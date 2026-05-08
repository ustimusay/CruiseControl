%% generate_mm_traceability_report.m
%  Mode_Manager テストトレーサビリティレポートを PDF で生成する
%
%  フェーズ分離:
%    Phase A: Mode_Manager 内部スクリーンショット (sim 不要)
%    Phase B: 波形プロット (harness sim × 2)
%    Phase C: Model Slicer ダイナミックスライス (crs_controller で実行)
%    Phase D: PDF 組み立て

projDir  = 'C:\work\demos\CruiseControl';
imgDir   = fullfile(projDir, 'reports', 'imgs');
outFile  = fullfile(projDir, 'reports', 'MM_traceability_report.pdf');
mmParent  = 'crs_controller/Mode_Manager';
chartBlk  = [mmParent '/ModeManager_chart'];
hName     = 'Mode_Manager_Harness';
seBlk    = [hName '/Harness Inputs'];
matFile  = fullfile(projDir, 'Mode_Manager_Harness_HarnessInputs.mat');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end

%% ── 1. アーティファクト読み込み ──────────────────────────────────────────
if ~bdIsLoaded('crs_controller'), open_system('crs_controller'); end
rs = slreq.load(fullfile(projDir, 'crs_controller_requirements.slreqx'));

tmPath = fullfile(projDir, 'crs_controller_tests.mldatx');
existingFiles = sltest.testmanager.getTestFiles();
tfObj = [];
for i = 1:numel(existingFiles)
    if strcmp(existingFiles(i).Name, 'crs_controller_tests')
        tfObj = existingFiles(i); break;
    end
end
if isempty(tfObj), tfObj = sltest.testmanager.load(tmPath); end

%% ── 2. リンクマップ・UUID→TC マップ ──────────────────────────────────────
verifyMap = containers.Map();
linkSets  = slreq.find('Type', 'LinkSet');
for i = 1:numel(linkSets)
    lnks = linkSets(i).getLinks();
    for j = 1:numel(lnks)
        lnk = lnks(j);
        if strcmp(lnk.Type, 'Verify')
            verifyMap(lnk.destination.id) = lnk.source.id;
        end
    end
end

tcMap  = containers.Map();
suites = tfObj.getTestSuites();
for s = 1:numel(suites)
    tcs = suites(s).getTestCases();
    for t = 1:numel(tcs)
        tcMap(tcs(t).UUID) = tcs(t);
    end
end

%% ── 3. MM 定義テーブル ───────────────────────────────────────────────────
mmDefs = {
    'MM-001', 'ModeManager_chart';
    'MM-002', 'ModeManager_chart';
};

%% ── 4. 要求リンクから Outport パスマップを構築 ──────────────────────────────
fprintf('Building Outport map from requirement links...\n');
outPortMap = containers.Map();
for k = 1:size(mmDefs, 1)
    reqId = mmDefs{k,1};
    req   = find(rs, 'Type', 'Requirement', 'Id', reqId);
    paths = {};
    for ls = 1:numel(linkSets)
        lnks = linkSets(ls).getLinks();
        for j = 1:numel(lnks)
            lnk = lnks(j);
            if strcmp(lnk.Type, 'Implement') && strcmp(lnk.destination.id, req.Id)
                try
                    sid   = strrep(lnk.source.id, ':', '');
                    h     = Simulink.ID.getHandle(['crs_controller:' sid]);
                    bpath = getfullname(h);
                    if strcmp(get_param(bpath, 'BlockType'), 'Outport') && ...
                       startsWith(bpath, mmParent)
                        paths{end+1} = bpath; %#ok<AGROW>
                    end
                catch
                end
            end
        end
    end
    outPortMap(reqId) = paths;
    pnames = cellfun(@(p) p(length(mmParent)+2:end), paths, 'UniformOutput', false);
    fprintf('  [%s] Outports: %s\n', reqId, strjoin(pnames, ', '));
end

%% ═══════════════════════════════════════════════════════════════════════════
%  Phase A: Mode_Manager 内部スクリーンショット
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('=== Phase A: Mode_Manager screenshots ===\n');
open_system(mmParent, 'force');
drawnow; pause(0.8);

for k = 1:size(mmDefs, 1)
    reqId  = mmDefs{k,1};
    ssFile = fullfile(imgDir, ['MM_' reqId '_subsys.png']);
    print(['-s' mmParent], '-dpng', '-r96', ssFile);
    mmDefs{k,3} = ssFile;
    fprintf('  [%s] screenshot saved\n', reqId);
end

%% ═══════════════════════════════════════════════════════════════════════════
%  Phase B: 波形プロット
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('\n=== Phase B: Waveform plots ===\n');
evalin('base', 'clear status key gear enbl cncl inc dec resume vehicle_speed brakeP');

reopenHarness(mmParent, hName);

for k = 1:size(mmDefs, 1)
    reqId    = mmDefs{k,1};
    iters    = tcMap(verifyMap(reqId)).getIterations();
    scenario = iters(1).Name;
    waveFile = fullfile(imgDir, ['MM_' reqId '_waveform.png']);

    ensureStopped(hName);
    set_param(seBlk, 'ActiveScenario', scenario);
    simOut = sim(hName, 'StopTime', '10');
    captureWaveform(scenario, matFile, simOut, waveFile, reqId);
    mmDefs{k,4} = waveFile;
    fprintf('  [%s] scenario=%s\n', reqId, scenario);
end
ensureStopped(hName);
try, bdclose(hName); catch, end

%% ═══════════════════════════════════════════════════════════════════════════
%  Phase C: Model Slicer ハイライト (動的スライス)
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('\n=== Phase C: Model Slicer highlights (dynamic) ===\n');
open_system(mmParent, 'force');
drawnow; pause(0.5);

for k = 1:size(mmDefs, 1)
    reqId        = mmDefs{k,1};
    outPortPaths = outPortMap(reqId);
    slicerFile      = fullfile(imgDir, ['MM_' reqId '_slicer.png']);
    slicerMmFile    = fullfile(imgDir, ['MM_' reqId '_slicer_mm.png']);
    slicerChartFile = fullfile(imgDir, ['MM_' reqId '_slicer_chart.png']);

    sc = [];
    try
        sc = slslicer('crs_controller');

        % Starting point をリセット
        existingSP = sc.StartingPoint;
        fprintf('  [%s] existing SPs before reset: %d\n', reqId, numel(existingSP));
        for m = 1:numel(existingSP)
            try
                sc.removeStartingPoint(existingSP(m).Path);
            catch re
                fprintf('    removeStartingPoint warn: %s\n', re.message);
            end
        end

        for m = 1:numel(outPortPaths)
            sc.addStartingPoint(outPortPaths{m});
        end
        fprintf('  [%s] SPs after add: %d\n', reqId, numel(sc.StartingPoint));

        sc.simulate();

        % シミュレーション完了を待機
        for w = 1:60
            try
                if strcmp(get_param('crs_controller', 'SimulationStatus'), 'stopped'), break; end
            catch, break; end
            pause(0.5);
        end
        try
            if ~strcmp(get_param('crs_controller', 'SimulationStatus'), 'stopped')
                set_param('crs_controller', 'SimulationCommand', 'stop'); pause(1);
            end
        catch, end
        drawnow; pause(0.8);

        % 1枚目: crs_controller ルートレベル
        open_system('crs_controller');
        drawnow; pause(0.8);
        print('-scrs_controller', '-dpng', '-r96', slicerFile);

        % 2枚目: Mode_Manager 内部
        open_system(mmParent, 'force');
        drawnow; pause(1.0);
        print(['-s' mmParent], '-dpng', '-r96', slicerMmFile);

        % 3枚目: ModeManager_chart (Stateflow チャート内部)
        open_system(chartBlk, 'force');
        drawnow; pause(1.0);
        print(['-s' chartBlk], '-dpng', '-r96', slicerChartFile);

        sc.terminate();
        drawnow; pause(0.5);
        fprintf('  [%s] slicer OK (dynamic)\n', reqId);
    catch e
        fprintf('  [%s] slicer WARN: %s\n', reqId, e.message);
        if ~isempty(sc), try, sc.terminate(); catch, end; end
        try
            if ~strcmp(get_param('crs_controller', 'SimulationStatus'), 'stopped')
                set_param('crs_controller', 'SimulationCommand', 'stop'); pause(1);
            end
        catch, end
        slicerFile      = '';
        slicerMmFile    = '';
        slicerChartFile = '';
    end
    mmDefs{k,5} = slicerFile;
    mmDefs{k,6} = slicerMmFile;
    mmDefs{k,7} = slicerChartFile;
end

%% ═══════════════════════════════════════════════════════════════════════════
%  Phase D: PDF 組み立て
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('\n=== Phase D: Building PDF report ===\n');
import mlreportgen.report.*
import mlreportgen.dom.*

rpt = Report(outFile, 'pdf');

tp          = TitlePage();
tp.Title    = 'CruiseControl';
tp.Subtitle = 'テストトレーサビリティレポート — Mode_Manager';
tp.Author   = sprintf('自動生成: %s', string(datetime('now'), 'yyyy-MM-dd'));
add(rpt, tp);
add(rpt, TableOfContents());

ch       = Chapter();
ch.Title = 'Mode_Manager テストケース（MM-001 〜 MM-002）';

for k = 1:size(mmDefs, 1)
    reqId    = mmDefs{k,1};
    req      = find(rs, 'Type', 'Requirement', 'Id', reqId);
    tc       = tcMap(verifyMap(reqId));
    iters_d  = tc.getIterations();
    scenario = iters_d(1).Name;

    descText = req.Description;
    if isempty(descText), descText = '—'; end

    ssFile          = mmDefs{k,3};
    waveFile        = mmDefs{k,4};
    slicerFile      = mmDefs{k,5};
    slicerMmFile    = mmDefs{k,6};
    slicerChartFile = mmDefs{k,7};

    sec       = Section();
    sec.Title = [reqId ' : ' tc.Name];

    % テストケース情報
    add(sec, makeHeading('テストケース情報'));
    add(sec, makeTable2Col({
        'テストケース名', tc.Name;
        'スイート',       tc.Parent.Name;
        'シナリオ',       scenario;
    }, '#D6EAF8'));

    % 紐づく要求
    add(sec, makeHeading('紐づく要求'));
    add(sec, makeTable2Col({
        '要求 ID',      req.Id;
        'Summary',     req.Summary;
        'Description', descText;
        '実装ブロック', mmParent;
    }, '#D5F5E3'));

    % Mode_Manager 内部
    add(sec, makeHeading('実装モデル内部: Mode_Manager'));
    add(sec, addImg(ssFile, '16cm'));

    % 波形
    add(sec, makeHeading(['シミュレーション波形  (シナリオ: ' scenario ')']));
    p = Paragraph('青: 入力シグナル (変化したもののみ)　　赤: 出力シグナル (変化したもののみ)');
    p.Style = {FontSize('8pt'), OuterMargin('0pt','0pt','0pt','2pt')};
    add(sec, p);
    add(sec, addImg(waveFile, '16cm'));

    % Model Slicer (1/2) ルートレベル
    opaths = outPortMap(reqId);
    pnames = cellfun(@(p) p(length(mmParent)+2:end), opaths, 'UniformOutput', false);
    spNote = strjoin(pnames, ', ');
    add(sec, makeHeading(['Model Slicer (1/3): crs_controller ルートレベル — starting point: ' spNote]));
    p2 = Paragraph('5 サブシステムのうち Mode_Manager (および依存ブロック) がハイライト');
    p2.Style = {FontSize('8pt'), OuterMargin('0pt','0pt','0pt','2pt')};
    add(sec, p2);
    add(sec, addImg(slicerFile, '16cm'));

    % Model Slicer (2/3) Mode_Manager 内部
    add(sec, makeHeading(['Model Slicer (2/3): Mode_Manager 内部 — ' reqId ' 関連信号がハイライト']));
    p3 = Paragraph('Mode_Manager 内部の Stateflow チャートと接続信号がハイライト');
    p3.Style = {FontSize('8pt'), OuterMargin('0pt','0pt','0pt','2pt')};
    add(sec, p3);
    add(sec, addImg(slicerMmFile, '16cm'));

    % Model Slicer (3/3) Stateflow チャート内部
    add(sec, makeHeading(['Model Slicer (3/3): ModeManager_chart 内部 — ' reqId ' 関連状態・遷移がハイライト']));
    p4 = Paragraph('Stateflow チャート内部の状態 (Disabled/Enabled/Activated) と遷移条件がハイライト');
    p4.Style = {FontSize('8pt'), OuterMargin('0pt','0pt','0pt','2pt')};
    add(sec, p4);
    add(sec, addImg(slicerChartFile, '16cm'));

    add(ch, sec);
    fprintf('  Section done: %s\n', tc.Name);
end

add(rpt, ch);
close(rpt);
fprintf('\nReport saved: %s\n', outFile);
rptview(rpt);

%% =========================================================================
%  ローカルヘルパー
%% =========================================================================
function reopenHarness(compPath, harnessName)
    if bdIsLoaded(harnessName)
        try, bdclose(harnessName); catch, end
    end
    sltest.harness.open(compPath, harnessName);
    drawnow; pause(1);
    try, set_param(harnessName, 'FastRestart', 'off'); catch, end
end

function ensureStopped(modelName)
    try
        status = get_param(modelName, 'SimulationStatus');
        if ~strcmp(status, 'stopped')
            set_param(modelName, 'SimulationCommand', 'stop');
            pause(0.5);
        end
    catch, end
end

function captureWaveform(scenario, matFile, simOut, waveFile, reqId)
    S  = load(matFile);
    ds = S.(scenario);

    inSigs = {};
    for i = 1:ds.numElements
        ts = ds{i};
        if range(double(ts.Data(:))) > 0
            inSigs{end+1} = ts; %#ok<AGROW>
        end
    end

    outSigs = {};
    for i = 1:simOut.yout.numElements
        el = simOut.yout{i};
        if range(double(el.Values.Data(:))) > 0
            outSigs{end+1} = el; %#ok<AGROW>
        end
    end

    nIn = numel(inSigs); nOut = numel(outSigs);
    if nIn + nOut == 0
        fig = figure('Visible','off','Position',[100 100 900 200]);
        text(0.5,0.5,'(変化するシグナルなし)','HorizontalAlignment','center');
        axis off; saveas(fig, waveFile); close(fig); return;
    end

    nTotal = nIn + nOut;
    fig = figure('Visible','off','Position',[100 100 900 max(300, nTotal*90)]);

    for i = 1:nIn
        ax = subplot(nTotal, 1, i);
        ts = inSigs{i};
        stairs(ts.Time, double(ts.Data(:)), 'Color', [0.1 0.4 0.8], 'LineWidth', 1.2);
        ylabel(ts.Name, 'Interpreter','none','FontSize',8);
        grid on; ax.XTickLabel = {};
        ax.Color = [0.94 0.97 1.0];
        if i == 1
            title(sprintf('[%s]  Scenario: %s', reqId, scenario), ...
                'Interpreter','none','FontSize',9);
        end
    end

    for i = 1:nOut
        ax = subplot(nTotal, 1, nIn+i);
        el = outSigs{i};
        stairs(el.Values.Time, double(el.Values.Data(:)), ...
               'Color', [0.8 0.2 0.1], 'LineWidth', 1.2);
        ylabel(el.Name, 'Interpreter','none','FontSize',8);
        grid on;
        if i < nOut, ax.XTickLabel = {}; end
        ax.Color = [1.0 0.96 0.94];
    end
    xlabel('Time (s)');

    saveas(fig, waveFile);
    close(fig);
end

function h = makeHeading(txt)
    import mlreportgen.dom.*
    h = Paragraph(txt);
    h.Style = {FontSize('11pt'), Bold, OuterMargin('0pt','0pt','8pt','3pt')};
end

function el = addImg(imgFile, width)
    import mlreportgen.dom.*
    if ~isempty(imgFile) && exist(imgFile, 'file')
        el = Image(imgFile);
        el.Style = {Width(width)};
    else
        el = Paragraph('(画像未生成)');
    end
end

function tbl = makeTable2Col(data, headerBg)
    import mlreportgen.dom.*
    tbl = Table();
    tbl.Style = {Width('100%'), Border('solid','#888888','1pt'), ...
                 ColSep('solid','#CCCCCC','1pt'), RowSep('solid','#CCCCCC','1pt')};
    for i = 1:size(data,1)
        row = TableRow();
        e1 = TableEntry();
        e1.Style = {BackgroundColor(headerBg), Width('30%'), ...
                    InnerMargin('4pt','4pt','4pt','4pt')};
        p1 = Paragraph(data{i,1}); p1.Style = {Bold};
        append(e1, p1); append(row, e1);
        e2 = TableEntry();
        e2.Style = {InnerMargin('4pt','4pt','4pt','4pt')};
        append(e2, Paragraph(data{i,2}));
        append(row, e2);
        append(tbl, row);
    end
end
