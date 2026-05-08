%% create_ic_port_links.m
%  Input_Conditioning の各要求に対し、直接関連する Inport / Outport ブロックへの
%  Implement リンクを slreq.createLink で作成する

projDir  = 'C:\work\demos\CruiseControl';
icParent = 'crs_controller/Input_Conditioning';

%% モデル・要求セットを開く
if ~bdIsLoaded('crs_controller'), open_system('crs_controller'); end
rs = slreq.load(fullfile(projDir, 'crs_controller_requirements.slreqx'));

%% リンク定義テーブル
%  { reqId, {Inport名}, {Outport名} }
linkDefs = {
    'IC-001', {'enbl','cncl','speed_set','resume','inc','dec'}, ...
              {'enbl_rise','cncl_rise','set_rise','resume_rise','inc_rise','dec_rise'};
    'IC-002', {'inc','dec'}, ...
              {'inc_held','dec_held'};
    'IC-003', {'brakeP'}, ...
              {'brake_active'};
    'IC-004', {'key'}, ...
              {'key_on'};
    'IC-005', {'gear'}, ...
              {'gear_drive'};
    'IC-006', {'vehicle_speed'}, ...
              {'speed_in_range'};
};

%% リンク作成
fprintf('=== Creating Inport/Outport links ===\n');
totalLinks = 0;

for k = 1:size(linkDefs, 1)
    reqId    = linkDefs{k,1};
    inPorts  = linkDefs{k,2};
    outPorts = linkDefs{k,3};

    req = find(rs, 'Type', 'Requirement', 'Id', reqId);
    if isempty(req)
        fprintf('  [%s] requirement not found — skipped\n', reqId);
        continue;
    end

    nIn = 0; nOut = 0;

    % Inport ブロックへのリンク
    for i = 1:numel(inPorts)
        blkPath = [icParent '/' inPorts{i}];
        try
            slreq.createLink(blkPath, req);
            nIn = nIn + 1;
        catch e
            fprintf('  [%s] Inport "%s" WARN: %s\n', reqId, inPorts{i}, e.message);
        end
    end

    % Outport ブロックへのリンク
    for i = 1:numel(outPorts)
        blkPath = [icParent '/' outPorts{i}];
        try
            slreq.createLink(blkPath, req);
            nOut = nOut + 1;
        catch e
            fprintf('  [%s] Outport "%s" WARN: %s\n', reqId, outPorts{i}, e.message);
        end
    end

    fprintf('  [%s] Inport x%d, Outport x%d linked\n', reqId, nIn, nOut);
    totalLinks = totalLinks + nIn + nOut;
end

%% リンクセット保存
linkSets = slreq.find('Type', 'LinkSet');
for i = 1:numel(linkSets)
    save(linkSets(i));
end

fprintf('\nTotal links created: %d\n', totalLinks);
fprintf('Link sets saved.\n');
