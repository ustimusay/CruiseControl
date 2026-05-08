%% create_mm_port_links.m
%  Mode_Manager の各要求に対し、直接関連する Inport / Outport ブロックへの
%  Implement リンクを slreq.createLink で作成する

projDir  = 'C:\work\demos\CruiseControl';
mmParent = 'crs_controller/Mode_Manager';

%% モデル・要求セットを開く
if ~bdIsLoaded('crs_controller'), open_system('crs_controller'); end
rs = slreq.load(fullfile(projDir, 'crs_controller_requirements.slreqx'));

%% リンク定義テーブル
%  { reqId, {Inport名}, {Outport名} }
linkDefs = {
    'MM-001', {'enbl_rise','cncl_rise','set_rise','resume_rise', ...
               'brake_active','key_on','gear_drive','speed_in_range','has_prev_target'}, ...
              {'op_mode','enable_pid','reset_pid','reqDrv','status'};
    'MM-002', {'driver_throttle'}, ...
              {'accel_override','init_pid_to_drv'};
};

%% リンク作成
fprintf('=== Creating Mode_Manager Inport/Outport links ===\n');
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

    for i = 1:numel(inPorts)
        blkPath = [mmParent '/' inPorts{i}];
        try
            slreq.createLink(blkPath, req);
            nIn = nIn + 1;
        catch e
            fprintf('  [%s] Inport "%s" WARN: %s\n', reqId, inPorts{i}, e.message);
        end
    end

    for i = 1:numel(outPorts)
        blkPath = [mmParent '/' outPorts{i}];
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
