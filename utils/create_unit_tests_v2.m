%% create_unit_tests_v2.m
%  crs_controller_tests.mldatx を要求と 1:1 の 19 TC 構造で再構築する
%
%  Suite              | TC (要求ID_名称)                     | Scenario
%  -------------------|--------------------------------------|------------------
%  Input_Conditioning | IC_001_Button_Rise_Detection         | rise_detection
%                     | IC_002_Button_Hold_Detection         | hold_detection
%                     | IC_003_Brake_Detection               | brake_active
%                     | IC_004_Key_Decode                    | key_decode
%                     | IC_005_Gear_Decode                   | gear_decode
%                     | IC_006_Speed_Range_Check             | speed_range
%  Mode_Manager       | MM_001_Mode_Transitions              | activate
%                     | MM_002_Accel_Override                | driver_override
%  Target_Speed_Mgr   | TSM_001_Set_Target                   | set_target
%                     | TSM_002_Resume_Target                | resume_prev
%                     | TSM_003_Increment                    | increment
%                     | TSM_004_Decrement                    | decrement
%                     | TSM_005_Accel_Release                | accel_release
%                     | TSM_006_Speed_Clamp                  | speed_clamp
%  PID_Throttle_Ctrl  | PID_001_Speed_Error                  | tracking_error
%                     | PID_002_PID_Core                     | bumpless_transfer
%                     | PID_003_Output_Gate                  | pid_disabled
%  Output_Logic       | OL_001_Activation_Check              | activated_no_override
%                     | OL_002_Throttle_Output               | saturation

projDir = 'C:\work\demos\CruiseControl';
cd(projDir);

%% Delete and recreate test file
tmPath = fullfile(projDir, 'crs_controller_tests.mldatx');
sltest.testmanager.clear();          % purge old file from memory
if exist(tmPath, 'file'), delete(tmPath); end

% Clear any base-workspace vars that conflict with crs_controller.sldd signals
evalin('base', 'clear status key gear enbl cncl inc dec resume vehicle_speed brakeP');
open_system('crs_controller');

tf = sltest.testmanager.TestFile(tmPath);

%% TC definition table
%  cols: {SuiteName, TCname, Component, HarnessName, SEblock, Scenario}
ic  = 'crs_controller/Input_Conditioning';
mm  = 'crs_controller/Mode_Manager';
tsm = 'crs_controller/Target_Speed_Manager';
pid = 'crs_controller/PID_Throttle_Controller';
ol  = 'crs_controller/Output_Logic';

tcDefs = {
  'Input_Conditioning',    'IC_001_Button_Rise_Detection', ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'rise_detection';
  'Input_Conditioning',    'IC_002_Button_Hold_Detection', ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'hold_detection';
  'Input_Conditioning',    'IC_003_Brake_Detection',       ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'brake_active';
  'Input_Conditioning',    'IC_004_Key_Decode',            ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'key_decode';
  'Input_Conditioning',    'IC_005_Gear_Decode',           ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'gear_decode';
  'Input_Conditioning',    'IC_006_Speed_Range_Check',     ic,  'Input_Conditioning_Harness',    'Input_Conditioning_Harness/Harness Inputs',    'speed_range';
  'Mode_Manager',          'MM_001_Mode_Transitions',      mm,  'Mode_Manager_Harness',          'Mode_Manager_Harness/Harness Inputs',          'activate';
  'Mode_Manager',          'MM_002_Accel_Override',        mm,  'Mode_Manager_Harness',          'Mode_Manager_Harness/Harness Inputs',          'driver_override';
  'Target_Speed_Manager',  'TSM_001_Set_Target',           tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'set_target';
  'Target_Speed_Manager',  'TSM_002_Resume_Target',        tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'resume_prev';
  'Target_Speed_Manager',  'TSM_003_Increment',            tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'increment';
  'Target_Speed_Manager',  'TSM_004_Decrement',            tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'decrement';
  'Target_Speed_Manager',  'TSM_005_Accel_Release',        tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'accel_release';
  'Target_Speed_Manager',  'TSM_006_Speed_Clamp',          tsm, 'Target_Speed_Manager_Harness',  'Target_Speed_Manager_Harness/Harness Inputs',  'speed_clamp';
  'PID_Throttle_Controller','PID_001_Speed_Error',         pid, 'PID_Throttle_Controller_Harness','PID_Throttle_Controller_Harness/Harness Inputs','tracking_error';
  'PID_Throttle_Controller','PID_002_PID_Core',            pid, 'PID_Throttle_Controller_Harness','PID_Throttle_Controller_Harness/Harness Inputs','bumpless_transfer';
  'PID_Throttle_Controller','PID_003_Output_Gate',         pid, 'PID_Throttle_Controller_Harness','PID_Throttle_Controller_Harness/Harness Inputs','pid_disabled';
  'Output_Logic',          'OL_001_Activation_Check',      ol,  'Output_Logic_Harness',          'Output_Logic_Harness/Harness Inputs',          'activated_no_override';
  'Output_Logic',          'OL_002_Throttle_Output',       ol,  'Output_Logic_Harness',          'Output_Logic_Harness/Harness Inputs',          'saturation';
};

%% Build test cases
currentSuite = '';
suiteObj     = [];
tcObjects    = cell(size(tcDefs,1), 1);

for k = 1:size(tcDefs,1)
    suiteName = tcDefs{k,1};
    tcName    = tcDefs{k,2};
    comp      = tcDefs{k,3};
    hname     = tcDefs{k,4};
    seBlk     = tcDefs{k,5};
    scenario  = tcDefs{k,6};

    % Create suite on first TC of each subsystem group
    if ~strcmp(suiteName, currentSuite)
        suiteObj     = tf.createTestSuite(suiteName);
        currentSuite = suiteName;
    end

    evalin('base', 'clear status');

    [tc, ~] = sltest.testmanager.createTestForComponent( ...
        'TestFile', suiteObj, ...
        'Component', comp, ...
        'TestType', 'simulation', ...
        'UseComponentInputs', false);

    tc.Name = tcName;
    tc.setProperty('HarnessOwner', comp, 'HarnessName', hname);

    iter = sltest.testmanager.TestIteration();
    iter.setModelParam(seBlk, 'ActiveScenario', scenario);
    tc.addIteration(iter, scenario);

    tcObjects{k} = tc;
    fprintf('  [%2d] %-45s <- %s\n', k, tcName, scenario);
end

%% Clean up auto-created harnesses
knownHarnesses = { ...
    'Input_Conditioning_Harness', 'Mode_Manager_Harness', ...
    'Target_Speed_Manager_Harness', 'PID_Throttle_Controller_Harness', ...
    'Output_Logic_Harness'};
allH = sltest.harness.find('crs_controller', 'SearchDepth', 0);
for k = 1:numel(allH)
    if ~any(strcmp(allH(k).name, knownHarnesses))
        sltest.harness.delete('crs_controller', allH(k).name);
        fprintf('  Removed auto-harness: %s\n', allH(k).name);
    end
end

tf.saveToFile();

%% Summary
fprintf('\n=== Test File Summary ===\n');
fprintf('File : %s\n', tmPath);
fprintf('TCs  : %d (1 per requirement)\n', numel(tcObjects));

%% Save TC objects to base workspace for link creation step
assignin('base', 'tcObjects', tcObjects);
fprintf('\ntcObjects saved to base workspace.\n');
