%% create_test_scenarios.m
%  Writes Signal Editor scenario data for each subsystem harness.
%  Run after create_unit_tests.m has created the harnesses.

projDir = 'C:\work\demos\CruiseControl';
cd(projDir);

%% --- Helper ---------------------------------------------------------------
function ts = mkTs(name, times, values)
    ts = timeseries(values(:), times(:));
    ts.Name = name;
end

function ds = addSig(ds, name, times, values)
    ds = addElement(ds, mkTs(name, times, values), name);
end

%% ==========================================================================
%  1. Input_Conditioning (10 inputs)
%     enbl,cncl,speed_set,resume,inc,dec: logical
%     brakeP,vehicle_speed: single   key,gear: uint8
%% ==========================================================================
fprintf('Building Input_Conditioning scenarios...\n');

% Default baseline: key=2(on), gear=2(drive), speed=70 km/h, all btns=0
function ds = ic_base()
    ds = Simulink.SimulationData.Dataset;
    ds = addSig(ds,'enbl',         [0;10], logical([0;0]));
    ds = addSig(ds,'cncl',         [0;10], logical([0;0]));
    ds = addSig(ds,'speed_set',    [0;10], logical([0;0]));
    ds = addSig(ds,'resume',       [0;10], logical([0;0]));
    ds = addSig(ds,'inc',          [0;10], logical([0;0]));
    ds = addSig(ds,'dec',          [0;10], logical([0;0]));
    ds = addSig(ds,'brakeP',       [0;10], single([0;0]));
    ds = addSig(ds,'key',          [0;10], uint8([2;2]));
    ds = addSig(ds,'gear',         [0;10], uint8([2;2]));
    ds = addSig(ds,'vehicle_speed',[0;10], single([70;70]));
end

% Scenario 1 – enbl rise detection: enbl 0→1→0 (one step pulse at t=1s)
rise_detection = ic_base();
rise_detection = modifyDataset(rise_detection,'enbl', ...
    mkTs('enbl',[0;1.0;1.01;10],logical([0;1;0;0])));

% Scenario 2 – hold detection: inc held 1.1 s (> threshold=50 samples×0.01=0.5 s)
hold_detection = ic_base();
hold_detection = modifyDataset(hold_detection,'inc', ...
    mkTs('inc',[0;1.0;2.1;10],logical([0;1;0;0])));

% Scenario 3 – brake active: brakeP steps to 10 > threshold=5
brake_active = ic_base();
brake_active = modifyDataset(brake_active,'brakeP', ...
    mkTs('brakeP',[0;1.0;10],single([0;10;10])));

% Scenario 4 – speed range: in range (70), then below Spd_Min=40 (30 km/h)
speed_range = ic_base();
speed_range = modifyDataset(speed_range,'vehicle_speed', ...
    mkTs('vehicle_speed',[0;1.0;5.0;10],single([70;70;30;30])));

ic_file = fullfile(projDir,'Input_Conditioning_Harness_HarnessInputs.mat');
save(ic_file,'rise_detection','hold_detection','brake_active','speed_range');
fprintf('  Saved: %s  (4 scenarios)\n', ic_file);

%% ==========================================================================
%  2. Mode_Manager (10 inputs)
%     logical: enbl_rise,cncl_rise,set_rise,resume_rise,brake_active,
%              key_on,gear_drive,speed_in_range,has_prev_target
%     single:  driver_throttle
%% ==========================================================================
fprintf('Building Mode_Manager scenarios...\n');

function ds = mm_base()
    ds = Simulink.SimulationData.Dataset;
    ds = addSig(ds,'enbl_rise',      [0;10], logical([0;0]));
    ds = addSig(ds,'cncl_rise',      [0;10], logical([0;0]));
    ds = addSig(ds,'set_rise',       [0;10], logical([0;0]));
    ds = addSig(ds,'resume_rise',    [0;10], logical([0;0]));
    ds = addSig(ds,'brake_active',   [0;10], logical([0;0]));
    ds = addSig(ds,'key_on',         [0;10], logical([1;1]));   % key always on
    ds = addSig(ds,'gear_drive',     [0;10], logical([1;1]));   % drive gear
    ds = addSig(ds,'speed_in_range', [0;10], logical([1;1]));   % speed OK
    ds = addSig(ds,'has_prev_target',[0;10], logical([0;0]));
    ds = addSig(ds,'driver_throttle',[0;10], single([10;10]));  % 10 < 15 (no override)
end

% Scenario 1 – enable: enbl_rise at t=1 → transitions Disabled→Enabled
enable = mm_base();
enable = modifyDataset(enable,'enbl_rise', ...
    mkTs('enbl_rise',[0;1.0;1.01;10],logical([0;1;0;0])));

% Scenario 2 – activate: enbl_rise at t=1, then set_rise at t=2 → Activated
activate = mm_base();
activate = modifyDataset(activate,'enbl_rise', ...
    mkTs('enbl_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
activate = modifyDataset(activate,'set_rise', ...
    mkTs('set_rise',[0;2.0;2.01;10],logical([0;1;0;0])));

% Scenario 3 – driver override: Enabled→Activated, then driver_throttle > 15
driver_override = mm_base();
driver_override = modifyDataset(driver_override,'enbl_rise', ...
    mkTs('enbl_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
driver_override = modifyDataset(driver_override,'set_rise', ...
    mkTs('set_rise',[0;2.0;2.01;10],logical([0;1;0;0])));
driver_override = modifyDataset(driver_override,'driver_throttle', ...
    mkTs('driver_throttle',[0;3.0;10],single([10;20;20])));  % 20 > Drv_Override_Hi=15

% Scenario 4 – cancel from activated: enbl→set→cncl → back to Enabled
cancel = mm_base();
cancel = modifyDataset(cancel,'enbl_rise', ...
    mkTs('enbl_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
cancel = modifyDataset(cancel,'set_rise', ...
    mkTs('set_rise',[0;2.0;2.01;10],logical([0;1;0;0])));
cancel = modifyDataset(cancel,'cncl_rise', ...
    mkTs('cncl_rise',[0;3.0;3.01;10],logical([0;1;0;0])));

mm_file = fullfile(projDir,'Mode_Manager_Harness_HarnessInputs.mat');
save(mm_file,'enable','activate','driver_override','cancel');
fprintf('  Saved: %s  (4 scenarios)\n', mm_file);

%% ==========================================================================
%  3. Target_Speed_Manager (9 inputs)
%     op_mode: opMode enum
%     logical: set_rise,resume_rise,inc_rise,dec_rise,inc_held,dec_held,accel_override
%     single:  vehicle_speed
%% ==========================================================================
fprintf('Building Target_Speed_Manager scenarios...\n');

function ds = tsm_base()
    ds = Simulink.SimulationData.Dataset;
    % op_mode = Activated from the start (skip the enable sequence)
    ds = addSig(ds,'op_mode',      [0;10], [opMode.Activated; opMode.Activated]);
    ds = addSig(ds,'set_rise',     [0;10], logical([0;0]));
    ds = addSig(ds,'resume_rise',  [0;10], logical([0;0]));
    ds = addSig(ds,'inc_rise',     [0;10], logical([0;0]));
    ds = addSig(ds,'dec_rise',     [0;10], logical([0;0]));
    ds = addSig(ds,'inc_held',     [0;10], logical([0;0]));
    ds = addSig(ds,'dec_held',     [0;10], logical([0;0]));
    ds = addSig(ds,'vehicle_speed',[0;10], single([70;70]));
    ds = addSig(ds,'accel_override',[0;10],logical([0;0]));
end

% Scenario 1 – set speed: set_rise at t=1, vehicle_speed=70 → target=70
set_target = tsm_base();
set_target = modifyDataset(set_target,'set_rise', ...
    mkTs('set_rise',[0;1.0;1.01;10],logical([0;1;0;0])));

% Scenario 2 – increment: set at t=1, inc_rise at t=2 → target = 70+1=71
increment = tsm_base();
increment = modifyDataset(increment,'set_rise', ...
    mkTs('set_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
increment = modifyDataset(increment,'inc_rise', ...
    mkTs('inc_rise',[0;2.0;2.01;10],logical([0;1;0;0])));

% Scenario 3 – decrement: set at t=1, dec_rise at t=2 → target = 70-1=69
decrement = tsm_base();
decrement = modifyDataset(decrement,'set_rise', ...
    mkTs('set_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
decrement = modifyDataset(decrement,'dec_rise', ...
    mkTs('dec_rise',[0;2.0;2.01;10],logical([0;1;0;0])));

% Scenario 4 – resume: set speed at t=1, then op_mode→Disabled at t=2,
%              then back to Activated at t=3 with resume_rise at t=3.5
resume_prev = tsm_base();
resume_prev = modifyDataset(resume_prev,'set_rise', ...
    mkTs('set_rise',[0;1.0;1.01;10],logical([0;1;0;0])));
resume_prev = modifyDataset(resume_prev,'op_mode', ...
    mkTs('op_mode',[0;1.0;2.0;3.0;10], ...
         [opMode.Activated;opMode.Activated;opMode.Enabled;opMode.Activated;opMode.Activated]));
resume_prev = modifyDataset(resume_prev,'resume_rise', ...
    mkTs('resume_rise',[0;3.5;3.51;10],logical([0;1;0;0])));

tsm_file = fullfile(projDir,'Target_Speed_Manager_Harness_HarnessInputs.mat');
save(tsm_file,'set_target','increment','decrement','resume_prev');
fprintf('  Saved: %s  (4 scenarios)\n', tsm_file);

%% ==========================================================================
%  4. PID_Throttle_Controller (6 inputs — all single or logical)
%% ==========================================================================
fprintf('Building PID_Throttle_Controller scenarios...\n');

function ds = pid_base()
    ds = Simulink.SimulationData.Dataset;
    ds = addSig(ds,'cc_target_speed',[0;10], single([70;70]));
    ds = addSig(ds,'vehicle_speed',  [0;10], single([70;70]));
    ds = addSig(ds,'enable_pid',     [0;10], logical([1;1]));
    ds = addSig(ds,'reset_pid',      [0;10], logical([0;0]));
    ds = addSig(ds,'init_pid_to_drv',[0;10], logical([0;0]));
    ds = addSig(ds,'driver_throttle',[0;10], single([25;25]));
end

% Scenario 1 – at target speed: target=speed=70, steady-state throttle ≈ 0 error
at_target_speed = pid_base();

% Scenario 2 – tracking error: target=80, speed=70 → positive throttle output
tracking_error = pid_base();
tracking_error = modifyDataset(tracking_error,'cc_target_speed', ...
    mkTs('cc_target_speed',[0;1.0;10],single([70;80;80])));

% Scenario 3 – bumpless transfer: init_pid_to_drv=T for first 0.5 s
%   PID output should start at driver_throttle=30, then track normally
bumpless_transfer = pid_base();
bumpless_transfer = modifyDataset(bumpless_transfer,'init_pid_to_drv', ...
    mkTs('init_pid_to_drv',[0;0.5;10],logical([1;0;0])));
bumpless_transfer = modifyDataset(bumpless_transfer,'driver_throttle', ...
    mkTs('driver_throttle',[0;10],single([30;30])));
bumpless_transfer = modifyDataset(bumpless_transfer,'cc_target_speed', ...
    mkTs('cc_target_speed',[0;10],single([70;70])));
bumpless_transfer = modifyDataset(bumpless_transfer,'vehicle_speed', ...
    mkTs('vehicle_speed',[0;10],single([70;70])));

pid_file = fullfile(projDir,'PID_Throttle_Controller_Harness_HarnessInputs.mat');
save(pid_file,'at_target_speed','tracking_error','bumpless_transfer');
fprintf('  Saved: %s  (3 scenarios)\n', pid_file);

%% ==========================================================================
%  5. Output_Logic (4 inputs)
%     pid_throttle, driver_throttle: single
%     accel_override: logical
%     op_mode: opMode enum
%% ==========================================================================
fprintf('Building Output_Logic scenarios...\n');

function ds = ol_base()
    ds = Simulink.SimulationData.Dataset;
    ds = addSig(ds,'pid_throttle',    [0;10], single([50;50]));
    ds = addSig(ds,'driver_throttle', [0;10], single([30;30]));
    ds = addSig(ds,'accel_override',  [0;10], logical([0;0]));
    ds = addSig(ds,'op_mode',         [0;10], [opMode.Activated;opMode.Activated]);
end

% Scenario 1 – Activated + no override → output = pid_throttle (50)
activated_no_override = ol_base();

% Scenario 2 – Activated + driver override → output = driver_throttle (30)
activated_override = ol_base();
activated_override = modifyDataset(activated_override,'accel_override', ...
    mkTs('accel_override',[0;1.0;10],logical([0;1;1])));

% Scenario 3 – Not activated (Enabled) → output = driver_throttle
not_activated = ol_base();
not_activated = modifyDataset(not_activated,'op_mode', ...
    mkTs('op_mode',[0;10],[opMode.Enabled;opMode.Enabled]));

% Scenario 4 – Saturation: pid_throttle=150, no override → clamp to 100
saturation = ol_base();
saturation = modifyDataset(saturation,'pid_throttle', ...
    mkTs('pid_throttle',[0;1.0;10],single([50;150;150])));

ol_file = fullfile(projDir,'Output_Logic_Harness_HarnessInputs.mat');
save(ol_file,'activated_no_override','activated_override','not_activated','saturation');
fprintf('  Saved: %s  (4 scenarios)\n', ol_file);

fprintf('\nAll scenario MAT files written.\n');

%% Helper: replace one element in a Dataset by name
function ds = modifyDataset(ds, sigName, newTs)
    for k = 1:ds.numElements
        if strcmp(ds{k}.Name, sigName)
            ds = ds.setElement(k, newTs, sigName);
            return;
        end
    end
    error('Signal "%s" not found in dataset', sigName);
end
