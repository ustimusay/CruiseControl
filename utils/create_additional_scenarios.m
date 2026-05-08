%% create_additional_scenarios.m
%  新規シナリオを追加して要求との 1:1 対応を完成させる
%
%  追加:
%    IC   : key_decode (IC-004), gear_decode (IC-005)
%    TSM  : accel_release (TSM-005), speed_clamp (TSM-006)
%    PID  : pid_disabled (PID-003)

projDir = 'C:\work\demos\CruiseControl';
cd(projDir);

%% Input_Conditioning -------------------------------------------------------
load('Input_Conditioning_Harness_HarnessInputs.mat');

% IC-004: key_decode — key 0→2 at t=1 → key_on should become true
key_decode = ic_base();
key_decode = modifyDataset(key_decode, 'key', ...
    mkTs('key', [0;1.0;10], uint8([0;2;2])));

% IC-005: gear_decode — gear 0→2 at t=1 → gear_drive should become true
gear_decode = ic_base();
gear_decode = modifyDataset(gear_decode, 'gear', ...
    mkTs('gear', [0;1.0;10], uint8([0;2;2])));

save('Input_Conditioning_Harness_HarnessInputs.mat', ...
    'rise_detection','hold_detection','brake_active','speed_range', ...
    'key_decode','gear_decode');
fprintf('IC: added key_decode, gear_decode\n');

%% Target_Speed_Manager -----------------------------------------------------
load('Target_Speed_Manager_Harness_HarnessInputs.mat');

% TSM-005: accel_release — set at t=1, accel_override true@t=2 then false@t=4
%   After override drops, target should reset to current vehicle_speed
accel_release = tsm_base();
accel_release = modifyDataset(accel_release, 'set_rise', ...
    mkTs('set_rise', [0;1.0;1.01;10], logical([0;1;0;0])));
accel_release = modifyDataset(accel_release, 'accel_override', ...
    mkTs('accel_override', [0;2.0;4.0;4.01;10], logical([0;1;1;0;0])));

% TSM-006: speed_clamp — vehicle_speed=30 (below Spd_Min=40), set at t=1
%   cc_target_speed should be clamped to Spd_Min (40 km/h)
speed_clamp = tsm_base();
speed_clamp = modifyDataset(speed_clamp, 'vehicle_speed', ...
    mkTs('vehicle_speed', [0;10], single([30;30])));
speed_clamp = modifyDataset(speed_clamp, 'set_rise', ...
    mkTs('set_rise', [0;1.0;1.01;10], logical([0;1;0;0])));

save('Target_Speed_Manager_Harness_HarnessInputs.mat', ...
    'set_target','increment','decrement','resume_prev', ...
    'accel_release','speed_clamp');
fprintf('TSM: added accel_release, speed_clamp\n');

%% PID_Throttle_Controller --------------------------------------------------
load('PID_Throttle_Controller_Harness_HarnessInputs.mat');

% PID-003: pid_disabled — enable_pid=false, target≠speed → output must be 0
pid_disabled = pid_base();
pid_disabled = modifyDataset(pid_disabled, 'enable_pid', ...
    mkTs('enable_pid', [0;10], logical([0;0])));
pid_disabled = modifyDataset(pid_disabled, 'cc_target_speed', ...
    mkTs('cc_target_speed', [0;10], single([80;80])));

save('PID_Throttle_Controller_Harness_HarnessInputs.mat', ...
    'at_target_speed','tracking_error','bumpless_transfer','pid_disabled');
fprintf('PID: added pid_disabled\n');

fprintf('\nAll additional scenarios created.\n');

%% =========================================================================
%  Local helpers
%% =========================================================================
function ts = mkTs(name, times, values)
    ts = timeseries(values(:), times(:));
    ts.Name = name;
end

function ds = modifyDataset(ds, sigName, newTs)
    for k = 1:ds.numElements
        if strcmp(ds{k}.Name, sigName)
            ds = ds.setElement(k, newTs, sigName);
            return;
        end
    end
    error('Signal "%s" not found in dataset', sigName);
end

function ds = ic_base()
    ds = Simulink.SimulationData.Dataset;
    for row = { ...
            {'enbl',          [0;10], logical([0;0])      }; ...
            {'cncl',          [0;10], logical([0;0])      }; ...
            {'speed_set',     [0;10], logical([0;0])      }; ...
            {'resume',        [0;10], logical([0;0])      }; ...
            {'inc',           [0;10], logical([0;0])      }; ...
            {'dec',           [0;10], logical([0;0])      }; ...
            {'brakeP',        [0;10], single([0;0])       }; ...
            {'key',           [0;10], uint8([2;2])        }; ...
            {'gear',          [0;10], uint8([2;2])        }; ...
            {'vehicle_speed', [0;10], single([70;70])     } }'
        c = row{1};
        ds = addElement(ds, mkTs(c{1}, c{2}, c{3}), c{1});
    end
end

function ds = tsm_base()
    ds = Simulink.SimulationData.Dataset;
    for row = { ...
            {'op_mode',       [0;10], [opMode.Activated;opMode.Activated]}; ...
            {'set_rise',      [0;10], logical([0;0])   }; ...
            {'resume_rise',   [0;10], logical([0;0])   }; ...
            {'inc_rise',      [0;10], logical([0;0])   }; ...
            {'dec_rise',      [0;10], logical([0;0])   }; ...
            {'inc_held',      [0;10], logical([0;0])   }; ...
            {'dec_held',      [0;10], logical([0;0])   }; ...
            {'vehicle_speed', [0;10], single([70;70])  }; ...
            {'accel_override',[0;10], logical([0;0])   } }'
        c = row{1};
        ds = addElement(ds, mkTs(c{1}, c{2}, c{3}), c{1});
    end
end

function ds = pid_base()
    ds = Simulink.SimulationData.Dataset;
    for row = { ...
            {'cc_target_speed', [0;10], single([70;70]) }; ...
            {'vehicle_speed',   [0;10], single([70;70]) }; ...
            {'enable_pid',      [0;10], logical([1;1])  }; ...
            {'reset_pid',       [0;10], logical([0;0])  }; ...
            {'init_pid_to_drv', [0;10], logical([0;0])  }; ...
            {'driver_throttle', [0;10], single([25;25]) } }'
        c = row{1};
        ds = addElement(ds, mkTs(c{1}, c{2}, c{3}), c{1});
    end
end
