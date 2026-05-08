% crs_controller_params.m
% Cruise control parameter workspace — run before opening crs_controller.slx

% PID gains
Kp = Simulink.Parameter(20);       Kp.DataType = 'single';
Ki = Simulink.Parameter(0.2);      Ki.DataType = 'single';
Kd = Simulink.Parameter(-0.0249);  Kd.DataType = 'single';

% Speed limits
tsp_min = Simulink.Parameter(40);   tsp_min.DataType = 'single';
tsp_max = Simulink.Parameter(100);  tsp_max.DataType = 'single';

% Throttle limits
throttle_min = Simulink.Parameter(0);    throttle_min.DataType = 'single';
throttle_max = Simulink.Parameter(100);  throttle_max.DataType = 'single';

% Brake threshold
brake_threshold = Simulink.Parameter(5);  brake_threshold.DataType = 'single';

% Button-press behavior
target_inc_short = Simulink.Parameter(1);   target_inc_short.DataType = 'single';
target_dec_short = Simulink.Parameter(1);   target_dec_short.DataType = 'single';
target_inc_hold  = Simulink.Parameter(5);   target_inc_hold.DataType  = 'single';
target_dec_hold  = Simulink.Parameter(5);   target_dec_hold.DataType  = 'single';
press_hold_threshold_samples = Simulink.Parameter(50);  % 500 ms / 10 ms

% Accelerator override thresholds
accel_override_on  = Simulink.Parameter(15);  accel_override_on.DataType  = 'single';
accel_override_off = Simulink.Parameter(5);   accel_override_off.DataType = 'single';

disp('crs_controller_params loaded.')
