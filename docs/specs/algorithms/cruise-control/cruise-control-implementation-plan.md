# Cruise Control Implementation Plan

## Status: In Progress
**Last Updated:** 2026-04-24
**Architecture Spec:** [cruise-control-architecture.md](cruise-control-architecture.md)
**Test Plan:** [cruise-control-test-plan.md](cruise-control-test-plan.md)

---

## 1. Progress Summary

| Phase | Status | Components |
|-------|--------|------------|
| Phase 0: Interface Contract & Stubs | ✅ Complete | All |
| Phase 1: Component Implementation | 🔲 Not Started | Input_Conditioning, Mode_Manager, Target_Speed_Manager, PID_Throttle_Controller |
| Phase 2: Integration | 🔲 Not Started | All components wired in root model |
| Phase 3: System Integration | 🔲 Not Started | Closed-loop with vehicle plant |

---

## 2. Model Hierarchy

```
crs_controller.slx (root)
├── Input_Conditioning          # Rising edge detection, threshold compares
│   ├── Button_RisingEdge       # Unit Delay-based edge detect (×6 buttons)
│   └── Hold_Timer              # 500ms hold counter for inc/dec
├── Mode_Manager                # Stateflow: Disabled/Enabled/Activated
├── Target_Speed_Manager        # Stateflow: target speed set/inc/dec/resume
├── PID_Throttle_Controller     # Discrete PID + bumpless IC + anti-windup
│   └── PID_Controller          # Simulink Discrete PID Controller block
└── Output_Logic                # Throttle mux (0 / pid / driver_override)
```

---

## 3. Dependencies

| Toolbox | Required For | Required? |
|---------|-------------|-----------|
| Simulink | All components | Yes |
| Stateflow | Mode_Manager, Target_Speed_Manager | Yes |
| Simulink Control Design | Discrete PID Controller block (if not built-in) | Check MATLAB version |

Verify available toolboxes before Phase 0:
```matlab
% Run in MATLAB before starting
ver('simulink')
ver('stateflow')
```

---

## 4. Workstream Graph

```
    Phase 0: Interface Contract & Stubs + Parameter Workspace
                              │
         ┌────────────────────┼──────────────────────┐
         ▼                    ▼                       ▼
  Input_Conditioning    Mode_Manager           PID_Throttle_Controller
  Target_Speed_Manager  (Stateflow)            (Subsystem)
  (parallel)                                    (can be parallel)
         │                    │                       │
         └────────────────────┼───────────────────────┘
                              ▼
                   Phase 2: Integration (wire root model)
                              │
                   Phase 3: System Integration (closed-loop plant)
```

---

## 5. Build Phases

### Phase 0: Interface Contract & Stubs
**Goal:** Freeze all interfaces, create parameter workspace, create compilable root model with stubs.
**Duration:** ~1 hour — **gates all parallel Phase 1 work**

| Step | Details |
|------|---------|
| 0.1 Create parameter workspace | Create `crs_controller_params.m` in `C:\work\demos\CruiseControl\` with all parameters as `Simulink.Parameter` objects |
| 0.2 Define enumerations | Create `opMode.m` and `reqMode.m` enumeration class files |
| 0.3 Create root model | New `crs_controller.slx` in `C:\work\demos\CruiseControl\` — solver: Fixed-step discrete, Ts=0.01 s |
| 0.4 Add root-level inports | 11 inports per system spec §6.1: enbl,cncl,set,resume,inc,dec,brakeP,key,gear,vehicle_speed,driver_throttle with correct data types |
| 0.5 Add root-level outports | 5 outports per system spec §6.2: reqDrv,status,mode,cc_target_speed,cc_throttle |
| 0.6 Create stub subsystems | Empty subsystem per component with final port names/types; pass-through or zero output |
| 0.7 Wire root model | Connect stubs with correct signal names; verify model compiles and runs for 1 second |

**Verification:**
- `sim('crs_controller', 1)` runs without error
- `model_overview` — verify 5-subsystem hierarchy matches §2
- `model_read` — verify all inports/outports present

**Checkpoint 0:** Root model with stubs compiles and simulates 1 second. All interfaces frozen.

---

**Parameter Workspace Script (`crs_controller_params.m`):**

```matlab
% PID gains
Kp = Simulink.Parameter(20);       Kp.DataType = 'single';
Ki = Simulink.Parameter(0.2);      Ki.DataType = 'single';
Kd = Simulink.Parameter(-0.0249);  Kd.DataType = 'single';

% Speed limits
tsp_min = Simulink.Parameter(40);  tsp_min.DataType = 'single';
tsp_max = Simulink.Parameter(100); tsp_max.DataType = 'single';

% Throttle limits
throttle_min = Simulink.Parameter(0);   throttle_min.DataType = 'single';
throttle_max = Simulink.Parameter(100); throttle_max.DataType = 'single';

% Brake threshold
brake_threshold = Simulink.Parameter(5); brake_threshold.DataType = 'single';

% Button behavior
target_inc_short = Simulink.Parameter(1);  target_inc_short.DataType = 'single';
target_dec_short = Simulink.Parameter(1);  target_dec_short.DataType = 'single';
target_inc_hold  = Simulink.Parameter(5);  target_inc_hold.DataType = 'single';
target_dec_hold  = Simulink.Parameter(5);  target_dec_hold.DataType = 'single';
press_hold_threshold_samples = Simulink.Parameter(50); % 500ms / 10ms

% Accelerator override thresholds
accel_override_on  = Simulink.Parameter(15); accel_override_on.DataType = 'single';
accel_override_off = Simulink.Parameter(5);  accel_override_off.DataType = 'single';
```

---

### Phase 1: Component Implementation
**Goal:** Implement all four algorithm components in parallel.
**Duration:** ~3–4 hours total (parallel)

#### Component 1A: Input_Conditioning

| Step | Details |
|------|---------|
| 1A.1 | Inside Input_Conditioning subsystem: add 6 rising-edge sub-subsystems (Unit Delay + Logic) for each button |
| 1A.2 | Add Hold_Timer sub-subsystem: Counter block (or MATLAB Function) counting consecutive samples of inc=true; output inc_held = (count >= press_hold_threshold_samples). Same for dec. |
| 1A.3 | Add Compare to Constant block: brakeP > brake_threshold → brake_active |
| 1A.4 | Add Compare to Constant blocks: key==2 → key_on; gear==2 → gear_drive |
| 1A.5 | Add Interval Test block (or two Compare + AND): tsp_min ≤ vehicle_speed ≤ tsp_max → speed_in_range |

**Verification:**
- `model_read('crs_controller/Input_Conditioning')` — confirm 12 output ports
- Simulation test: inc=true for 60 samples → inc_held=true at sample 50

---

#### Component 1B: Mode_Manager (Stateflow)

| Step | Details |
|------|---------|
| 1B.1 | Add Stateflow Chart block to Mode_Manager subsystem; set sample time to -1 (inherited) |
| 1B.2 | Define chart data: inputs (enbl_rise, cncl_rise, set_rise, resume_rise, brake_active, key_on, gear_drive, speed_in_range, has_prev_target, driver_throttle); outputs (mode, enable_pid, reset_pid, init_pid_to_drv, accel_override, reqDrv, status) |
| 1B.3 | Create three states: Disabled, Enabled, Activated per architecture §4.2 |
| 1B.4 | Implement transition conditions per architecture §5.2 (priority order in Activated) |
| 1B.5 | Implement accel_override hysteresis in Activated during action using local variable `override_active` |
| 1B.6 | Assign mode, reqDrv enum outputs in entry/during actions |

**Stateflow chart skeleton:**
```
state Disabled {
  entry: mode = opMode.Disabled; enable_pid = false; reset_pid = true;
         status = false; reqDrv = reqMode.None; accel_override = false;
  
  [enbl_rise && key_on && gear_drive && speed_in_range] → Enabled / reqDrv = reqMode.EnableReq;
}

state Enabled {
  entry: mode = opMode.Enabled; enable_pid = false; reset_pid = false;
         status = true; init_pid_to_drv = false;
  during: reqDrv = reqMode.None; accel_override = false;
  
  [!key_on || !gear_drive || enbl_rise] → Disabled / reqDrv = reqMode.DisableReq; reset_pid = true; status = false;
  [set_rise && speed_in_range && gear_drive] → Activated / reqDrv = reqMode.ActivateReq; init_pid_to_drv = true;
  [resume_rise && has_prev_target && speed_in_range && gear_drive] → Activated / reqDrv = reqMode.ResumeReq; init_pid_to_drv = true;
}

state Activated {
  entry: mode = opMode.Activated; enable_pid = true; status = true;
  during: {
    % Accelerator override hysteresis
    if driver_throttle > accel_override_on.Value
        accel_override = true;
    elseif driver_throttle < accel_override_off.Value
        accel_override = false;
    end
    enable_pid = !accel_override;
    init_pid_to_drv = false;   % clear after entry step
  }
  
  [!key_on || enbl_rise || !gear_drive] → Disabled / ...   % priority 1
  [brake_active || cncl_rise || !speed_in_range] → Enabled / reqDrv = reqMode.DeactivateReq; enable_pid = false;  % priority 2
}
```

**Verification:**
- Simulate all transitions from scenario list S1–S16
- Confirm init_pid_to_drv is high only for one sample on Activated entry

---

#### Component 1C: Target_Speed_Manager (Stateflow)

| Step | Details |
|------|---------|
| 1C.1 | Add Stateflow Chart to Target_Speed_Manager subsystem |
| 1C.2 | Define local data: `target` (single, IC=40), `prev_target` (single, IC=0), internal prev_override flag |
| 1C.3 | Implement target update logic per architecture §4.3 in a single active state with conditional actions |
| 1C.4 | Output cc_target_speed = target; has_prev_target = has_prev |

**Verification:**
- S2: set_rise → target = vehicle_speed (60 km/h)
- S5: inc_rise (short) → target = 61 km/h
- S6: inc_held (50 samples) → target increases 0.05 km/h/sample

---

#### Component 1D: PID_Throttle_Controller

| Step | Details |
|------|---------|
| 1D.1 | Inside PID_Throttle_Controller subsystem: add Subtract block for `e = cc_target_speed − vehicle_speed` |
| 1D.2 | Add Simulink Discrete PID Controller block (parallel form, Ts=0.01, Forward Euler, anti-windup=Clamping) |
| 1D.3 | Connect external initial condition: IC port ← driver_throttle (active when init_pid_to_drv=true) |
| 1D.4 | Connect external reset: reset port ← reset_pid |
| 1D.5 | Add Switch block: output = pid_out when enable_pid=true, else 0 |
| 1D.6 | Set Kp, Ki, Kd from workspace parameters |

**Verification:**
- `model_query_params('crs_controller/PID_Throttle_Controller/PID_Controller')` — verify Kp=20, Ki=0.2, Kd=−0.0249
- Step response test: error=+20 km/h step, enable_pid=true → output increases, settles

---

### Phase 2: Integration
**Goal:** Replace stubs with real components, verify full signal flow.
**Duration:** ~1 hour

| Step | Details |
|------|---------|
| 2.1 | Replace stub subsystems with implemented components |
| 2.2 | Wire all signals per architecture §3.3 signal flow diagram |
| 2.3 | Add signal names to all wires in root model (match architecture spec signal names) |
| 2.4 | Verify model compiles and runs 30 seconds without error |
| 2.5 | Check outputs at steady state: Disabled mode → cc_throttle=0, status=false |

**Verification:**
- `model_read('crs_controller')` — confirm all 5 components connected
- Run 30 s simulation with all inputs = 0 (initial state) — no errors, mode=Disabled, cc_throttle=0
- Run scenario S2 (§5): mode transitions to Activated within 1 sample of set_rise

**Checkpoint 2:** All components wired. Root model simulates all scenarios S1–S16 without error.

---

### Phase 3: System Integration (Closed-Loop)
**Goal:** Connect crs_controller to a vehicle longitudinal dynamics plant for closed-loop performance validation.
**Duration:** ~2 hours

| Step | Details |
|------|---------|
| 3.1 | Create test harness `crs_controller_harness.slx` containing crs_controller + simple vehicle plant |
| 3.2 | Simple vehicle plant: `dv/dt = (F_throttle − F_drag) / m` — first-order speed response to throttle |
| 3.3 | Drive initial conditions: vehicle_speed IC = 40 km/h, key=2 (ON), gear=2 (Drive) |
| 3.4 | Sequence: t=1s enbl press → t=2s set press → observe speed regulation |
| 3.5 | Run scenario S3 (40→60 km/h step): verify convergence within 20 s to 60±3 km/h |
| 3.6 | Run scenario S4 (80→60 km/h step): verify convergence within 20 s |

**Verification:**
- Closed-loop test per test plan §4.1 and §4.2
- Plot vehicle_speed, cc_target_speed, cc_throttle vs. time

**Checkpoint 3:** Closed-loop simulation meets G1 performance: speed tracking ±5% within 20 s for both S3 and S4 scenarios.

---

## 6. Parameter Table

| Parameter | Symbol | Value | Unit | Source | Calibratable? | Block Path |
|-----------|--------|-------|------|--------|---------------|------------|
| Proportional gain | Kp | 20 | — | Req §4.3 ROM | Yes | crs_controller/PID_Throttle_Controller/PID_Controller |
| Integral gain | Ki | 0.2 | — | Req §4.3 ROM | Yes | crs_controller/PID_Throttle_Controller/PID_Controller |
| Derivative gain | Kd | −0.0249 | — | Req §4.3 ROM | Yes | crs_controller/PID_Throttle_Controller/PID_Controller |
| Brake threshold | brake_threshold | 5 | kPa | Req §4.3 ROM | Yes | crs_controller/Input_Conditioning |
| Min target speed | tsp_min | 40 | km/h | Req §4.3 ROM | No | crs_controller/Input_Conditioning, Target_Speed_Manager |
| Max target speed | tsp_max | 100 | km/h | Req §4.3 ROM | No | crs_controller/Input_Conditioning, Target_Speed_Manager |
| Min throttle | throttle_min | 0 | % | Req §4.3 ROM | No | crs_controller/Output_Logic |
| Max throttle | throttle_max | 100 | % | Req §4.3 ROM | No | crs_controller/Output_Logic, PID_Throttle_Controller |
| Short inc step | target_inc_short | 1 | km/h | Req §4.3 ROM | No | crs_controller/Target_Speed_Manager |
| Short dec step | target_dec_short | 1 | km/h | Req §4.3 ROM | No | crs_controller/Target_Speed_Manager |
| Hold inc rate | target_inc_hold | 5 | km/h/s | Req §4.3 ROM | No | crs_controller/Target_Speed_Manager |
| Hold dec rate | target_dec_hold | 5 | km/h/s | Req §4.3 ROM | No | crs_controller/Target_Speed_Manager |
| Hold press threshold | press_hold_threshold_samples | 50 | samples | Req §3.5 (500ms/10ms) | No | crs_controller/Input_Conditioning/Hold_Timer |
| Accel override ON | accel_override_on | 15 | % | Req §3.9 | No | crs_controller/Mode_Manager |
| Accel override OFF | accel_override_off | 5 | % | Req §3.9 | No | crs_controller/Mode_Manager |

---

## 7. Sync Points

After each phase:
1. `model_read` — verify block topology matches architecture spec component catalog
2. `model_query_params` — spot-check Kp, Ki, Kd, brake_threshold
3. Run validation test for that phase (from test plan)
4. Update Progress Summary table above
5. Proceed only when checkpoint criteria met

**If mode logic deviation found:** Update architecture spec §5.2 transitions, re-verify Mode_Manager.
**If interface mismatch found:** Update architecture spec §4 component interfaces, re-stub, re-verify.

---

## 8. Definition of Done

### Interface Contract Complete (Phase 0)
- [ ] `crs_controller_params.m` created and loads without error
- [ ] `opMode.m` and `reqMode.m` enum classes created
- [ ] Root model has 11 inports, 5 outports with correct names and data types
- [ ] All 5 stub subsystems present with correct ports
- [ ] `sim('crs_controller', 1)` runs without error

### Component Complete (Phase 1)
- [ ] Input_Conditioning: rising edge for all 6 buttons verified; hold timer verified at 50 samples; brake, key, gear, speed_in_range correct
- [ ] Mode_Manager: all transitions S1–S16 verified in isolation; accel_override hysteresis correct; init_pid_to_drv high for exactly 1 sample
- [ ] Target_Speed_Manager: Set, Inc (short/long), Dec (short/long), Resume all verified; target clamped to [40, 100]
- [ ] PID_Throttle_Controller: Kp=20, Ki=0.2, Kd=−0.0249 confirmed; anti-windup active; reset and IC ports wired

### Algorithm Integrated (Phase 2)
- [ ] All 5 components wired in root model per architecture §3.3
- [ ] `sim('crs_controller', 30)` runs clean
- [ ] Scenarios S1–S16 all pass in open-loop test harness

### System Integration Complete (Phase 3)
- [ ] Closed-loop harness simulates vehicle speed regulation
- [ ] S3 (40→60 km/h): speed ∈ [57, 63] km/h within 20 s ✓
- [ ] S4 (80→60 km/h): speed ∈ [57, 63] km/h within 20 s ✓

---

## 9. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Stateflow transition priority conflict (Disable vs. Deactivate in Activated) | Assign explicit priority order in Stateflow chart; test both conditions simultaneously in test plan |
| PID bumpless transfer IC not loading correctly | Verify with single-step sim: at t=activation, cc_throttle ≈ driver_throttle ±2% |
| inc_held / dec_held false triggering | Ensure Hold_Timer resets immediately on button release; test with exact 49- and 50-sample pulses |
| Target speed not clamping at tsp_max during long hold | Verify clamp block is downstream of accumulation; test hold for 30+ seconds |
| Accelerator override hysteresis state not initializing | Set accel_override initial value = false in Stateflow data properties |
