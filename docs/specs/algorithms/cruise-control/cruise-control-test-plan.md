# Cruise Control Test Plan

## Status: Draft
**Last Updated:** 2026-04-24
**Architecture Spec:** [cruise-control-architecture.md](cruise-control-architecture.md)

---

## 1. Overview

**Validation Stages:**
- **Component Validation (MIL)** — Test each subsystem in isolation with scripted inputs
- **Integrated Algorithm Validation (MIL)** — All components wired together, scripted inputs, no plant
- **System-in-Loop Validation (MIL)** — Algorithm connected to vehicle longitudinal dynamics plant
- **Robustness & Sensitivity** — PID gain variation, boundary conditions, noise

**Validation Philosophy:** Each stage must pass before proceeding. Failures at higher stages are debugged by dropping back to lower stages.

**Model under test:** `crs_controller.slx`
**Test harness:** `crs_controller_harness.slx` (closed-loop with plant for Stage 3)
**Solver:** Fixed-step discrete, Ts = 0.01 s

---

## 2. Component Validation

### 2.1 Input_Conditioning

**What is validated:** Rising edge detection, hold timer, brake/key/gear decoding, speed range detection.

#### Steady-State Tests

| Test | Input | Expected Output | Acceptance Criterion | Basis |
|------|-------|-----------------|---------------------|-------|
| IC-SS-01: Brake above threshold | brakeP = 10 kPa | brake_active = true | brake_active == true | §4.3: brakeP > 5 kPa |
| IC-SS-02: Brake below threshold | brakeP = 3 kPa | brake_active = false | brake_active == false | §4.3 |
| IC-SS-03: Key ON | key = 2 | key_on = true | key_on == true | Arch §4.1 |
| IC-SS-04: Key not ON | key = 1 | key_on = false | key_on == false | Arch §4.1 |
| IC-SS-05: Gear Drive | gear = 2 | gear_drive = true | gear_drive == true | Arch §4.1 |
| IC-SS-06: Speed in range | vehicle_speed = 60 km/h | speed_in_range = true | speed_in_range == true | Req §3.1 |
| IC-SS-07: Speed below min | vehicle_speed = 35 km/h | speed_in_range = false | speed_in_range == false | Req §3.1 |
| IC-SS-08: Speed above max | vehicle_speed = 105 km/h | speed_in_range = false | speed_in_range == false | Req §3.1 |

#### Transient Tests

| Test | Excitation | Expected Response | Acceptance Criterion | Basis |
|------|-----------|-------------------|---------------------|-------|
| IC-TR-01: Rising edge on enbl | enbl: 0→1 at t=1s | enbl_rise = true for exactly 1 sample | enbl_rise high for 1 sample only | Arch §4.1 |
| IC-TR-02: No edge while held | enbl = true for 10 samples | enbl_rise = true only at first sample | enbl_rise == false for samples 2–10 | Arch §4.1 |
| IC-TR-03: Hold timer — short | inc = true for 40 samples, then false | inc_held = false throughout | inc_held == false at sample 40 | Req §3.5 (500ms = 50 samples) |
| IC-TR-04: Hold timer — long | inc = true for 60 samples | inc_held = true from sample 50 onward | inc_held == true at samples 50–60 | Req §3.5 |
| IC-TR-05: Hold timer reset | inc = true 60 samples, false 5 samples, true 60 samples | inc_held resets on release; requires 50 new samples | inc_held == false at sample 65 after second press start | Arch §4.1 |

---

### 2.2 Mode_Manager

**What is validated:** All state transitions, output assignments, priority, accelerator override.

#### Stateflow Transition Tests

| Test | Trigger | Expected Transition | Exit State | Basis |
|------|---------|--------------------|-----------| ------|
| MM-TR-01: Enable from Disabled | enbl_rise && key_on && gear_drive && speed_in_range=true | Disabled → Enabled | Enabled | Req §3.1 |
| MM-TR-02: Enable fails — speed OOR | enbl_rise, speed_in_range=false | No transition | Disabled | Req §3.1 |
| MM-TR-03: Enable fails — gear wrong | enbl_rise, gear_drive=false | No transition | Disabled | Req §3.1 |
| MM-TR-04: Enable fails — key off | enbl_rise, key_on=false | No transition | Disabled | Req §3.1 |
| MM-TR-05: Disable via key | mode=Enabled, key_on→false | Enabled → Disabled | Disabled | Req §3.2 |
| MM-TR-06: Disable via enbl toggle | mode=Enabled, enbl_rise | Enabled → Disabled | Disabled | Req §3.2 |
| MM-TR-07: Activate via Set | mode=Enabled, set_rise, speed_in_range, gear_drive | Enabled → Activated | Activated | Req §3.3 |
| MM-TR-08: Activate fails — speed OOR | mode=Enabled, set_rise, speed_in_range=false | No transition to Activated | Enabled | Req §3.3 |
| MM-TR-09: Deactivate via brake | mode=Activated, brake_active=true | Activated → Enabled | Enabled | Req §3.4 |
| MM-TR-10: Deactivate via cancel | mode=Activated, cncl_rise=true | Activated → Enabled | Enabled | Req §3.4 |
| MM-TR-11: Deactivate via speed OOR | mode=Activated, speed_in_range→false | Activated → Enabled | Enabled | Req §3.4 |
| MM-TR-12: Disable from Activated | mode=Activated, key_on→false | Activated → Disabled (NOT Enabled) | Disabled | Req §3.2 priority |
| MM-TR-13: Resume | mode=Enabled, has_prev_target=true, resume_rise, speed_in_range, gear_drive | Enabled → Activated | Activated | Req §3.10 |
| MM-TR-14: Resume fails — no prev | mode=Enabled, has_prev_target=false, resume_rise | No transition | Enabled | Req §3.10 |

#### Output Assignment Tests

| Test | Condition | Expected Output | Acceptance Criterion |
|------|-----------|-----------------|---------------------|
| MM-OUT-01: status=false in Disabled | mode=Disabled | status=false | status == false |
| MM-OUT-02: status=true in Enabled | mode=Enabled | status=true | status == true |
| MM-OUT-03: enable_pid=false in Enabled | mode=Enabled | enable_pid=false | enable_pid == false |
| MM-OUT-04: enable_pid=true in Activated | mode=Activated, accel_override=false | enable_pid=true | enable_pid == true |
| MM-OUT-05: init_pid_to_drv one-shot | Enabled→Activated transition | init_pid_to_drv=true for 1 sample, then false | init_pid_to_drv high exactly 1 sample |
| MM-OUT-06: accel_override ON | mode=Activated, driver_throttle=20% | accel_override=true | accel_override == true |
| MM-OUT-07: accel_override OFF hysteresis | accel_override active, driver_throttle drops to 10% | accel_override=true (not yet released) | accel_override == true at 10% |
| MM-OUT-08: accel_override released | accel_override active, driver_throttle drops to 3% | accel_override=false | accel_override == false |

---

### 2.3 Target_Speed_Manager

**What is validated:** Target speed set, inc/dec short and long, resume, clamp, override release behavior.

#### Steady-State & Functional Tests

| Test | Input | Expected Output | Acceptance Criterion | Basis |
|------|-------|-----------------|---------------------|-------|
| TSM-01: Set on activation | set_rise, vehicle_speed=72 km/h | cc_target_speed=72 km/h | target == 72 ± 0.1 km/h | Req §3.3 |
| TSM-02: Short Inc | mode=Activated, inc_rise (not held) | cc_target_speed += 1 km/h | target increases by exactly 1.0 km/h | Req §3.5 |
| TSM-03: Short Dec | mode=Activated, dec_rise (not held) | cc_target_speed −= 1 km/h | target decreases by exactly 1.0 km/h | Req §3.6 |
| TSM-04: Long Inc | mode=Activated, inc_held for 100 samples (1s) | cc_target_speed += 5 km/h over 1s | target increases by 5.0 ± 0.1 km/h after 1s | Req §3.7 |
| TSM-05: Long Dec | mode=Activated, dec_held for 100 samples (1s) | cc_target_speed −= 5 km/h over 1s | target decreases by 5.0 ± 0.1 km/h after 1s | Req §3.8 |
| TSM-06: Clamp at tsp_max | mode=Activated, target=99 km/h, inc_rise | cc_target_speed=100 km/h (not 101) | target == 100 km/h after press | Req §3.1 |
| TSM-07: Clamp at tsp_min | mode=Activated, target=41 km/h, dec_rise | cc_target_speed=40 km/h (not 39) | target == 40 km/h after press | Req §3.1 |
| TSM-08: Resume restores prev | Activated→Enabled→Activated(resume) | cc_target_speed = prev_target | target == prev_target ± 0.1 km/h | Req §3.10 |
| TSM-09: Override release updates target | Activated, accel_override ends, vehicle_speed=75 | cc_target_speed = 75 km/h | target == 75 ± 0.1 km/h | Req §3.9 |
| TSM-10: has_prev_target cleared on Disable | mode→Disabled | has_prev_target = false | has_prev_target == false | Arch §4.3 |

---

### 2.4 PID_Throttle_Controller

**What is validated:** PID output, anti-windup, reset, IC preload for bumpless transfer.

#### Control Law Tests

| Test | Excitation | Expected Response | Acceptance Criterion | Basis |
|------|-----------|-------------------|---------------------|-------|
| PID-01: Positive error → positive output | e=+10 km/h step, enable_pid=true | pid_throttle > 0, increases | pid_throttle > 0 within 1 sample | Kp=20 > 0 |
| PID-02: Negative error → negative (clamped to 0) | e=−10 km/h, enable_pid=true | pid_throttle clamped to 0 | pid_throttle == 0 | throttle_min=0 |
| PID-03: Disabled → zero output | enable_pid=false | pid_throttle = 0 | pid_throttle == 0 | Arch §4.4 |
| PID-04: Reset clears integrator | enable_pid=true for 5s, reset_pid=true | pid_throttle drops to Kp*e immediately | Integral contribution = 0 after reset | Arch §4.4 |

#### Edge Cases

| Test | Condition | Expected Behavior | Acceptance Criterion |
|------|-----------|-------------------|---------------------|
| PID-EC-01: Anti-windup | e=+100 km/h (deep saturation) for 10s, then e=0 | Integrator not wound up; output recovers quickly | Recovery to e=0 within 2s (no overshoot >10%) |
| PID-EC-02: IC preload | Enabled→Activated, driver_throttle=30% | cc_throttle ≈ 30% at t=0 of Activated | |cc_throttle − driver_throttle| < 2% at activation step |

---

## 3. Integrated Algorithm Validation

### 3.1 Operating Mode Tests

| Mode | Entry Condition | Input Signals | Expected Behavior | Acceptance Criterion |
|------|----------------|---------------|-------------------|---------------------|
| Disabled | Default | All inputs = 0 | cc_throttle=0, status=false, mode=Disabled | mode==Disabled, cc_throttle==0 |
| Enabled | key=2, gear=2, speed=60, enbl_rise | Per above | status=true, cc_throttle=0, mode=Enabled | mode==Enabled within 1 sample |
| Activated | Enabled + set_rise | Per above | cc_throttle from PID, mode=Activated, status=true | mode==Activated within 1 sample |

### 3.2 Mode Transition Scenarios

| Transition | From → To | Trigger | Expected Behavior | Bumpless Transfer Check |
|-----------|-----------|---------|-------------------|------------------------|
| Enable | Disabled → Enabled | enbl_rise | status=true, cc_throttle=0 | N/A (throttle was 0) |
| Activate | Enabled → Activated | set_rise | PID starts, target=vehicle_speed | |cc_throttle at t=0| ≈ driver_throttle ± 2% |
| Deactivate (brake) | Activated → Enabled | brake_active | cc_throttle=0 immediately | N/A (driver takes over) |
| Deactivate (cancel) | Activated → Enabled | cncl_rise | cc_throttle=0 immediately | N/A |
| Disable from Activated | Activated → Disabled | key_on→false | cc_throttle=0, status=false | N/A |
| Resume | Enabled → Activated | resume_rise | target=prev_target, PID resumes | |cc_throttle at t=0| ≈ driver_throttle ± 2% |

### 3.3 Saturation / Anti-Windup Recovery

| Test | Condition | Recovery Input | Expected Behavior | Acceptance Criterion |
|------|-----------|---------------|-------------------|---------------------|
| INT-AW-01 | mode=Activated, target=100, vehicle_speed=40 (20% error for 30s) | target=60, vehicle_speed=60 | Recovers to ~0% throttle without sustained overshoot | Recovery within 10 s, overshoot < 10% |

### 3.4 Full Scenario Trace (S1–S16)

Each scenario from system spec §5 is run as an integrated algorithm test. Pass criteria: outputs match "Expected Algorithm Behavior" column within "Performance Criteria" tolerance.

---

## 4. System-in-Loop Validation

### 4.1 Speed Tracking from Below (Req §3.11 Scenario A)

**Setup:** Simple vehicle plant (first-order throttle-to-speed response), IC: vehicle_speed=40 km/h, key=ON, gear=Drive, driver_throttle=0

**Scenario:**
1. t=0–1s: mode=Disabled (enbl not pressed)
2. t=1s: enbl_rise → mode=Enabled
3. t=2s: set_rise → mode=Activated, target=40 km/h
4. t=2s: apply step in target to 60 km/h (simulate driver immediately setting 60 via Inc, or: model sets target=vehicle_speed=40 at Set, then operator increments… alternatively harness can set target=60 directly post-Set)
5. Run until t=25s

**Acceptance Criteria:**

| Metric | Target | Basis |
|--------|--------|-------|
| Speed within 5% of 60 km/h | |vehicle_speed − 60| ≤ 3 km/h by t=22s | Req §3.11 (20 s settling, ±5%) |
| No sustained oscillation | Speed variance < 1 km/h² after t=22s | Stability requirement |

### 4.2 Speed Tracking from Above (Req §3.11 Scenario B)

**Setup:** vehicle_speed IC = 80 km/h, target = 60 km/h

**Scenario:**
1. t=1s: Enable → Activate at 80 km/h; target set to 80 km/h
2. t=2s: Target stepped to 60 km/h
3. Run until t=25s

**Acceptance Criteria:**

| Metric | Target | Basis |
|--------|--------|-------|
| Speed within 5% of 60 km/h | |vehicle_speed − 60| ≤ 3 km/h by t=22s | Req §3.11 |
| Throttle does not go negative | cc_throttle ≥ 0 always | throttle_min=0 saturation |

### 4.3 Accelerator Override and Resume

**Setup:** Activated at 60 km/h

**Scenario:**
1. t=5s: driver_throttle ramps to 20% (override ON)
2. t=10s: driver_throttle ramps back to 2% (override OFF)
3. Observe: target updates to current vehicle_speed; PID resumes

**Acceptance Criteria:**
- Override activates within 1 sample of driver_throttle crossing 15%
- cc_throttle = driver_throttle during override (within 1%)
- After override ends: target = vehicle_speed at that moment; PID resumes without discontinuity > 5%

---

## 5. Robustness & Sensitivity

### 5.1 PID Gain Sensitivity

| Parameter | Nominal | Range | Test | Acceptance Criterion |
|-----------|---------|-------|------|---------------------|
| Kp | 20 | [10, 40] | S3: 40→60 km/h step | Settling ≤ 25 s, no sustained oscillation |
| Ki | 0.2 | [0.05, 0.5] | S3: 40→60 km/h step | Zero steady-state error; no windup |
| Kd | −0.0249 | [−0.1, 0] | S3 | No unstable oscillation |

### 5.2 Boundary Conditions

| Test | Condition | Expected Behavior | Acceptance Criterion |
|------|-----------|-------------------|---------------------|
| ROB-01: Speed at exact tsp_min=40 | Activated, vehicle_speed=40.0 km/h | speed_in_range=true (boundary inclusive) | mode stays Activated |
| ROB-02: Speed just below tsp_min | vehicle_speed=39.9 km/h | Deactivation triggers | mode→Enabled within 1 sample |
| ROB-03: Speed at exact tsp_max=100 | Activated, target clamp at 100 | cc_target_speed=100, no further increase | target == 100 |
| ROB-04: Simultaneous brake + cancel | Both brake_active and cncl_rise=true | Deactivate (either condition sufficient) | mode→Enabled within 1 sample |
| ROB-05: Disable + Deactivate same step | brake_active=true AND key_on=false | → Disabled (disable wins) | mode==Disabled (not Enabled) |
| ROB-06: Long press Dec at tsp_min | target=40, dec_held=true for 5s | target stays at 40 km/h | target == 40 throughout |

---

## 6. Implementation Equivalence

Not applicable — simulation-only v1. Skip.

---

## 7. Simulation Configuration

| Setting | Value | Rationale |
|---------|-------|-----------|
| Solver | Fixed-step discrete | Architecture spec C1 |
| Step size | 0.01 s | 10 ms base sample time |
| Stop time | Varies by test (see scenarios) | Per scenario |
| Initial conditions | All inputs = 0, mode = Disabled | Safe default |
| Signal logging | mode, status, cc_throttle, cc_target_speed, vehicle_speed, enable_pid, accel_override | All outputs + key internal signals |

---

## 8. Input Signal Definitions

| Signal ID | Type | Parameters | Used In |
|-----------|------|-----------|---------|
| ENBL_PULSE_01 | Pulse | Amplitude=1, Width=0.02s (2 samples), Period=10s | MM-TR-01, S1, S2 |
| SET_PULSE_01 | Pulse | Amplitude=1, Width=0.02s, Period=10s, Delay=2s | S2, TSM-01 |
| RESUME_PULSE_01 | Pulse | Amplitude=1, Width=0.02s, Delay=5s | MM-TR-13, TSM-08 |
| INC_SHORT_01 | Pulse | Amplitude=1, Width=0.03s (3 samples) | IC-TR-03, TSM-02 |
| INC_LONG_01 | Step | On at t=2s, Off at t=3s (100 samples) | IC-TR-04, TSM-04 |
| DEC_SHORT_01 | Pulse | Amplitude=1, Width=0.03s | TSM-03 |
| DEC_LONG_01 | Step | On at t=2s, Off at t=3s | TSM-05 |
| BRAKE_STEP_01 | Step | Amplitude=10 kPa, Time=5s | MM-TR-09, S9 |
| SPEED_60 | Constant | 60 km/h | S2, S3 (setpoint) |
| SPEED_STEP_40_TO_60 | Step | IC=40, Final=40 (speed is state; plant driven) | S3, §4.1 |
| SPEED_STEP_80_TO_60 | Step | IC=80 | S4, §4.2 |
| ACCEL_RAMP_20 | Ramp | From 0 to 20% over 2s, t=5s | S13, §4.3 |

---

## 9. Gherkin Scenario Templates

### 9.1 Mode Manager — Enable and Activate

```gherkin
Feature: Cruise Control - Mode Management

  Scenario: Enable cruise control from Disabled
    Given the model "crs_controller.slx" is loaded
    And the solver is "Fixed-step" with step size 0.01
    And parameter "key" is set to 2
    And parameter "gear" is set to 2
    And parameter "vehicle_speed" is set to 60

    When a pulse input of 1 is applied to "enbl" at t=1.0s for 0.02s

    Then at t=1.01s, "mode" shall be "Enabled"
    And at t=1.01s, "status" shall be 1
    And at t=1.01s, "cc_throttle" shall be 0 within 0.1

  Scenario: Activate cruise control via Set button
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Enabled" (pre-condition)
    And parameter "vehicle_speed" is set to 60
    And parameter "gear" is set to 2

    When a pulse input of 1 is applied to "set" at t=2.0s for 0.02s

    Then at t=2.01s, "mode" shall be "Activated"
    And at t=2.01s, "cc_target_speed" shall be 60 km/h within 0.5

  Scenario: Deactivate cruise control via brake
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Activated"

    When a step input of 10 kPa is applied to "brakeP" at t=5.0s

    Then at t=5.01s, "mode" shall be "Enabled"
    And at t=5.01s, "cc_throttle" shall be 0 within 0.1

  Scenario: Disable overrides deactivate (priority check)
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Activated"

    When "brakeP" is set to 10 kPa at t=5.0s
    And "key" is set to 0 at t=5.0s (same step)

    Then at t=5.01s, "mode" shall be "Disabled"
```

### 9.2 Target Speed Manager — Short/Long Press

```gherkin
Feature: Cruise Control - Target Speed Management

  Scenario: Short press Inc increases target by 1 km/h
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Activated"
    And "cc_target_speed" is 60 km/h

    When a pulse input of 1 is applied to "inc" at t=3.0s for 0.03s (3 samples)

    Then at t=3.05s, "cc_target_speed" shall be 61 km/h within 0.1

  Scenario: Long press Inc increases target at 5 km/h per second
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Activated"
    And "cc_target_speed" is 60 km/h

    When "inc" is held at 1 from t=3.0s to t=4.0s (100 samples)

    Then at t=4.0s, "cc_target_speed" shall be 65 km/h within 0.2

  Scenario: Target speed clamps at tsp_max
    Given the model "crs_controller.slx" is loaded
    And cruise control mode is "Activated"
    And "cc_target_speed" is 99 km/h

    When a pulse input of 1 is applied to "inc" at t=3.0s for 0.03s

    Then at t=3.05s, "cc_target_speed" shall be 100 km/h within 0.1
    And "cc_target_speed" shall never exceed 100 km/h
```

### 9.3 PID Throttle — Speed Tracking Performance

```gherkin
Feature: Cruise Control - Speed Tracking

  Scenario: Track 60 km/h target from 40 km/h initial speed
    Given the model "crs_controller_harness.slx" is loaded
    And vehicle_speed initial condition is 40 km/h
    And cruise control is Activated with target 60 km/h at t=2.0s

    When simulation runs to t=25.0s

    Then at t=22.0s, "vehicle_speed" shall be 60 km/h within 3.0
    And "vehicle_speed" shall not oscillate with amplitude > 1 km/h after t=22.0s

  Scenario: Track 60 km/h target from 80 km/h initial speed
    Given the model "crs_controller_harness.slx" is loaded
    And vehicle_speed initial condition is 80 km/h
    And cruise control is Activated with target 60 km/h at t=2.0s

    When simulation runs to t=25.0s

    Then at t=22.0s, "vehicle_speed" shall be 60 km/h within 3.0
    And "cc_throttle" shall be greater than or equal to 0 at all times
```

---

## Appendix A: Test Execution Commands

```
# Component: Mode Manager
Use model_test with: model="crs_controller.slx", gherkin_file="tests/mode_manager.feature"

# Component: Target Speed Manager  
Use model_test with: model="crs_controller.slx", gherkin_file="tests/target_speed.feature"

# Component: PID Controller
Use model_test with: model="crs_controller.slx", gherkin_file="tests/pid_controller.feature"

# Integrated algorithm validation
Use model_test with: model="crs_controller.slx", gherkin_file="tests/integrated.feature"

# System-in-loop: speed tracking
Use model_test with: model="crs_controller_harness.slx", gherkin_file="tests/speed_tracking.feature"
```
