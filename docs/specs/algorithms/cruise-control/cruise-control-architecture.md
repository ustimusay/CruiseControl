# Cruise Control Architecture Spec

## Status: Draft
**Last Updated:** 2026-04-24
**Author:** Claude Code
**Parent Spec:** [cruise-control-system.md](cruise-control-system.md)

---

## 1. Overview

This document defines the functional decomposition of the cruise control algorithm. The algorithm accepts driver button inputs, vehicle speed, brake pressure, key/gear state, and driver throttle; and produces a regulated throttle command plus mode/status signals. It integrates with a vehicle longitudinal dynamics plant model in closed-loop simulation. Primary inputs: 11 signals per system spec §6.1. Primary outputs: 5 signals per system spec §6.2.

---

## 2. Goals, Non-Goals & Constraints

### 2.1 Design Goals

| ID | Goal |
|----|------|
| G1 | Isolate mode management (Stateflow) from throttle control (PID) for independent development and tuning |
| G2 | Isolate target speed management to allow independent testing of button-press logic |
| G3 | All calibration parameters accessible as named workspace variables (Simulink.Parameter objects) |
| G4 | PID integrator anti-windup and bumpless mode transfer fully specified |

### 2.2 Non-Goals

| ID | Non-Goal | Rationale |
|----|----------|-----------|
| NG1 | Fixed-point arithmetic | Simulation-only v1 |
| NG2 | Multi-rate design | All components run at single 10 ms base rate |
| NG3 | Adaptive gain scheduling | Fixed gains from ROM table |

### 2.3 Constraints

| Constraint | Description |
|------------|-------------|
| C1 | Fixed-step discrete solver, 10 ms sample time |
| C2 | Single-precision floating-point (single) for all plant-facing signals, per interface spec §6.1 |
| C3 | PID gains fixed at Kp=20, Ki=0.2, Kd=−0.0249 (calibratable but not adaptive) |
| C4 | Target speed range enforced: [tsp_min, tsp_max] = [40, 100] km/h |
| C5 | Throttle output range enforced: [throttle_min, throttle_max] = [0, 100] % |

---

## 3. Architecture

### 3.1 Functional Decomposition Diagram

```
Inputs (11 signals)
     │
     ▼
┌─────────────────────┐
│  Input Conditioning │  — Rising edge detect for 6 buttons; brake threshold compare
└─────────────────────┘
     │ button_rise[6], brake_active, key_on, gear_drive, speed_in_range
     ▼
┌─────────────────────┐        ┌──────────────────────────┐
│   Mode Manager      │──────→ │  Target Speed Manager    │
│  (Stateflow Chart)  │  mode  │  (Stateflow Chart)       │
└─────────────────────┘        └──────────────────────────┘
     │ mode, enable_pid,                │ cc_target_speed
     │ reset_pid, init_pid_to_drv       ▼
     │                        ┌──────────────────────────┐
     └───────────────────────→│   PID Throttle Controller│
                               │  (Simulink PID block)    │
                               └──────────────────────────┘
                                        │ pid_throttle
                                        ▼
                               ┌──────────────────────────┐
                               │    Output Logic           │
                               │  (Throttle mux + clamp)  │
                               └──────────────────────────┘
                                        │
                                        ▼
                               Outputs: cc_throttle, mode,
                               status, cc_target_speed, reqDrv
```

### 3.2 Component Catalog

| Component | Implementation | Function | Ports (key I/O) | Rate | DFT | Dependencies |
|-----------|---------------|----------|-----------------|------|-----|--------------|
| **Input_Conditioning** | Subsystem | Rising edge detection for buttons; threshold compare for brake; boolean decode for key/gear | In: enbl,cncl,set,resume,inc,dec,brakeP,key,gear,vehicle_speed → Out: enbl_rise,cncl_rise,set_rise,resume_rise,inc_rise,dec_rise,inc_held,dec_held,brake_active,key_on,gear_drive,speed_in_range | 10 ms | No | — |
| **Mode_Manager** | Stateflow Chart | Supervisory state machine: Disabled/Enabled/Activated; outputs mode, enable/reset signals for PID and Target Speed | In: enbl_rise,cncl_rise,set_rise,resume_rise,brake_active,key_on,gear_drive,speed_in_range,has_prev_target → Out: mode,enable_pid,reset_pid,init_pid_to_drv,accel_override,reqDrv,status | 10 ms | No | Input_Conditioning |
| **Target_Speed_Manager** | Stateflow Chart | Manages cc_target_speed: Set on activate, inc/dec with short/long press, resume to prev_target, update on accel override release | In: mode,set_rise,resume_rise,inc_held,dec_held,vehicle_speed,accel_override → Out: cc_target_speed,has_prev_target | 10 ms | No | Mode_Manager |
| **PID_Throttle_Controller** | Subsystem (Simulink PID block) | Discrete PID controller on speed error; clamping anti-windup; output enabled only in Activated mode and when override inactive | In: cc_target_speed,vehicle_speed,enable_pid,reset_pid,init_pid_to_drv,driver_throttle → Out: pid_throttle | 10 ms | No | Mode_Manager, Target_Speed_Manager |
| **Output_Logic** | Subsystem | Throttle mux (PID vs. driver override vs. 0); saturate [0,100]; compose status/mode outputs | In: pid_throttle,driver_throttle,accel_override,mode → Out: cc_throttle | 10 ms | No | Mode_Manager, PID_Throttle_Controller |

### 3.3 Signal Flow

```
vehicle_speed ──────────────────────────────────────────────→ [PID error: target − speed]
                                                                        │
buttons/key/gear/brakeP ──→ [Input_Conditioning] ──→ [Mode_Manager] ──→ enable_pid, reset_pid
                                                              │
                                                              ▼
                                                    [Target_Speed_Manager]
                                                              │ cc_target_speed
                                                              ▼
                                                    [PID_Throttle_Controller]
                                                              │ pid_throttle
                                                              ▼
driver_throttle ─────────────────────────────────→ [Output_Logic] ──→ cc_throttle
                                                    (mux: 0 / pid / drv)
```

No algebraic loops: all feedback paths pass through Unit Delay blocks in Stateflow (Moore-style outputs).

---

## 4. Component Details

### 4.1 Input_Conditioning

**Purpose:** Convert raw boolean button signals to single-sample rising-edge pulses; decode key/gear enumerations; compare brake pressure to threshold; detect speed in range.

**Implementation:** Subsystem with Unit Delay for each button (rising edge = current AND NOT previous).

**Interface:**

| Port | Direction | Signal Name | Unit | Data Type | Sample Time |
|------|-----------|-------------|------|-----------|-------------|
| u1–u6 | Input | enbl,cncl,set,resume,inc,dec | — | boolean | 10 ms |
| u7 | Input | brakeP | kPa | single | 10 ms |
| u8 | Input | key | — | uint8 | 10 ms |
| u9 | Input | gear | — | uint8 | 10 ms |
| u10 | Input | vehicle_speed | km/h | single | 10 ms |
| y1–y6 | Output | enbl_rise,cncl_rise,set_rise,resume_rise,inc_rise,dec_rise | — | boolean | 10 ms |
| y7,y8 | Output | inc_held, dec_held | — | boolean | 10 ms |
| y9 | Output | brake_active | — | boolean | 10 ms |
| y10 | Output | key_on | — | boolean | 10 ms |
| y11 | Output | gear_drive | — | boolean | 10 ms |
| y12 | Output | speed_in_range | — | boolean | 10 ms |

**Behavior:**
- Rising edge: `button_rise = button_current AND NOT button_prev` (Unit Delay, IC=false)
- `inc_held`: inc button has been continuously held for ≥ 500 ms (50 samples). Implemented as a counter that increments while inc=true and resets when inc=false. `inc_held = (counter >= 50)`.
- `dec_held`: same logic for dec button.
- `brake_active = brakeP > brake_threshold` (brake_threshold = 5 kPa)
- `key_on = (key == 2)` (ON position)
- `gear_drive = (gear == 2)` (Drive position)
- `speed_in_range = (vehicle_speed >= tsp_min) AND (vehicle_speed <= tsp_max)`

**Dependencies:** brake_threshold, tsp_min, tsp_max from workspace parameters.

---

### 4.2 Mode_Manager

**Purpose:** Stateflow supervisory chart implementing the Disabled/Enabled/Activated state machine. Outputs mode, PID control signals, and driver request interpretation.

**Implementation:** Stateflow Chart (Moore semantics — outputs assigned in state actions, not transitions).

**Interface:**

| Port | Direction | Signal Name | Unit | Data Type | Sample Time |
|------|-----------|-------------|------|-----------|-------------|
| u1 | Input | enbl_rise | — | boolean | 10 ms |
| u2 | Input | cncl_rise | — | boolean | 10 ms |
| u3 | Input | set_rise | — | boolean | 10 ms |
| u4 | Input | resume_rise | — | boolean | 10 ms |
| u5 | Input | brake_active | — | boolean | 10 ms |
| u6 | Input | key_on | — | boolean | 10 ms |
| u7 | Input | gear_drive | — | boolean | 10 ms |
| u8 | Input | speed_in_range | — | boolean | 10 ms |
| u9 | Input | has_prev_target | — | boolean | 10 ms |
| u10 | Input | driver_throttle | % | single | 10 ms |
| y1 | Output | mode | — | Enum:opMode | 10 ms |
| y2 | Output | enable_pid | — | boolean | 10 ms |
| y3 | Output | reset_pid | — | boolean | 10 ms |
| y4 | Output | init_pid_to_drv | — | boolean | 10 ms |
| y5 | Output | accel_override | — | boolean | 10 ms |
| y6 | Output | reqDrv | — | Enum:reqMode | 10 ms |
| y7 | Output | status | — | boolean | 10 ms |

**Behavior — State Machine:**

```
State: Disabled
  Entry: enable_pid=false; reset_pid=true; status=false; mode=Disabled
  During: reqDrv=None; accel_override=false
  Transitions out:
    [enbl_rise && key_on && gear_drive && speed_in_range] → Enabled
      (reqDrv=EnableReq on this step)

State: Enabled
  Entry: enable_pid=false; reset_pid=false; status=true; mode=Enabled; init_pid_to_drv=false
  During: reqDrv=None; accel_override=false
  Transitions out (priority order):
    1. [!key_on || !gear_drive] → Disabled
       (reqDrv=DisableReq on this step)
    2. [enbl_rise] → Disabled
       (reqDrv=DisableReq on this step)
    3. [set_rise && speed_in_range && gear_drive] → Activated
       (reqDrv=ActivateReq; init_pid_to_drv=false)
    4. [resume_rise && has_prev_target && speed_in_range && gear_drive] → Activated
       (reqDrv=ResumeReq; init_pid_to_drv=false)

State: Activated
  Entry: enable_pid=true; reset_pid=false; status=true; mode=Activated; init_pid_to_drv=true (one step)
  init_pid_to_drv is true only on the entry step; then false.
  Transitions out (priority order):
    1. [!key_on || enbl_rise || !gear_drive] → Disabled     ← Disable (highest priority)
       (reqDrv=DisableReq)
    2. [brake_active || cncl_rise || !speed_in_range] → Enabled   ← Deactivate
       (reqDrv=DeactivateReq)
  Accelerator override sub-behavior (within Activated, not a state transition):
    accel_override = (driver_throttle > accel_override_on) ||
                     (accel_override_prev && driver_throttle >= accel_override_off)
    (hysteresis: latch on at 15%, release below 5%)
    enable_pid = !accel_override
```

**Dependencies:** opMode enum, reqMode enum, workspace parameters.

---

### 4.3 Target_Speed_Manager

**Purpose:** Maintains and updates `cc_target_speed` in response to mode transitions and driver button inputs.

**Implementation:** Stateflow Chart with internal data for target speed and prev_target.

**Interface:**

| Port | Direction | Signal Name | Unit | Data Type | Sample Time |
|------|-----------|-------------|------|-----------|-------------|
| u1 | Input | mode | — | Enum:opMode | 10 ms |
| u2 | Input | set_rise | — | boolean | 10 ms |
| u3 | Input | resume_rise | — | boolean | 10 ms |
| u4 | Input | inc_rise | — | boolean | 10 ms |
| u5 | Input | dec_rise | — | boolean | 10 ms |
| u6 | Input | inc_held | — | boolean | 10 ms |
| u7 | Input | dec_held | — | boolean | 10 ms |
| u8 | Input | vehicle_speed | km/h | single | 10 ms |
| u9 | Input | accel_override | — | boolean | 10 ms |
| u10 | Input | accel_override_prev | — | boolean | 10 ms |
| y1 | Output | cc_target_speed | km/h | single | 10 ms |
| y2 | Output | has_prev_target | — | boolean | 10 ms |

**Behavior:**

Internal data:
- `target`: single, range [tsp_min, tsp_max], IC = tsp_min
- `prev_target`: single, IC = 0
- `has_prev`: boolean, IC = false

Logic (evaluated each 10 ms step):

```
if mode transitions to Activated via set_rise:
    prev_target = target   (save before overwrite)
    target = clamp(vehicle_speed, tsp_min, tsp_max)
    has_prev = true

elif mode transitions to Activated via resume_rise:
    target = prev_target   (restore)

elif mode == Activated && !accel_override:
    if inc_held:
        target = clamp(target + target_inc_hold * 0.01, tsp_min, tsp_max)   # +5 km/h/s × 10ms
    elif inc_rise && !inc_held:
        target = clamp(target + target_inc_short, tsp_min, tsp_max)
    if dec_held:
        target = clamp(target - target_dec_hold * 0.01, tsp_min, tsp_max)
    elif dec_rise && !dec_held:
        target = clamp(target - target_dec_short, tsp_min, tsp_max)

elif mode == Activated && accel_override && !accel_override_prev:
    # override just ended (falling edge)
    prev_target = target
    target = clamp(vehicle_speed, tsp_min, tsp_max)

elif mode == Disabled:
    has_prev = false

cc_target_speed = target
has_prev_target = has_prev
```

**Note on long-press increment:** `target_inc_hold * Ts = 5 km/h/s × 0.01 s = 0.05 km/h per sample`. This accumulates to 5 km/h after 1 second of holding, as specified.

**Dependencies:** tsp_min, tsp_max, target_inc_short, target_dec_short, target_inc_hold, target_dec_hold from workspace.

---

### 4.4 PID_Throttle_Controller

**Purpose:** Compute engine throttle command from speed error using discrete PID with clamping anti-windup. Active only when enable_pid=true.

**Implementation:** Subsystem containing a Simulink Discrete PID Controller block.

**Interface:**

| Port | Direction | Signal Name | Unit | Data Type | Sample Time |
|------|-----------|-------------|------|-----------|-------------|
| u1 | Input | cc_target_speed | km/h | single | 10 ms |
| u2 | Input | vehicle_speed | km/h | single | 10 ms |
| u3 | Input | enable_pid | — | boolean | 10 ms |
| u4 | Input | reset_pid | — | boolean | 10 ms |
| u5 | Input | init_pid_to_drv | — | boolean | 10 ms |
| u6 | Input | driver_throttle | % | single | 10 ms |
| y1 | Output | pid_throttle | % | single | 10 ms |

**Behavior:**

Speed error: `e = cc_target_speed − vehicle_speed`

PID control law (discrete, backward Euler integration):
```
u(k) = Kp * e(k) + Ki * I(k) + Kd * (e(k) − e(k−1)) / Ts
I(k) = I(k−1) + e(k) * Ts      (before anti-windup clamp)
```

Anti-windup (clamping):
- Output is saturated to [throttle_min, throttle_max] = [0, 100]%
- Integration is suspended when saturated AND the error would drive further saturation:
  `integrate = !(output_saturated_high && e > 0) && !(output_saturated_low && e < 0)`

Bumpless transfer on Activated entry:
- When `init_pid_to_drv=true` (one step only on entry to Activated): initialize the integrator so that `u(0) = driver_throttle`. This sets `I(0) = driver_throttle / Ki` (with P and D terms subtracted).
- When `reset_pid=true` (Disabled): clear integrator to 0.

When `enable_pid=false`: `pid_throttle = 0`.

**Simulink PID block configuration:**
- Controller form: Parallel
- Time domain: Discrete-time
- Sample time: 0.01 s
- Integrator method: Forward Euler (or Backward Euler — consistent with anti-windup implementation)
- Filter: No derivative filter (N = ∞, i.e., pure discrete derivative)
- Anti-windup: Clamping
- Initial condition source: External (for bumpless transfer)
- External reset: Rising edge on reset_pid

**Dependencies:** Kp, Ki, Kd, throttle_min, throttle_max from workspace.

---

### 4.5 Output_Logic

**Purpose:** Select and limit final throttle output based on operating mode and accelerator override.

**Implementation:** Simple Subsystem (Switch blocks + Saturation).

**Interface:**

| Port | Direction | Signal Name | Unit | Data Type | Sample Time |
|------|-----------|-------------|------|-----------|-------------|
| u1 | Input | pid_throttle | % | single | 10 ms |
| u2 | Input | driver_throttle | % | single | 10 ms |
| u3 | Input | accel_override | — | boolean | 10 ms |
| u4 | Input | mode | — | Enum:opMode | 10 ms |
| y1 | Output | cc_throttle | % | single | 10 ms |

**Behavior:**

```
if mode == Activated:
    if accel_override:
        cc_throttle = clamp(driver_throttle, 0, 100)
    else:
        cc_throttle = clamp(pid_throttle, 0, 100)
else:
    cc_throttle = 0
```

**Dependencies:** throttle_min, throttle_max.

---

## 5. Mode Logic

### 5.1 Mode Definitions

| Mode | Description | Entry Condition |
|------|-------------|-----------------|
| **Disabled** | CC fully off; no throttle output | Default; key≠ON; enbl toggle from Enabled/Activated; gear≠Drive |
| **Enabled** | CC ready; CRUISE indicator on; no throttle control | enbl_rise from Disabled with key_on+gear_drive+speed_in_range |
| **Activated** | CC actively controls throttle; SET indicator on | set_rise or resume_rise from Enabled with speed_in_range+gear_drive |

**Default mode:** Disabled

### 5.2 Transitions

| From | To | Condition | Action on Entry |
|------|----|-----------|-----------------|
| Disabled | Enabled | enbl_rise && key_on && gear_drive && speed_in_range | status=true; reqDrv=EnableReq |
| Enabled | Disabled | !key_on OR enbl_rise OR !gear_drive | reset_pid=true; status=false; reqDrv=DisableReq |
| Enabled | Activated (Set) | set_rise && speed_in_range && gear_drive | target=vehicle_speed; enable_pid=true; init_pid_to_drv=true; reqDrv=ActivateReq |
| Enabled | Activated (Resume) | resume_rise && has_prev_target && speed_in_range && gear_drive | target=prev_target; enable_pid=true; init_pid_to_drv=true; reqDrv=ResumeReq |
| Activated | Enabled (Deactivate) | brake_active OR cncl_rise OR !speed_in_range OR (gear≠Drive but key=ON) | enable_pid=false; prev_target=target; reqDrv=DeactivateReq |
| Activated | Disabled (Disable) | !key_on OR enbl_rise OR !gear_drive [higher priority than Deactivate] | enable_pid=false; reset_pid=true; status=false; has_prev=false; reqDrv=DisableReq |

### 5.3 Bumpless Transfer

On entry to Activated: `init_pid_to_drv=true` for one step. The PID block uses external IC = driver_throttle to preload its integrator, ensuring cc_throttle ≈ driver_throttle at t=0 of Activated mode. Discontinuity target: < 2% throttle.

On exit from Activated to Enabled: enable_pid=false → cc_throttle=0 immediately. No bumpless transfer needed since the vehicle is decelerating (brake or cancel).

---

## 6. Cross-Cutting Concerns

### 6.1 Anti-Windup & Integrator Management

| Concern | Approach |
|---------|----------|
| **Anti-windup strategy** | Clamping — disable integration when output saturated and error would deepen saturation |
| **Integrator reset** | reset_pid=true when entering Disabled state |
| **Integrator IC** | Preloaded from driver_throttle on entry to Activated (one-step init_pid_to_drv flag) |

### 6.2 Saturation & Rate Limiting

| Signal | Limit Type | Values | Rationale |
|--------|-----------|--------|-----------|
| cc_throttle | Saturation | [0, 100] % | Physical actuator range |
| cc_target_speed | Saturation | [40, 100] km/h | Requirement §3.1 |

No rate limiting on throttle (PID derivative term provides implicit rate limiting via Kd).

### 6.3 Numerical Safety

| Concern | Approach |
|---------|----------|
| **Division-by-zero** | No divisions in algorithm; not applicable |
| **Overflow protection** | All signals bounded by saturation blocks; single precision sufficient for km/h and % ranges |
| **Precision** | single precision (7 decimal digits) sufficient for 0–100% throttle and 0–200 km/h speed |

### 6.4 Rate Transitions

Single-rate design at 10 ms. No rate transitions required.

### 6.5 Calibration Parameter Management

| Approach | Details |
|----------|---------|
| **Storage** | MATLAB workspace script `crs_controller_params.m` creating `Simulink.Parameter` objects |
| **Naming convention** | Flat namespace matching ROM table names from system spec §6.3 (e.g., `Kp`, `Ki`, `brake_threshold`) |
| **Grouping** | PID gains (Kp, Ki, Kd), speed limits (tsp_min, tsp_max, throttle_min, throttle_max), button behavior (target_inc_short, etc.) |
| **Tunability** | Kp, Ki, Kd, brake_threshold: CalibrationParameter (tunable); all others: ModelParameter (fixed at compile time) |

### 6.6 Code Generation Considerations

Not applicable (simulation-only v1).

### 6.7 Algebraic Loop Prevention

| Concern | Approach |
|---------|----------|
| **Stateflow outputs** | Moore-style chart — outputs updated in state `during` and `entry` actions, never in transitions that read their own outputs |
| **Mode_Manager → Target_Speed_Manager** | mode output from Mode_Manager feeds Target_Speed_Manager; no feedback path back |
| **PID feedback** | vehicle_speed feeds PID; comes from plant (external); no algebraic loop within algorithm |
| **accel_override** | Computed in Mode_Manager from driver_throttle (external input); no feedback loop |

---

## 7. Key Decisions

| # | Decision | Options Considered | Choice | Rationale |
|---|----------|-------------------|--------|-----------|
| 1 | Mode_Manager implementation | (a) Stateflow, (b) MATLAB Function, (c) enabled subsystems | Stateflow Chart | State machines are clearest in Stateflow; supports temporal logic operators |
| 2 | Accelerator override location | (a) Sub-state in Activated, (b) separate signal from Mode_Manager | Signal from Mode_Manager | Keeps Stateflow chart flat; override is a continuous condition, not a state |
| 3 | PID anti-windup method | (a) Back-calculation, (b) Clamping | Clamping | Simpler to implement with built-in PID block; adequate for this application |
| 4 | Target speed manager | (a) Stateflow, (b) MATLAB Function | Stateflow Chart | Transition-based updates (set/resume) map cleanly to Stateflow transitions |
| 5 | Bumpless transfer approach | (a) Integrator preload via IC, (b) output tracking | Integrator preload (IC) | Direct: PID IC = driver_throttle ensures zero bump on entry |

---

## 8. Known Limitations & Deferred Items

| Item | Description | Rationale for Deferral |
|------|-------------|------------------------|
| No derivative filter | Pure discrete derivative on speed error; may amplify quantization noise | Speed sensor assumed clean in simulation; add filter in v2 if needed |
| No rate limit on cc_target_speed changes | Abrupt target speed changes possible during override release | Comfort not a v1 requirement |
| Gear/key enumerations hardcoded | Gear=2 for Drive, Key=2 for ON assumed | Enumerations to be confirmed with vehicle interface spec |

---

## Appendix A: Related Documents

- [System Spec](cruise-control-system.md)
- [Implementation Plan](cruise-control-implementation-plan.md)
- [Test Plan](cruise-control-test-plan.md)
- [Functional Requirements](../../../../CruiseControlFunctionalRequirements.md)

## Appendix B: API Verification Notes

The following Simulink blocks and MATLAB APIs will be used. Verification against existing `crs_controller_test.mldatx` references in the MATLAB project:

- **Simulink Discrete PID Controller block** (Simulink Control Design or built-in): parallel form, discrete-time, external IC, external reset — standard block, verified by toolbox documentation.
- **Stateflow Chart**: `after(N, msec)` temporal operator for 500 ms hold threshold — available in Stateflow since R2016a.
- **`Simulink.Parameter`**: standard Simulink API for calibratable parameters.
- **`Simulink.Bus`**: not required (no bus signals in this interface).

All APIs are standard Simulink/Stateflow. No third-party or custom blocks required.
