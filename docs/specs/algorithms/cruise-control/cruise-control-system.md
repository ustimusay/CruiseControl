# Cruise Control System Spec

## Status: Draft
**Last Updated:** 2026-04-24
**Author:** Claude Code
**Reviewers:** —

---

## 1. Executive Summary

This document specifies a discrete-time cruise control algorithm for an automobile. The algorithm automatically maintains a driver-selected vehicle speed by commanding an engine throttle value via a PID controller. It is a multi-mode supervisory feedback controller that manages three operating modes (Disabled, Enabled, Activated), interprets driver button inputs, and outputs a throttle command and system status signals. The algorithm integrates with a vehicle plant model and a driver input interface. Source requirements: `CruiseControlFunctionalRequirements.md`.

---

## 2. Problem Statement

### Current Pain Points

1. **Manual speed maintenance** — The driver must continuously adjust the accelerator pedal to maintain a constant speed on highways, causing fatigue.
2. **No automated target speed management** — There is no mechanism to set, increment, decrement, or resume a target speed through button inputs.
3. **No mode supervision** — There is no logic to safely enable, activate, deactivate, and disable speed control based on vehicle state (key, gear, speed, brake).

### Opportunity

This algorithm addresses all three issues by providing a fully supervised cruise control function with PID-based throttle regulation, driver-intent interpretation, and safe mode transition logic.

---

## 3. Goals & Success Metrics

### Goals

| Goal | Description |
|------|-------------|
| **G1: Speed Tracking** | Regulate vehicle speed to target ±5% within 20 seconds for step inputs |
| **G2: Safe Mode Management** | Enable/disable/activate/deactivate CC based on vehicle conditions without discontinuous throttle commands |
| **G3: Driver Intent Interpretation** | Correctly interpret Set/Inc/Dec/Resume short and long button presses |
| **G4: Accelerator Override** | Yield throttle control when driver depresses accelerator >15%, resume when <5% |

### Success Metrics

- **Speed tracking accuracy**: |vehicle_speed − cc_target_speed| ≤ 5% of target within 20 s for a 20 km/h step at 40 km/h and at 80 km/h (method: closed-loop MIL simulation)
- **Mode transition correctness**: All mode entry/exit conditions verified against §3 requirements (method: Stateflow chart MIL test)
- **Bumpless transfer**: |cc_throttle discontinuity at Enabled→Activated| < 2% (method: MIL simulation)
- **Button response latency**: Target speed update occurs within 1 sample period of button press detection (method: MIL test)

---

## 4. Non-Goals (v1)

| Non-Goal | Rationale |
|----------|-----------|
| Adaptive PID gain scheduling | Fixed PID gains are specified; gain scheduling not required in this version |
| Fault detection / sensor diagnostics | No sensor fault detection; algorithm trusts all inputs |
| Fuel efficiency optimization | Throttle command optimized for speed tracking only |
| Fixed-point / code generation | Simulation-only in v1 |

---

## 5. Operating Scenarios

| ID | Scenario | Input Conditions | Expected Algorithm Behavior | Performance Criteria |
|----|----------|-----------------|----------------------------|---------------------|
| S1 | Enable CC | key=ON, gear=Drive, speed=60 km/h, enbl pressed (mode=Disabled) | Mode transitions Disabled→Enabled; CRUISE indicator turns on | Transition within 1 sample |
| S2 | Activate CC (Set) | mode=Enabled, set pressed, speed=60 km/h, gear=Drive | Mode transitions Enabled→Activated; cc_target_speed=60 km/h; PID begins regulating | target_speed set within 1 sample |
| S3 | Speed tracking from below | Activated, vehicle_speed=40 km/h, target=60 km/h | Throttle increases; speed converges to 60±3 km/h | Settled within 20 s |
| S4 | Speed tracking from above | Activated, vehicle_speed=80 km/h, target=60 km/h | Throttle decreases; speed converges to 60±3 km/h | Settled within 20 s |
| S5 | Short press Inc | Activated, inc pressed and released <500 ms | cc_target_speed increases by 1 km/h | Update within 1 sample of release |
| S6 | Long press Inc | Activated, inc held ≥500 ms | cc_target_speed increases by 5 km/h/s while held | Speed begins increasing 500 ms after press |
| S7 | Short press Dec | Activated, dec pressed and released <500 ms | cc_target_speed decreases by 1 km/h | Update within 1 sample of release |
| S8 | Long press Dec | Activated, dec held ≥500 ms | cc_target_speed decreases by 5 km/h/s while held | Speed begins decreasing 500 ms after press |
| S9 | Brake deactivation | Activated, brakeP > 5 kPa | Mode transitions Activated→Enabled; throttle returns to 0 | Deactivate within 1 sample |
| S10 | Cancel deactivation | Activated, cncl pressed | Mode transitions Activated→Enabled | Deactivate within 1 sample |
| S11 | Speed out of range | Activated, vehicle_speed < 40 km/h | Mode transitions Activated→Enabled | Deactivate within 1 sample |
| S12 | Resume | mode=Enabled (was Activated), resume pressed, speed∈[40,100], gear=Drive | Mode transitions Enabled→Activated with previous target speed | Resume within 1 sample |
| S13 | Accelerator override | Activated, driver_throttle > 15% | CC releases throttle; cc_throttle = driver_throttle; mode stays Activated | Override within 1 sample |
| S14 | Override release | Override active, driver_throttle < 5% | CC resumes; target_speed = current vehicle_speed; PID resumes | Resume within 1 sample |
| S15 | Disable from Activated | Activated, key transitions from ON | Mode transitions Activated→Disabled; throttle=0 | Within 1 sample |
| S16 | Toggle disable | mode=Enabled or Activated, enbl pressed | Mode transitions to Disabled | Within 1 sample |

---

## 6. External Interface Contract

### 6.1 Inputs

| Name | Description | Unit | Data Type | Sample Time | Sign Convention | Source |
|------|-------------|------|-----------|-------------|-----------------|--------|
| enbl | Cruise button | — | boolean | 10 ms | true = pressed | Driver button |
| cncl | Cancel button | — | boolean | 10 ms | true = pressed | Driver button |
| set | Set button | — | boolean | 10 ms | true = pressed | Driver button |
| resume | Resume button | — | boolean | 10 ms | true = pressed | Driver button |
| inc | Inc button | — | boolean | 10 ms | true = pressed | Driver button |
| dec | Dec button | — | boolean | 10 ms | true = pressed | Driver button |
| brakeP | Brake pressure | kPa | single | 10 ms | 0 = no braking, positive = braking | Brake sensor |
| key | Key position | — | uint8 | 10 ms | 0=Lock, 1=Acc, 2=ON, 3=Start | Ignition switch |
| gear | Shift position | — | uint8 | 10 ms | 0=Park, 1=Neutral, 2=Drive, 3=Reverse | Transmission |
| vehicle_speed | Vehicle speed | km/h | single | 10 ms | Positive = forward motion | Vehicle speed sensor |
| driver_throttle | Accelerator pedal throttle | % | single | 10 ms | 0=released, 100=fully depressed | Accelerator pedal sensor |

### 6.2 Outputs

| Name | Description | Unit | Data Type | Sample Time | Sign Convention | Destination |
|------|-------------|------|-----------|-------------|-----------------|-------------|
| reqDrv | Interpreted driver request | — | Enum: reqMode | 10 ms | See enum definition below | Diagnostic / HMI |
| status | CRUISE indicator (system enabled) | — | boolean | 10 ms | true = CC enabled or activated | Instrument cluster |
| mode | Current operation mode | — | Enum: opMode | 10 ms | See enum definition below | Diagnostic / HMI |
| cc_target_speed | Target speed commanded by CC | km/h | single | 10 ms | Positive, range [40, 100] | Diagnostic / HMI |
| cc_throttle | Engine throttle command from CC | % | single | 10 ms | 0=closed, 100=fully open | Engine throttle actuator |

**Enum: opMode**

| Value | Name | Description |
|-------|------|-------------|
| 0 | Disabled | CC is off |
| 1 | Enabled | CC ready, not controlling throttle |
| 2 | Activated | CC actively controlling throttle |

**Enum: reqMode**

| Value | Name | Description |
|-------|------|-------------|
| 0 | None | No driver request |
| 1 | EnableReq | Driver pushed Cruise to enable |
| 2 | DisableReq | Driver pushed Cruise to disable |
| 3 | ActivateReq | Driver pushed Set to activate |
| 4 | DeactivateReq | Driver pushed Cancel or brake applied |
| 5 | ResumeReq | Driver pushed Resume |
| 6 | SpeedIncReq | Driver pushed Inc |
| 7 | SpeedDecReq | Driver pushed Dec |

### 6.3 Parameters

#### Calibratable Parameters

| Name | Description | Unit | Default | Range | Tuning Guidance |
|------|-------------|------|---------|-------|-----------------|
| Kp | PID proportional gain | — | 20 | [0, 100] | Increase for faster response; watch for overshoot |
| Ki | PID integral gain | — | 0.2 | [0, 10] | Increase to eliminate steady-state error; watch for windup |
| Kd | PID derivative gain | — | −0.0249 | [−1, 0] | Negative value provides damping of speed error |
| brake_threshold | Min brake pressure recognized as braking | kPa | 5 | [1, 20] | Lower = more sensitive to light braking |

#### Fixed Parameters

| Name | Description | Unit | Value | Source / Rationale |
|------|-------------|------|-------|--------------------|
| tsp_min | Minimum allowable target speed | km/h | 40 | Requirement §3.1 |
| tsp_max | Maximum allowable target speed | km/h | 100 | Requirement §3.1 |
| throttle_min | Minimum throttle output | % | 0 | Physical actuator limit |
| throttle_max | Maximum throttle output | % | 100 | Physical actuator limit |
| target_inc_short | Speed increment on short Inc press | km/h | 1 | Requirement §3.5 |
| target_dec_short | Speed decrement on short Dec press | km/h | 1 | Requirement §3.6 |
| target_inc_hold | Speed increment rate on long Inc press | km/h/s | 5 | Requirement §3.7 |
| target_dec_hold | Speed decrement rate on long Dec press | km/h/s | 5 | Requirement §3.8 |
| press_hold_threshold | Duration to distinguish short from long press | ms | 500 | Requirements §3.5–3.8 |
| accel_override_on | Accelerator % above which CC releases throttle | % | 15 | Requirement §3.9 |
| accel_override_off | Accelerator % below which CC resumes | % | 5 | Requirement §3.9 |

---

## 7. Operating Modes

| Mode | Entry Condition | Exit Condition | Active Behavior | Initialization on Entry |
|------|----------------|----------------|-----------------|------------------------|
| **Disabled** | Default on startup; key≠ON; enbl pressed while Enabled or Activated; gear≠Drive | enbl pressed AND key=ON AND gear=Drive AND speed∈[40,100] | cc_throttle=0; status=false | Reset target speed, clear resume history |
| **Enabled** | From Disabled: enbl AND key=ON AND gear=Drive AND speed∈[40,100] | To Disabled: key≠ON OR enbl (toggle) OR gear≠Drive; To Activated: set AND speed∈[40,100] AND gear=Drive (or resume conditions) | cc_throttle=0; status=true; CRUISE indicator on | No integrator active; retain prev_target if was Activated |
| **Activated** | From Enabled: set AND speed∈[40,100] AND gear=Drive; Resume: resume AND has_prev_target AND speed∈[40,100] AND gear=Drive | To Enabled (deactivate): brakeP>threshold OR cncl OR speed∉[40,100] OR gear≠Drive; To Disabled (disable): key≠ON OR enbl OR gear≠Drive | PID regulates vehicle_speed to cc_target_speed; cc_throttle output active | cc_target_speed=vehicle_speed (on Set); cc_target_speed=prev_target (on Resume); initialize PID to avoid bump |

**Accelerator Override sub-behavior within Activated:**
- If driver_throttle > 15%: suspend PID, cc_throttle = driver_throttle
- If override active AND driver_throttle < 5%: end override, cc_target_speed = vehicle_speed, PID resumes

---

## 8. Code Generation & Execution Constraints

Simulation-only (v1). No code generation target.

**Solver:** Fixed-step discrete, sample time 10 ms.

---

## 9. Open Questions

| # | Question | Options | Decision |
|---|----------|---------|----------|
| 1 | Sample time | (a) 10 ms, (b) 20 ms, (c) 50 ms | 🟡 Pending — defaulting to 10 ms |
| 2 | Gear enumeration values (uint8) | (a) Park=0/Neutral=1/Drive=2/Reverse=3, (b) other | 🟡 Pending — using (a) |
| 3 | Key enumeration values (uint8) | (a) Lock=0/Acc=1/ON=2/Start=3, (b) other | 🟡 Pending — using (a) |
| 4 | reqDrv output — is it needed for control logic? | (a) Diagnostic output only, (b) Feeds back into control | ✅ Decided: diagnostic output only |

---

## 10. Future Considerations

- **Adaptive gains**: Gain scheduling vs. vehicle speed or road grade for improved performance
- **Sensor fault handling**: Detect and respond to implausible vehicle speed or brake sensor readings
- **Hill hold / grade compensation**: Feed-forward throttle based on estimated road grade
- **Fixed-point / Code generation**: Deploy to ECU via Embedded Coder

---

## Appendix A: Related Documents

- [Functional Requirements](../../../../CruiseControlFunctionalRequirements.md)
- [Architecture Spec](cruise-control-architecture.md)
- [Implementation Plan](cruise-control-implementation-plan.md)
- [Test Plan](cruise-control-test-plan.md)

## Appendix B: Research Notes

- PID control for automotive cruise control is standard. Negative Kd dampens oscillation when speed error derivative is positive (speed rising toward target).
- Hysteresis on accelerator override (15% on, 5% off) prevents chattering at the threshold.
- 500 ms short/long press threshold is consistent with HMI standards for automotive button hold detection.
