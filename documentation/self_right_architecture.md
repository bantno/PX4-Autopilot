# Autonomous Self-Righting Mode — Controller Architecture & Gate Logic

> Tilt-wing twin-tractor. Recovers the aircraft from an **inverted, at-rest float on the water** to
> upright, using **propeller thrust only**, then hands back to MANUAL. This document is the design
> reference for the `self_right` module and its commander gate. Keep it in sync with the code.

## 0. Problem & approach

When the aircraft capsizes it floats **stably inverted**. The manual recovery is: rotate the wing so
the props point up, throttle to ~80–90% until the airframe pitches over, hold until upright, then
**cut throttle before the props dip into the water**. The pilot's eyes are both the feedback loop and
the safety cutoff.

This mode automates that as a **gated flight mode**, selectable **only** when the estimator reports
the aircraft is **inverted AND at rest at the instant of the mode transition**. Key physical insight:
the inverted float and the upright float are **both stable equilibria**, separated by an unstable
**tipping point**. We therefore do not need to "flip and catch" — we only need enough thrust to push
the CG **past over-center**, after which buoyancy + gravity settle the aircraft upright on their own.
The controller's job is to detect over-center and then *get out of the way*.

## 1. System context (reused infrastructure)

- Wing (carrying both tractor props) rotates about **BodyY (pitch)** via a reversible-ESC motor driven
  through `Peripheral_via_Actuator_Set1` — i.e. a `vehicle_command` `DO_SET_ACTUATOR` (param7 = 0,
  normalized command in param1). Identical command path to `SunTracker::publishActuator()`.
- Wing-angle feedback: AS5600 magnetic encoder → `sensor_encoder` topic (`angle`, boot-zeroed at
  wing-level). In SITL the plant is the `wing_tilt_sim` module (integrates the actuator command into an
  angle and republishes `sensor_encoder`).
- Props: driven via the `actuator_motors` topic (`control[i]`), published directly by the module
  (the `rover_differential` direct-publish pattern), active only while the mode owns the outputs.

## 2. Reference frames & key quantities

- Body frame FRD; `vehicle_attitude.q` is body→NED (Hamilton, `q[0]=w`).
- **Pitch-from-upright `θ`** is derived from `Quaternion(q).dcm_z()` (body +Z expressed in NED):
  upright → `dcm_z()(2) ≈ +1`, inverted → `≈ −1`. Using `dcm_z` avoids Euler gimbal-lock at ±90°
  pitch — exactly the region traversed. The in-plane components give the flip direction.
- **Pitch rate `q_body`** from `vehicle_angular_velocity` (body Y) — the trusted signal under high
  thrust, when the accelerometer is expected to clip (see §6).
- **Tilt angle `θ_tilt`** from `sensor_encoder.angle`.

## 3. Gate logic (latched)

Enterable **only** when inverted AND at rest, but **not droppable mid-maneuver**. Implemented as a
commander `HealthAndArmingCheck` (`selfRightingCheck`) that owns the `can_run` bit for
`NAVIGATION_STATE_SELF_RIGHT`:

```
inverted = dcm_z(q)(2) < -SR_INV_THR                          # e.g. SR_INV_THR = 0.7
           && accel_body_z sign agrees (vehicle_acceleration) # cross-check vs. a bad quaternion
at_rest  = vehicle_land_detected.at_rest
           && estimator_status_flags.cs_tilt_align
entry_ok = inverted && at_rest
can_run  = entry_ok || (vehicle_status.nav_state == NAVIGATION_STATE_SELF_RIGHT)   # ← LATCH
```

- The `|| already-in-mode` term is the **latch**: `at_rest` goes false the instant the flip starts,
  but because we are already in the mode `can_run` stays true and commander does not eject us.
- Mode requirements: `mode_req_attitude` + `mode_req_angular_velocity` only — **no** position / global
  / home (none are trustworthy inverted on water).
- Exit is driven by the module (→ MANUAL), never by the gate going false.

## 4. Maneuver controller — state machine

Active only while `nav_state == NAVIGATION_STATE_SELF_RIGHT`. One `ScheduledWorkItem` loop. The
`RIGHTING` state dispatches to the law chosen by `SR_STRATEGY`; everything else is shared.

```
        ┌─────────┐ enter mode (gate already passed)
        │ ROTATE  │  tilt PID drives θ_tilt → SR_TILT_SP (~π/2, props up)
        │  WING   │  next when |θ_tilt-SR_TILT_SP| < SR_TILT_TOL  or  t > SR_TILT_TIMEOUT
        └────┬────┘
             ▼
        ┌─────────┐  SR_STRATEGY dispatch:
        │RIGHTING │    0 OPEN_LOOP_REPLAY → replay recorded thr(t)  (× adaptive factor)
        │         │    1 ATTITUDE_PID     → thr = clamp(PID(θ_err), 0, SR_THR_MAX)
        │         │  done when over_center && θ < tol   (or trajectory end)
        └────┬────┘  fail (adaptive) when t > SR_TIMEOUT && !over_center → bump SR_THR_LRN
             ▼
        ┌─────────┐  throttle → 0, wing → SR_TILT_PARK (retracts props from the swept arc)
        │   CUT   │  buoyancy + gravity settle the upright float
        └────┬────┘
             ▼
        ┌─────────┐  publish vehicle_command → set MANUAL
        │  DONE   │
        └─────────┘
```

**`over_center`** (shared success/stop signal): `θ` has crossed below `SR_OVERCTR` and keeps
decreasing — i.e. the CG passed the unstable tipping point and the upright float will complete the
settle without thrust.

## 5. Control laws

- **Tilt loop (ROTATE_WING / park):** small position PID on `sensor_encoder.angle` → normalized
  `DO_SET_ACTUATOR` param1, same shape as `SunTracker`'s PID (the actuator is a *rate* plant, so a
  position loop is required; reuse those gains as a starting point).
- **Prop moment:** **symmetric** thrust — `actuator_motors.control[i] = thr` (equal on both tractors).
  Equal thrust through the tilted (~90°) thrust line, offset from the CG along Z, produces a
  **pitching** moment — exactly the flip axis. Differential thrust is *not* used for righting (it is a
  yaw control).
- **Strategy 0 — OPEN_LOOP_REPLAY:** `thr(t)` is a time-indexed table extracted from a successful
  manual/SITL run (`Tools/self_right/extract_trajectory.py`). No live attitude in the loop → most
  robust to the estimator degradation in §6. The recorded trajectory already ends in a safe cut. With
  adaptive enabled, the table is scaled by the learned peak-thrust factor.
- **Strategy 1 — ATTITUDE_PID:** `thr = clamp(Kp·θ_err + Ki·∫θ_err + Kd·θ̇_err, 0, SR_THR_MAX)` with
  target upright (`θ_err = θ`). One-directional: thrust can only add a flip moment, never reverse, so
  it relies on buoyancy to arrest past center; throttle naturally decays as `θ → 0`.
- **Adaptive thrust learning** (wraps OPEN_LOOP / fixed ramp): monotone **step-up line search** across
  attempts. `thr_peak = SR_THR_LRN` (init `SR_THR_INIT`); on **fail** (timeout, no `over_center`)
  `SR_THR_LRN += SR_THR_STEP`; on **success** latch; bisect once bracketed; hard cap `SR_THR_MAX`.
  Safe because a failed (under-thrust) attempt never crosses center — the props stay up and the
  airframe re-settles inverted, re-arming the gate for the next, slightly stronger attempt. Converges
  to the **minimum** sufficient thrust = the gentlest, least-slam flip.

## 6. Sensor-trust model during the maneuver

- EKF2 `vehicle_attitude` (`dcm_z`) is trusted for the gate, ROTATE_WING, the PID strategy, and the
  over-center test — all relatively low rate. **OPEN_LOOP_REPLAY uses no in-loop attitude**, so it is
  the fallback when the estimator is least trustworthy.
- Under high thrust the accelerometer is expected to exceed 1 g and **clip**; EKF gravity fusion
  self-disables and attitude rides on gyro integration for a few seconds. Body pitch-rate `q_body`
  (gyro) is the most reliable in-maneuver signal and backs the over-center "keeps decreasing" test.
- Yaw is **not** trusted inverted (mag / GNSS-yaw degraded); the controller never uses heading.

## 7. Abort & override (all fail safe: CUT → MANUAL)

- **Stick override:** any `manual_control_setpoint` axis beyond `SR_STICK_DZ` → immediate cut.
- **Rangefinder backstop:** downward `distance_sensor` clearance < `SR_RNG_MIN` → reflex cut (if a
  sensor is present; `SR_RNG_MIN = 0` disables). The only signal that directly senses the water.
- **Estimator unsafe:** `vehicle_imu.delta_velocity_clipping != 0` OR
  `estimator_status_flags.fs_bad_acc_*` → cut for ATTITUDE_PID (attitude untrustworthy); OPEN_LOOP may
  continue (operator-selectable).
- **Backstop timeout:** elapsed > `SR_TIMEOUT` → cut; also the per-attempt fail signal for adaptive.

On any abort: `actuator_motors` → 0, wing → `SR_TILT_PARK`, command MANUAL.

## 8. Parameters (`SR_*`, declared in `self_right/module.yaml`)

| Group | Params |
|---|---|
| Enable / select | `SR_EN`, `SR_STRATEGY` (0 = open-loop replay, 1 = attitude PID) |
| Gate | `SR_INV_THR` (inverted `dcm_z` threshold) |
| Tilt | `SR_TILT_SP`, `SR_TILT_PARK`, `SR_TILT_TOL`, `SR_TILT_TIMEOUT`, tilt PID gains |
| Righting / cut | `SR_THR_MAX` (hard ceiling), `SR_RAMP_T`, `SR_OVERCTR` (tipping angle), `SR_TIMEOUT` |
| Attitude PID | `SR_PID_P`, `SR_PID_I`, `SR_PID_D` |
| Adaptive | `SR_ADAPT_EN`, `SR_THR_INIT`, `SR_THR_STEP`, `SR_THR_LRN` (persisted learned thrust) |
| Safety | `SR_STICK_DZ`, `SR_RNG_MIN` (rangefinder clearance cut, 0 = disabled) |

## 9. Telemetry

`SelfRightStatus` publishes: state-machine state, active strategy, `θ` (pitch-from-upright),
`q_body`, `θ_tilt`, commanded throttle, `over_center` flag, adaptive `SR_THR_LRN` and attempt
number/result, and active abort reason — for live tuning and post-flight log analysis.

## 10. Bring-up order (safety)

1. **Phase 0 (no code):** record 2–3 manual rightings in MANUAL mode; extract thrust profile, peak
   pitch rate, tipping angle, total duration, and whether EKF attitude survives the thrust. These seed
   `SR_THR_*`, `SR_OVERCTR`, `SR_TIMEOUT`, tilt PID.
2. **SITL:** validate the latched gate (rejected upright, accepted inverted+at-rest), then each
   strategy; confirm the latch holds when `at_rest` drops, stick override, and all aborts.
3. **Hardware bench:** confirm the wing reaches ≥90° and props spin up under the mode.
4. **First water trials:** OPEN_LOOP replay of a known-good manual trajectory, RC manual-cut mapped as
   the ultimate backstop. Enable adaptive only after replay is trusted.
