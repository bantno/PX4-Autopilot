# Autonomous Self-Righting Mode — Controller Architecture & Gate Logic

> Tilt-wing twin-tractor. Recovers the aircraft from an **inverted, at-rest float on the water** to
> upright, using **propeller thrust only**, then **safely disarms**. This document is the design
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

The design is deliberately minimal: **one righting law** (open-loop throttle ramp with an
attitude-based cut), an explicit **verify** step before anything moves, and **every exit path ends in
a forced disarm** — success, timeout, verify failure, or pilot stick override.

## 1. System context (reused infrastructure)

- Wing (carrying both tractor props) rotates about **BodyY (pitch)** via a reversible-ESC motor.
  The **`wing_tilt` module is the single owner of that ESC**: it closes the position loop on the
  AS5600 encoder (`sensor_encoder`, de-geared by 1/`TILT_GEAR`) and publishes `DO_SET_ACTUATOR` →
  `Peripheral_via_Actuator_Set1`. Client modules never command the ESC directly — they publish
  wing-angle setpoints on `wing_tilt_setpoint`, arbitrated by priority (**self_right > console >
  sun_tracker**) and freshness (a source releases the wing by not republishing for 0.5 s).
- Wing-angle feedback: AS5600 magnetic encoder → `sensor_encoder` topic (`angle`, boot-zeroed at
  wing-level). In SITL the plant is the `wing_tilt_sim` module (integrates the actuator command into an
  angle and republishes `sensor_encoder`).
- Props: driven via the `actuator_motors` topic (`control[i]`), published directly by the module
  (the `rover_differential` direct-publish pattern), active only while the mode owns the outputs.

## 2. Reference frames & key quantities

- Body frame FRD; `vehicle_attitude.q` is body→NED (Hamilton, `q[0]=w`).
- **Pitch-from-upright `θ`** is derived from `Quaternion(q).dcm_z()` (body +Z expressed in NED):
  upright → `dcm_z()(2) ≈ +1`, inverted → `≈ −1`. Using `dcm_z` avoids Euler gimbal-lock at ±90°
  pitch — exactly the region traversed.
- **Pitch rate `q_body`** from `vehicle_angular_velocity` (body Y) — logged for diagnostics.
- **Tilt angle `θ_tilt`** from `sensor_encoder.angle`.

## 3. Gate logic (latched) + module-side verify

Enterable **only** when inverted AND at rest, but **not droppable mid-maneuver**. Implemented as a
commander `HealthAndArmingCheck` (`selfRightingCheck`) that owns the `can_run` bit for
`NAVIGATION_STATE_SELF_RIGHT`:

```
inverted = dcm_z(q)(2) < -SR_INV_THR                          # e.g. SR_INV_THR = 0.7
at_rest  = vehicle_land_detected.at_rest
           && estimator_status_flags.cs_tilt_align
entry_ok = inverted && at_rest
can_run  = entry_ok || (vehicle_status.nav_state == NAVIGATION_STATE_SELF_RIGHT)   # ← LATCH
```

- The `|| already-in-mode` term is the **latch**: `at_rest` goes false the instant the flip starts,
  but because we are already in the mode `can_run` stays true and commander does not eject us.
- Mode requirements: `mode_req_attitude` + `mode_req_angular_velocity` only — **no** position / global
  / home (none are trustworthy inverted on water).
- Exit is driven by the module (forced disarm), never by the gate going false.

On top of the gate, the module runs its own **VERIFY state** on entry, before anything moves. All of
the following must hold within a 1 s window, else it disarms without moving:

1. **Inverted** — `cos(θ) < -SR_INV_THR` (same test as the gate).
2. **At rest** — `vehicle_land_detected.at_rest`.
3. **EKF tilt aligned** — `estimator_status_flags.cs_tilt_align`.
4. **Encoder fresh + trustworthy** — `sensor_encoder` newer than 1 s, `valid`, `zeroed`. The wing is
   never driven open-loop.
5. **Battery OK** — `battery_status` (primary instance) fresh, `connected`, warning below
   `WARNING_LOW`.

## 4. Maneuver controller — state machine

Active only while `nav_state == NAVIGATION_STATE_SELF_RIGHT`. One `ScheduledWorkItem` loop (100 Hz).

```
        ┌─────────┐ enter mode (gate already passed)
        │ VERIFY  │  preconditions (§3), motors off, wing untouched
        │         │  pass → ROTATE WING;  >1 s without pass → DISARM (nothing moved)
        └────┬────┘
             ▼
        ┌─────────┐
        │ ROTATE  │  tilt PID drives θ_tilt → SR_TILT_SP (−π/2, props up), motors off
        │  WING   │  |θ_tilt−SP| < SR_TILT_TOL → RIGHTING
        └────┬────┘  t > SR_TILT_TMO → CUT (FAILURE — never thrust with the props misplaced)
             ▼
        ┌─────────┐
        │RIGHTING │  hold wing up; thr = min(t/SR_RAMP_T, 1) · SR_THR_MAX
        │         │  over_center (θ < SR_OVERCTR, latched) → CUT (success)
        └────┬────┘  t > SR_TIMEOUT → CUT (failure)
             ▼
        ┌─────────┐  throttle → 0, wing → SR_TILT_PARK (retracts props from the swept arc)
        │   CUT   │  buoyancy + gravity settle the float
        └────┬────┘  parked (or SR_TILT_TMO) → DISARM
             ▼
        ┌─────────┐  neutral tilt command, forced COMPONENT_ARM_DISARM (param2 = 21196),
        │ DISARM  │  re-sent every 500 ms until commander reports disarmed → IDLE
        └─────────┘
```

**Pilot stick override** (any roll/pitch/yaw beyond `SR_STICK_DZ`) is checked in VERIFY, ROTATE WING,
RIGHTING and CUT: it immediately routes to CUT (or straight to DISARM if nothing has moved yet).
Switching flight modes on the RC also ends the maneuver instantly — the module resets, stops
publishing motors and sends a neutral tilt command.

**`over_center`**: `θ` has crossed below `SR_OVERCTR` while thrusting — the CG passed the unstable
tipping point and the upright float will complete the settle without thrust.

## 5. Control laws

- **Tilt (ROTATE_WING / park):** the module publishes wing-angle setpoints (`SR_TILT_SP`,
  `SR_TILT_PARK`) to the **`wing_tilt` controller**, which owns the 20 Hz encoder PID
  (`TILT_KP/KI/KD`, `TILT_DB` error deadband, `TILT_GEAR`). Ownership is released by simply not
  republishing; the controller then stops the rate-plant actuator with one neutral command, so
  the wing can never run away on a stale correction.
- **Prop moment:** **symmetric** thrust — `actuator_motors.control[0..1] = thr` (equal on both
  tractors). Equal thrust through the tilted (~90°) thrust line, offset from the CG along Z, produces
  a **pitching** moment — exactly the flip axis.
- **Righting law (the only one):** ramp the throttle linearly over `SR_RAMP_T` to `SR_THR_MAX` and
  hold; cut on over-center or `SR_TIMEOUT`. Seed `SR_THR_MAX` / `SR_RAMP_T` / `SR_OVERCTR` /
  `SR_TIMEOUT` from a logged successful manual righting.

## 6. Sensor-trust model during the maneuver

- EKF2 `vehicle_attitude` (`dcm_z`) is trusted for the gate, VERIFY and the over-center test.
- Under high thrust the accelerometer may exceed 1 g and clip; EKF gravity fusion self-disables and
  attitude rides on gyro integration for a few seconds — good enough for the single over-center
  threshold test. `SR_TIMEOUT` backstops the case where attitude goes bad entirely.
- Yaw is **not** trusted inverted (mag / GNSS-yaw degraded); the controller never uses heading.

## 7. Abort & override (all paths end in a forced disarm)

- **Stick override:** any `manual_control_setpoint` axis beyond `SR_STICK_DZ` → immediate cut → park
  → disarm.
- **Tilt timeout:** wing not at props-up within `SR_TILT_TMO` → abort **before any thrust**.
- **Righting timeout:** no over-center within `SR_TIMEOUT` → cut → park → disarm.
- **Verify failure:** preconditions (§3) not met within 1 s of mode entry → disarm, nothing moved.

The disarm is a `VEHICLE_CMD_COMPONENT_ARM_DISARM` with **param2 = 21196 (force)** — this skips
`Commander::disarm()`'s "not landed" refusal, which cannot be trusted floating on water. The command
is re-sent every 500 ms until commander reports disarmed.

## 8. Parameters (`SR_*`, declared in `self_right/module.yaml`)

| Group | Params |
|---|---|
| Enable | `SR_EN` |
| Gate / verify | `SR_INV_THR` (inverted `dcm_z` threshold) |
| Tilt | `SR_TILT_SP`, `SR_TILT_PARK`, `SR_TILT_TOL`, `SR_TILT_TMO`; the position loop lives in the `wing_tilt` module (`TILT_KP/KI/KD`, `TILT_DB`, `TILT_GEAR`) |
| Righting / cut | `SR_THR_MAX` (peak throttle), `SR_RAMP_T`, `SR_OVERCTR` (tipping angle), `SR_TIMEOUT` |
| Safety | `SR_STICK_DZ` (pilot override deadzone) |

## 9. Telemetry

`SelfRightStatus` publishes: state-machine state, `θ` (pitch-from-upright), `q_body`, `θ_tilt`,
commanded throttle, `over_center` flag, and the active abort reason — for live tuning and post-test
log analysis.

## 10. Bring-up order (safety)

1. **Manual reference:** record a successful manual righting; extract peak throttle, ramp time,
   tipping angle and total duration to seed `SR_THR_MAX`, `SR_RAMP_T`, `SR_OVERCTR`, `SR_TIMEOUT`.
2. **Hardware bench (props off):** `self_right tilt` regression; gate rejects upright; inverted entry
   runs VERIFY → ROTATE WING → RIGHTING; stick wiggle aborts and **disarms** (even with the land
   detector not reporting landed); unplugged encoder fails VERIFY without moving the wing.
3. **First water trials:** pilot places the aircraft inverted wings-level, arms, selects SELF_RIGHT;
   RC sticks are the ultimate backstop (any deflection = cut + disarm).
