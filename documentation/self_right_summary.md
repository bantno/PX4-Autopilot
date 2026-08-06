# Self-Righting Controller — Summary

Autonomous inverted-recovery mode for the tilt-wing twin-tractor aircraft.
Module: `src/modules/self_right/`. Full design reference: [self_right_architecture.md](self_right_architecture.md).

## Overview

- **Purpose:** Autonomous inverted-recovery mode (`NAVIGATION_STATE_SELF_RIGHT`) for the tilt-wing twin-tractor. Flips the aircraft from a stable upside-down float on water back to upright using **prop thrust only**, then **safely disarms**.
- **Key insight:** Inverted and upright floats are both stable equilibria separated by a tipping point — so it only needs enough thrust to push the CG **past over-center**, then cut and let buoyancy finish.
- **Gate (latched):** Enterable **only** when the estimator says inverted **AND** at rest; once entered, a latch keeps it from being ejected mid-flip when `at_rest` drops. The module then re-verifies (inverted, at rest, EKF tilt-aligned, fresh wing encoder, battery OK) before anything moves.
- **Actuation:** Rotate wing props-up (setpoint → `wing_tilt` controller, the single owner of the tilt ESC), then **symmetric** thrust on both tractors → pure pitching moment. One law only: ramp to `SR_THR_MAX` over `SR_RAMP_T`, cut on over-center or `SR_TIMEOUT`.
- **Safety:** Every exit path — success, verify failure, tilt timeout, righting timeout, pilot stick override — cuts throttle, parks the wing and **force-disarms** (param2 = 21196, so a "not landed" refusal can't strand it armed). Any stick deflection beyond `SR_STICK_DZ` overrides the autonomy at any point.

## Flow Chart

Rendered version: [self_right_states.svg](self_right_states.svg). Parameter provenance from the
2026-07-24 manual pool flip: [self_right_param_seeding.png](self_right_param_seeding.png)
(regenerate with `Tools/self_right/plot_param_seeding.py`).


```
                    ┌─────────────────────────────┐
                    │  Commander gate (latched)   │
                    │  inverted && at_rest        │
                    │  || already-in-mode         │
                    └──────────────┬──────────────┘
                                   │ pilot enters SELF_RIGHT
                                   ▼
                          ┌─────────────────┐
                          │     VERIFY      │  inverted + at_rest + tilt_align
                          │   motors off    │  + encoder fresh + battery OK
                          └────────┬────────┘
                    all pass       │        >1 s without pass ──────────┐
                                   ▼                                    │
                          ┌─────────────────┐                           │
                          │   ROTATE WING   │  tilt PID → props-up      │
                          │   motors off    │  (~90°)                   │
                          └────────┬────────┘                           │
                 |θ_tilt−SP|<tol   │        t > SR_TILT_TMO → CUT       │
                                   ▼        (failure: never thrust)     │
                          ┌─────────────────┐                           │
                          │    RIGHTING     │  hold wing up + ramp to   │
                          │                 │  SR_THR_MAX               │
                          └───┬────────┬────┘                           │
                over_center   │        │  t > SR_TIMEOUT                │
              (flip committed)│        │  or stick override             │
                              ▼        ▼                                │
                          ┌─────────────────┐                           │
                          │      CUT        │  throttle→0, wing→park    │
                          │                 │  buoyancy settles it      │
                          └────────┬────────┘                           │
                                   ▼                                    │
                          ┌─────────────────┐                           │
                          │     DISARM      │  neutral tilt, forced  ◀──┘
                          │                 │  disarm (retried 500 ms)
                          └─────────────────┘
```
