# Self-Right Autonomous Pool Test — Procedure

Hardware: Pixhawk 4 (px4_fmu-v5), 4244_twin_tractor airframe. Design: [self_right_architecture.md](self_right_architecture.md).
Defaults seeded from `pool_self-right_test_22_05_44.ulg` (2026-07-24 manual session).

## Phase 0 — one-time setup (bench, USB)

1. `param set SYS_AUTOSTART 4244`, `param set SR_EN 1`, (`param set SENS_EN_XM125 1` if the radar
   is wired), `reboot`. The airframe then auto-starts `as5600`, `sun_tracker` (muted), `self_right`,
   `ina226_charger` on every boot, and puts SELF_RIGHT on RC flight-mode slot 6.
2. Confirm: `self_right status` (state 0), `listener sensor_encoder` (`valid`, `zeroed`),
   `param show SR_*` (THR_MAX 1.0, OVERCTR 1.4, TIMEOUT 5.0, TILT_SP 1.57, STICK_DZ 0.15),
   `SUN_TRK_EN` = 0.
3. **Boot-position convention: power on with the wing LEVEL, every time.** The encoder zeroes at
   boot; `SR_TILT_SP 1.57` means "+90 deg from boot position". (Pool day 2026-07-24 booted
   props-up — that is why the encoder read ~0 during the flip.)

## Phase 1 — bench checks (props OFF)

Monitor with `listener self_right_status` (QGC MAVLink console).

1. **Tilt regression:** `self_right tilt 45` -> wing drives to +45 deg and holds; `self_right tilt off`
   -> wing stops, no creep.
2. **Gate rejects upright:** level aircraft, RC slot 6 -> mode switch refused.
3. **Verify-fail:** hold inverted, arm in MANUAL, `as5600 stop`, switch to slot 6 -> within ~1 s
   `abort_reason: 4` (VERIFY_FAIL), wing never moves, vehicle disarms itself.
   Restore: `as5600 start -X -b 4`.
4. **Happy path + stick override:** inverted, armed, sticks centered, slot 6 -> states
   1 (Verify) -> 2 (RotateWing, wing to +90 deg) -> 3 (Righting, ramp to full over 1 s). While
   motors spin, wiggle roll -> instant cut, wing parks, `abort_reason: 1` (STICK), disarm.
5. **Simulated flip:** during Righting, physically rotate the airframe past ~80 deg from inverted
   -> `over_center: True`, cut, park, disarm with no stick input.
6. Untouched, Righting times out at 5 s (`abort_reason: 3`) and disarms — correct behavior.

## Phase 2 — pool test (props ON)

Prep: fresh battery (Verify refuses on any low-battery warning), RC in hand, pool clear.

1. Power on dry, wing level. Wait for EKF ready.
2. Arm in MANUAL, throttle zero, then hands off the sticks (>15% deflection aborts).
3. Place inverted, wings level. Let it settle 3-5 s (at-rest detector needs stillness).
4. Flip to slot 6, hands off. Expected: <=1 s verify -> wing props-up (<=5 s) -> 1 s ramp ->
   flip commits within ~3 s of throttle-up -> cut at 80 deg -> buoyancy settles -> wing parks ->
   disarms itself. Total under ~15 s.
5. **Abort at any moment = wiggle any stick** (instant cut + park + disarm). Mode switch also
   stops it; QGC force-disarm is the last resort.
6. After the attempt the vehicle is disarmed but **still in SELF_RIGHT mode**: re-arming while
   still inverted restarts the attempt immediately. Deliberate retries only — otherwise switch
   back to MANUAL before re-arming.
7. Pull the log; `self_right_status` (20 Hz) carries the state/theta/throttle trace for tuning.
