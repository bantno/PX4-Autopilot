# Self-Right Autonomous Pool Test — Procedure

Hardware: Pixhawk 4 (px4_fmu-v5), 4244_twin_tractor airframe. Design: [self_right_architecture.md](self_right_architecture.md).
Righting defaults seeded from `pool_self-right_test_22_05_44.ulg` (2026-07-24 manual session).
**2026-08-13 lake test findings + revised gains/timeout for open water:
[self_right_lake_test_2026-08-13.md](self_right_lake_test_2026-08-13.md)** — in short:
KP 3.0, `SR_TILT_TMO` 10, and `SR_TILT_TOL` 0.1 in water (0.05 below is bench-only).
The wing tilt loop now lives in the **`wing_tilt` module** — the single owner of the tilt ESC.
sun_tracker and self_right publish setpoints to it; priority: **self_right > console > sun_tracker**.

## Phase 0 — one-time setup (bench, USB)

1. Select the airframe and enable the mode:
   ```
   param set SYS_AUTOSTART 4244
   param set SR_EN 1
   param set SENS_EN_XM125 1      # only if the radar is wired
   ```
2. Set the tuned tilt-loop values (bench-tuned 2026-08-11, supersedes the 2026-08-10 pass —
   the old SUN_* params are gone, and the logged SUN_GEAR_RATIO 0.8 turned out to be the
   reciprocal — the bevel gears the encoder UP off the wing spar. KP/KI/KD were then retuned
   together: raising KI alone for steady-state accuracy caused overshoot during the throttle-on
   Righting hold, so KD was added to damp it, which then allowed KI to go higher still):
   ```
   param set TILT_GEAR 1.2        # encoder turns per wing turn (measured; was wrongly 0.8)
   param set TILT_KP 1.8          # proportional gain (was default 2.0)
   param set TILT_KI 0.45         # integral gain — kills steady-state droop; raise with KD together
   param set TILT_KD 0.11         # derivative gain — damps overshoot from raising KP/KI
   param set SR_TILT_TOL 0.05     # props-up/park tolerance, rad (was default 0.1)
   param set PWM_MAIN_MIN1 1000   # motor min = ESC zero-throttle (was 1100 = armed idle spin):
   param set PWM_MAIN_MIN2 1000   # throttle 0 must NOT spin the props while the wing rotates
   ```
   `TILT_DB 0.01` default still matches the tune — it's the deadband floor below which the loop
   stops correcting (avoids ESC dither); runtime-adjustable if the deadzone ever needs revisiting.
   `TILT_GEAR` is read once at startup, so:
3. `reboot`. The airframe auto-starts `wing_tilt`, `as5600`, `sun_tracker`, `self_right`,
   `ina226_charger` on every boot, and puts SELF_RIGHT on RC flight-mode slot 6.
4. **Boot-position convention: power on with the wing LEVEL, every time.** The encoder zeroes at
   boot; all setpoints (`SR_TILT_SP -1.57` = props-up) are wing angle relative to that zero.
5. **Tilt ESC arms itself** ~`TILT_ARM_DLY` (5 s) after boot: a slow sine wiggle about neutral
   (`TILT_ARM_V 0.1`, `TILT_ARM_N 5` cycles of `TILT_ARM_T 2.0` s — bench-confirmed 2026-08-13)
   runs and the ESC chirps; the wing then drives itself back to the boot-zero position. No QGC
   actuators-tab dance. If the ESC was power-cycled separately, re-run with `wing_tilt esc_arm`.
   (The greenjay firmware arms on pulses within ±10 µs of its stored center, which is slightly
   off 1500 — steady neutral never arms it; the wiggle's slow center crossings do.)
6. Confirm after reboot:
   - `wing_tilt status` → `owner: none`, `encoder: ok`
   - `self_right status` → state 0
   - `listener sensor_encoder` → `valid: True`, `zeroed: True`
   - `param show SR_*` → THR_MAX 1.0, OVERCTR 1.4, TIMEOUT 5.0, TILT_SP -1.57, TILT_TOL 0.05,
     STICK_DZ 0.15
   - `param show TILT_*` → KP 1.8, KI 0.45, KD 0.11, DB 0.01, GEAR 1.2

   `SUN_TRK_EN` may stay whatever you like now — the wing_tilt arbitration means the sun tracker
   can never fight self_right or a console hold. Set it 0 anyway for quieter logs during testing.

## Phase 1 — bench checks (props OFF)

Monitor with `listener self_right_status` and `wing_tilt status` (QGC MAVLink console).

1. **Gear/tilt regression:** `self_right tilt 45` → the wing drives to a **physical** +45° and
   holds against a gentle push; `wing_tilt status` shows `owner: console`, `measured ~45 deg`,
   small error. (The raw encoder will read ~54° = 45° × 1.2 — the de-gearing is correct if the
   *wing* is at 45°.) `self_right tilt off` → wing stops within ~0.5 s, `owner: none`.
2. **Arbitration:** with `SUN_TRK_EN 1` and the tracker in any state, `self_right tilt 20` must
   take the wing (`owner: console`) and hold it steadily — no jitter, no fighting. `tilt off`
   returns ownership (`owner: sun_tracker` if it is publishing, else `none`).
3. **Gate rejects upright:** aircraft level, try RC flight-mode slot 6 → mode switch refused.
4. **Verify-fail path:** hold the aircraft inverted, arm in MANUAL, `as5600 stop`, switch to
   slot 6 → within ~1 s `abort_reason: 4` (VERIFY_FAIL), the wing never moves, and the vehicle
   **disarms itself**. Restore with `as5600 start -X -b 4`.
5. **Happy path + stick override:** inverted, armed, sticks centered, switch to slot 6 → states
   walk 1 (Verify) → 2 (RotateWing, `wing_tilt` owner becomes `self_right`, wing to −90°) →
   3 (Righting, motors ramp to full over 1 s). While motors spin, wiggle the roll stick →
   instant cut, wing parks, `abort_reason: 1` (STICK), disarm.
6. **Simulated flip:** repeat entry, and during Righting physically rotate the airframe past
   ~80° from inverted → `over_center: True`, motors cut, wing parks, disarm — no stick input.
7. Untouched, Righting times out at 5 s (`abort_reason: 3`) and disarms — correct behavior,
   not a bug.

## Phase 2 — pool test (props ON)

Prep: fresh battery (Verify refuses on any low-battery warning), RC in hand, pool clear.

1. Power on dry land, **wing level**. Wait for EKF ready.
2. Arm in MANUAL, throttle zero, then hands off the sticks (>15% deflection aborts).
3. Place the plane in the pool inverted, wings level. Let it settle 3–5 s. (No firmware at-rest
   gate anymore — the land detector is unreliable on water, so the checks were removed 2026-08-13.
   Settling before entry is now on you, the operator.)
4. Flip to slot 6, hands off. Expected: ≤1 s verify → wing props-up (≤5 s) → 1 s throttle ramp →
   flip commits within ~3 s of throttle-up (the manual one took 2.7 s) → cut at 80° → buoyancy
   settles it → wing parks → **disarms itself**. Total under ~15 s.
5. **Abort at any moment = wiggle any stick** (instant cut + park + disarm). Mode switch also
   stops it; QGC force-disarm or `commander disarm -f` is the last resort.
6. After the attempt the vehicle is disarmed but **still in SELF_RIGHT mode**: re-arming while
   still inverted restarts the attempt immediately. Deliberate retries only — otherwise switch
   back to MANUAL before re-arming.
7. Pull the log; `self_right_status` (20 Hz) has the state/θ/throttle trace and
   `wing_tilt_status` (20 Hz) the tilt setpoint/measured/error/output trace for tuning.
