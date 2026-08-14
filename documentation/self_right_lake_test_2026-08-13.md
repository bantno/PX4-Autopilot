# Self-Right Lake Test 2026-08-13 — Findings & Next Test Plan

First open-water attempt of the autonomous self-right sequence (procedure:
[self_right_test_procedure.md](self_right_test_procedure.md)). The maneuver aborted in
RotateWing and the props correctly never spun. Logs: `~/Documents/2026-08-13/`, key files
`18_27_11.ulg` (the attempt), `18_29_52.ulg` (encoder failure), `18_30_27/18_30_38.ulg`
(verify-refused retries).

## Finding 1 — tilt timeout was a tune/time problem, NOT actuator saturation

The attempt (`18_27_11.ulg`, params: KP 1.8 / KI 0.45 / KD 0.11, `SR_TILT_TMO` 5,
`SR_TILT_TOL` 0.1) aborted `ABORT_TILT_TIMEOUT` with the wing at **−82.1°** vs the −90°
setpoint — 2.4° outside the ±5.7° tolerance band, still converging when time ran out.

`wing_tilt_status` trace during RotateWing:

| phase | error | output u | wing motion |
|---|---|---|---|
| 0–1.7 s | 81° → 34° | −1.00 (saturated) | up to 69°/s |
| 1.7–3.6 s | 28° → 8.7° | −0.89 → −0.52 | decelerating |
| 3.6–5.0 s | stuck ~7.8° | −0.52 → −0.57 (integral creeping) | **zero** |

The actuator has plenty of authority: at full command it slews the wing at 60–70°/s
(the whole 80° transit takes <2 s). The stall happened at only **52–57% command** — the
loop simply doesn't ask for enough drive at small errors under water load.

**Water roughly doubles the command needed to keep the wing moving:** on the bench
(`60_percent_bench_test_114`, 2026-08-12) the wing still moved at |u| ≈ 0.28 and entered
tolerance in 2.7 s; in the water the motion threshold is ~0.55–0.6. The bench-tuned gains
put the loop below that threshold at small error, leaving only the integral (winding at
Ki·e ≈ 0.06/s, quasi-static creep time constant Kp/Ki = 4 s) to push through — it needed
roughly 2–3 s more than the 5 s timeout allowed.

**Conclusion: no actuator change needed. Fix = more gain + more time.**

## Finding 2 — AS5600 encoder failed (water), verify gate worked as designed

`TILT_KP` was raised to 3.0 on-site after the abort, but **was never exercised**:

- `18_29_52.ulg`: the encoder raw count starts toggling instantaneously between exactly
  two values — 2795 (−9°) and 1535 = 0x5FF (−119.8°) — on adjacent 10 ms samples. That is
  an apparent 11,000°/s, ~150× the wing's physical slew: an electrical/I2C fault, not
  motion (`wing_tilt` output was 0.00 throughout). The driver's multi-turn unwrap
  swallowed the jumps as real rotation and walked the accumulated angle to −480° raw
  (−307° wing). The wing never physically moved.
- `18_30_27` / `18_30_38`: `sensor_encoder` absent entirely (driver stopped publishing —
  bus dead). Both SELF_RIGHT retries held Verify for the full 1 s window, failed the
  fresh-encoder precondition, aborted `ABORT_VERIFY_FAIL`, and force-disarmed. Motors
  never moved. **The safety gate did its job.**
- Post-mortem: encoder burned out from water intrusion. Replaced; new unit reads.

**Open safety gap:** during the failure the encoder briefly published `valid=1` with the
poisoned −307° angle, and `zeroed` stayed true throughout. If a corrupted-but-"valid"
reading persisted, Verify would pass and the position loop would drive the wing hard
against a phantom ~300° error — the setpoint clamp does not protect against corrupted
*feedback*. TODO: plausibility check in the `as5600` driver (reject single-sample jumps
faster than the wing can physically move, ~100°/s at the encoder, before the unwrap).

## Params for the Sunday water test

| param | value | note |
|---|---|---|
| `TILT_KP` | **3.0** | already set on the vehicle; bench-verify first (see below) |
| `TILT_KI` | 0.45 | unchanged; raise to 0.7 only as an on-site fallback |
| `TILT_KD` | 0.11 | unchanged |
| `SR_TILT_TMO` | **10** | was 5; the log shows convergence needed ~2–3 s more |
| `SR_TILT_TOL` | **0.1** | as flown. The procedure's 0.05 is bench-only — far too tight for water (the wing stalled 0.14 rad out). Widen to 0.15 on-site if a timeout recurs at ≥−82°. |

## Sunday checklist

1. **Bench, props off (new encoder + KP 3.0):**
   - `listener sensor_encoder`: `valid: True`, `zeroed: True`, angle tracks hand motion
     smoothly, no jumps. Wing-level boot as always.
   - `self_right tilt 45` regression per the procedure (Phase 1 step 1).
   - KP 3.0 oscillation check: hold at 0°, −45°, −90° — the light bench load is the
     twitchy regime; watch for limit cycling. If it oscillates, drop to 2.5.
   - Verify-fail path (Phase 1 step 4) still aborts and disarms with the encoder stopped.
2. **Waterproof the encoder**: conformal coat / seal the AS5600 board and I2C connector —
   water killed the last one.
3. **Water test**: Phase 2 of the procedure unchanged. Expected: wing in tolerance in
   ~4–8 s (watch `wing_tilt_status.output` — it should stay above ~0.6 magnitude until
   inside tolerance), then ramp → over-center → cut → disarm.
4. **If it times out again**: pull the log, check the stall angle and `output` at stall.
   Fallbacks in order: `SR_TILT_TOL 0.15`, then `TILT_KI 0.7` (windup is guarded by
   conditional integration, but expect more overshoot on the bench).
5. Pull all logs; `wing_tilt_status` at 20 Hz has the setpoint/measured/error/output
   trace for the next tuning pass.

## Contingency plan — if the autonomous strategy doesn't right it

The hardware is proven: the 2026-07-24 manual session flipped the plane by hand-flying the
same physics (wing props-up + symmetric throttle, flip committed in 2.7 s). So every failure
on Sunday is either a parameter problem (fix on-site) or falls back to the manual method.

### On-site triage by abort reason

Read it live over telemetry: QGC MAVLink console → `self_right status` (or
`listener self_right_status`). The vehicle force-disarms after every abort and **stays in
SELF_RIGHT mode — re-arming while still inverted restarts the attempt immediately**, so
retries are cheap; switch back to MANUAL first if you *don't* want an immediate retry.

| abort_reason | meaning | on-site response, in order |
|---|---|---|
| 4 VERIFY_FAIL | a precondition failed for the full 1 s window; nothing moved | `listener sensor_encoder` (valid? zeroed? fresh?) → power-cycle + wing-level reboot re-zeros; check battery warning (fresh pack); check EKF tilt align. |
| 2 TILT_TIMEOUT | wing didn't reach −90°±tol in `SR_TILT_TMO` | Check stall angle in `wing_tilt status`. Escalate one at a time: `SR_TILT_TOL 0.15` → `TILT_KI 0.7` → `TILT_KP 3.5`. (TMO is already 10 s.) |
| 3 RIGHTING_TIMEOUT | full thrust for `SR_TIMEOUT` didn't cross over-center | Confirm the wing *held* −90° during thrust (`wing_tilt_status` during state 3). Then `SR_TIMEOUT 8`; if it rocks close to vertical but falls back, `SR_OVERCTR 1.3` (cut earlier ≈ 75°) is available — change deliberately, it's the commit threshold. |
| 1 STICK | you (or a drifting TX) crossed the 15% deadzone | Set the TX down / check trims; `SR_STICK_DZ 0.25` if the radio is noisy. |

### Manual fallback (the demonstrated method, updated for current firmware)

Rehearse this on the bench props-off BEFORE Sunday — the July manual flip predates the
`wing_tilt` module, `PWM_MAIN_MIN 1000`, and the ESC auto-arm changes:

1. Switch to MANUAL (leave SELF_RIGHT), stay armed (or re-arm), throttle zero.
2. QGC console: `self_right tilt -90` — the console hold outranks sun_tracker and keeps
   holding while armed in MANUAL; wing drives props-up and resists prop thrust.
3. Watch `wing_tilt status` until measured ≈ −90°, then advance throttle on the RC.
   **Keep yaw/rudder centered** — differential-thrust yaw mixing will unbalance the motors.
4. Cut throttle as it passes ~vertical (the manual session cut at ~90° and buoyancy
   finished the flip; holding past ~107° fell back over).
5. `self_right tilt 0` to park the wing, then `self_right tilt off`, then disarm.

### Prepare before Sunday (bench, props off)

- [ ] Rehearse the manual fallback sequence above end-to-end on current firmware.
- [ ] Walk each triage row once: kill the encoder (`as5600 stop`) → see abort 4; wiggle a
      stick in Righting → abort 1; let Righting run out → abort 3. Practice reading
      `self_right status` for the code.
- [ ] Verify the QGC MAVLink console works over the telemetry radio at lake-like range —
      the whole contingency plan runs through it.
- [ ] Pre-stage the fallback `param set` lines (the table above) in a note on the field
      laptop so they're copy-paste, not composed on the water.
- [ ] Two fresh flight packs (Verify refuses on WARNING_LOW; retries burn charge).
- [ ] Physical retrieval plan for a disarmed inverted plane away from shore (waders /
      kayak / throw line — props are cut whenever disarmed, but treat them as live).
- [ ] Field laptop with `pyulog` for a quick log pull between attempts if triage over
      the console isn't conclusive.
