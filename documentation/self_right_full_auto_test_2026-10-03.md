# Self-right full-auto water test — 2026-10-03 (ledger + hypotheses)

Logs: `~/Logs/10-3-Self-Righting-Full-Auto/` (37 files). Firmware `ae0308c71f` on every log; flight board GUID `…46002b`.
Site ≈ 34.0563, −84.6904. Log file names are UTC; local time (EDT) = UTC − 4 h.

**Status: data only, no conclusions.** Operator summaries go in the "Operator notes" lines.

## Which logs are the in-water attempts

`self_right_status` shows 17 activations. Only five reached Righting → over-center → Cut with `abort_reason = 0`:

| # | Log | Arm (UTC / EDT) | Power-on session |
|---|-----|-----------------|-------------------|
| 1 | `18_49_03.ulg` | 18:49 / 14:49 | B (boot that started at 18_39 UTC) |
| 2 | `18_53_54.ulg` | 18:54 / 14:54 | B |
| 3 | `18_55_11.ulg` | 18:55 / 14:55 | B |
| 4 | `19_17_59.ulg` (attempt at +304 s) | 19:23 / 15:23 | C (boot at 19_17_59) |
| 5 | `19_24_55.ulg` | 19:25 / 15:25 | C |

The other 12 activations, all between 15:39 and 15:55 UTC, are summarized in the appendix. They show 0 A battery current throughout.

Parameters common to all five: `SR_THR_MAX 1.0`, `SR_RAMP_T 1.0 s`, `SR_OVERCTR 1.4 rad (80.2°)`, `SR_TIMEOUT 5 s`,
`SR_TILT_SP −1.57`, `SR_TILT_TOL 0.10 rad (5.7°)`, `SR_TILT_TMO 10 s`, `SR_TILT_PARK 0`, `TILT_KI 0`, `TILT_KD 0.3`,
`TILT_GEAR 1.0`. **Differs:** `TILT_KP` = 20 for #1–3, 15 for #4–5 (changed in `19_06_15.ulg`).

## Ledger

Times are seconds from the start of the Righting phase unless stated otherwise. θ = angle of body +Z from upright (180° = fully inverted).

| | #1 18_49_03 | #2 18_53_54 | #3 18_55_11 | #4 19_17_59 | #5 19_24_55 |
|---|---|---|---|---|---|
| TILT_KP | 20 | 20 | 20 | 15 | 15 |
| θ at mode entry | 159.8° | 160.6° | 161.0° | 172.0° | 175.5° |
| Inverted attitude (roll / pitch) | −173 / −19 | −174 / −18 | −173 / −18 | −174 / +5 | +178 / +4 |
| Verify | passed in 1 cycle | 1 cycle | 1 cycle | 1 cycle | 1 cycle |
| RotateWing duration (0 → within tol) | 4.04 s | 3.30 s | 3.30 s | 3.20 s | 3.28 s |
| Wing at Righting entry | −84.5° | −85.3° | −85.1° | −83.5° | −83.9° |
| θ change during RotateWing | 160 → 149° | 161 → 149° | 161 → 150° | 172 → 170° | 176 → 173° |
| Wing range while thrusting | −92.3…−85.7° | −92.5…−85.3° | −94.8…−85.1° | −92.4…−85.1° | −92.7…−85.6° |
| Tilt cmd \|u\| max while thrusting | 1.00 | 1.00 | 1.00 | 1.00 | 0.96 |
| First hull motion (>3°) | +0.66 s @ thr 0.66 | +0.64 s @ 0.64 | +0.64 s @ 0.64 | +0.68 s @ 0.68 | +0.66 s @ 0.67 |
| Over-center latched | +1.24 s @ thr 1.00 | +1.22 s | +1.28 s | +1.42 s | +1.44 s |
| Peak rate (\|q\| pitch / \|p\| roll) | 8.5 / 3.3 rad/s | 7.0 / 2.1 | 9.3 / 2.6 | 9.1 / 2.5 | 6.8 / 5.2 |
| Peak accel magnitude | 43.5 m/s² | 43.7 | 49.7 | 31.2 | 27.0 |
| Pitch extreme during flip | −83° | −85° | −84° | −89° | −78° |
| Battery before / min / peak I | 25.08 V / 21.88 V / 139 A | 25.04 / 21.87 / 140 | 25.02 / 21.91 / 135 | 24.92 / 21.88 / 138 | 24.88 / 21.81 / 140 |
| mAh used by the maneuver | 24 | 23 | 25 | 30 | 32 |
| Motor PWM | 2000 peak; 1000 after Cut | same | same | same | same |
| Cut duration | 0.17 s | 0.23 s | 0.23 s | 0.19 s | 3.01 s |
| How Cut ended | mode switch → Manual (+0.16 s after over-center) | mode switch → Manual (+0.20 s) | mode switch → Manual (+0.21 s) | mode switch → Manual (+0.17 s) | wing reached park → Disarm state |
| Wing parked? | no (encoder lost, last −92°) | no, released at −52.6° | no, released at −49.9° | no, released at −59.7° | yes, −3.8° → −0.4° |
| Disarm | "Disarmed by landing" +4.6 s after over-center | "Disarmed by landing" +4.6 s | "Disarmed by landing" +4.5 s | "Disarmed by landing" +4.1 s | self_right force-disarm (cmd 400, p2 = 21196), "Disarmed by internal command" +3.0 s |
| Encoder dropouts | invalid 0.1 s after over-center, did not recover in log | none | two short ones, 1.6 s and 3.2 s after Idle | none during attempt | 1.1 s, starting 0.2 s into Cut |
| Settled attitude before disarm (roll / pitch) | +1.5 / +19.4 | +5.3 / +20.6 | +4.4 / +19.9 | −8.8 / −4.5 | −4.6 / −10.6 |
| Sticks (R/P/Y) during maneuver | centered (≤0.03), throttle −1 | same | same | same | same |
| Operator notes | Operator switched out of Self Right to stop a prop slam | same | same | same | No switch-out; **prop hit the water and broke the mount** |

Every attempt logged "Takeoff detected" during Righting, then "Landing detected" about 2 s before the land-detector disarm.

## Hypotheses (unconfirmed)

**H1 — Exit path.** In #1–4 the mode switch went back to Manual about 0.2 s after over-center. That ended self_right's active mode, so the
state machine dropped from Cut straight to Idle and skipped both park and force-disarm. The land detector's auto-disarm is
what disarmed those four. #5 is the only run where the designed exit ran (Cut → park → force-disarm). Was the
switch-back a deliberate procedure step or a reflex?

**H2 — Exposure after the switch-back.** In #1–4 the vehicle was armed in Manual for about 4–5 s after the flip. The props stayed at PWM 1000
only because the throttle stick was at the bottom. Whether that is a hazard depends on where the throttle stick sits during
the procedure.

**H3 — Thrust margin.** The hull starts moving at a commanded throttle of 0.64–0.68 every time. Over-center happens only after the ramp has
reached 1.0. This could mean full throttle is genuinely needed. It could also be a lag from the water's added mass and inertia
during a fast 1 s ramp. A fixed ramp can't tell these apart.

**H4 — Two populations (#1–3 vs #4–5).** These are different power-ons, and they show different floating trims:

- Inverted pitch: −18° vs +5°.
- Upright pitch after the flip: +20° vs −5…−11°.
- Time to over-center: 1.22–1.28 s vs 1.42–1.44 s.
- Peak acceleration: 4.4–5.1 g vs 2.8–3.2 g.

Several changes coincide with the split: TILT_KP 20 → 15, a lower pack voltage, a new boot, and any physical change made between
sessions (ballast, battery position, water ingress, how the vehicle was set down). A vehicle that starts closer to
fully inverted also has more rotation to cover before 80°.

**H5 — Encoder dropouts are linked to the flip.** #1, #3 and #5 all lost the encoder at or shortly after the flip. The timing lines up with
the 3–5 g peak and splashing. The valid flag dropped cleanly, without the Aug-13 unwrap garbage. Separately, four boots this
afternoon logged `as5600: I2C init failed` (18_27, 18_59, 19_00, 19_01), which points to an intermittent connection. In #5
the dropout paused the park for about 1.1 s; Cut still finished.

**H6 — Zero reference after #1.** Closed. The operator confirmed the encoder zero was set correctly before every trial, so wing angles are comparable across all five.

**H7 — Wing loading under thrust.** The wing enters Righting at −83…−85° because the 5.7° tolerance is reached while it is
still moving. It then overshoots to −92…−95° with the tilt command saturated, in 4 of 5 runs. This could be overshoot
from the KP 15–20 / KI 0 loop, or reaction torque and aerodynamic load as the props spin up.

**H8 — Wing rotation changes the inverted float.** In #1–3, RotateWing alone pitched the hull about 11° (θ 160 → 149°); in #4–5 it was about 2°.
This could be related to H4.

**H9 — Flip mode.** All five runs flip mainly about body pitch (\|q\| 6.8–9.3 rad/s vs \|p\| ≤ 5.2). They pass through about −80…−89° Euler
pitch, and heading changes about 170–180°. That looks like pitching end-over-end, not rolling.

**H10 — TILT_GEAR.** Closed. The operator confirmed 1.0 is correct: the encoder drive was changed to 1:1, so encoder angle equals wing angle.

**H11 — Battery.** Each attempt costs about 23–32 mAh, peaks at 135–140 A and sags to 21.8–21.9 V (about 3.64 V/cell on 6S). That
implies roughly 22 mΩ of pack + wiring resistance. `battery_status.warning` stayed at 0 throughout.

## Prop-strike analysis (added 2026-10-04)

- In all five trials, self_right cut the motors at over-center (θ ≈ 76–79°). Commanded throttle went to 0 that cycle, PWM read 1000 by the next 10 Hz sample, and battery current read ≈0 within about 0.1 s.
- In trials 1–4 the operator switched out 0.16–0.21 s after that cut, by which point the props were already unpowered and coasting. The switch-out did not change prop power.
- What the switch-out did change was the wing park:
  - Trials 1–4: Cut was cut short, wing_tilt kept the stale park setpoint for about 0.5 s and then released. The wing travelled 34–40° (−90° → −50…−60°). In trial 1 the encoder dropped, so it barely moved.
  - Trial 5: Cut ran to completion. The wing held still during an encoder dropout (+0.15…+1.2 s), then was driven at saturated command from −86° to 0° between +1.2 and +3.0 s. That is 91° of travel, with the hull already upright and the props pointing roughly down.
- Hypothesis (needs the operator's account of when the strike happened): the trial-5 strike happened during the late park swing (+1.2…+3 s), not during the slam at +0.2 s. The first 1.2 s of trial 5 involved less wing motion than trials 1–4.

## Appendix — other activations in the folder (not in the ledger)

| Log | Outcome |
|---|---|
| 15_39_56 | ABORT_VERIFY_FAIL (Verify → Disarm) |
| 15_43_31, 15_44_45, 15_48_24, 15_49_19, 15_53_37 | ABORT_TILT_TIMEOUT (TMO 5 s, later 10 s; TOL 0.05, then 0.10) |
| 15_52_00, 15_54_31, 15_55_36 | Reached Righting → ABORT_RIGHTING_TIMEOUT after 5 s |

All of these show θ ≈ 175°, `battery_status.current_a` = 0 and a flat 23.9 V, so the motors look unpowered (bench or dry
run?). Some starting wing readings were odd (−118°, −278°).
Afternoon logs with no activation (18_27–18_46, 18_59–19_09) contain console `tilt` tests. 18_39 set `COM_FLTMODE6 = 17`
(Self Right on slot 6). 18_46_53 was armed upright, drew 32 A and was ended by the kill switch.

## Follow-up change (2026-10-04)

The operator reports the trial-5 prop strike happened during the slam, with the props still spinning. The motor command
was already an instant step to 0 at θ ≈ 77–79°. To give the props more time to spin down, the motors now cut earlier in
the rotation:

- `SR_OVERCTR` default changed 1.4 → **1.833 rad (105°)**, in `src/modules/self_right/module.yaml`. The operator's
  reasoning is that the stronger props now give the hull enough momentum.
- **If the next test fails to get over center** (it falls back inverted after the cut, or hits `ABORT_RIGHTING_TIMEOUT`),
  this is the knob: **lower** `SR_OVERCTR` toward 1.4 to keep thrust on longer. 1.4 rad completed all five trials here.
  Note that the 2026-07-24 pool flip that only reached about 107° fell back. That was before the prop upgrade, and 105° is close to it.
