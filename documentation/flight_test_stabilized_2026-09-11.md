# AHAB Stabilized-Mode Flight Test — 2026-09-11, Cobb County RC Club

**Goal:** fly the AHAB twin-tractor in PX4 **Stabilized** mode and come home with a log that shows
whether the roll/pitch attitude loops track. Every earlier Stabilized attempt (flights 229, 239)
flew on wrong gains or a reversed pitch servo; this is the first flight with both fixed.

**Crew:** RC pilot on the transmitter. GCS operator (you) runs QGroundControl, the checklists,
the video camera, and calls the test points. Nobody flies until both of you have finished
Section 5.

**Rule of the day:** any sign of instability in Stabilized — oscillation, growing bank, nose
dropping, anything that does not look like a gentle correction — the pilot flips straight back
to **Manual**. Do not try to ride it out. Do not change gains in the air.

---

## 1. Flash the flight controller (before the plane leaves the bench)

The firmware for this test lives on branch **`flight-test-2026-09-11`** of
`https://github.com/bantno/PX4-Autopilot`. It carries the corrected gains, the RC-loss
failsafe below, and the airframe config. Flash it from Brian's laptop (toolchain installed) or
any machine with the PX4 toolchain.

**Option A: build and flash from the terminal**

```
cd ~/PX4/PX4-Autopilot                      # or a fresh: git clone --recursive https://github.com/bantno/PX4-Autopilot.git
git fetch origin
git checkout flight-test-2026-09-11
git submodule update --init --recursive
git rev-parse --short HEAD                  # write this hash down; you check it on the plane below
make px4_fmu-v6x_default upload             # close QGC first; plug the FC USB in when it says "waiting for bootloader"
```

`make ... upload` builds (a few minutes the first time), then waits for the flight controller to
appear on USB, flashes, verifies, and reboots it. QGC must be closed while it runs; QGC holding
the USB port makes the upload wait forever.

**Option B: flash the prebuilt file from QGC** (no terminal, needs the file from a machine that
built the branch): QGC → Vehicle Setup → Firmware → plug in USB → tick *Advanced settings* →
*Custom firmware file* → choose `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`.

**Then verify on the plane** (QGC → Analyze Tools → MAVLink Console):

```
ver all                     # "PX4 git-hash" must start with the hash you wrote down above
param show FW_RR_P          # 0.0500   (old bad value was 0.015)
param show FW_PR_FF         # 0.4000   (old bad value was 0.005)
param show NAV_RCL_ACT      # 5        (RC loss -> motors off + spiral)
param show COM_RC_LOSS_T    # 5.0
param show PWM_AUX_FAIL3    # 1460
param show PWM_AUX_FAIL5    # 1650
param show PWM_AUX_REV      # 40
param show SYS_AUTOSTART    # 4244
```

If `NAV_RCL_ACT`, `COM_RC_LOSS_T` or the `PWM_AUX_FAIL*` values did not take (they were user-set
on the board, and a user-set value beats the airframe default), set them by hand:

```
param set NAV_RCL_ACT 5
param set COM_RC_LOSS_T 5
param set PWM_AUX_FAIL1 1000
param set PWM_AUX_FAIL2 1000
param set PWM_AUX_FAIL3 1460
param set PWM_AUX_FAIL4 1460
param set PWM_AUX_FAIL5 1650
param set PWM_AUX_FAIL6 1350
param save
```

Flashing keeps the saved parameters (calibrations, RC setup, output map). It does not wipe them.

**Logging settings for a tuning flight** (set once, on the ground, same console):

```
param set SDLOG_PROFILE 2139    # was 2123: adds the "High rate" set -> attitude, setpoints, servo
                                # commands and stick input at full rate instead of 10-20 Hz
param set SDLOG_MODE 1          # log from boot, so the props-off Stabilized direction check is in a log
param save
```

With the old profile the log had the rate loop at 400 Hz but attitude and its setpoint at 20 Hz
and the servo outputs at 10 Hz, which is too coarse to judge the attitude loop. Logging from boot
makes one log per power-up; fine on the card, and you get the ground checks recorded.

## 2. Packing list

Spares are already packed; this is the verify-it-is-in-the-car list.

**Aircraft**
- [ ] AHAB airframe, wing, tail, all fasteners; wing tilt hardware is **not** installed (correct)
- [ ] 10-inch Master Airscrew props, one per motor, plus spares; prop tool
- [ ] Launch cart, painter's tape or masking tape for taping the plane to the cart
- [ ] Pitot tube cover (remove before flight!) and a spare
- [ ] Servo, ESC, motor spares (packed)

**Power**
- [ ] 4 × 6S 5500 mAh packs, all charged to ≥ 4.1 V/cell (≥ 24.6 V pack) before the first flight
- [ ] Cell checker / voltmeter
- [ ] Field charger + supply if you intend to recharge

**Control & ground station**
- [ ] RadioMaster GX12, charged. Model **"AHAB FC"** (name is close to that; it sits just above the
      currently selected model in the model list). Select it and confirm the model name on the
      screen before powering the aircraft.
- [ ] Laptop with QGroundControl, charged, plus charger
- [ ] 915 MHz telemetry radio (ground end) + its USB cable
- [ ] USB-C cable that reaches the flight controller (data cable, not charge-only)
- [ ] microSD card **in the flight controller** (arming is refused without one) + spare card
- [ ] Phone or camera on a tripod to video every flight, especially the Stabilized engagement

**Field**
- [ ] AMA card, FAA registration, and whatever the club requires at the flight line
- [ ] Hex drivers, small screwdrivers, zip ties, tape, CA glue, multimeter
- [ ] Notebook / this document printed
- [ ] Fire extinguisher or LiPo bag, first aid

## 3. Aircraft configuration (as it is on the plane)

### 3.1 Flight controller connections

Flight controller: Holybro Pixhawk 6X (fmu-v6x), airframe `4244_twin_tractor`.

| Port on FC | What plugs in |
|---|---|
| POWER1 | Power module from the flight battery (feeds FC and the 5 V rail) |
| FMU PWM OUT ("AUX") pins 1–6 | Motors and servos, see table below. Signal + ground only; servo power is from the rail |
| I/O PWM OUT ("MAIN") | **Nothing.** Leave empty |
| GPS1 | GPS / compass / safety-switch / buzzer module |
| TELEM1 | 915 MHz telemetry radio |
| I2C | MS4525DO pitot airspeed sensor |
| RC IN | ELRS receiver |
| USB-C | Laptop, pre-flight only |
| microSD slot | Log card, required to arm |

### 3.2 Servo and motor channels (FMU PWM OUT rail)

| AUX pin | Function | Notes |
|---|---|---|
| 1 | Motor 2 (right) | ESC signal. Range 1000–1900 µs (the 1900 is the 90 % cap) |
| 2 | Motor 1 (left) | ESC signal. Range 1000–1900 µs |
| 3 | Left aileron (servo 1) | |
| 4 | Right aileron (servo 2) | Output **reversed** in firmware |
| 5 | Ruddervator (servo 3) | Pitch + yaw |
| 6 | Ruddervator (servo 4) | Pitch + yaw, output **reversed** in firmware |
| 7 | Wing-tilt ESC | Not installed today; pin stays empty |

`PWM_AUX_REV = 40` is what reverses pins 4 and 6. It was changed from 24 before flight 239 and
that fixed the pitch runaway of flight 229. **Do not change it.**

**10-inch Master Airscrew props: throttle is capped at 90 %.** That cap is the 1900 µs
`PWM_AUX_MAX1/2` on the motor outputs. Do not raise it and do not extend the throttle endpoint
on the transmitter. Full stick on the GX12 = 1900 µs = 90 %.

### 3.3 Radio channels (GX12 model "AHAB FC")

| Channel | Function | What to verify on the QGC **Radio** page |
|---|---|---|
| 1 | Roll | Bar moves the right way, centers near 1500 |
| 2 | Pitch | Same |
| 3 | Throttle | Low = 1000, high = 2000 |
| 4 | Yaw | Same as roll |
| 5 | Flight mode | Low position = **Manual**, high = **Stabilized**. Middle is unassigned |
| 7 | Arm switch | One end arms; find which |
| 8 | Kill switch | One end kills the motors instantly; find which |

Nobody remembers which physical switch is on which channel. **Verify all of them** in Section 5
before the plane leaves the table. The flight-mode assignment is on the QGC **Flight Modes**
page; it will show you live which position gives which mode.

Transmitter trims are **not** zeroed on this model. Leave them alone (zeroing them without a
recalibration broke the model config last time). Consequence: in Stabilized a centered stick is
not exactly "wings level, nose level"; see 6.2.

### 3.4 Key parameters

| Parameter | Value | Meaning |
|---|---|---|
| FW_RR_P / FW_RR_I / FW_RR_FF | 0.05 / 0.10 / 0.50 | Roll rate loop (PX4 defaults, FF from flight 214) |
| FW_PR_P / FW_PR_I / FW_PR_FF | 0.08 / 0.10 / 0.40 | Pitch rate loop |
| FW_YR_P / FW_YR_I / FW_YR_FF | 0.05 / 0.10 / 0.30 | Yaw rate loop (untested) |
| FW_MAN_R_MAX / FW_MAN_P_MAX | 45° / 30° | Full stick in Stabilized = this bank / pitch |
| FW_R_RMAX / FW_P_RMAX_* | 56 / 60 °/s | Rate limits |
| FW_AIRSPD_TRIM / MIN / MAX | 22 / 13 / 35 m/s | Airspeed envelope used for gain scaling |
| FW_AIRSPD_STALL | 7 m/s | Placeholder; gain scaling clamps here. See 6.3 |
| TRIM_ROLL / TRIM_PITCH | 0 / 0.025 | Firmware trims. TRIM_ROLL stays 0 |
| PWM_AUX_MAX1/2 | 1900 | The 90 % throttle cap |
| NAV_RCL_ACT / COM_RC_LOSS_T | 5 / 5 s | RC loss: motors off, surfaces to spiral |
| BAT1_N_CELLS | 6 | |
| COM_ARM_WO_GPS | 1 | GPS lock not required to arm |
| COM_PREARM_MODE | 2 | Servos move while disarmed (that is how you do the ground checks) |

## 4. Failsafe behaviour (know this before arming)

**Kill switch (ch 8):** motors stop immediately, surfaces go to neutral (1500 µs). In the air
this is a dead-stick with neutral tail, and the plane is nose-heavy, so it will nose down. Use
it on the ground or if a prop is about to hurt someone, not as a "get out of Stabilized" button.

**Getting out of Stabilized:** the **mode switch to Manual**. That is the abort. Practice the
motion on the ground.

**RC loss (receiver loses the transmitter for 5 s):** the FC enters *Terminate*: motors to
zero, ailerons to a slight right bank, ruddervators to about the flight-239 level trim plus a
little nose-up. Intent: a descending right-hand spiral instead of a straight glide off the field.
It never clears: once triggered, the flight is over even if the link returns. This was set from
log data and **has never been flown**; the surface directions are checked in 5.6.

**Low battery:** warning only, no automatic action. You watch the voltage.

**Datalink loss (telemetry radio):** no action. The plane keeps flying on RC.

## 5. Procedure

Work through it in order. Tick every box. If a check fails, stop and fix it or do not fly.

### 5.1 Arrival, aircraft assembly (props OFF)

- [ ] Assemble wing and tail. Check every control linkage, clevis, and servo screw.
- [ ] Check CG: the plane balances at approximately the **front spar clamp**. Do this with the
      battery in its flight position. Nose-heavy is the known tendency; do not fly tail-heavy.
- [ ] Pitot cover ON for now (dust). Do not blow into the pitot.
- [ ] Props stay off until 5.7.

### 5.2 Power up and connect

- [ ] Transmitter first. Select model **"AHAB FC"**, confirm the name on screen, all switches to
      the safe/off end, throttle at zero.
- [ ] Plug in a battery. Check it reads ≥ 24.6 V (4.1 V/cell) on the checker first.
- [ ] Plug the ground telemetry radio into the laptop, start QGC, wait for the vehicle to connect.
      If QGC will not connect over the radio, use the USB cable for the ground checks.
- [ ] QGC status bar: it should say **Disarmed**, mode **Manual**, GPS acquiring. Ignore
      "Operation timeout" messages; they are a transfer retry, not a fault.
- [ ] MAVLink Console (Analyze Tools → MAVLink Console), run and write down:
  ```
  ver all
  param show FW_RR_P
  param show NAV_RCL_ACT
  ```
  `FW_RR_P` must be `0.0500`. If it reads `0.0150` the aircraft is on old firmware; **do not fly
  Stabilized**, call Brian.

### 5.3 Radio checks (QGC → Vehicle Setup → Radio)

- [ ] Move each stick; the roll, pitch, yaw, throttle bars follow, correct sense (roll stick
      right → roll bar right; pitch stick forward → pitch bar down is PX4 convention).
- [ ] Sticks released: roll and pitch bars sit near center. Note how far off they are; more than
      about 10 % means the TX trims are large — tell Brian, do not zero them yourself.
- [ ] Flip every switch and watch which channel bar moves. Write down: mode switch = ___,
      arm switch = ___, kill switch = ___, which end is which.
- [ ] Flight Modes page: mode switch low reads **Manual**, high reads **Stabilized**. Middle
      does nothing (keeps the current mode); the pilot should treat the switch as two-position.
- [ ] Walk the transmitter 50 m away, confirm no "RC lost" message and RSSI stays healthy.

### 5.4 Surface checks in Manual (still props OFF, disarmed)

Servos move while disarmed on this aircraft. Stand clear of the surfaces.

- [ ] Mode switch to **Manual**.
- [ ] Roll stick right: left aileron trailing edge **down**, right aileron **up**.
- [ ] Pitch stick back (nose up): both ruddervator trailing edges **up**.
- [ ] Yaw stick right: ruddervators split, the pair deflects to yaw the nose right (left
      trailing edge up, right trailing edge down, as seen from behind).
- [ ] Surfaces centered with sticks centered, within a few degrees. Note any offset.
- [ ] Throttle stick up: nothing spins (disarmed). Good.

### 5.5 Stabilization direction check (props OFF) — the most important check of the day

Do this **before** the first flight. The point is to prove the attitude loop pushes the surfaces
the right way. A reversed axis in Stabilized is a crash.

- [ ] Mode switch to **Stabilized**. QGC shows "Stabilized".
- [ ] Sticks centered. Hold the plane level. Surfaces near neutral.
- [ ] **Roll**: bank the plane right wing down ~30°. The FC must command roll-left: **right
      aileron trailing edge goes down, left aileron goes up.** Bank left: opposite. The
      deflection should be obvious (tens of percent), not a twitch.
- [ ] **Pitch**: raise the nose ~20°. FC must command nose-down: **both ruddervator trailing
      edges go down.** Lower the nose: both go up.
- [ ] **Yaw**: rotate the nose right (flat yaw). FC commands yaw-left: ruddervators split the
      opposite way to the Manual yaw-right check. This one is weakest and has never been flown;
      if it looks wrong, note it and tell Brian, but it is not a no-go on its own — roll and pitch
      are.
- [ ] Put the plane down level. Roll stick right: plane wants to bank right, so the surfaces
      command roll-right (same directions as Manual roll stick right) and hold there.
- [ ] Back to **Manual**. Surfaces follow sticks again.

If roll or pitch moves the wrong way in Stabilized while Manual is correct: **do not fly.**
The fix is `PWM_AUX_REV` (or a control-allocation sign), and that is a phone call to Brian, not
a field guess.

If the surfaces barely move in Stabilized (a few percent of travel while Manual gives full
travel): wrong gains / old firmware. Do not fly. Recheck 5.2.

### 5.6 RC-loss failsafe check (props OFF, ARMED)

- [ ] Everybody clear of the props' arc, even with props off. Throttle zero.
- [ ] Arm with the arm switch. Confirm "Armed" in QGC.
- [ ] Throttle up slightly: motors spin. Throttle back to zero.
- [ ] Switch the transmitter **off**.
- [ ] Within ~5 s: QGC shows a failsafe/termination message, motors stay off even if you had
      throttle in, ailerons deflect for a **right** bank (right aileron up, left down, small),
      ruddervators move **up** noticeably (nose-up trim).
- [ ] Switch the transmitter on. The state does not clear (by design). Disarm with the switch
      if it lets you; otherwise unplug the battery.
- [ ] Power-cycle the aircraft before continuing.

### 5.7 Props on, final checks

- [ ] Props on, correct rotation per side, tight. Rotation check: with props on, arm, throttle
      to about 10 % for two seconds, confirm thrust blows backwards on both motors, throttle zero.
- [ ] Pitot cover OFF. In the MAVLink Console: `listener airspeed_validated` — airspeed reads
      about 0 to 2 m/s sitting still. If it reads NaN or a large number, do not fly Stabilized
      (gain scaling uses it).
- [ ] Battery ≥ 24.6 V. Telemetry connected. GPS lock is not required for this test but is nice.
- [ ] Video camera rolling, framed on the runway and the sky above it.
- [ ] Pilot brief (5.8) done out loud.

### 5.8 Pilot brief (say it out loud, both of you)

1. Takeoff in **Manual**. Climb in Manual to at least **100 m** above the field. Fly a couple of
   laps in Manual to get the trim feel. The plane is nose-heavy: expect to hold some up.
2. Straight and level, wings level, upwind leg, at cruise throttle (~45–60 %), call "engaging".
   Switch to **Stabilized** and centre the sticks.
3. Watch for 3 seconds: the plane should hold roughly level and fly straight. Small bank or
   pitch offsets from the TX trims are expected. Oscillation, a bank that keeps growing, or the
   nose dropping steadily is **not** — switch to Manual immediately.
4. If it holds: gentle roll inputs, ~half stick, left and right. Then gentle pitch. Then a
   coordinated turn with roll stick only, around 30° bank.
5. Back to **Manual** for the landing approach. Land in Manual.
6. Never below **60 m** in Stabilized on this flight.
7. The kill switch is for the ground. The abort is the mode switch.

### 5.9 Cart launch

- [ ] Runway clear well past the expected liftoff point; the cart free-rolls with no steering
      and continues after the plane leaves it. Somebody retrieves it after the plane is airborne
      and clear.
- [ ] Plane on the cart, lightly taped with two short strips that tear away. It must not lift
      the cart.
- [ ] Nose into wind. Pilot in Manual, arm, confirm "Armed".
- [ ] Smooth throttle to full over ~2 s (the 90 % cap is in the firmware; full stick is fine).
- [ ] Keep wings level with aileron, hold the nose straight with **rudder** (yaw stick) — this
      airframe has ground-looped on every landing so far and the cart has no steering.
- [ ] Rotate gently around 16 m/s; last time it lifted off after ~40 m / 5 s of ground roll.
- [ ] Climb straight ahead, no turns below 30 m.

### 5.10 Landing

- Manual mode. Into wind. Slow, flat approach; touch down at the slowest comfortable speed.
- Expect a yaw swing at touchdown; the pilot should be on the rudder before it happens.
- After stopping: throttle zero, disarm with the switch, then kill switch on, then walk out.

### 5.11 Between flights

- [ ] Battery voltage under no load: swap the pack if any cell is below 3.7 V. Plan ~8 minute
      flights; flight 239 used about 670 mAh in 6 minutes, so a 5500 mAh pack has plenty of margin
      at that length, but do not stretch it.
- [ ] Motors and ESCs: touch test. Warm is fine, too-hot-to-hold means stop and cool.
- [ ] Check props tight, linkages, tape residue on the cart contact points.
- [ ] Pull the log (Section 7) **now**, not at the end of the day. Copy the `.ulg` off the SD
      card or download it in QGC (Analyze Tools → Log Download).
- [ ] Look at the roll/pitch tracking plots (Section 7). Decide with Brian whether to change a
      gain. One parameter per flight, on the ground, written down.

### 5.12 Abort / no-go list

- Wind or gusts that make the Manual laps uncomfortable → land, done for the day. Pilot's call.
- Any Stabilized instability → Manual immediately, land, review the log before trying again.
- Airspeed reads wrong on the ground → no Stabilized.
- Surface direction check fails → no flight.
- Firmware check fails (`FW_RR_P` 0.015) → no Stabilized.
- Battery below 4.1 V/cell before takeoff → swap.

## 6. Tuning the stabilization

### 6.1 What Stabilized does

In Stabilized the roll and pitch sticks command an **attitude** (bank angle up to 45°, pitch
up to ±30°), not a surface deflection. Two loops per axis:

- Attitude loop: `FW_R_TC` / `FW_P_TC` (time constants, 0.4 s default) turn attitude error into
  a rate setpoint. Leave these alone tomorrow.
- Rate loop: `FW_RR_P/I/FF` (roll) and `FW_PR_P/I/FF` (pitch) turn rate error into surface
  torque. This is what you tune.

Throttle is still manual. Yaw stick adds rudder on top of automatic turn coordination.

### 6.2 What to expect

- Sticks centered → about wings-level, nose-level. Because the TX trims are not zero, expect a
  small steady bank or pitch offset. A roll offset of 10 % stick is about 4.5° of bank.
- Release the roll stick from a turn → the plane rolls level in about a second.
- The plane should not "hunt": a slow wallow of ±5° is a tuning issue (see table), a fast
  buzz of the surfaces is too much P, a divergence is a reversed axis (should have been caught
  on the ground).

### 6.3 Airspeed matters

The rate gains are scaled by airspeed: deflection multiplies by (22 / airspeed)². At 15 m/s
that is 2.2× the cruise deflection, at 13 m/s 2.9×. `FW_AIRSPD_STALL` is 7 m/s, which lets
the scaling reach nearly 10× at very low speed. Consequence: **fly Stabilized at cruise speed
(18–24 m/s), not slow.** If the plane looks twitchy only when slow and fine at cruise, that is
the airspeed scaling, not the gains; tell Brian, the fix is raising `FW_AIRSPD_STALL` toward
the real stall speed (roughly 10–12 m/s), not lowering P.

### 6.4 Gain changes: only on the ground, one at a time, after looking at the log

| Symptom in the log / in the air | Change | Range |
|---|---|---|
| Rate follows setpoint slowly, lazy response, no oscillation | Raise FF | FW_RR_FF 0.5 → up to 0.7; FW_PR_FF 0.4 → up to 0.6 |
| Fast oscillation (several Hz), surfaces buzzing | Lower P by 25 % | FW_RR_P 0.05 → 0.04; FW_PR_P 0.08 → 0.06 |
| Slow wallow, overshoots and comes back | Lower P a little, or lower I | FW_RR_I / FW_PR_I 0.1 → 0.05 |
| Holds a steady offset from the setpoint | Raise I a little | 0.1 → 0.15 |
| Everything fine but soft | Raise P by 25 % | max 0.12 on either axis |

Current values and the sane range: **P 0.03–0.12, I 0.05–0.15, FF 0.3–0.7.** Outside that,
stop and think. Change one number per flight, note it on paper with the flight number, and
only after looking at the plots in Section 7. Do it with `param set FW_RR_P 0.04` in the
MAVLink Console or on the QGC Parameters page; it takes effect immediately and is saved.

Do not touch `PWM_AUX_REV`, `CA_*`, `TRIM_ROLL` (stays 0), `FW_MAN_*`, or the airspeed
parameters at the field.

## 7. Reading the log

The flight controller logs to the SD card from arming to disarm. File names look like
`log_241_2026-9-11-14-32-10.ulg`. Get them off the card or via QGC Log Download.

### 7.1 Quick look: the flight report tool

On the laptop, in the PX4 repo:
```
python3 Tools/flight_report.py path/to/log_241_....ulg
```
It writes a `figures/` folder next to the log with attitude, rates, actuator, airspeed, power
and event plots, and auto-detects arm / throttle-up / liftoff / landing. Open the attitude and
rate figures first.

### 7.2 PlotJuggler: what to plot

Open the `.ulg` in PlotJuggler (it has a ULog loader). Put these on the same time axis:

| Question | Topics / fields |
|---|---|
| When was Stabilized on? | `vehicle_status/nav_state` (0 = Manual, 8 = Stabilized) |
| Does roll track? | `vehicle_attitude_setpoint/roll_body` vs roll from `vehicle_attitude/q` (PlotJuggler converts quaternion → Euler; or use `vehicle_attitude_setpoint/roll_body` vs `vehicle_local_position`… simplest: the report tool plots it) |
| Does pitch track? | `vehicle_attitude_setpoint/pitch_body` vs pitch from `vehicle_attitude/q` |
| Rate loop | `vehicle_rates_setpoint/roll` vs `vehicle_angular_velocity/xyz[0]`; same with `pitch` vs `xyz[1]` |
| What the controller asked for | `vehicle_torque_setpoint/xyz[0..2]` (roll, pitch, yaw torque, ±1) |
| What the servos got | `actuator_servos/control[0..3]` (±1, full rate with the High-rate profile) or `actuator_outputs` instance 1 (the AUX rail), `output[2..5]` in µs at 10 Hz |
| Pilot input | `manual_control_setpoint/roll`, `pitch`, `throttle` (throttle is −1…+1, mid = 0) |
| Airspeed | `airspeed_validated/calibrated_airspeed_m_s` |
| Integrator wind-up | `rate_ctrl_status/rollspeed_integ`, `pitchspeed_integ` |
| Battery | `battery_status/voltage_v`, `current_a`, `discharged_mah` |

### 7.3 How to judge it

- Good: measured roll rate follows `vehicle_rates_setpoint/roll` with a small lag
  (~0.1 s) and settles without overshoot. Attitude reaches the setpoint in about a second.
- Too little FF: rate lags the setpoint and the torque command builds slowly.
- Too much P: torque and rate oscillate against each other at several Hz.
- Too much I / too little damping: slow overshoot that takes seconds to settle; the integrator
  trace ramps up and down.
- Reversed axis (should never reach the air): torque goes one way and the rate goes the other,
  growing. Flight 229 looked exactly like that on pitch.

Compare against flight 239 if you want a "before" picture: its Stabilized segment shows rate
setpoints pinned at the limit and torque only 5–9 % because of the old gains.

## 8. Common problems

| Problem | Check |
|---|---|
| Will not arm | QGC message tells you why. Usual ones: no SD card, kill switch engaged, throttle not at zero, RC not detected, sensors uncalibrated. GPS lock is **not** required. |
| Arms, motors do not spin | Kill switch. Throttle stick direction. ESCs beeping = no signal: check the AUX1/2 leads. |
| One motor spins, other does not | Swap the ESC signal leads between AUX1/AUX2 to see if the problem follows the lead or the motor. |
| Surfaces do nothing in Manual | Mode really Manual? Radio page shows channels moving? Servo power (rail) present, 5 V at a servo lead? |
| Surfaces move in Manual but barely in Stabilized | Old firmware / gains. `param show FW_RR_P` must be 0.05. |
| Stabilized pushes a surface the wrong way | `PWM_AUX_REV` must be 40. Do not fly. Call Brian. |
| Mode switch does nothing | You are in the middle position (unassigned), or the switch is on another channel. Flight Modes page shows it live. |
| "Manual control lost" on the ground | Receiver not bound / wrong model selected on the GX12 / antenna. Model must be "AHAB FC". |
| Airspeed NaN or wild | Pitot cover, I2C lead loose, `SENS_EN_MS4525DO` must be 1. `listener airspeed_validated`. Blow gently past (not into) the tip: it should rise. |
| Telemetry will not connect | Ground radio plugged in before QGC started? Right COM port, 57600 baud. Fall back to USB. Only one program may hold the port. |
| QGC spams "Operation timeout" | Transfer retry noise, ignore unless the vehicle keeps dropping. |
| Compass / mag warnings | Not needed for Stabilized (no heading control). Note it, fly. |
| Plane yaws hard on landing rollout | Known: four ground loops so far. Rudder on rollout, land slow and straight. |
| Plane wants to bank / pitch with sticks centered in Stabilized | TX trim offset becoming an attitude command. Small is fine. Large: note it for Brian; do not zero trims at the field. |
| Stabilized twitchy only at low speed | Airspeed gain scaling (6.3). Fly faster; tell Brian. |
| Motor or ESC too hot to hold after a flight | Stop. Check the 90 % cap is still 1900 on `PWM_AUX_MAX1/2` and that the props are the 10-inch ones. |

## 9. What changed since the last flight (for context)

- Firmware now carries the corrected fixed-wing rate gains (PX4 defaults + measured FF). Flights
  229 and 239 flew on gains ~20× too low; that is why Stabilized "did nothing" in 239.
- Pitch servo polarity was fixed before flight 239 (`PWM_AUX_REV` 24 → 40) and verified in that
  log. Yaw polarity follows from the same change and is checked on the ground in 5.5.
- RC-loss behaviour changed from Disarm to Terminate-with-spiral (Section 4).
- Airspeed sensor (MS4525DO) has flown twice and is healthy.
- Wing tilt, solar tracker, self-right and the camera are all off or absent for this test.

## 10. After the day

- Copy every `.ulg` and the video into the flight-test folder, named by flight number.
- Write down per flight: battery start/end voltage, duration, what was tested, what the pilot
  felt, any parameter changed.
- Send Brian the logs and this sheet.
