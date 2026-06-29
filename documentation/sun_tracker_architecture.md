# Sun-Tracking Tilt-Wing System — Architecture & Implementation

## The problem

The fixed-wing (twin-tractor) has a wing that can tilt, driven by a brushless motor on a
reversible ESC, with the wing's mechanical angle measured by an AS5600 magnetic encoder. The
goal: keep the wing's solar surface pointed at the sun during flight. That requires three things
the autopilot already half-has — *where is the sun* (needs GPS position + UTC time), *how is the
aircraft oriented* (attitude estimate), and *where is the wing right now* (the encoder) — fused
into a closed-loop command to one extra actuator, **without disturbing the existing
differential-thrust flight control allocation**.

## High-level architecture

Three decoupled pieces connected only by uORB topics, rather than one monolith. The seam choices
are the interesting part:

```
  vehicle_attitude ──────────┐
  vehicle_global_position ───┤
  sensor_gps (UTC) ──────────┤
                             ▼
  AS5600 ─► as5600 drv ─► sensor_encoder ─► sun_tracker ─► vehicle_command
   (I2C)    (hardware)      (new msg)       (controller)   (DO_SET_ACTUATOR)
                                                                  │
                            [SITL only: wing_tilt_sim ◄───────────┤
                             models plant, republishes            │
                             sensor_encoder]                      ▼
                                              mixer FunctionActuatorSet
                                              ─► PWM ch7 (FUNC=301) ─► ESC
```

Why split it this way:
- **The encoder is a generic sensor, not part of the controller.** A standalone I2C driver
  publishing a small generic topic. Independently testable (`listener sensor_encoder`, rotate the
  wing by hand) and reusable for anything else that wants a shaft angle.
- **The controller never touches the flight allocation.** It emits to a *peripheral* actuator
  path, so the differential-thrust allocator owns motors/servos and the sun-tracker owns exactly
  one extra output. No shared mutable state.
- **The plant model is swappable.** In sim, a fake plant publishes the same `sensor_encoder`
  topic the real driver would, so the controller is byte-for-byte identical in SITL and on
  hardware — only the encoder *source* changes.

## Component 1 — `sensor_encoder` uORB message

A new, intentionally generic message (`msg/SensorEncoder.msg`):

```
uint64 timestamp
uint32 device_id
float32 angle      # tilt [rad] relative to boot zero, wrapped to [-PI, PI)
uint16 raw_count   # raw 0..4095, for diagnostics
bool valid         # magnet present AND zero latched
bool zeroed        # boot reference captured
```

Chose a new message over reusing `wheel_encoders` (a 2-element rover odometry topic) because the
semantics differ and overloading invites unit/index confusion. The `valid`/`zeroed` split lets
the controller gate cleanly: `valid` is the single boolean the consumer checks, while `zeroed` is
diagnostic insight into *why* it might be invalid.

## Component 2 — `as5600` I2C driver

`src/drivers/encoder/as5600/`, built on PX4's modern `I2CSPIDriver<>` base (same pattern as the
VL53L1X rangefinder). Key points:

- **Bus plumbing.** Inherits `device::I2C` + `I2CSPIDriver<AS5600>`. The base handles bus
  enumeration, probing, and work-queue scheduling; the driver provides `probe()`, `RunImpl()`,
  and register reads. The AS5600 uses **8-bit register addressing** (unlike the VL53L1X's
  16-bit), so the read helper writes a single pointer byte then reads, relying on the chip's
  auto-increment to grab the high/low angle bytes in one transfer.
- **No WHO_AM_I.** The AS5600 has no chip-ID register, so `probe()` validates presence by a
  successful STATUS (0x0B) read rather than matching an ID.
- **The boot-zero assumption.** The AS5600 is single-turn absolute but its zero is mechanically
  arbitrary. Instead of requiring calibration, the driver assumes **the wing is level with the
  fuselage longitudinal (body-X) axis at startup = tilt 0**. On the first reading with the magnet
  detected, it latches `_zero_count = raw_count`. Every subsequent sample is reported relative to
  that: `angle = wrap_pi((count - _zero_count) * 2*PI/4096)`, giving a signed angle in [-PI, PI).
  Until that latch happens it publishes `valid=false`, keeping the controller safely gated. An
  `as5600 reset` runtime verb (dispatched through the I2CSPI `custom_method` mechanism, setting a
  `volatile bool _zero_request`) re-latches on demand — e.g. after re-seating the wing.
- **Rate.** 100 Hz (`ScheduleOnInterval(10ms)`) — plenty for the slow tilt dynamics and light on
  the bus.
- New device type `DRV_ENC_DEVTYPE_AS5600` registered in `drv_sensor.h`.

## Component 3 — `sun_tracker` controller

`src/modules/sun_tracker/`, a `ModuleBase` + `ModuleParams` + `ScheduledWorkItem` running a fixed
**20 Hz** loop (`ScheduleOnInterval(50ms)`). Fixed-rate rather than attitude-triggered because
the dynamics are slow and a steady cadence makes the PID dt well-behaved. Each cycle:

**1. Resolve UTC.** Prefers `sensor_gps.time_utc_usec`; if absent (0 in several sim GPS paths),
falls back to the **system realtime clock** (`px4_clock_gettime(CLOCK_REALTIME)`), but only if it
reads as plausibly current (epoch > 2020) so a default 1970 clock can't drive a garbage sun
position. This makes SITL usable *and* hardens the real vehicle (PX4 sets the system clock from
GPS, so it's a valid secondary source).

**2. Compute the sun direction.** A self-contained ephemeris (`solar_position.cpp`) — there was no
astronomy code anywhere in PX4. Standard NOAA/Almanac low-precision algorithm: UTC → days since
J2000 → solar mean longitude/anomaly → ecliptic longitude → right ascension/declination →
sidereal time → hour angle → topocentric azimuth (clockwise from North) and elevation. Validated
against first-principles geometry — e.g. solstice-noon at the equator gives elevation
66.56 deg = 90 - 23.44 exactly; Greenwich winter solstice 15.08 deg; azimuths point due south at
local noon.

Subtle but important: the algorithm needs **double precision only for the Julian-day reduction**
(the day count is ~1e4 and gets multiplied by per-day rates ~360, so the fractional degree after
the mod-360 reduction would be destroyed by float). Everything downstream — all the trig on
bounded angles — runs in **single precision** (`sinf`/`cosf`/`atan2f`). This matters because
fmu-v5/v6x have a single-precision FPU and double trig is software-emulated and bulky. The float
version produces identical results to ~0.1 deg.

**3. Rotate into the body frame.** Convert sun az/el to a NED unit vector, then
`s_body = Dcm(q)^T * s_ned` — the attitude quaternion's DCM maps body->NED, so its transpose maps
NED->body.

**4. Derive the desired tilt.** The wing rotates about one body axis (configurable, `SUN_AXIS`:
body-Y/pitch default, or body-X/roll). At tilt 0 the panel normal points "up" (body -Z); the tilt
that aims the normal at the sun is the bearing of the sun's projection onto the rotation plane:
`theta = atan2(s_body_x, -s_body_z)`. Because the *driver* already reports angle in the boot-zero
frame, `theta=0` means "panel level," and `SUN_TILT_OFF` is only a small residual mechanical trim
— it does **not** carry the encoder zero. The setpoint is constrained to
`[SUN_TILT_MIN, SUN_TILT_MAX]`. If the sun is below the horizon, it parks at `SUN_PARK`.

**5. Close the loop.** A PID on the wrapped error `wrap_pi(setpoint - measured)`, with a deadband
(`SUN_DEADBAND`) to stop ESC dither on target, integral anti-windup (clamped so the integral term
alone can't saturate), and output clamped to `u in [-1, 1]`. Because the ESC is reversible,
negative `u` actively drives the wing back — it holds position against wind/gravity in both
directions, which is why the design needs a reversible ESC rather than a one-way drive.

## The output path — and the one real tradeoff

The controller drives the ESC by publishing `vehicle_command` with
`VEHICLE_CMD_DO_SET_ACTUATOR` (param1 = `u`, param7 = 0 -> "Actuator Set 1"). PX4's mixer has a
`FunctionActuatorSet` provider that consumes exactly this and exposes it as the
`Peripheral_via_Actuator_Set1` output function (enum 301). A physical pin is mapped to it with
`PWM_MAIN_FUNCx = 301`. **Zero changes to control allocation** — the whole point.

The cost: **commander ACKs every `DO_SET_ACTUATOR`**. At 20 Hz that's command-ack traffic. So
`publishActuator()` only re-emits when `u` changes by >0.005 since the last publish, plus a 200 ms
heartbeat — steady-state spam drops to ~5 Hz while still feeding the actuator. The alternative (a
dedicated output function backed by its own topic, like the gimbal) is cleaner for continuous
control but touches shared `mixer_module` code; the decoupled path was chosen knowing this
tradeoff.

**Parameters** (`module.yaml`): `SUN_TRK_EN`, `SUN_KP/KI/KD`, `SUN_TILT_OFF`,
`SUN_TILT_MIN/MAX`, `SUN_DEADBAND`, `SUN_PARK`, `SUN_AXIS`.

## Build integration

- Enabled on **fmu-v6x** and **fmu-v5** (`default.px4board`). fmu-v5 was a problem: its default
  firmware is essentially flash-full upstream, and the new code (~15 KB) overflowed by exactly
  that. The single-precision trig wasn't the culprit (double libm was already linked elsewhere —
  it saved only 240 bytes). To fit, we disabled the two **UUV (underwater vehicle) controllers** —
  dead weight on an airplane — and the **gimbal** module, landing at 99.40%. fmu-v6x has ample
  room.
- **SITL gets the controller but not the driver**, because I2C device drivers don't compile for
  posix (no `CONFIG_I2C` / no `device::I2C`). A structural fact, not a config choice — which is
  exactly why the sim plant exists.
- The 4244 jsbsim airframe wires `PWM_MAIN_FUNC7 = 301`, `SUN_TRK_EN = 0` (opt-in), and autostarts
  the modules. `as5600 start` is a harmless no-op in SITL; `wing_tilt_sim start` is a no-op on
  hardware — symmetric.

## The test harness — `wing_tilt_sim`

A SITL-only "digital twin" of the actuator (`src/modules/simulation/wing_tilt_sim/`). It
subscribes to the *same* `vehicle_command`/`DO_SET_ACTUATOR` the real ESC would receive, models
the tilt motor as a **rate source** (`angle += MAX_RATE * u * dt`, clamped to mechanical stops —
`u=0` holds, matching a non-backdrivable drive), and republishes `sensor_encoder` at 100 Hz with
`valid/zeroed` set. This closes the entire `controller -> command -> plant -> encoder ->
controller` loop in pure software, so gains can be tuned and convergence watched before risking
hardware. The controller can't tell the difference — it just sees `sensor_encoder`.

`sun_tracker status` was also enriched to print live sun az/el, setpoint, measured angle, error,
and output — which is how an "el -64.8 deg" reading is immediately diagnosable as simply nighttime
at the sim location (parked, working correctly).

## Code state

- **Committed and pushed** (branch `sun-tracker`, commit `d706ff4d`): the controller, driver,
  message, device-id, board enables, and airframe wiring — 19 files, ~1,134 lines.
- **Uncommitted** (left in the working tree): the `wing_tilt_sim` plant, the UTC fallback, and the
  rich status. Both rounds build green on SITL and fmu-v6x.
- Pre-existing in-progress work (the `ActuatorEffectivenessFixedWingDiffThrust` edits, the airframe
  FW tuning, `CLAUDE.md`) was kept untouched — the commit staged only the sun-tracker's own
  airframe hunk so the uncommitted tuning stayed out of it.

## Hardware validation checklist (only HW can prove the real loop)

1. **AS5600 bring-up:** hold wing level, `as5600 start -X`, `as5600 status` (zeroed: yes),
   `listener sensor_encoder` — `angle ~0` at level; tilt by hand, confirm sign and [-PI, PI)
   wrapping; test `as5600 reset`.
2. **Output/ESC config:** `PWM_MAIN_FUNC7=301`, set `PWM_MAIN_MIN7/MAX7/DIS7` so the reversible
   ESC reads center = neutral and +/-1 = full forward/reverse; put the ESC in 3D/bidirectional
   mode.
3. **Bench closed loop (props off, armed — outputs only go live when armed):** `SUN_TRK_EN=1`,
   step `SUN_TILT_OFF` (daytime) or `SUN_PARK` (any time), confirm the wing drives to target and
   holds against a hand disturbance; tune `SUN_KP/KI/KD`.
4. **Flight:** confirm commanded tilt tracks the computed sun direction as heading/attitude change.
