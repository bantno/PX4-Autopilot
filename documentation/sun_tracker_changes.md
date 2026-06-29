# Solar Tracking (sun_tracker) — Code Change Log

Inventory of the code that implements the **sun-tracking tilt-wing controller** and its supporting
hardware driver. For the design/algorithm rationale see
[sun_tracker_architecture.md](sun_tracker_architecture.md); this file is the *what-changed*
reference (paralleling [self_right_changes.md](self_right_changes.md)).

Delivered across two commits:
- `d706ff4d17` — *Add sun-tracking tilt-wing module (sun_tracker + AS5600 encoder driver)* (initial)
- `6f1ecafec3` — *sun_tracker: status logging, heading-variance gate, tilt reversal, FMU7* (refinements)

The feature has three parts: (A) the **control module**, (B) the **AS5600 encoder driver** + its
message, and (C) **logging, board, and airframe** wiring. The SITL tilt-wing plant
(`wing_tilt_sim`, commit `908f6c2cf1`) is documented in [jsbsim_sitl.md](jsbsim_sitl.md) since it is
shared with the self-righting work.

---

## A. Control module: `src/modules/sun_tracker/` (all new)

| File | Purpose |
|---|---|
| `SunTracker.hpp` / `SunTracker.cpp` | `ModuleBase` + `ModuleParams` + `ScheduledWorkItem` running at 20 Hz. Computes the sun direction from GPS position + UTC time, rotates it into the body frame using the vehicle attitude + north-referenced heading, derives the wing tilt angle that points the panel normal at the sun, and closes a **position PID on the AS5600 encoder feedback** (`sensor_encoder`). Output is a normalized command sent to a dedicated reversible ESC via `vehicle_command`/`DO_SET_ACTUATOR` (Peripheral_via_Actuator_Set1). Gates on attitude/position/time/encoder validity and on heading quality. |
| `solar_position.hpp` / `solar_position.cpp` | Self-contained solar-position algorithm: UTC microseconds + lat/lon → sun azimuth/elevation (NED). No external deps; used by the tracker each cycle. |
| `module.yaml` | Declares the `SUN_*` parameters (below). |
| `CMakeLists.txt` | `px4_add_module(MODULE modules__sun_tracker …)`, depends on `px4_work_queue` + `mathlib`. |
| `Kconfig` | `menuconfig MODULES_SUN_TRACKER` (default n). |

### Key parameters (`SUN_*`)
| Group | Params |
|---|---|
| Enable | `SUN_TRK_EN` (reboot) |
| Tilt position PID | `SUN_KP`, `SUN_KI`, `SUN_KD`, `SUN_DEADBAND` |
| Geometry / limits | `SUN_AXIS` (BodyY pitch / BodyX roll), `SUN_TILT_OFF` (trim), `SUN_TILT_MIN/MAX`, `SUN_PARK` (sun-down angle), `SUN_TILT_REV` (mounting-direction flip), `SUN_GEAR_RATIO` (encoder→wing de-gearing) |
| Heading gate | `SUN_HDG_VAR_MAX` (max `vehicle_local_position.heading_var` accepted — proxy for "yaw aligned to an absolute reference") |
| Debug (bench, remove before flight) | `SUN_DBG_SWP`, `SUN_DBG_AMP` (sine sweep that bypasses the GPS/sun gate to exercise the encoder + ESC) |

### Notable design points (added in the refinement commit)
- **Heading-variance gate** (`SUN_HDG_VAR_MAX`) instead of `heading_good_for_control`: the latter only
  asserts after an in-flight mag alignment, so it never becomes true for a stationary tracker. A
  bounded `heading_var` doubles as a "yaw is aided by mag/GPS-yaw" check.
- **Tilt reversal** (`SUN_TILT_REV`) reconciles the geometry convention (positive tilt = panel normal
  toward +body-X) with the physical wing/encoder mounting, without touching the encoder/ESC loop sign.
- **Encoder de-gearing** (`SUN_GEAR_RATIO`): the AS5600 may be geared faster than the wing shaft;
  the controller divides measured encoder angle by the ratio so setpoint/limits/deadband stay in real
  wing angle.
- **UTC fallback**: prefers GPS time, falls back to the system realtime clock (plausibility-checked)
  so SITL without a fix still tracks.
- **Command rate-limiting**: `DO_SET_ACTUATOR` is only emitted on meaningful change + a slow
  heartbeat, keeping the rate of commander-ACK'd commands low.

---

## B. AS5600 encoder driver + message

| File | Change |
|---|---|
| `src/drivers/encoder/as5600/AS5600.{hpp,cpp}` | **New** I2C driver for the AMS AS5600 12-bit magnetic rotary encoder (fixed addr 0x36). Latches a boot-time "wing level" zero on first valid magnet reading; publishes wrapped angle relative to that zero, raw count, and validity flags. Has a `reset` verb to re-latch the level reference at runtime. |
| `src/drivers/encoder/as5600/CMakeLists.txt`, `Kconfig` | Build + `menuconfig DRIVERS_ENCODER_AS5600`. |
| `src/drivers/encoder/Kconfig` | Source the new `as5600` Kconfig under the encoder driver group. |
| `src/drivers/drv_sensor.h` | Added the AS5600 device-type id. |
| `msg/SensorEncoder.msg` | **New** topic: `angle` [rad] (relative to boot zero, wrapped), `raw_count`, `valid`, `zeroed`. |
| `msg/SunTrackerStatus.msg` | **New** telemetry: sun az/el, heading + heading_var, tilt setpoint/measured/error/output, and the gating flags (`enabled`, `debug_sweep`, `pose_valid`, `heading_valid`, `encoder_valid`). |
| `msg/CMakeLists.txt` | Registered both new messages. |

---

## C. Logging, boards, airframes

| File | Change |
|---|---|
| `src/modules/logger/logged_topics.cpp` | Log `sun_tracker_status` so tracking behaviour is captured in ULogs. |
| `boards/px4/{fmu-v5,fmu-v6x,sitl}/default.px4board` | Enable `CONFIG_MODULES_SUN_TRACKER=y` (and the AS5600 encoder driver on hardware boards). |
| `ROMFS/.../init.d-posix/airframes/4244_jsbsim_twin_tractor` | SITL airframe: map the tilt ESC to ch7 via `PWM_MAIN_FUNC7=301` (Peripheral_via_Actuator_Set1), `SUN_TRK_EN 0`, start `as5600` (no-op in SITL) + `wing_tilt_sim` + `sun_tracker`. |
| `ROMFS/.../init.d/airframes/4244_twin_tractor` | **New** hardware airframe (added in the refinement commit): tilt ESC on FMU aux ch7 (`PWM_AUX_FUNC7=301`, neutral disarmed/failsafe), `SUN_TRK_EN 0`, start `as5600 -X -b 4` (external I2C bus 4) + `sun_tracker`. |
| `ROMFS/.../init.d/airframes/CMakeLists.txt` | Register the new hardware airframe id. |

---

## Command/data flow (summary)

```
GPS pos + UTC ──► solar_position() ──► sun az/el (NED)
vehicle_attitude (q) + heading ──► sun vector in body frame ──► desired wing tilt [rad]
                                              │
                         (clamp to SUN_TILT_MIN/MAX, gate on heading_var)
                                              ▼
   position PID  vs  sensor_encoder.angle / SUN_GEAR_RATIO  ──► normalized ESC cmd [-1,1]
                                              ▼
        vehicle_command DO_SET_ACTUATOR (param1, param7=0)  ──► Peripheral_via_Actuator_Set1
                                              ▼
                 reversible ESC ──► tilt motor ──► AS5600 encoder ──► sensor_encoder (loop closed)
```

In SITL `wing_tilt_sim` replaces the ESC+motor+AS5600 with a plant model that integrates the command
into an angle and republishes `sensor_encoder`.

## Build status
Builds clean in SITL and on fmu-v5/fmu-v6x; `sun_tracker_status` and `sensor_encoder` are logged.

## Relationship to other features
- Shares the tilt-wing actuator (Peripheral_via_Actuator_Set1) and AS5600 encoder with the
  **self-righting mode** ([self_right_changes.md](self_right_changes.md) /
  [self_right_architecture.md](self_right_architecture.md)).
- Runs on the **twin_tractor** airframe simulated in JSBSim ([jsbsim_sitl.md](jsbsim_sitl.md)).
