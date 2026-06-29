# Acconeer XM125 Radar Rangefinder — Code Change Log

Inventory of the code that adds the **Acconeer XM125 60 GHz pulsed-coherent radar** as a
downward-facing distance sensor, intended as an altimeter to **aid the landing controller**,
including **landing on water**, where the reading is only trusted within **7 m of the surface**.
This is the *what-changed* reference0; design rationale is inlined below since the
feature is a single self-contained driver.

The XM125 is an Acconeer A121 radar paired with an on-board STM32 running Acconeer's **I2C Distance
Detector** application. It exposes a register-map interface over I2C (default address **0x52**), so the
driver is a register read/write device — it does not process raw radar sweeps. The module's firmware
runs the detector and returns up to 10 ranked peaks per measurement; the driver selects one and
publishes it on the standard `distance_sensor` uORB topic.

The feature has three parts: (A) the **driver**, (B) **device-id / startup / board wiring**, and
(C) an **fmu-v5 flash-budget adjustment** required to fit it.

---

## A. Driver: `src/drivers/distance_sensor/acconeer_xm125/` (all new)

| File | Purpose |
|---|---|
| `acconeer_xm125.hpp` / `acconeer_xm125.cpp` | `device::I2C` + `I2CSPIDriver<>` + `ModuleParams` work-item driver. Drives a **configure → measure → collect** state machine (non-blocking; calibration is polled across cycles), reads the closest/strongest peak, and publishes via `PX4Rangefinder` as a **RADAR** (`MAV_DISTANCE_SENSOR_RADAR`) sensor. Standard `start`/`stop`/`status` verbs and `-R` rotation arg via `BusCLIArguments`/`BusInstanceIterator`. |
| `parameters.c` | Declares the `SENS_EN_XM125` enable flag and the `XM125_*` configuration parameters (below). |
| `CMakeLists.txt` | `px4_add_module(MODULE drivers__distance_sensor__acconeer_xm125 …)`, depends on `drivers_rangefinder` + `px4_work_queue`. |
| `Kconfig` | `menuconfig DRIVERS_DISTANCE_SENSOR_ACCONEER_XM125` (default n). |

### Parameters
| Param | Default | Purpose |
|---|---|---|
| `SENS_EN_XM125` | 0 | Start the driver at boot (read by `rc.sensors`; reboot required). |
| `XM125_MIN_DIST` | 0.30 m | Near-range blanking. Sets the detector `START` range and the reported `min_distance`. Reflections closer than this (the fuselage wall/radome the sensor sits behind, plus the radar's ~6 cm direct-leakage zone) are **not reported as peaks**. Set just beyond the mounting standoff. |
| `XM125_MAX_DIST` | 7.0 m | Sets the detector `END` range and the reported `max_distance` (the trust ceiling). |
| `XM125_REFL_SHP` | 2 (Planar) | Detector reflector shape: Planar weights large flat specular surfaces (water) by amplitude·range; Generic (1) weights by amplitude·range². |
| `XM125_STR_MIN` | 0 | Minimum raw peak strength to count as valid. 0 accepts any peak that passes the on-sensor CFAR threshold; raise to suppress weak/spurious returns. |

### I2C register protocol (Distance Detector application)
- 16-bit register address (big-endian, 2 bytes) + 32-bit register data (big-endian, 4 bytes).
  A read writes the 2 address bytes then does a repeated-start read of 4 data bytes
  (`transfer(addr,2,rx,4)`); a write sends 2 address + 4 data bytes in one transfer.
- Registers used: `0x0000` VERSION (probe), `0x0003` DETECTOR_STATUS (bit31 `BUSY`,
  `CONFIG_APPLY_OK`, `DETECTOR_CALIBRATE_OK`, error bits), `0x0010` RESULT
  (`num_distances`, `near_start_edge`, `measure_distance_error`), `0x0011…` PEAK0..9 distance (mm),
  `0x001b…` PEAK0..9 strength, `0x0040` START, `0x0041` END, `0x0046` THRESHOLD_METHOD,
  `0x0047` PEAK_SORTING, `0x004b` REFLECTOR_SHAPE, `0x0100` COMMAND.
- COMMAND values: `APPLY_CONFIG_AND_CALIBRATE` (1), `MEASURE_DISTANCE` (2).

### Notable design points
- **Detector configuration for radar-over-water**: `PEAK_SORTING = STRONGEST` (the near-nadir
  specular water return dominates), `THRESHOLD_METHOD = CFAR` (adapts to the changing noise floor with
  altitude/sea state), `REFLECTOR_SHAPE` from `XM125_REFL_SHP` (Planar by default). `START`/`END` come
  from `XM125_MIN_DIST`/`XM125_MAX_DIST`. Close-range leakage cancellation is left at the module default
  (off) since `START` already blanks the near zone — saves measurement time/power.
- **Fuselage rejection**: because the sensor is mounted inside the fuselage, "closest peak" would be the
  fixed fuselage-wall reflection. The `START` blanking window excludes it at the source, so the module
  never reports it.
- **Peak selection vs. gating split**: the `distance_sensor` topic carries a single distance, so the
  driver does **candidate selection only** — it publishes PEAK0 with a binary, strength-gated
  `signal_quality` (`XM125_STR_MIN` → quality 100, else 0; EKF2 only distinguishes 0 vs >0). When the
  detector reports no peak or a measure error (e.g. specular dropout off-nadir), it publishes quality 0
  so EKF2's quality hysteresis blocks fusion and holds the last terrain estimate.
- **Gating is delegated to EKF2** (deliberately not duplicated in the driver). EKF2's range-finder
  pipeline already provides the outlier/consistency rejection this use-case needs — see operator notes
  below.
- **State machine**: `init()` does `I2C::init()` (probe on VERSION) then schedules. `Configure` writes
  the config registers + `APPLY_CONFIG_AND_CALIBRATE`; `WaitConfig` polls `DETECTOR_STATUS` until
  calibration completes (or retries on error); `Measure` issues `MEASURE_DISTANCE`; `Collect` polls
  `BUSY`, then reads RESULT + PEAK0 and publishes. Runs ~10 Hz; `status` prints the range window,
  reflector shape, last distance, peak count, and the near-start-edge flag.

---

## B. Device id, startup, and board wiring

| File | Change |
|---|---|
| `src/drivers/drv_sensor.h` | Added the distance-sensor device-type id `DRV_DIST_DEVTYPE_XM125 0xE1`. |
| `ROMFS/px4fmu_common/init.d/rc.sensors` | Start the driver when enabled: `if param compare -s SENS_EN_XM125 1 then acconeer_xm125 start -X fi` (external I2C bus), alongside the other I2C rangefinders. |
| `boards/px4/fmu-v6x/default.px4board` | Enable `CONFIG_DRIVERS_DISTANCE_SENSOR_ACCONEER_XM125=y`. |
| `boards/px4/fmu-v5/default.px4board` | Enable the driver (see part C for the accompanying flash-budget changes). |

**SITL is intentionally not enabled.** PX4's `device::I2C` base class is compiled only under
`#if defined(CONFIG_I2C)`, which the POSIX/SITL target does not define — an I2C driver cannot be built
for SITL at all (this is a build-time limitation, not just "no live data"). The driver is therefore a
hardware-only (NuttX) feature.

---

## C. fmu-v5 flash-budget adjustment

fmu-v5 (STM32F765, 2016 KB `FLASH_AXIM`) was already at/over 100% on this branch from the other added
modules — it overflowed by ~1.8 KB even **without** this driver, and by ~4.9 KB with it. To fit the
XM125 (and restore a building fmu-v5), two driver families were trimmed:

| File | Change |
|---|---|
| `boards/px4/fmu-v5/default.px4board` | Replaced `CONFIG_COMMON_DISTANCE_SENSOR=y` with `CONFIG_DRIVERS_DISTANCE_SENSOR_ACCONEER_XM125=y` — keeps the XM125, drops the 8 other rangefinder drivers (vl53l0x, vl53l1x, lightware_laser_i2c, lightware_laser_serial, ll40ls, tf02pro, tfmini, ulanding_radar). |
| `boards/px4/fmu-v5/default.px4board` | Removed `CONFIG_COMMON_OPTICAL_FLOW=y` (paa3905, paw3902, pmw3901, px4flow, thoneflow). |

These only remove which *other* sensor models the firmware can talk to; they have no effect on the
XM125 path (the shared `distance_sensor` topic, `PX4Rangefinder`, and EKF2 fusion are unchanged). Any
specific dropped driver can be cheaply re-added per vehicle (e.g.
`CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C=y`) now that there is headroom. fmu-v6x (2 MB flash)
was untouched. fmu-v5 builds at **98.4%** flash after the change.

---

## Command/data flow (summary)

```
Acconeer XM125 (I2C 0x52, Distance Detector app)
  configure: START=XM125_MIN_DIST, END=XM125_MAX_DIST, PEAK_SORTING=STRONGEST,
             THRESHOLD=CFAR, REFLECTOR_SHAPE=XM125_REFL_SHP, then APPLY_CONFIG_AND_CALIBRATE
  per cycle: MEASURE_DISTANCE ─► poll BUSY ─► read RESULT (+ PEAK0 distance/strength)
                                                   │
        no peak / measure error ──► publish quality 0 (hold last terrain)
        valid peak ──► distance = PEAK0 mm/1000, quality = (strength ≥ XM125_STR_MIN ? 100 : 0)
                                                   ▼
        PX4Rangefinder.update() ──► distance_sensor (type=RADAR, downward-facing)
                                                   ▼
   EKF2 range-finder fusion: innovation gate (EKF2_RNG_GATE) + kinematic-consistency check
   vs. vz (EKF2_RNG_K_GATE) + quality hysteresis (EKF2_RNG_QLTY_T) + range aid (EKF2_RNG_A_*)
                                                   ▼
        terrain / height estimate ──► vehicle_local_position.dist_bottom (used by landing)
```

## EKF2 operator tuning notes (parameters, not code)
To use the XM125 as a height/landing aid, set on the vehicle:
- `EKF2_RNG_CTRL` to enable range-finder height fusion (sensor must be downward-facing).
- `EKF2_RNG_PITCH` = the sensor's pitch offset from nadir for the fuselage mount.
- `EKF2_RNG_A_HMAX` ≈ 7 (default 5) if range should act as the primary height source up to the 7 m
  ceiling; `EKF2_RNG_A_VMAX` to taste for approach speed.
- `EKF2_RNG_GATE` / `EKF2_RNG_K_GATE` / `EKF2_RNG_QLTY_T` defaults are a good start; the kinematic
  K-gate is what rejects multipath ghosts and specular-dropout jumps.

## Build status
- **fmu-v6x**: builds and links clean (firmware produced).
- **fmu-v5**: builds and links clean after the part-C trim — `FLASH_AXIM` at 98.4%.
- **SITL**: green (driver not built there — I2C unavailable on POSIX, see part B).

## Bench / hardware verification
Wire the XM125 to an external I2C bus, set `SENS_EN_XM125=1`, reboot. Then:
- `acconeer_xm125 status` → running, sample/comm-error perf counters, range window, last distance,
  peak count, near-start-edge flag.
- `listener distance_sensor` → `type` = 3 (RADAR), `max_distance` = 7.0; move a target between ~0.3 m
  and 7 m and confirm `current_distance` follows in metres; confirm `signal_quality` drops to 0 when
  the target is removed / out of range or, in the mount, that the fuselage wall is not reported.

## Relationship to other features
Independent driver; shares only the generic PX4 rangefinder infrastructure
(`PX4Rangefinder`, `distance_sensor`, EKF2 range fusion). Runs on the **twin_tractor** airframe
([jsbsim_sitl.md](jsbsim_sitl.md)) and is intended to support its landing controller.
