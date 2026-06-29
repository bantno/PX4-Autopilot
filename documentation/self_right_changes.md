# Self-Righting Mode — Code Change Log

Complete inventory of the code changes that implement the autonomous self-righting flight mode, on
branch **`self-right-mode`** (branched off `sun-tracker`). For the *design* rationale see
[self_right_architecture.md](self_right_architecture.md); this file is the *what-changed* reference.

The change has three layers: (A) a new control **module**, (B) a new **gated flight mode** wired
through commander/MAVLink/RC, and (C) **config/tooling** to enable and tune it.

---

## A. New module: `src/modules/self_right/` (all new files)

| File | Purpose |
|---|---|
| `SelfRight.hpp` | Module class (`ModuleBase` + `ModuleParams` + `ScheduledWorkItem`). Declares the `State` enum (Idle/RotateWing/Righting/Cut/Done), all uORB subs/pubs, maneuver/PID/over-center/adaptive state, and the `DEFINE_PARAMETERS` block for the `SR_*` params. |
| `SelfRight.cpp` | The controller. Runs at 100 Hz; only acts while armed + `nav_state == NAVIGATION_STATE_SELF_RIGHT`. State machine: rotate wing to props-up (`SR_TILT_SP`) via an encoder position PID → run the `SR_STRATEGY` righting law → cut + park wing → request MANUAL. Includes over-center detection, open-loop and attitude-PID strategies, the adaptive thrust-learning step-up search, and aborts (stick / rangefinder / accel-clipping / timeout). Publishes `actuator_motors` (symmetric on control[0]/[1]), the tilt `DO_SET_ACTUATOR`, and `self_right_status`. |
| `module.yaml` | Declares the `SR_*` parameters (enable/strategy, gate threshold, tilt setpoints + PID, righting/cut, attitude-PID gains, adaptive layer, safety). |
| `CMakeLists.txt` | `px4_add_module(MODULE modules__self_right …)`, depends on `px4_work_queue` + `mathlib`. |
| `Kconfig` | `menuconfig MODULES_SELF_RIGHT` (default n). |

### New message
- **`msg/SelfRightStatus.msg`** (new) — telemetry: state, strategy, abort reason (with `STATE_*`,
  `STRATEGY_*`, `ABORT_*` constants), pitch-from-upright, pitch rate, tilt angle, throttle,
  `over_center`, `active`, learned thrust, attempt counter.
- **`msg/CMakeLists.txt`** — registered `SelfRightStatus.msg` (alphabetical, after `SatelliteInfo`).

---

## B. New gated flight mode `NAVIGATION_STATE_SELF_RIGHT`

The mode reuses the previously-unused **`NAVIGATION_STATE_FREE1` slot (value 16)** so
`NAVIGATION_STATE_MAX` stays 31 and no `static_assert`s need bumping. The group bit (`1u<<16` =
`65536`) and `navigation_mode_t` index `25` were free.

### Nav-state definition & generated enums
| File | Change |
|---|---|
| `msg/versioned/VehicleStatus.msg` | Renamed `NAVIGATION_STATE_FREE1 = 16` → `NAVIGATION_STATE_SELF_RIGHT = 16`. |
| `src/lib/events/enums.json` | Added `self_right` to **both** generated enums: `navigation_mode_group_t` (bit `65536`) and `navigation_mode_t` (index `25`). Required for the `NavModes` gating mask and for events/UI naming. |

### Commander: gate, control-mode, requirements
| File | Change |
|---|---|
| `src/modules/commander/HealthAndArmingChecks/checks/selfRightingCheck.{hpp,cpp}` | **New check.** The latched entry gate. "Inverted" = `Quaternion(att.q).dcm_z()(2) < -SR_INV_THR`; "at rest" = `vehicle_land_detected.at_rest && estimator_status_flags.cs_tilt_align`. Reads `SR_EN`/`SR_INV_THR` via `param_find_no_notification` (no build dependency on the module). `clearArmingBits(SelfRight)` unless entry_ok; `clearCanRunBits(SelfRight)` unless entry_ok **or already in the mode** (the latch — so a moving aircraft whose `at_rest` dropped is not ejected). |
| `HealthAndArmingChecks/HealthAndArmingChecks.hpp` | Include `selfRightingCheck.hpp`; add `_self_righting_checks` member; add it to the `_checks[]` run list. |
| `HealthAndArmingChecks/CMakeLists.txt` | Added `checks/selfRightingCheck.cpp` to the build. |
| `HealthAndArmingChecks/Common.hpp` | Added `NavModes::SelfRight = navigation_mode_group_t::self_right` so the check can address the mode's group bit. |
| `ModeUtil/mode_requirements.cpp` | `NAVIGATION_STATE_SELF_RIGHT` requires `mode_req_angular_velocity` + `mode_req_attitude` only — deliberately **no** position/global/home (untrustworthy inverted on water). |
| `ModeUtil/control_mode.cpp` | New `case NAVIGATION_STATE_SELF_RIGHT:` that leaves **all** control + allocation flags disabled. Because `control_allocator` only publishes `actuator_motors` when `flag_control_allocation_enabled` is set, this lets the module own the outputs with no allocator edit. |
| `ModeUtil/conversions.hpp` | Map the nav_state → `navigation_mode_t::self_right` (events). |

### UI / mode naming
| File | Change |
|---|---|
| `src/lib/modes/ui.hpp` | `nav_state_names[16]` "16: UNUSED2" → "Self Right"; added the bit to `getValidNavStates()`. (`isAdvanced()` default already returns true.) |

### Activation paths (RC + GCS)
| File | Change |
|---|---|
| `src/modules/commander/px4_custom_mode.h` | Added `PX4_CUSTOM_MAIN_MODE_SELF_RIGHT` enum value; mapped the nav_state in `get_px4_custom_mode()`. |
| `src/modules/commander/Commander.cpp` | In the `VEHICLE_CMD_DO_SET_MODE` handler, map `custom_main_mode == PX4_CUSTOM_MAIN_MODE_SELF_RIGHT` → the nav_state (GCS / MAVLink entry). This is also the command the module publishes to hand back to MANUAL on completion. |
| `src/modules/manual_control/ManualControl.cpp` | `navStateFromParam()` case `17` → `NAVIGATION_STATE_SELF_RIGHT` (RC flight-mode slot value). |
| `src/modules/commander/module.yaml` | Added `17: Self Right` to the `COM_FLTMODE*` enum metadata. |

---

## C. Simulation, boards, airframes, tooling

| File | Change |
|---|---|
| `src/modules/simulation/wing_tilt_sim/WingTiltSim.cpp` | Raised `TRAVEL_LIMIT_RAD` 1.5 → 1.8 so the SITL tilt plant can reach the ~90° props-up self-right setpoint (sun-tracking only needed ~±69°). |
| `boards/px4/{sitl,fmu-v5,fmu-v6x}/default.px4board` | `CONFIG_MODULES_SELF_RIGHT=y` on each (the boards that already run the tilt-wing stack). |
| `ROMFS/.../init.d-posix/airframes/4244_jsbsim_twin_tractor` | Start `self_right`; set `SR_EN 0`, `SR_TILT_SP 1.57`, `COM_FLTMODE6 17` (RC slot 6 selects the mode). |
| `ROMFS/.../init.d/airframes/4244_twin_tractor` | Same startup + defaults on the hardware airframe. |
| `Tools/self_right/extract_trajectory.py` | **New.** Reads a ULog of a manual/SITL righting; prints suggested `SR_THR_MAX`/`SR_RAMP_T`/`SR_OVERCTR`/`SR_TIMEOUT` seeds and writes a per-sample `thr(t)`/pitch CSV for the open-loop replay strategy. |

---

## Actuator-ownership note (why there is no `control_allocator` edit)

The original wiring plan considered suppressing `control_allocator`. It proved unnecessary:
`ControlAllocator::publish_actuator_controls()` early-returns unless
`_publish_controls == vehicle_control_mode.flag_control_allocation_enabled`. Since the SELF_RIGHT
control-mode leaves that flag false, the allocator never publishes `actuator_motors` in this mode and
the `self_right` module is the sole publisher. No race, no allocator change.

## Build status (at implementation time)
- `make px4_sitl_default` — clean (module lib built, `SR_*` params generated).
- `make px4_fmu-v6x_default` — clean (module in NuttX firmware).
- astyle clean on all new/changed files.

## Not yet done (follow-ups)
- **Parameter seeding**: `SR_*` defaults are placeholders — run `extract_trajectory.py` on a Phase-0
  manual-righting log to set `SR_OVERCTR`, `SR_THR_MAX`, `SR_RAMP_T`, `SR_TIMEOUT`, tilt PID.
- **Open-loop replay table**: `SR_STRATEGY=0` currently uses a ramp-hold (scaled by the adaptive
  factor); loading an arbitrary recorded `thr(t)` table from `extract_trajectory.py` output into the
  module is a future enhancement.
- **SITL/HW verification**: the gate (reject-upright / accept-inverted), latch, strategies, and
  aborts have not yet been exercised in simulation.
