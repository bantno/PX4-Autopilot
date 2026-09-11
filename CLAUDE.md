# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

PX4 is an OS-independent flight control stack (NuttX, Linux, macOS) for drones, planes, VTOL, rovers, etc. Flight logic lives in `src/modules`; reusable algorithms in `src/lib`; hardware drivers in `src/drivers`; OS shims in `platforms`.

## Build & run

Builds go through `make`, which is a thin wrapper that drives CMake/Ninja per **board config**. A target is `<vendor>_<board>_<config>` derived from `boards/*/*.px4board`; the `_default` suffix may be omitted.

```bash
make px4_sitl_default          # build SITL (host simulation)
make px4_fmu-v6x_default       # build NuttX firmware for a flight controller
make px4_fmu-v6x_default upload   # build + flash over USB
make list_config_targets       # list all board targets
make clean                     # clean all build dirs
```

Build artifacts land in `build/<target>/`. CMake is reconfigured automatically when needed; don't hand-edit `build/`.

### Running SITL (simulation)

The simulator is selected as the make target's second word; the airframe via `PX4_SIM_MODEL` (set by the startup script chosen with the airframe id).

```bash
make px4_sitl jmavsim                              # default quad in jMAVSim
make px4_sitl gz                                   # Gazebo (gz)
make px4_sitl jsbsim                               # JSBSim (used for the fixed-wing/twin_tractor work)
HEADLESS=1 make px4_sitl jsbsim                    # no GUI
PX4_SIM_MODEL=jsbsim_twin_tractor make px4_sitl jsbsim
```

Airframe startup scripts: `ROMFS/px4fmu_common/init.d-posix/airframes/` (SITL) and `.../init.d/airframes/` (hardware). The numeric prefix is the airframe id. JSBSim aircraft/bridge live under `Tools/simulation/jsbsim/jsbsim_bridge/`.

## Test, format, lint

```bash
make tests                                  # full SITL unit/functional suite (build/px4_sitl_test)
make tests TESTFILTER=AttitudeControl       # run a subset by ctest name filter
make px4_sitl_test                           # configure the test build, then:
cd build/px4_sitl_test && ctest -R <name> -V # run a single test directly

make format          # apply astyle (the canonical formatter — config in Tools/astyle/)
make check_format    # verify formatting (CI gate)
make clang-tidy      # static analysis (configures a clang build first)
```

Style is enforced by **astyle**, not clang-format. C/C++ uses **tabs (width 8)**, max line length **120**, `insert_final_newline`. Standard is **C++14 / C11**. Always run `make format` before committing C/C++.

## Architecture

**uORB is the backbone.** Modules don't call each other directly — they communicate by publishing/subscribing to uORB topics (an async pub/sub message bus). A topic is defined by a `.msg` file in `msg/`; the build generates C++ structs from these. To trace data flow, follow the topic name, not function calls. `make uorb_graphs` renders the dependency graph.

- **`src/modules/`** — the running tasks/work-items: `commander` (arming/state machine), `ekf2` (state estimation), `navigator`, `mc_*`/`fw_*`/`vtol_*` (multicopter/fixed-wing/VTOL controllers), `control_allocator`, `sensors`, `mavlink`, `logger`, `simulation`. Each module is a self-contained app registered into the firmware.
- **`src/lib/`** — pure libraries reused across modules: `matrix` (linear algebra), `mathlib`, `geo`, `control_allocation`, `rate_control`, `pid`, `parameters`, `mixer_module`, etc. No uORB I/O here — keep libs side-effect-free and unit-testable.
- **`src/drivers/`** — sensor/peripheral drivers (IMU, baro, GPS, ESC/DShot, etc.), publishing into uORB.
- **`platforms/`** — OS abstraction: `nuttx`, `posix` (SITL/Linux), `qurt` (Qualcomm DSP), plus `common`. The same module code runs on all of them.
- **`boards/<vendor>/<board>/`** — per-board config: `*.px4board` (Kconfig selecting modules/drivers), pin/init, NuttX defconfig, bootloader.
- **`ROMFS/px4fmu_common/`** — shell startup scripts (`init.d`) that configure a vehicle at boot and select which modules start; airframe files map an airframe id to params + module setup.

### Module anatomy

A typical module dir contains: the `.cpp/.hpp` (often a class extending `ModuleBase` + `ScheduledWorkItem`), `CMakeLists.txt` (declares the module via `px4_add_module`), `Kconfig` (enable/disable per board), and **`module.yaml`** — declares parameters and the actuator/output config. Parameters are also defined inline via `PARAM_DEFINE_*` / `module.yaml` and accessed through generated `_param_*` accessors. After adding params or topics, a rebuild regenerates the bindings.

### Control allocation (relevant to this branch)

Fixed-wing/VTOL/multirotor output mixing flows: controller → `control_allocator` module → `src/lib/control_allocation/` (effectiveness matrix + allocation) → `actuator_outputs` → driver/sim. New airframe geometries are added as an `ActuatorEffectiveness*` class under `src/lib/control_allocation/actuator_effectiveness/` and wired through `module.yaml`. This branch (`twin-tractor`) adds differential-thrust fixed-wing allocation and JSBSim twin-tractor airframes.

## Conventions & gotchas

- Branch off `main` for new work; PRs target `main`.
- Submodules are required (`make` aborts without `.git`). Use `make submodulesupdate` to sync, `make distclean` for a hard reset.
- Generated code (uORB headers, params, mixers) lives only in `build/` — never commit it; change the `.msg`/`.yaml`/source instead.
- Don't add a module to a board by editing CMake alone — gate it through the board's `*.px4board`/Kconfig.
