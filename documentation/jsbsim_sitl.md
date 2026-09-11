# JSBSim SITL — twin_tractor Tilt-Wing Aircraft

How the **twin_tractor** fixed-wing tilt-wing aircraft is simulated in JSBSim, and the PX4-side SITL
changes that drive it. This is the *what-and-where* reference for the simulation; the flight-mode
features that run on top of it are documented in
[sun_tracker_changes.md](sun_tracker_changes.md) and [self_right_changes.md](self_right_changes.md).

Delivered across commits `fe79e293d5` (twin tractor airframes + differential-thrust allocation),
`8795f48d9f` (diff-thrust ground-yaw + twin_tractor tuning), and `908f6c2cf1` (`wing_tilt_sim`), plus
the JSBSim aircraft model/tooling that lives in the `Tools/simulation/jsbsim/jsbsim_bridge` submodule.

---

## 0. Quick start

```bash
make px4_sitl_default                 # build firmware
make px4_sitl jsbsim PX4_SIM_MODEL=jsbsim_twin_tractor    # or: make jsbsim_twin_tractor
HEADLESS=1 make px4_sitl jsbsim PX4_SIM_MODEL=jsbsim_twin_tractor   # no FlightGear visuals
```

`PX4_SIM_MODEL=jsbsim_twin_tractor` selects the airframe (SYS_AUTOSTART 4244) and the JSBSim model.

---

## 1. The aircraft model (JSBSim)

**`Tools/.../jsbsim_bridge/models/TwinTractor/twin_tractor.xml`** — the full flight-dynamics model.
Lines ~331–931 are **auto-generated** by the aero pipeline (§3); everything else is hand-maintained.

- **Mass & inertia (real 4 kg airframe):** empty 4.0 kg; Ixx 0.1012, Iyy 0.0521, Izz 0.073 kg·m²;
  no cross-products. **CG at 0.445 m from the nose datum (x = 17.52 in), with AERORP = CG** so the
  VSPAERO moment reference matches. *The CG is canonical — keep it fixed; relocate placeholder
  gear/engines to match it, never move the CG.*
- **Reference geometry (VSPAERO-normalized):** wing area 0.425 m², span 1.7 m, mean chord 0.25 m.
- **Aerodynamics:** VSPAERO-derived, generated from a `.stab` sweep. 2D lookup tables on
  (alpha, beta) for static (CL, CD, CS, Cl, Cm, Cn), rate damping (Clp/Clr/Cmq/Cnp/Cnr/CYp/CYr), and
  control derivatives (Clda/Cldr/Cmde/Cnda/Cndr/CLde/CYdr). Sign map is identity (VSPAERO body axes
  match JSBSim). `Cmadot` is a hand-set constant; no post-stall model (inviscid VLM has no stall).
  **Trusted envelope:** α −10..+20°, β ±20° — but the fine 2° grid only covers α 0..+20° (β mirrored
  from the positive side); α −10..0° comes from the coarse 5° grid and is edge-clamped for |β| > 10°.
  Tables clamp flat beyond the grid edges. *Note: an advanced aero + stall model from a separate repo
  is slated to replace this generated block (see the aero-swap boundary section in `tools/README.md`).*
- **Propulsion (real T-Motor Cine66 2812 925KV + 9×5×3 prop):** two tractor engines at y = ±0.25 m.
  Uses the **`brushless_dc_motor` (FGBrushLessDCMotor)** engine model — this replaced a simple
  `FGElectric` model whose power/omega torque spike caused a roll-divergence at throttle-up.
  `cine66_925kv.xml`: 22.2 V (6S nominal), KV 925, R 0.069 Ω, no-load 1.05 A → ~2.9 kgf static per
  motor. `9x5x3.xml`: 3-blade, APC 9×5E data ×1.16 for the third blade, calibrated to ~17.3 kRPM /
  ~2.9 kgf static. **Counter-rotating**: left prop sense +1, right −1 (cancels net torque).
- **Landing gear:** taildragger (two mains + tailwheel); currently a Rascal-frame placeholder
  relocated to the real frame after the nose-over fix — still approximate.
- **Flight control:** reads per-surface commands from the bridge
  (`fcs/left|right-aileron-cmd-norm`, `fcs/left|right-ruddervator-cmd-norm`), reconstructs virtual
  roll/pitch/yaw, and drives elevator/aileron/rudder deflections (V-tail ruddervators).

### Supporting model files (same `TwinTractor/` dir)
| File | Role |
|---|---|
| `Engines/cine66_925kv.xml` | Real T-Motor Cine66 2812 925KV brushless-DC engine, datasheet-calibrated. |
| `Engines/9x5x3.xml` | 9", 3-blade fixed-pitch prop; C_THRUST/C_POWER calibrated to the motor. |
| `TwinTractor-set.xml`, `Models/TwinTractor.xml` | FlightGear visual config + animations (surfaces, props). The mesh `Models/TwinTractor.ac` is exported from the OpenVSP AHAB project — see `Models/README.md` for the export checklist. |

---

## 2. The bridge config (PX4 ↔ JSBSim)

**`Tools/.../jsbsim_bridge/configs/twin_tractor.xml`** maps PX4 `HIL_ACTUATOR_CONTROLS` channels to
JSBSim FCS properties and defines the sensor set (IMU/GPS/baro/mag/airspeed).

| idx (chan−1) | PX4 output | JSBSim property |
|---|---|---|
| 0 | Motor LEFT | `fcs/throttle-cmd-norm[0]` |
| 1 | Motor RIGHT | `fcs/throttle-cmd-norm[1]` |
| 2 | Servo1 / CS0 | `fcs/left-aileron-cmd-norm` |
| 3 | Servo2 / CS1 | `fcs/right-aileron-cmd-norm` |
| 4 | Servo3 / CS2 | `fcs/left-ruddervator-cmd-norm` |
| 5 | Servo4 / CS3 | `fcs/right-ruddervator-cmd-norm` |

Per-channel `scale` signs can be flipped here if a surface/motor responds backwards.

---

## 3. The aero generation pipeline (VSPAERO → JSBSim)

The aerodynamics are produced from VSPAERO stability runs, not hand-tuned. Regenerate with one
command after a new VSPAERO run:

```bash
cd Tools/.../jsbsim_bridge/models/TwinTractor/tools
./stab_to_jsbsim.py --stab vspaero/AHAB_full_sweep.stab --mirror-beta --merge vspaero/AHAB.stab
```

That is the canonical invocation that produced the current aero block: fine positive-quadrant sweep,
β mirrored by lateral symmetry, negative-α rows grafted from the coarse run. Caveat: `--merge`
sources are *not* mirrored — they need their own negative-β coverage (the coarse run has it).

| File | Role |
|---|---|
| `tools/stab_to_jsbsim.py` | Pure-stdlib converter: parses a VSPAERO `.stab` (α/β grid of derivative tables) and emits the 2D JSBSim `<table>` XML in place (rewrites only the auto-gen block of `twin_tractor.xml`). Key options: `--mirror-beta` (synthesize −β from +β by lateral symmetry), `--merge` (graft extended-α rows from a finer secondary `.stab` onto the primary grid). Prints a cruise stability self-check. |
| `tools/refs.json` | Single source of truth: reference geometry, AERORP/CG, cruise condition, control-group→FCS map, sign map, rate convention, `Cmadot`, post-stall toggle. **Edit this to retune, not the converter.** |
| `tools/README.md` | Pipeline walkthrough + conventions. |
| `tools/vspaero/AHAB.stab` | Coarse 5×5 run, α,β ∈ ±10° (negative-α merge source). |
| `tools/vspaero/AHAB_full_sweep.stab` | Fine 11×11 run, α,β ∈ 0..+20° at 2° (positive quadrant only). |

Both `.stab` files are co-located with the converter so regeneration works in place; the copies in
the repo `documentation/` folder are archival. `vspaero_plan_v2.md` (the original VSPAERO port plan)
is superseded.

> To update aero: run a new VSPAERO sweep → drop the `.stab` in `tools/vspaero/` → re-run
> `stab_to_jsbsim.py` → rebuild. Mass stays 4 kg; the sign map is identity.

---

## 4. PX4 SITL airframe — `4244_jsbsim_twin_tractor`

**`ROMFS/px4fmu_common/init.d-posix/airframes/4244_jsbsim_twin_tractor`** configures the vehicle at
boot. Highlights:

- **Control allocation:** `CA_AIRFRAME=16` → the differential-thrust fixed-wing effectiveness (§6).
  `CA_ROTOR_COUNT=2` at PX 0.3 m / PY ±0.25 m; `CA_SV_CS_COUNT=4` (two ailerons CS0/CS1, two V-tail
  ruddervators CS2/CS3 with roll/pitch/yaw torque signs); `CA_DTHR_SC=1.0` (diff-thrust yaw scale).
- **Output mapping:** `PWM_MAIN_FUNC1/2 = 101/102` (motors L/R), `FUNC3..6 = 201..204` (CS0..CS3),
  `FUNC7 = 301` (Peripheral_via_Actuator_Set1 — the tilt-wing reversible ESC for sun_tracker /
  self_right).
- **Rate-loop tuning for 4 kg:** `FW_PR_P/FW_RR_P/FW_YR_P` scaled ~26–40× down from the Rascal
  placeholder to match the real inertia (intentionally sluggish; FW autotune from this baseline).
  Airspeed envelope set to the real glide speeds (`FW_AIRSPD_TRIM 22`, MIN 15, MAX 35).
- **Tilt-wing stack (opt-in):** starts `as5600` (HW driver, no-op in SITL), `wing_tilt_sim` (SITL
  plant, §7), and `sun_tracker`; plus the self-righting `self_right` module (`SR_*`, `COM_FLTMODE6`).
- **TEST-ONLY relaxed arming** for air-launch ICs (`NAV_DLL_ACT 0`, `NAV_RCL_ACT 0`,
  `COM_RC_IN_MODE 4`, `COM_ARM_WO_GPS 1`, `CBRK_AIRSPD_CHK`) lives in the
  `4244_jsbsim_twin_tractor.post` overlay (sourced only by the posix rcS, excluded from airframe
  matching) — the canonical airframe file stays clean; delete the `.post` for realistic arming.

Sibling Gazebo airframes added alongside: `4242_gz_twin_tractor_seaplane`, `4243_gz_twin_cessna`.
All are registered in `init.d-posix/airframes/CMakeLists.txt`.

---

## 5. SITL integration glue

| File | Role |
|---|---|
| `Tools/simulation/jsbsim/sitl_run.sh` | Launcher: sets `PX4_SIM_MODEL=jsbsim_<model>`, starts the JSBSim bridge (+ optional FlightGear visuals, suppressed by `HEADLESS=1`), runs PX4, cleans up on exit. twin_tractor auto-selects the `TwinTractor` FlightGear visual once `models/TwinTractor/Models/TwinTractor.ac` exists (export checklist in `Models/README.md`), falling back to the Rascal model until then. FlightGear from a snap VS Code terminal needs `unset GTK_PATH` (or a native terminal). |
| `src/modules/simulation/simulator_mavlink/sitl_targets_jsbsim.cmake` | Adds the `jsbsim_bridge` submodule as an ExternalProject; lists `twin_tractor` among the models; auto-matches `*_jsbsim_*` airframes; generates the `jsbsim_twin_tractor[ __WORLD]` make targets. |
| `Tools/simulation/jsbsim/jsbsim_bridge` | Submodule holding the bridge binary, the TwinTractor model, configs, and the aero tools. |

---

## 6. Differential-thrust fixed-wing allocation

`src/lib/.../actuator_effectiveness/ActuatorEffectivenessFixedWingDiffThrust.{hpp,cpp}` (selected by
`CA_AIRFRAME=16`, wired in `ControlAllocator.cpp` and `control_allocator/module.yaml`).

- Behaves like the standard fixed-wing effectiveness but **gates yaw-by-differential-thrust on the
  land detector**: `enabled = landed || gear_switch_on`. On the ground this gives steering authority
  for takeoff/taxi (a tail-dragger twin has no nosewheel); airborne it is off by default (rudder does
  yaw), unless the pilot forces it via the gear switch (`RC_MAP_GEAR_SW`).
- Rebuilds the effectiveness matrix on state change and scales the yaw column by `CA_DTHR_SC`.
- Subscribes to `manual_control_switches`, `vehicle_land_detected`, `flaps_setpoint`,
  `spoilers_setpoint`; delegates to the rotor + control-surface effectiveness helpers.

The `8795f48d9f` commit refined this (ground-yaw enable via land detector) together with the
twin_tractor airframe rate/airspeed defaults.

---

## 7. `wing_tilt_sim` — SITL tilt-wing plant

`src/modules/simulation/wing_tilt_sim/{WingTiltSim.hpp,WingTiltSim.cpp,CMakeLists.txt,Kconfig}`
(commit `908f6c2cf1`; enabled via `CONFIG_MODULES_SIMULATION_WING_TILT_SIM=y` on the sitl board).

- SITL-only stand-in for the tilt ESC + motor + AS5600 encoder (the real `as5600` I2C driver isn't
  built for SITL).
- Subscribes to the same `vehicle_command` `DO_SET_ACTUATOR` (Peripheral_via_Actuator_Set1, param1)
  that `sun_tracker` / `self_right` publish; models the actuator as a **rate source** (`|u|=1` →
  1.5 rad/s), integrates the wing angle at 100 Hz, clamps to `±TRAVEL_LIMIT_RAD`, and republishes
  `sensor_encoder` — closing the tilt control loop in simulation.
- `TRAVEL_LIMIT_RAD` was widened **1.5 → 1.8 rad** so the wing can reach the ~90° props-up
  self-righting setpoint (`SR_TILT_SP = 1.57`), not just the sun-tracking range.

---

## 8. Notable decisions / gotchas

- **Engine model matters:** the `brushless_dc_motor` model fixed an arm-time roll divergence that the
  generic `FGElectric` model caused (power/omega torque spike). Confirmed on the rascal model, then
  applied to twin.
- **CG is canonical (0.445 m):** keep it fixed; move placeholder gear/engines to match, never the CG.
- **Aero is regenerated, not edited:** treat `twin_tractor.xml`'s aero block as build output of
  `stab_to_jsbsim.py`; change `refs.json` / the `.stab` and re-run.
- **Valid envelope is bounded:** inviscid VLM aero → no stall; only trust the model inside the swept
  α/β range.
- **Relaxed arming params in the airframe are TEST-ONLY** (air-launch ICs) — strip before ground/real
  flights.
- **Shared tilt path:** `wing_tilt_sim` (SITL) ↔ `as5600` (hardware) publish the same
  `sensor_encoder`, so `sun_tracker` and `self_right` are identical across sim and hardware.
- **The TwinTractor model lives in the `jsbsim_bridge` submodule** on its local `ahab` branch
  (committed; the superproject pins that SHA). The pinned commit exists only locally until the
  submodule branch is pushed to a fork and `.gitmodules` updated — do that before sharing the
  superproject branch, and never run a forced submodule update/clean from an older superproject
  commit that predates the pin.

## Build status
`make px4_sitl_default` builds clean with the twin_tractor model, diff-thrust allocation, and
`wing_tilt_sim`. `make jsbsim_twin_tractor` launches the sim.
