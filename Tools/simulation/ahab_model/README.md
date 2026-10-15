# AHAB Gazebo model (`twin_tractor_seaplane`, airframe 4242)

```bash
make px4_sitl gz_twin_tractor_seaplane          # GUI
HEADLESS=1 make px4_sitl gz_twin_tractor_seaplane
```

PX4 finds this model through `PX4_GZ_MODELS_EXTRA` (`src/modules/simulation/gz_bridge/gz_env.sh.in`).
`px4-rc.gzsim` falls back to it when a model isn't in the upstream `Tools/simulation/gz` submodule.

## Layout

| Path | What |
|---|---|
| `twin_tractor_seaplane/model.sdf` | **Generated** by `tools/build_model.py`. Don't edit it; change the inputs and re-run. |
| `twin_tractor_seaplane/meshes/` | `hull.glb` / `wing.glb` meshed from the multibody STEP (`AHAB_FINAL_simv2.stp`, 2026-10-13) by `tools/step_to_glb.py`; aileron, ruddervator and prop meshes by Nick Trujillo (`ac86632ced` on `gazebo_sim`) |
| `mass_properties.yaml` | Measured mass/CG/inertia (procedure: `documentation/ahab_mass_properties_procedure.md`) |
| `rate_derivatives.yaml` | *(not yet created)* Rate derivatives to be identified from the flight logs; until then `stab_to_gz.py` uses flagged VSPAERO fallbacks |
| `collision_boxes.json` | Buoyancy/contact boxes and hydro surface panels, written by the STEP tools below |
| `tools/mass_properties.py` | Pendulum data → link inertials (`--selftest` for a known-answer check) |
| `tools/stab_to_gz.py` | VSPAERO `.stab` → AdvancedLiftDrag coefficients |
| `tools/build_model.py` | Writes `model.sdf` and `hydro_panels.txt` (`AHAB_HYDRO` / `AHAB_TILT_EFFORT` env overrides for sweeps) |
| `tools/float_to_model.py`, `tools/fin_to_model.py` | Wingtip floats / ventral fin from their STEP files |
| `tools/water_scenarios.py` | Water calibration scenarios (headless or `--gui`) |
| `tools/selfright_video.py` | Self-right demo recorded from the three cameras → `videos/*.webm` (local only, gitignored) |
| `tools/step_to_glb.py`, `tools/step_bodies.py` | STEP → `hull.glb` / `wing.glb` (needs `cadquery-ocp` in a venv; usage in the docstring) |
| `tools/step_to_collision.py` | STEP → volume-exact boxes (fuselage outer envelope, solid wing/V-tail) + hull/wing surface panels |
| `worlds/ahab_lake.sdf` | Lake world: graded buoyancy, lakebed at −10 m, home at the 2026-10-03 test site |

### Source CAD and regeneration

The CAD files are not in the repo: `AHAB_FINAL_simv2.stp` (multibody aircraft), `Float_Mount_Assm.stp` (one wingtip
float with struts) and `ventral fin.stp` (keel rudder), all Inventor exports from 2026-10-13. To regenerate after a
CAD change, run in this order, with `~/occ_venv` holding `cadquery-ocp numpy`:

```bash
cd Tools/simulation/ahab_model/tools
~/occ_venv/bin/python step_to_glb.py       AHAB_FINAL_simv2.stp    . ../twin_tractor_seaplane/meshes
~/occ_venv/bin/python step_to_collision.py AHAB_FINAL_simv2.stp    . ../collision_boxes.json   # ~4 min
~/occ_venv/bin/python float_to_model.py    Float_Mount_Assm.stp    . ../twin_tractor_seaplane/meshes ../collision_boxes.json
~/occ_venv/bin/python fin_to_model.py      "ventral fin.stp"       . ../twin_tractor_seaplane/meshes ../collision_boxes.json
python3 build_model.py
```

## Model

```
base_link   hull + V-tail + tilt drive + battery + avionics; IMU/baro/mag/GPS; AdvancedLiftDrag
├─ servo_2 / servo_3       ruddervators L/R
└─ wing_tilt_joint         axis -y at the tilt pivot, +-pi (wiring seam), + = leading edge up
   └─ wing_link            outer wing panels + nacelles + motors (centre section stays on the hull)
      ├─ servo_0 / servo_1 ailerons L/R
      └─ rotor_0 / rotor_1 props L/R (motor 1 / motor 2), thrust along the wing
```

- **Frame**: x forward, y left, z up (Nick's mesh frame). The nose tip is at x = +0.483 m and the keel bottom at
  z = −0.1415 m.
- **Surfaces**: + = trailing edge up on every surface, the same as the PX4 servo outputs.
- **Tilt wing**: same signal path as the hardware.
  - **Command**: `wing_tilt` → `DO_SET_ACTUATOR` → function 301 → `SIM_GZ_SV_FUNC5` → `servo_4`. A
    `JointController` applies that value as a joint velocity (`SIM_GZ_SV_MIN/MAXA5` = ±65 °/s), so it behaves as a
    rate plant like the reversible ESC.
  - **Feedback**: `gz_bridge` (`SIM_GZ_EN_TILT`) publishes `sensor_encoder` = joint angle × `TILT_GEAR`, zero at
    spawn.
  - **Sign**: `SR_TILT_SP` −1.57 points the props toward the belly, i.e. up while inverted.
- **Aero**: whole-aircraft `AdvancedLiftDrag` on `base_link`. This is valid because the wing is level whenever
  the aircraft flies. It was chosen over standard `LiftDrag`, which has no CD0 or induced drag, changes only lift
  with control deflection, and has no derivatives.
  - **Static terms**: fits over α −4…12°, β −6…6°.
  - **Control derivatives**: per surface, per degree.
  - **Rate derivatives**: from `rate_derivatives.yaml`, falling back to VSPAERO window medians, which are flagged
    in the SDF header.
- **Propulsion**: the JSBSim Cine66 925KV + 9x5x3 operating point: 29.2 N at 1811 rad/s per motor, torque/thrust
  0.0173.

## Cameras

Three camera sensors (`CAMERAS` in `tools/build_model.py`; placeholder optics, no housing mass):

| Sensor | Link | Where | Rate | gz topic |
|---|---|---|---|---|
| `camera_forward` | `base_link` | front face of the pylon, under the wing leading edge, looking forward | 30 Hz, 1280x720, 100 deg | default (`/world/ahab_lake/model/<model>/link/base_link/sensor/camera_forward/image`) |
| `camera_wing` | `wing_link` (tilts with the wing) | under the left wing at y 0.50 m, looking inboard at the hull, 13 deg down | 30 Hz, 1280x720, 100 deg | `/ahab/camera_wing/image` |
| `camera_down` | `base_link` | just under the keel at x 0.20 m, nadir, image top = forward | 30 Hz, 1280x960, 69 deg | `/ahab/camera_down/image` |

Only the forward camera uses the default topic pattern, so it is the one the world's `GstCameraSystem` streams to
QGC (UDP 5600). The down camera is underwater while the aircraft floats, and the lake is a flat plane, so there is
nothing to survey yet.

Rendering is not free even with no subscribers (unthrottled gz server, model floating, 2026-10-14):
5.4x real time without cameras, 4.0x with cameras and no subscribers, 2.2x with all three subscribed.
Self-right demo video (inverted start → SELF_RIGHT + arm, all three feeds tiled, flip metrics printed):

```bash
python3 Tools/simulation/ahab_model/tools/selfright_video.py            # -> videos/selfright_cameras_<time>.webm
python3 Tools/simulation/ahab_model/tools/selfright_video.py --stills   # + one JPEG per sim second
```

For speed runs, regenerate with `AHAB_CAMERAS=0 python3 tools/build_model.py` (and back without it afterwards).

## Water (`PX4_GZ_WORLD=ahab_lake`)

```bash
PX4_GZ_WORLD=ahab_lake make px4_sitl gz_twin_tractor_seaplane
python3 Tools/simulation/ahab_model/tools/water_scenarios.py [float drop taxi autotakeoff selfright mission] [--gui]
```

- **Buoyancy**: gz-sim graded `Buoyancy` on the box collisions. The boxes are volume-exact, from the STEP: the
  fuselage uses its sealed outer envelope (the CAD part is an open thin shell), and the wing panels, floats and fin
  use their solid volume.
- **Hydrodynamics**: `src/modules/simulation/gz_plugins/water_hydrodynamics`, a per-panel model after Kerner,
  "Water interaction model for boats in video games" (Just Cause 3, 2015). It has the same force set and parameter
  names as Mainwaring's `asv_wave_sim` HydrodynamicsPlugin:
  - pressure drag along each panel normal, and suction on lee-side panels
  - ITTC-57 viscous drag
  - small damping scaled by the submerged area
  - Planing lift comes from the pressure term. With `fPDrag` = 1 it is close to Savitsky's (1964) C_L ~ τ^1.1 at
    planing speed.
- **Why per-panel**: the lumped Fossen model (VRX, gz-sim `Hydrodynamics`) has no planing lift and acts in air too.
  Zarnick 1978 / Savitsky are more accurate but assume prismatic hulls and cost much more.
- **Panels** (`twin_tractor_seaplane/hydro_panels.txt`, about 5.6k): the real hull surface from the STEP, with true
  normals. Box faces cannot work: they give a staircase bow with no lift.
- **Numerics, built into the plugin**:
  - submerged fraction over each panel's height, which stops forces switching on and off at the waterline
  - drag limited to what stops a link's motion in one step, which stops the jitter of the light wing link
  - a per-link force/torque cap for violent impacts
- **Coefficients** (`HYDRO` in `tools/build_model.py`):
  - cPDrag 100/100, fPDrag 1
  - cSDrag 20/20: suction a fifth of pressure, otherwise the planing drag stalls the takeoff run at 9 m/s
  - cDampR1 20: pitch damping against porpoising
- **Floats**: 3° nose-up incidence. **Ventral fin** (keel rudder): 50 mm behind the step, as on the aircraft. At
  planing speed the wetted forebody puts the water side force ahead of the CG, so the hull water-loops, as the real
  one did. The fin plus ≥ 6° trim makes it directionally stable, so airframe 4242 holds `RWTO_PSP 8` on the water
  run.

Results (`water_scenarios.py`, placeholder mass properties):

| Scenario | Result |
|---|---|
| float | trim +5.8°, level roll (with floats) |
| drop 0.5 m | settles within 5 mm in 2.5 s (5 bobs) |
| taxi 15 / 30 / 50 / 80 / 100 % | 0.7 / 1.8 / 3.6 / 7.4 / 8.9 m/s, nose-up trim, stable |
| PX4 auto takeoff from the water | off in 4.1 s / 42 m at 22 m/s, heading held within 5°, climbs to 100 m |
| self-right (`SR_OVERCTR` 1.4) | flips and settles level; slower than 2026-10-03 (gap 7) |

## Known gaps (also flagged in the generated SDF header)

1. **Mass properties** are placeholders until `mass_properties.yaml` is filled from the measurement session. The
   JSBSim inertia (0.101/0.052/0.073 kg·m²) is not used: it is too small to contain the wing.
2. **Tilt pivot** is taken from the 16.2 mm spanwise bore in the STEP's fuselage centre section (model x 0.000,
   z 0.1427 m). Confirm it with the tape measurement (`tilt_pivot` in `mass_properties.yaml`).
3. **Drag**: VSPAERO drag is about 4–5× too low compared with flight. The model gives L/D ≈ 27 near cruise, but
   the measured best on 2026-10-07 was 5.3. It's kept as-is to match JSBSim. This is the first thing for the
   incoming aero model to fix.
4. **Rate derivatives**: the VSPAERO values are noisy, and their window median for Cnr is positive
   (destabilizing). Use the flight-identified values once `rate_derivatives.yaml` is filled.
5. **Prop thrust doesn't drop with airspeed** (`MulticopterMotorModel`), unlike JSBSim's advance-ratio prop.
   The mesh shows the 10x9x3 that is actually flown, while the constants are the JSBSim 9x5x3.
6. **Stall**: VSPAERO has none. `alpha_stall` is a 15° placeholder.
7. **Water flip dynamics wait on mass properties.** In the water, self-right reaches over-center at 1.7 s, against
   1.2–1.4 s on 2026-10-03. Flip onset and timing are set by the placeholder CG height and inertia, so re-check them
   with `water_scenarios.py selfright` after the measurement session. Lower rotational water damping (`cDampR1` 2)
   matches the real peak pitch rate (8.2 vs 6.8–9.3 rad/s), but it brings back porpoising at 100 % taxi. Taxi and
   takeoff were kept working, so `cDampR1` stays at 20.
8. **The tilt drive is an ideal speed source with a 10 N·m torque limit.** It rotates the wing in water in 1.4 s,
   against about 3.3 s on 2026-10-03. Put the measured stall torque in `tilt_drive.stall_torque_Nm`.
9. **Water**: flat, with no waves, spray or added mass.
10. **Landing on the water is untested.** A first try with PX4 `commander land` (no mission) took about 3.5 min
    to descend, the simulated battery reached warning, and flight termination activated before touchdown. Look at
    the FW land-mode approach and the battery and failsafe settings before using it in the MDP stack.

## Next steps

1. **Mass-properties session**: fill `mass_properties.yaml` (procedure in
   `documentation/ahab_mass_properties_procedure.md`, including surface throws and tilt stall torque), then run
   `tools/build_model.py` and re-check `water_scenarios.py selfright` against 2026-10-03.
2. **Rate derivatives from the flight logs** (needs step 1's inertia). Write `rate_derivatives.yaml` and document
   the method.
3. **Tilt drive**: set the measured stall torque, so the wing rotates in water in about 3.3 s as logged.
4. **Water landing** (gap 10). Then use `water_scenarios.py mission` (takeoff → land → flip → self-right) as the
   end-to-end check for the MDP stack.
