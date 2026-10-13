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
| `twin_tractor_seaplane/meshes/` | Nick Trujillo's GLB meshes (from `ac86632ced` on `gazebo_sim`) |
| `mass_properties.yaml` | Measured mass/CG/inertia (procedure: `documentation/ahab_mass_properties_procedure.md`) |
| `rate_derivatives.yaml` | Rate derivatives identified from flight logs (`documentation/ahab_rate_derivative_identification.md`) |
| `tools/mass_properties.py` | Pendulum data → link inertials (`--selftest` for a known-answer check) |
| `tools/stab_to_gz.py` | VSPAERO `.stab` → AdvancedLiftDrag coefficients |
| `tools/build_model.py` | Writes `model.sdf` |

## Model

```
base_link   hull + V-tail + tilt drive + battery + avionics; IMU/baro/mag/GPS; AdvancedLiftDrag
├─ servo_2 / servo_3       ruddervators L/R
└─ wing_tilt_joint         axis -y at the tilt pivot, +-pi (wiring seam), + = leading edge up
   └─ wing_link            wing + nacelles + motors
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

## Known gaps (also flagged in the generated SDF header)

1. **Mass properties** are placeholders until `mass_properties.yaml` is filled from the measurement session. The
   JSBSim inertia (0.101/0.052/0.073 kg·m²) is not used: it is too small to contain the wing.
2. **The wing visual doesn't tilt** until Nick exports `ahab_main.glb` split into `meshes/hull.glb` (hull + V-tail)
   and `meshes/wing.glb` (wing + nacelles), in the same frame and units. `build_model.py` picks them up
   automatically. Props, ailerons and the wing collision box already tilt.
3. **Drag**: VSPAERO drag is about 4–5× too low compared with flight. The model gives L/D ≈ 27 near cruise, but
   the measured best on 2026-10-07 was 5.3. It's kept as-is to match JSBSim. This is the first thing for the
   incoming aero model to fix.
4. **Rate derivatives**: the VSPAERO values are noisy, and their window median for Cnr is positive
   (destabilizing). Use the flight-identified values once `rate_derivatives.yaml` is filled.
5. **Prop thrust doesn't drop with airspeed** (`MulticopterMotorModel`), unlike JSBSim's advance-ratio prop.
   The mesh shows the 10x9x3 that is actually flown, while the constants are the JSBSim 9x5x3.
6. **Stall**: VSPAERO has none. `alpha_stall` is a 15° placeholder.
7. **No water or buoyancy.** Self-right can be exercised for sequencing and signs only.
