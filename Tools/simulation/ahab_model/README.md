# AHAB visual model

Visual geometry for the AHAB seaplane, by Nick Trujillo. These are the visual
parts of commit `ac86632ced` on `origin/gazebo_sim`. The full ROS 2 package
(launch files, Gazebo plugins, bridge config, world) is still on that branch.

- `meshes/`
  - `ahab_main.glb`: hull, wing and fixed V-tail
  - `aileron.glb`, `ruddervator.glb`: control surfaces
  - `prop_10x9x3.glb`: prop
  - All are glTF Y-up exports from Blender, in metres.
- `ahab_base.xacro`
  - Links and joints: frame, two ailerons, two ruddervators and two props.
  - Mesh orientations.
  - Hinge and motor positions measured from the cutouts in `ahab_main.glb`.

Frame convention is x forward, y left, z up. Positive surface deflection is
trailing edge up on all four surfaces. The left meshes are reused on the right side.

## Not included (intentionally)

The placeholder physics was left out:
- mass and inertia (4.0 kg, estimated CG)
- box collisions
- the prop motor model (JSBSim 9x5x3 constants)
- the generic `advanced_plane` lift/drag
- body drag

These should come from the measured vehicle and the AHAB aero model when this is
integrated with PX4. Use the 5 kg AUW and the canonical CG at 0.445 m from the
nose.

## Integration notes (PX4 gz SITL)

- `mesh_dir` still points at `package://ahab_description/...`. Change it to a
  `model://` path when the model moves into the PX4 gz models tree.
- PX4's gz bridge publishes motor speeds on `/<model>/command/motor_speed` and
  servos on `/model/<model>/<joint_name>` (`gz.msgs.Double`).
- In the original xacro, the props' joint is pitched so link +z is the thrust
  axis, as `MulticopterMotorModel` expects.
