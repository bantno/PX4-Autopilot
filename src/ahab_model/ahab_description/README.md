# AHAB Description

This ROS 2 package defines the physical characteristics of the AHAB seaplane
for visualization, control, and simulation. It follows the layout of
`asdl_roomba_description`.

## Usage

The `*.urdf.xacro` files in the `urdf` directory are converted to proper
`*.urdf` files upon building. They are found in the relevant `share` directory.

```bash
xacro urdf/ahab.urdf.xacro namespace:=ahab00 sim:=true
ros2 launch ahab_description description.launch.py   # robot_state_publisher only
```

## Files

`config` - ROS <-> Gazebo bridge configuration

`launch` - `description.launch.py` publishes the robot description and TF

`models/ahab_base` - airframe xacro, meshes and Gazebo plugins (`simulation/gz.xacro`)

`urdf` - top-level xacro (`base_link` -> `frame_link` + surfaces + props)

## Model

| Link | Mesh | Joint | Axis |
|------|------|-------|------|
| `frame_link` | `ahab_main.glb` (hull, wing, fixed V-tail) | fixed to `base_link` | - |
| `aileron_{left,right}` | `aileron.glb` (Alr_L) | revolute +-0.52 rad | y (hinge) |
| `ruddervator_{left,right}` | `ruddervator.glb` (vtail_alr) | revolute +-0.52 rad | y along tail panel |
| `rotor_{left,right}` | `prop_10x9x3.glb` | continuous | x (thrust) |

Frame: x forward, y left, z up. The GLB files are glTF Y-up exports in metres,
so the hull visual is rolled +90 deg, same as the x650 in `asdl_roomba`.
Hinge and motor locations were measured from the cutouts in `ahab_main.glb`;
they are xacro properties at the top of `models/ahab_base/ahab_base.xacro`.
The left meshes are reused on the right side for now.

Mass (4.0 kg) and inertia are taken from the JSBSim TwinTractor model. The CG
is an estimate at roughly the wing quarter chord. Collisions are temporary boxes.
