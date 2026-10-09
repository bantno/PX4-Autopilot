# AHAB Model

ROS 2 + Gazebo (gz-sim) packages for the AHAB seaplane, laid out like
`asdl-roomba`:

- `ahab_description` - xacro/URDF, GLB meshes, Gazebo plugins, bridge config
- `ahab_bringup` - launch files (`sim.launch.py` spawns the plane in Gazebo)

Worlds live in a separate package at `src/worlds` (`ahab_worlds`).

## Build

PX4-Autopilot has its own `package.xml` at the repo root, so colcon run from
the repo root stops there. Point colcon at these folders explicitly:

```bash
cd ~/workspace/PX4-Autopilot
colcon build --base-paths src/ahab_model src/worlds --symlink-install
source install/setup.bash
```

These folders are not part of the PX4 CMake build, so `make px4_sitl` is unaffected.

## Run

```bash
ros2 launch ahab_bringup sim.launch.py                     # blank world
ros2 launch ahab_bringup sim.launch.py headless:=true      # no Gazebo window
ros2 launch ahab_bringup sim.launch.py world:=/path/to.sdf vehicle_namespace:=ahab01

# props [rad/s]: velocity[0]=left, velocity[1]=right (max 1811 ~ 29 N each)
ros2 topic pub --once /ahab00/command/motor_speed actuator_msgs/msg/Actuators "{velocity: [1811.0, 1811.0]}"

# surfaces [rad], positive = trailing edge up
ros2 topic pub --once /ahab00/cmd/ruddervator_left std_msgs/msg/Float64 "{data: 0.15}"   # both + = pitch up
ros2 topic pub --once /ahab00/cmd/ruddervator_right std_msgs/msg/Float64 "{data: 0.15}"
ros2 topic pub --once /ahab00/cmd/aileron_left std_msgs/msg/Float64 "{data: -0.1}"       # roll right
ros2 topic pub --once /ahab00/cmd/aileron_right std_msgs/msg/Float64 "{data: 0.1}"
```

Control signs: roll right = aileron_left < 0, aileron_right > 0; pitch up = both
ruddervators > 0; yaw right = ruddervator_left > 0, ruddervator_right < 0.

At full throttle on the ground it reaches ~27 m/s in about 2.5 s; with up
ruddervator it lifts off from there. Without elevator it stays on the ground until
~43 m/s because the props sit above the CG and push the nose down.

Without ROS, the same works from gz directly:

```bash
gz topic -t /ahab00/command/motor_speed -m gz.msgs.Actuators -p "velocity: [900, 900]"
```

## Simulation model

- Props: gz `MulticopterMotorModel`, constants from the JSBSim TwinTractor 9x5x3
  prop. Thrust does not fall off with airspeed.
- Wing halves and V-tail panels: gz `LiftDrag`, each tied to its control surface.
- Body drag: an extra `LiftDrag` element with no lift (CD0 ~0.07 on wing area),
  so top speed levels out (~45 m/s at full throttle).
- Hull friction is low (mu 0.2) so it can slide on the ground plane.

Requires ROS 2 Humble + Gazebo Harmonic (the container is Ubuntu 22.04).
Install with `scripts/install_deps.sh`.
