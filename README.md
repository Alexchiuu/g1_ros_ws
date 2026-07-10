# g1_ros_ws

ROS 2 (Foxy) workspace for the Unitree G1 humanoid with Tetheria/Aero dexterous
hands: URDF description, Gazebo Classic simulation, and bridges for viewing
and commanding the real robot.

## Packages

| Package | Purpose |
|---|---|
| [`g1_description`](src/g1_description) | URDF/xacro model (converted from a MuJoCo MJCF source via `mjcf_to_urdf.py`), meshes, RViz config. |
| [`g1_gazebo`](src/g1_gazebo) | Gazebo Classic bringup: world, `ros2_control` position controllers, spawn launch file. |
| [`g1_state_bridge`](src/g1_state_bridge) | Bridges the real robot's DDS `LowState` + Aero hand ZMQ relay + neck ZMQ server to `sensor_msgs/JointState`, and ships the hand/neck slider GUIs. |
| [`g1_zed_bridge`](src/g1_zed_bridge) | Decodes the G1's ZED Mini H.264 network stream and republishes `Image`/`PointCloud2`. |

## Prerequisites

- Ubuntu 20.04 + ROS 2 Foxy (`/opt/ros/foxy`)
- Gazebo Classic 11 + `gazebo_ros2_control`, `controller_manager`, `position_controllers`, `joint_state_broadcaster`
- `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`
- For the real robot: `unitree_hg` message package and a sourced `unitree_ros2` CycloneDDS workspace (`/opt/unitree_ros2/cyclonedds_ws`), plus `python3-zmq` and `python3-tk`
- For the ZED bridge: ZED SDK Python bindings (`pyzed`) and `cv_bridge`

## Build

```bash
cd ~/g1_ros_ws
colcon build --symlink-install
source install/setup.bash
```

Re-source `install/setup.bash` in every new terminal (the helper scripts in
`script/` do this for you).

## Viewing the description only (no simulation, no robot)

```bash
ros2 launch g1_description display.launch.py
```

Opens RViz2 + `joint_state_publisher_gui` so you can drag sliders and see the
model move. No physics, no controllers.

## Gazebo simulation

```bash
ros2 launch g1_gazebo g1_gazebo.launch.py
```

Spawns the robot in Gazebo Classic with `ros2_control` position controllers
(`body_controller`, `left_hand_controller`, `right_hand_controller`), a
position-command bridge, an overlap monitor, and a sim-fed RViz. Command the
robot by dragging sliders in the "command_gui" `joint_state_publisher_gui`
window — they publish to `/g1/position_command`, which gets re-sorted into
each controller's `commands` topic.

Launch arguments (all optional):

| Argument | Default | Meaning |
|---|---|---|
| `world` | `g1_gazebo/worlds/g1_world.world` | Gazebo world file to load |
| `spawn_z` | `0.85` | Spawn height in meters |
| `rviz` | `true` | Start the sim-fed RViz window |
| `command_gui` | `true` | Start the joint slider GUI |

Example: run headless, no RViz, no sliders:

```bash
ros2 launch g1_gazebo g1_gazebo.launch.py rviz:=false command_gui:=false
```

## Real robot

These require the robot reachable on the network and its onboard helper
processes already running (see each script's header for exact requirements —
`aero_hand_relay.py` for the hands, `neck_server.py` for the neck, both run
on the robot itself, outside this workspace).

The `script/` directory has ready-to-run wrappers that source ROS, the
CycloneDDS workspace, and this workspace's `install/setup.bash` for you:

```bash
./script/view_robot.sh           # RViz fed by the real robot's live state
./script/aero_hand_controller.sh # Hand slider GUI (7 sliders/hand)
./script/neck_controller.sh      # Neck slider GUI (yaw/pitch)
```

`view_robot.sh` also sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and a
`CYCLONEDDS_URI` pinned to a specific network interface
(`enx001122683161`) — edit that interface name in the script if your machine's
Ethernet adapter to the robot is named differently.

Equivalent manual invocation (if you'd rather not use the wrapper):

```bash
source /opt/ros/foxy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch g1_state_bridge real_state.launch.py
```

`real_state.launch.py` also brings up the ZED bridge and accepts neck
calibration overrides if you've re-zeroed the neck with the robot-side
`neck.py`:

| Argument | Default | Meaning |
|---|---|---|
| `neck_yaw_zero_ticks` | `2023` | Raw Dynamixel tick treated as yaw zero |
| `neck_pitch_zero_ticks` | `3688` | Raw Dynamixel tick treated as pitch zero |
| `neck_yaw_sign` | `-1` | Sign flip between tick and radian direction |
| `neck_pitch_sign` | `-1` | Sign flip between tick and radian direction |

## ZED camera bridge

Usually brought up automatically by `real_state.launch.py`. To run it
standalone:

```bash
ros2 launch g1_zed_bridge zed_bridge.launch.py stream_ip:=192.168.123.164
```

| Argument | Default | Meaning |
|---|---|---|
| `stream_ip` | `192.168.123.164` | IP the ZED H.264 stream is opened on (by `calex/deploy/zed.py` on the robot) |
| `stream_port` | `30000` | Stream port |
| `frame_id` | `zed_camera_frame` | TF frame for published image/cloud |
| `depth_mode` | `ULTRA` | One of `NONE`, `PERFORMANCE`, `QUALITY`, `ULTRA`, `NEURAL` |

## MJCF → URDF conversion

`g1_description`'s URDF/xacro was generated from a MuJoCo MJCF source with:

```bash
ros2 run g1_description mjcf_to_urdf.py <input.xml> <output.urdf> [--xacro] [--mesh-pkg PKG_URI]
```

Only needed if you're regenerating the model from a new MJCF source, not for
normal use.

## Troubleshooting

- **Gazebo GUI never finishes loading / "No mesh specified" spam**: the
  launch file sets `GAZEBO_MODEL_PATH` for you; if you're launching Gazebo
  some other way, set it manually one directory above `g1_description`.
- **RViz shows "No transform from [X]" for most of the robot in sim**: make
  sure nothing has `use_sim_time:=true` — Gazebo's `/clock` isn't reliable in
  this setup, so every node in `g1_gazebo.launch.py` deliberately runs on the
  wall clock.
- **Robot falls over / links visibly interpenetrate in sim**: expected —
  simulated joints are position-controlled with no torque limit, so nothing
  stops one commanded pose from driving links into each other.
  `overlap_monitor.py` logs a warning when this happens; it doesn't prevent it.
