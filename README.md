# g1_ros_ws

ROS 2 (Foxy) workspace for the Unitree G1 humanoid with Tetheria/Aero dexterous
hands: URDF description, an IsaacGym-based simulation, and bridges for
viewing and commanding the real robot.

## Packages

| Package | Purpose |
|---|---|
| [`g1_description`](src/g1_description) | URDF/xacro model, meshes, RViz configs, `view.launch.py`. |
| [`g1_isaacgym`](src/g1_isaacgym) | IsaacGym simulation scripts. Not a ROS/colcon package -- run with the `dexman_isaacgym` conda env. |
| [`g1_state_bridge`](src/g1_state_bridge) | Real-robot state bridge + hand/neck slider GUIs. |
| [`g1_zed_bridge`](src/g1_zed_bridge) | Real robot's ZED camera stream decoder. |

## Prerequisites

- Ubuntu 20.04 + ROS 2 Foxy (`/opt/ros/foxy`)
- `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`
- Simulation: IsaacGym + PyTorch (`dexman_isaacgym` conda env)
- Real robot: `unitree_hg`, a sourced `unitree_ros2` CycloneDDS workspace, `python3-zmq`, `python3-tk`
- ZED bridge: `pyzed`, `cv_bridge`

## Build

```bash
cd ~/g1_ros_ws
colcon build --symlink-install
source install/setup.bash
```

## View the description only (no sim, no robot)

```bash
ros2 launch g1_description display.launch.py
```

## Run the IsaacGym sim

```bash
./script/run_isaacgym.sh --viewer                          # interactive window
./script/run_isaacgym.sh --headless --video /tmp/g1_stand.mp4
```

| Argument | Default | Meaning |
|---|---|---|
| `--duration` | `12.0` | Sim seconds to run |
| `--spawn_z` | `0.80` | Spawn height in meters |
| `--headless` / `--viewer` | headless | Viewer window vs headless |
| `--video PATH` | none | Record an mp4 (needs `pip install imageio-ffmpeg` in the conda env) |
| `--ros_bridge_port` | `0` (off) | UDP port for joint states + pelvis pose (for RViz) |
| `--camera_port` | `0` (off) | UDP port for the simulated depth camera (for RViz) |
| `--camera_hz` | `10.0` | Simulated camera capture rate |
| `--teleop_gui` | off | Listen for the hand/neck controller GUIs |

## Watch the sim in RViz

```bash
# Terminal A
./script/run_isaacgym.sh --viewer --ros_bridge_port 5555 --camera_port 5556

# Terminal B
./script/view.sh sim
```

Drop `--camera_port` if you don't want the depth camera image/point cloud.

## Sim hand + neck controller GUIs

```bash
./script/run_isaacgym.sh --viewer --teleop_gui   # sim, with the GUI servers listening
./script/aero_hand_controller_sim.sh              # 7 sliders/hand
./script/neck_controller_sim.sh                   # yaw/pitch sliders
```

## Real robot

```bash
./script/view.sh real            # RViz fed by the real robot's live state
./script/aero_hand_controller.sh # Hand slider GUI (7 sliders/hand)
./script/neck_controller.sh      # Neck slider GUI (yaw/pitch)
```

Requires the robot reachable on the network and its onboard helper processes
running (`aero_hand_relay.py` for the hands, `neck_server.py` for the neck,
both run on the robot).

Manual invocation (if not using the wrapper):

```bash
source /opt/ros/foxy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch g1_state_bridge real_state.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `neck_yaw_zero_ticks` | `2023` | Raw Dynamixel tick treated as yaw zero |
| `neck_pitch_zero_ticks` | `3688` | Raw Dynamixel tick treated as pitch zero |
| `neck_yaw_sign` | `-1` | Sign flip between tick and radian direction |
| `neck_pitch_sign` | `-1` | Sign flip between tick and radian direction |

## ZED camera bridge (real robot)

```bash
ros2 launch g1_zed_bridge zed_bridge.launch.py stream_ip:=192.168.123.164
```

| Argument | Default | Meaning |
|---|---|---|
| `stream_ip` | `192.168.123.164` | IP the ZED H.264 stream is opened on |
| `stream_port` | `30000` | Stream port |
| `frame_id` | `zed_camera_frame` | TF frame for published image/cloud |
| `depth_mode` | `ULTRA` | `NONE`, `PERFORMANCE`, `QUALITY`, `ULTRA`, or `NEURAL` |

## MJCF → URDF conversion

```bash
ros2 run g1_description mjcf_to_urdf.py <input.xml> <output.urdf> [--xacro] [--mesh-pkg PKG_URI]
```
