# g1_ros_ws

ROS 2 (Foxy) workspace for the Unitree G1 humanoid with Tetheria/Aero dexterous
hands: URDF description, an IsaacGym-based simulation, and bridges for
viewing and commanding the real robot.

## Packages

| Package | Purpose |
|---|---|
| [`g1_description`](src/g1_description) | URDF/xacro model (converted from a MuJoCo MJCF source via `mjcf_to_urdf.py`), meshes, RViz config. |
| [`g1_isaacgym`](src/g1_isaacgym) | Standalone IsaacGym simulation: loads this package's URDF and runs Unitree's pretrained leg policy to stand the robot up. Not a ROS node -- run with the `dexman_isaacgym` conda env, not colcon. |
| [`g1_state_bridge`](src/g1_state_bridge) | Bridges the real robot's DDS `LowState` + Aero hand ZMQ relay + neck ZMQ server to `sensor_msgs/JointState`, and ships the hand/neck slider GUIs. |
| [`g1_zed_bridge`](src/g1_zed_bridge) | Decodes the G1's ZED Mini H.264 network stream and republishes `Image`/`PointCloud2`. |

## Prerequisites

- Ubuntu 20.04 + ROS 2 Foxy (`/opt/ros/foxy`)
- `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`
- For simulation: an IsaacGym install + PyTorch (this machine already has one in the `dexman_isaacgym` conda env -- see [Simulation](#simulation) below)
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

## Simulation

IsaacGym, not Gazebo. `g1_isaacgym` is a plain Python script (not a colcon
package -- it runs against the `dexman_isaacgym` conda env's Python 3.8 /
CUDA IsaacGym install, which lives outside this workspace):

```bash
./script/run_isaacgym.sh --headless --video /tmp/g1_stand.mp4
./script/run_isaacgym.sh --viewer          # interactive window instead
```

(`script/run_isaacgym.sh` just activates `dexman_isaacgym` and forwards args
to `src/g1_isaacgym/scripts/stand_g1.py` -- run that directly if you'd rather
manage the conda env yourself.)

Loads `g1_description`'s URDF straight from `src/` (mesh `package://` URIs
resolve against `asset_root=src/`, same convention IsaacGym's own bundled
assets use), strips the URDF's `world` link/`floating_base_joint` so `pelvis`
becomes a free-floating root (see `stand_g1.py`'s docstring -- same fix
`g1_gazebo.launch.py` used to use for Gazebo's SDF pose graph), then holds
the robot standing with Unitree's own pretrained leg policy
(`src/g1_isaacgym/policies/g1_legs_stand_walk_policy.pt`, see
`policies/NOTICE.md` for provenance) run closed-loop with a zero velocity
command. A fixed-pose PD hold with no policy reliably topples the robot
forward within about a second in this sim -- a static leg pose alone isn't
enough to balance a floating-base biped, hence the policy. Everything the
policy doesn't drive (waist/arms/neck/hands) is PD-held at a fixed default
pose.

### Watching it in RViz too

The IsaacGym viewer window (`--viewer`) is the sim itself; RViz can mirror
the same motion alongside it, fed over localhost UDP (kept separate from the
conda env to avoid its CUDA/libstdc++ colliding with system ROS). Two
terminals:

```bash
# Terminal A -- the sim, streaming joint states (5555) + a simulated depth
# camera (5556) mounted on neck_link, matching where the real ZED rides
./script/run_isaacgym.sh --viewer --ros_bridge_port 5555 --camera_port 5556

# Terminal B -- RViz (system ROS, not the conda env)
./script/view.sh sim
```

The camera is optional -- drop `--camera_port` if you just want joint
states. When it's on, it publishes to `/zed/rgb/image_raw` and `/zed/points`,
the exact topics + frame (`zed_camera_frame`) the real robot's ZED bridge
uses, so RViz's existing Image/PointCloud2 displays (already in
`isaacgym_live.rviz`, carried over from the real robot's `display.rviz`)
light up with no config changes. It's a coarse 128x96 render (kept small
enough that one frame fits in a single UDP datagram) at 10 Hz, not a
photorealistic sensor -- good enough to sanity-check what the robot "sees",
not for training anything.

`view.sh` also covers the [real robot](#real-robot) below (`./script/view.sh
real`) -- one script, one launch file (`g1_description`'s `view.launch.py`),
since both cases ultimately just need `robot_state_publisher` + rviz2 pointed
at whatever's currently publishing `/joint_states`. `source:=sim`
brings up `rviz_bridge.py` + rviz2 on `isaacgym_live.rviz` (Fixed Frame
`world`, since IsaacGym has ground-truth pelvis pose); `source:=real`
delegates entirely to `g1_state_bridge`'s `real_state.launch.py` (see
[Real robot](#real-robot) below) with its own rviz2 on `display.rviz` (Fixed
Frame `pelvis`, since the real robot has no global-pose source). Start
either order -- RViz just shows nothing until the sim/robot side starts
streaming.

| Argument | Default | Meaning |
|---|---|---|
| `--duration` | `12.0` | Sim seconds to run |
| `--spawn_z` | `0.80` | Spawn height in meters |
| `--headless` / `--viewer` | headless | Open an interactive viewer window instead of running headless |
| `--video PATH` | none | Record an mp4 via a headless camera sensor (needs `pip install imageio-ffmpeg` in the conda env) |
| `--ros_bridge_port` | `0` (off) | UDP port to stream joint states + pelvis pose to (see above) |
| `--camera_port` | `0` (off) | UDP port to stream the simulated depth camera to (see above) |
| `--camera_hz` | `10.0` | Simulated camera capture rate |
| `--teleop_gui` | off | Listen for the hand/neck controller GUIs below |

### Hand + neck controller GUIs

The real robot's hand/neck slider GUIs ([below](#real-robot)) work against
the sim unmodified -- just point them at localhost instead of the robot,
since they talk to a documented ZMQ wire protocol rather than to the robot
specifically, and `stand_g1.py --teleop_gui` implements that same protocol
(`ZmqTeleopLink` in that script):

```bash
./script/run_isaacgym.sh --viewer --teleop_gui   # sim, with the servers listening
./script/aero_hand_controller_sim.sh              # 7 sliders/hand
./script/neck_controller_sim.sh                   # yaw/pitch sliders
```

Note: the 32 tendon-driven hand joints carry `effort="0"` in the URDF (fine
for Gazebo's old kinematic `SetPosition()`, which ignores effort limits, but
IsaacGym's PD drive is real torque-based dynamics and *does* enforce it --
`stand_g1.py` gives them a small nonzero torque budget for exactly this
reason, see its `dof_props["effort"]` override).

## Real robot

These require the robot reachable on the network and its onboard helper
processes already running (see each script's header for exact requirements —
`aero_hand_relay.py` for the hands, `neck_server.py` for the neck, both run
on the robot itself, outside this workspace).

The `script/` directory has ready-to-run wrappers that source ROS, the
CycloneDDS workspace, and this workspace's `install/setup.bash` for you:

```bash
./script/view.sh real            # RViz fed by the real robot's live state
./script/aero_hand_controller.sh # Hand slider GUI (7 sliders/hand)
./script/neck_controller.sh      # Neck slider GUI (yaw/pitch)
```

`./script/view.sh real` also sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
a `CYCLONEDDS_URI` pinned to a specific network interface
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

- **`stand_g1.py` falls over / topples forward**: check `--spawn_z` is close
  to `0.80` (matches the leg default pose's natural standing height) and
  that `src/g1_isaacgym/policies/g1_legs_stand_walk_policy.pt` actually
  loaded (a missing/corrupt policy file would fail at `torch.jit.load`, not
  silently no-op).
- **`ImportError`/`ExpatError` from `stand_g1.py`**: it parses
  `g1_tether.urdf` with Python's strict `xml.dom.minidom` to strip the
  `world`/`floating_base_joint` before handing it to IsaacGym -- any
  hand-edit to that file that introduces non-well-formed XML (e.g. `--`
  inside an XML comment, which is invalid) will break this step.
