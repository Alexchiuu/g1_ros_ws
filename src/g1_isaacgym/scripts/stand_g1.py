#!/usr/bin/env python3
"""Load g1_description's URDF into IsaacGym and make it stand still.

Leg balance is driven by Unitree's own pretrained locomotion policy
(policies/g1_legs_stand_walk_policy.pt, copied verbatim from the official
unitreerobotics/unitree_rl_gym repo's deploy/pre_train/g1/motion.pt) run
closed-loop with a zero velocity command -- this is the exact control loop
`deploy_mujoco.py`/`deploy_real.py` use in that repo, just re-hosted on
IsaacGym instead of MuJoCo/real hardware. A pure fixed-pose PD hold (no
policy) was tried first and reliably topples the robot forward within ~1s in
this sim: a static leg pose alone isn't enough to balance a floating-base
biped without closed-loop correction, hence the policy.

Everything the policy doesn't touch (waist/arms/neck/tendon-hand joints --
this URDF has 63 DOFs, the policy only actuates the 12 leg joints) is
PD-held at a fixed pose with gains from the same source config
(unitree_rl_gym/deploy/deploy_real/configs/g1.yaml, arm_waist_* fields, also
identical to DexMan-corl's enter_default_pose.py). Neck/hands aren't covered
by that config either; they get soft gains holding them at 0.

Run (from repo root, dexman_isaacgym conda env):

    conda activate dexman_isaacgym
    python src/g1_isaacgym/scripts/stand_g1.py --headless --video /tmp/g1_stand.mp4
    python src/g1_isaacgym/scripts/stand_g1.py --viewer          # interactive window

Pass --ros_bridge_port to also stream joint states + pelvis pose over
localhost UDP as JSON, for rviz_bridge.py (run separately, under system ROS,
not this conda env -- see that script's docstring) to republish as
sensor_msgs/JointState + a world->pelvis TF so RViz can show the same motion.
UDP keeps IsaacGym's conda env (its own libstdc++/CUDA/etc.) fully out of the
ROS process's address space instead of importing rclpy in-process here.

Pass --camera_port to also stream a simulated depth camera (mounted on
neck_link, matching where the real ZED rides) as raw-binary UDP packets, for
rviz_bridge.py to republish as sensor_msgs/Image + PointCloud2 on the same
topics (/zed/rgb/image_raw, /zed/points) the real robot's g1_zed_bridge
uses -- RViz's existing displays for those topics just light up, no config
changes needed. See CAMERA_* constants below for the mount geometry and the
pixel->3D formula's derivation (empirically verified, not guessed).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import tempfile
import xml.dom.minidom as minidom
from pathlib import Path

# isaacgym must be imported before torch (it loads its own .so's first).
from isaacgym import gymapi

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[3]
G1_DESCRIPTION_DIR = REPO_ROOT / "src" / "g1_description"
URDF_PATH = G1_DESCRIPTION_DIR / "urdf" / "g1_tether.urdf"
POLICY_PATH = Path(__file__).resolve().parent.parent / "policies" / "g1_legs_stand_walk_policy.pt"

# ---------------------------------------------------------------------------
# Joint groups + gains (see module docstring for provenance).
# ---------------------------------------------------------------------------

LEG_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
LEG_DEFAULT_ANGLES = np.array(
    [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], dtype=np.float32
)
LEG_KP = [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40]
LEG_KD = [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2]

ARM_WAIST_JOINT_NAMES = [
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
ARM_WAIST_KP = [300, 300, 300, 100, 100, 50, 50, 20, 20, 20, 100, 100, 50, 50, 20, 20, 20]
ARM_WAIST_KD = [3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1]

SOFT_KP = 5.0  # neck + tendon-hand joints, not covered by Unitree's config
SOFT_KD = 0.3

# Policy I/O constants, copied from unitree_rl_gym's g1.yaml.
ANG_VEL_SCALE = 0.25
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
ACTION_SCALE = 0.25
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
PHASE_PERIOD = 0.8
SIM_DT = 0.002
CONTROL_DECIMATION = 10  # -> 50 Hz control, matching real hardware control_dt

# ---------------------------------------------------------------------------
# Simulated depth camera, mounted on neck_link (same parent link the real
# ZED's static TF uses). Kept small (128x96) so one frame's raw bytes fit in
# a single UDP datagram (color: 4+8+128*96*3 = 36,876 B; depth:
# 4+16+128*96*4 = 49,172 B; both well under the ~65,507 B loopback limit) --
# no chunking/reassembly needed.
#
# neck_link's own local axes are X-forward, Y-up, Z-right (see
# g1_zed_bridge/launch/zed_bridge.launch.py's docstring -- it's rotated +90
# deg about X from the "usual" X-forward/Y-left/Z-up convention). Mounting
# the IsaacGym camera with local_transform.r = -90 deg about local X exactly
# cancels that (empirically verified: this is the same -90 deg roll that
# file's own comment independently derived for the real camera), so the
# camera's rendered pixels land directly in neck_link's own axis convention
# with no further rotation needed. Confirmed by test: with an identity-
# rotation camera, a world-frame marker at +Y appeared on the image's LEFT
# side, i.e. image-right <-> -Y in the camera's own local frame; the
# formula below is derived from that (and matching IMAGE_DEPTH's documented
# sign: it returns *negative* distance).
CAMERA_WIDTH = 128
CAMERA_HEIGHT = 96
CAMERA_HFOV_DEG = 87.0
CAMERA_FAR_PLANE = 5.0  # meters; matches typical indoor depth camera range
# Forward/up offset from neck_link's origin -- a rough "forehead" position,
# not measured (same placeholder status as zed_bridge.launch.py's own
# camera_xyz defaults). Needs to clear the head mesh itself (a too-small
# forward offset puts the camera inside/behind the head, occluding most of
# the frame -- confirmed by rendering a test frame).
CAMERA_MOUNT_OFFSET = (0.18, 0.08, 0.0)  # (forward, up, right) in neck_link's own axes

CAM_MAGIC_COLOR = b"COLR"
CAM_MAGIC_DEPTH = b"DPTH"


def build_target_maps() -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    target, kp, kd = {}, {}, {}
    for name, angle, p, d in zip(LEG_JOINT_NAMES, LEG_DEFAULT_ANGLES, LEG_KP, LEG_KD):
        target[name], kp[name], kd[name] = float(angle), p, d
    for name, p, d in zip(ARM_WAIST_JOINT_NAMES, ARM_WAIST_KP, ARM_WAIST_KD):
        target[name], kp[name], kd[name] = 0.0, p, d
    return target, kp, kd


def get_gravity_orientation(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """World gravity direction expressed in the body frame, from a wxyz quat."""
    g = np.zeros(3, dtype=np.float32)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


def rotate_world_to_body(qw: float, qx: float, qy: float, qz: float, v: np.ndarray) -> np.ndarray:
    def qmul(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return np.array([
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ])
    q = np.array([qw, qx, qy, qz])
    q_inv = np.array([qw, -qx, -qy, -qz])
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    return qmul(qmul(q_inv, v_quat), q)[1:]


# ---------------------------------------------------------------------------
# Asset prep: strip world/floating_base_joint so `pelvis` is the root link
# (fix_base_link=False then gives it a free 6-DOF floating base). Same fix
# g1_gazebo.launch.py used for Gazebo's SDF pose graph (see git history).
# ---------------------------------------------------------------------------


def make_isaacgym_asset_root() -> tuple[str, str]:
    doc = minidom.parseString(URDF_PATH.read_text())
    root_el = doc.documentElement
    for link in root_el.getElementsByTagName("link"):
        if link.getAttribute("name") == "world":
            root_el.removeChild(link)
    for joint in root_el.getElementsByTagName("joint"):
        if joint.getAttribute("name") == "floating_base_joint":
            root_el.removeChild(joint)

    tmp_root = Path(tempfile.mkdtemp(prefix="g1_isaacgym_asset_"))
    pkg_dir = tmp_root / "g1_description"
    (pkg_dir / "urdf").mkdir(parents=True)
    (pkg_dir / "urdf" / "g1_tether_isaacgym.urdf").write_text(doc.toxml())
    os.symlink(G1_DESCRIPTION_DIR / "meshes", pkg_dir / "meshes")
    return str(tmp_root), "g1_description/urdf/g1_tether_isaacgym.urdf"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, default=12.0, help="sim seconds to run")
    p.add_argument("--spawn_z", type=float, default=0.80)
    p.add_argument("--viewer", action="store_true", help="open an interactive IsaacGym viewer window")
    p.add_argument("--headless", action="store_true", help="no viewer window (default)")
    p.add_argument("--video", type=str, default=None, help="path to write an mp4 via a headless camera sensor")
    p.add_argument("--device", type=int, default=0, help="CUDA device index")
    p.add_argument("--ros_bridge_port", type=int, default=0,
                    help="if set, stream joint states + pelvis pose as UDP JSON to 127.0.0.1:PORT "
                         "for rviz_bridge.py to pick up (0 = disabled)")
    p.add_argument("--camera_port", type=int, default=0,
                    help="if set, stream a simulated depth camera (neck_link-mounted) as UDP binary "
                         "to 127.0.0.1:PORT for rviz_bridge.py to pick up (0 = disabled)")
    p.add_argument("--camera_hz", type=float, default=10.0, help="camera capture rate")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    use_viewer = args.viewer and not args.headless

    gym = gymapi.acquire_gym()

    sim_params = gymapi.SimParams()
    sim_params.dt = SIM_DT
    sim_params.substeps = 1
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.contact_offset = 0.01
    sim_params.physx.rest_offset = 0.0
    sim_params.physx.use_gpu = True
    sim_params.use_gpu_pipeline = False

    sim = gym.create_sim(args.device, args.device, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        raise RuntimeError("gym.create_sim failed")

    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    plane_params.static_friction = 1.0
    plane_params.dynamic_friction = 1.0
    gym.add_ground(sim, plane_params)

    asset_root, asset_file = make_isaacgym_asset_root()
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = False
    asset_options.collapse_fixed_joints = True
    asset_options.armature = 0.01
    asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

    dof_names = gym.get_asset_dof_names(asset)
    num_dofs = gym.get_asset_dof_count(asset)
    print(f"[stand_g1] loaded asset with {num_dofs} DOFs, "
          f"{gym.get_asset_rigid_body_count(asset)} rigid bodies")

    target_map, kp_map, kd_map = build_target_maps()
    dof_props = gym.get_asset_dof_properties(asset)
    default_targets = np.zeros(num_dofs, dtype=np.float32)
    name_to_idx = {}
    for i, name in enumerate(dof_names):
        name_to_idx[name] = i
        dof_props["driveMode"][i] = gymapi.DOF_MODE_POS
        dof_props["stiffness"][i] = kp_map.get(name, SOFT_KP)
        dof_props["damping"][i] = kd_map.get(name, SOFT_KD)
        default_targets[i] = target_map.get(name, 0.0)
    leg_idx = [name_to_idx[n] for n in LEG_JOINT_NAMES]

    env = gym.create_env(sim, gymapi.Vec3(-2, -2, 0), gymapi.Vec3(2, 2, 2), 1)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, args.spawn_z)
    pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
    actor = gym.create_actor(env, asset, pose, "g1_tether", 0, 1)
    gym.set_actor_dof_properties(env, actor, dof_props)

    # Start already in the target pose (no mid-air transient into a crouch).
    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    dof_states["pos"] = default_targets
    dof_states["vel"] = 0.0
    gym.set_actor_dof_states(env, actor, dof_states, gymapi.STATE_ALL)
    gym.set_actor_dof_position_targets(env, actor, default_targets)

    pelvis_handle = gym.find_actor_rigid_body_handle(env, actor, "pelvis")

    depth_cam_handle = None
    depth_cam_fx = depth_cam_fy = depth_cam_cx = depth_cam_cy = None
    depth_cam_pixel_u = depth_cam_pixel_v = None
    if args.camera_port:
        neck_body_handle = gym.find_actor_rigid_body_handle(env, actor, "neck_link")
        cam_props = gymapi.CameraProperties()
        cam_props.width = CAMERA_WIDTH
        cam_props.height = CAMERA_HEIGHT
        cam_props.horizontal_fov = CAMERA_HFOV_DEG
        cam_props.far_plane = CAMERA_FAR_PLANE
        depth_cam_handle = gym.create_camera_sensor(env, cam_props)
        local_transform = gymapi.Transform()
        fwd, up, right = CAMERA_MOUNT_OFFSET
        local_transform.p = gymapi.Vec3(fwd, up, right)
        local_transform.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1.0, 0.0, 0.0), -np.pi / 2.0)
        gym.attach_camera_to_body(depth_cam_handle, env, neck_body_handle, local_transform,
                                   gymapi.FOLLOW_TRANSFORM)

        proj = np.array(gym.get_camera_proj_matrix(sim, env, depth_cam_handle))
        depth_cam_fx = float(proj[0, 0]) * CAMERA_WIDTH / 2.0
        depth_cam_fy = float(proj[1, 1]) * CAMERA_HEIGHT / 2.0
        depth_cam_cx = CAMERA_WIDTH / 2.0
        depth_cam_cy = CAMERA_HEIGHT / 2.0
        print(f"[stand_g1] streaming depth camera to udp://127.0.0.1:{args.camera_port} "
              f"({CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {args.camera_hz} Hz)")

    policy = torch.jit.load(str(POLICY_PATH))
    policy.eval()
    cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # zero velocity command = stand in place
    action = np.zeros(12, dtype=np.float32)

    ros_sock = None
    ros_addr = None
    if args.ros_bridge_port:
        ros_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ros_addr = ("127.0.0.1", args.ros_bridge_port)
        print(f"[stand_g1] streaming to rviz_bridge.py at udp://127.0.0.1:{args.ros_bridge_port}")

    cam_sock = None
    cam_addr = None
    if args.camera_port:
        cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cam_addr = ("127.0.0.1", args.camera_port)

    viewer = None
    if use_viewer:
        viewer = gym.create_viewer(sim, gymapi.CameraProperties())
        gym.viewer_camera_look_at(viewer, env, gymapi.Vec3(2.0, 2.0, 1.5), gymapi.Vec3(0.0, 0.0, 0.8))

    video_writer = None
    cam_handle = None
    cam_props = None
    if args.video is not None:
        import imageio
        cam_props = gymapi.CameraProperties()
        cam_props.width, cam_props.height = 640, 480
        cam_handle = gym.create_camera_sensor(env, cam_props)
        gym.set_camera_location(cam_handle, env, gymapi.Vec3(2.0, 2.0, 1.5), gymapi.Vec3(0.0, 0.0, 0.8))
        video_writer = imageio.get_writer(args.video, fps=30)

    camera_decimation = max(1, round(1.0 / (args.camera_hz * SIM_DT))) if depth_cam_handle is not None else None

    num_steps = int(args.duration / SIM_DT)
    counter = 0
    sim_time = 0.0
    pelvis_heights = []
    fell_over = False

    try:
        for step in range(num_steps):
            gym.simulate(sim)
            gym.fetch_results(sim, True)
            gym.step_graphics(sim)
            counter += 1
            sim_time += SIM_DT

            if counter % CONTROL_DECIMATION == 0:
                body_state = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)
                q = body_state["pose"]["r"][pelvis_handle]
                v_ang = body_state["vel"]["angular"][pelvis_handle]
                qw, qx, qy, qz = float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"])
                omega_body = rotate_world_to_body(qw, qx, qy, qz, np.array([v_ang["x"], v_ang["y"], v_ang["z"]]))
                gravity = get_gravity_orientation(qw, qx, qy, qz)

                dof_state = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
                qj = np.array([dof_state["pos"][i] for i in leg_idx], dtype=np.float32)
                dqj = np.array([dof_state["vel"][i] for i in leg_idx], dtype=np.float32)

                phase = (sim_time % PHASE_PERIOD) / PHASE_PERIOD
                obs = np.zeros(47, dtype=np.float32)
                obs[0:3] = omega_body * ANG_VEL_SCALE
                obs[3:6] = gravity
                obs[6:9] = cmd * CMD_SCALE
                obs[9:21] = (qj - LEG_DEFAULT_ANGLES) * DOF_POS_SCALE
                obs[21:33] = dqj * DOF_VEL_SCALE
                obs[33:45] = action
                obs[45:47] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]

                with torch.no_grad():
                    action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze().astype(np.float32)
                leg_targets = action * ACTION_SCALE + LEG_DEFAULT_ANGLES

                full_targets = default_targets.copy()
                for j, li in enumerate(leg_idx):
                    full_targets[li] = leg_targets[j]
                gym.set_actor_dof_position_targets(env, actor, full_targets)

                if ros_sock is not None:
                    p = body_state["pose"]["p"][pelvis_handle]
                    payload = {
                        "t": sim_time,
                        "joint_names": dof_names,
                        "joint_pos": [float(dof_state["pos"][i]) for i in range(num_dofs)],
                        "pelvis_pos": [float(p["x"]), float(p["y"]), float(p["z"])],
                        "pelvis_quat_xyzw": [qx, qy, qz, qw],
                    }
                    try:
                        ros_sock.sendto(json.dumps(payload).encode("utf-8"), ros_addr)
                    except OSError:
                        pass

            body_state = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_POS)
            pelvis_z = float(body_state["pose"]["p"][pelvis_handle]["z"])
            pelvis_heights.append(pelvis_z)
            if pelvis_z < 0.4:
                fell_over = True

            want_video_frame = video_writer is not None and step % 10 == 0
            want_depth_frame = depth_cam_handle is not None and step % camera_decimation == 0
            if want_video_frame or want_depth_frame:
                gym.render_all_camera_sensors(sim)

            if want_video_frame:
                img = gym.get_camera_image(sim, env, cam_handle, gymapi.IMAGE_COLOR)
                img = img.reshape(cam_props.height, cam_props.width, 4)[:, :, :3]
                video_writer.append_data(img)

            if want_depth_frame:
                color = gym.get_camera_image(sim, env, depth_cam_handle, gymapi.IMAGE_COLOR)
                color = color.reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 4)[:, :, :3]
                raw_depth = gym.get_camera_image(sim, env, depth_cam_handle, gymapi.IMAGE_DEPTH)
                depth = -raw_depth.astype(np.float32)  # IMAGE_DEPTH is negative distance
                depth[~np.isfinite(depth)] = 0.0  # no-hit pixels -> invalid marker (0 = never a real depth)
                depth[depth > CAMERA_FAR_PLANE] = 0.0

                color_header = CAM_MAGIC_COLOR + struct.pack("<II", CAMERA_WIDTH, CAMERA_HEIGHT)
                depth_header = CAM_MAGIC_DEPTH + struct.pack(
                    "<IIffff", CAMERA_WIDTH, CAMERA_HEIGHT,
                    depth_cam_fx, depth_cam_fy, depth_cam_cx, depth_cam_cy)
                try:
                    cam_sock.sendto(color_header + np.ascontiguousarray(color, dtype=np.uint8).tobytes(), cam_addr)
                    cam_sock.sendto(depth_header + np.ascontiguousarray(depth, dtype=np.float32).tobytes(), cam_addr)
                except OSError:
                    pass

            if viewer is not None:
                gym.draw_viewer(viewer, sim, True)
                gym.sync_frame_time(sim)
                if gym.query_viewer_has_closed(viewer):
                    break

            if step % (CONTROL_DECIMATION * 50) == 0:
                print(f"[stand_g1] t={sim_time:6.2f}s  pelvis z = {pelvis_z:.3f} m")
    finally:
        if video_writer is not None:
            video_writer.close()
            print(f"[stand_g1] wrote video to {args.video}")
        if viewer is not None:
            gym.destroy_viewer(viewer)
        gym.destroy_sim(sim)

    heights = np.array(pelvis_heights)
    settled = heights[len(heights) // 2:]
    print(f"[stand_g1] pelvis z: start={heights[0]:.3f}  end={heights[-1]:.3f}  "
          f"settled mean={settled.mean():.3f}  settled std={settled.std():.4f}")
    if not fell_over and 0.6 < settled.mean() < 0.95:
        print("[stand_g1] PASS: robot stood without falling for the full run.")
    else:
        print("[stand_g1] WARNING: robot fell over during the run.")


if __name__ == "__main__":
    main()
