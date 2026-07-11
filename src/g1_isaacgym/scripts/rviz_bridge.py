#!/usr/bin/env python3
"""UDP -> ROS2 bridge for the live IsaacGym sim.

stand_g1.py (run under the dexman_isaacgym conda env, see that script's
--ros_bridge_port / --camera_port) streams joint positions + pelvis pose as
UDP JSON, and optionally a simulated depth camera as raw-binary UDP. This
node republishes:

  - joint positions + pelvis pose -> sensor_msgs/JointState on /joint_states
    + a world->pelvis TF, so RViz shows the same motion the IsaacGym viewer
    window shows.
  - the depth camera -> sensor_msgs/Image on /zed/rgb/image_raw + a
    PointCloud2 on /zed/points, frame_id "zed_camera_frame" -- the same
    topics/frame g1_zed_bridge's real ZED node publishes, so RViz's existing
    displays for them just light up (see g1_description/rviz/*.rviz).

UDP is used instead of importing rclpy directly in stand_g1.py's process to
keep the conda env's own libstdc++/CUDA/etc. out of the ROS process
entirely.

Run under system ROS, NOT the dexman_isaacgym conda env:

    source /opt/ros/foxy/setup.bash
    source ~/g1_ros_ws/install/setup.bash
    python3 src/g1_isaacgym/scripts/rviz_bridge.py --port 5555 --camera_port 5556
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

CAM_MAGIC_COLOR = b"COLR"
CAM_MAGIC_DEPTH = b"DPTH"
CAMERA_FRAME_ID = "zed_camera_frame"

POINT_CLOUD_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]


class RvizBridge(Node):
    def __init__(self, port: int, camera_port: int):
        super().__init__("g1_isaacgym_rviz_bridge")
        self._lock = threading.Lock()
        self._latest: dict | None = None

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", port))
        self._sock.settimeout(1.0)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / 30.0, self._publish)

        self._cam_lock = threading.Lock()
        self._latest_color: np.ndarray | None = None  # (H, W, 3) uint8
        self._latest_depth: tuple[np.ndarray, float, float, float, float] | None = None
        if camera_port:
            self._cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cam_sock.bind(("127.0.0.1", camera_port))
            self._cam_sock.settimeout(1.0)
            threading.Thread(target=self._camera_recv_loop, daemon=True).start()
            self._image_pub = self.create_publisher(Image, "zed/rgb/image_raw", 10)
            self._cloud_pub = self.create_publisher(PointCloud2, "zed/points", 10)
            self.create_timer(1.0 / 15.0, self._publish_camera)
            self.get_logger().info(
                f"Listening for stand_g1.py's camera on udp://127.0.0.1:{camera_port}")

        self.get_logger().info(f"Listening for stand_g1.py on udp://127.0.0.1:{port}")

    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            with self._lock:
                self._latest = msg

    def _camera_recv_loop(self) -> None:
        while rclpy.ok():
            try:
                data, _ = self._cam_sock.recvfrom(1 << 20)
            except socket.timeout:
                continue
            except OSError:
                break

            magic = data[:4]
            if magic == CAM_MAGIC_COLOR:
                width, height = struct.unpack_from("<II", data, 4)
                color = np.frombuffer(data, dtype=np.uint8, offset=12).reshape(height, width, 3)
                with self._cam_lock:
                    self._latest_color = color
            elif magic == CAM_MAGIC_DEPTH:
                width, height, fx, fy, cx, cy = struct.unpack_from("<IIffff", data, 4)
                depth = np.frombuffer(data, dtype=np.float32, offset=28).reshape(height, width)
                with self._cam_lock:
                    self._latest_depth = (depth.copy(), fx, fy, cx, cy)

    def _publish(self) -> None:
        with self._lock:
            msg = self._latest
        if msg is None:
            return

        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = msg["joint_names"]
        js.position = msg["joint_pos"]
        self._joint_pub.publish(js)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"
        x, y, z = msg["pelvis_pos"]
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = msg["pelvis_quat_xyzw"]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _publish_camera(self) -> None:
        with self._cam_lock:
            color = self._latest_color
            depth_entry = self._latest_depth
        if color is None and depth_entry is None:
            return

        now = self.get_clock().now().to_msg()

        if color is not None:
            height, width, _ = color.shape
            img = Image()
            img.header.stamp = now
            img.header.frame_id = CAMERA_FRAME_ID
            img.height = height
            img.width = width
            img.encoding = "rgb8"
            img.is_bigendian = 0
            img.step = width * 3
            img.data = color.tobytes()
            self._image_pub.publish(img)

        if depth_entry is not None:
            depth, fx, fy, cx, cy = depth_entry
            height, width = depth.shape
            # Pixel -> 3D in neck_link's own (X-fwd, Y-up, Z-right) convention
            # -- see stand_g1.py's CAMERA_* comment for the derivation. Static
            # TF neck_link->zed_camera_frame (translation-only, see
            # view.launch.py) makes this frame_id valid.
            v, u = np.mgrid[0:height, 0:width]
            valid = depth > 0.0
            x = depth[valid]
            y = (cy - v[valid]) * depth[valid] / fy
            z = (u[valid] - cx) * depth[valid] / fx

            if color is not None and color.shape[:2] == depth.shape:
                rgb = color[valid].astype(np.uint32)
            else:
                rgb = np.full((valid.sum(), 3), 200, dtype=np.uint32)
            packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
            rgb_float = packed.astype(np.uint32).view(np.float32)

            points = np.empty((x.shape[0], 4), dtype=np.float32)
            points[:, 0] = x
            points[:, 1] = y
            points[:, 2] = z
            points[:, 3] = rgb_float

            msg = PointCloud2()
            msg.header.stamp = now
            msg.header.frame_id = CAMERA_FRAME_ID
            msg.height = 1
            msg.width = points.shape[0]
            msg.fields = POINT_CLOUD_FIELDS
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = 16 * points.shape[0]
            msg.is_dense = False
            msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
            self._cloud_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--camera_port", type=int, default=0,
                         help="0 = don't listen for camera data")
    args = parser.parse_args()

    rclpy.init()
    node = RvizBridge(args.port, args.camera_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
