#!/usr/bin/env python3
"""UDP -> ROS2 bridge for the live IsaacGym sim.

stand_g1.py (run under the dexman_isaacgym conda env, see that script's
--ros_bridge_port) streams joint positions + pelvis pose as UDP JSON. This
node republishes that as sensor_msgs/JointState on /joint_states and
broadcasts a world->pelvis TF, so RViz can show the same motion the IsaacGym
viewer window shows. UDP is used instead of importing rclpy directly in
stand_g1.py's process to keep the conda env's own libstdc++/CUDA/etc. out of
the ROS process entirely.

Run under system ROS, NOT the dexman_isaacgym conda env:

    source /opt/ros/foxy/setup.bash
    source ~/g1_ros_ws/install/setup.bash
    python3 src/g1_isaacgym/scripts/rviz_bridge.py --port 5555
"""

from __future__ import annotations

import argparse
import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RvizBridge(Node):
    def __init__(self, port: int):
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5555)
    args = parser.parse_args()

    rclpy.init()
    node = RvizBridge(args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
