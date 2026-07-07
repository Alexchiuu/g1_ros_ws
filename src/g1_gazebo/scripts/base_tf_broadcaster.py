#!/usr/bin/env python3
"""Broadcast the sim robot's floating-base pose as TF world->pelvis.

The gazebo overlay's p3d_pelvis plugin (see g1_tether.gazebo.xacro) publishes
ground-truth pelvis odometry on /g1/base_pose -- the sim's stand-in for the
real robot's torso IMU. robot_state_publisher can't provide this transform:
the launch feeds it the URDF with world/floating_base_joint stripped (see
g1_gazebo.launch.py), so its TF tree is rooted at pelvis and this node owns
the world->pelvis edge. With it, RViz (fixed frame "world") shows the robot's
actual attitude -- falling over, lying down -- instead of a pelvis-locked
upright view.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BaseTfBroadcaster(Node):
    def __init__(self):
        super().__init__("g1_base_tf_broadcaster")
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/g1/base_pose", self._on_pose, 10)
        self.get_logger().info("broadcasting /g1/base_pose -> TF world->pelvis")

    def _on_pose(self, msg: Odometry):
        # Deliberately re-stamped with this node's own clock rather than
        # forwarding msg.header.stamp (gazebo sim time): nothing else in this
        # launch runs on sim time (see g1_gazebo.launch.py's use_sim_time
        # comment), so keeping this transform on the same wall clock as
        # robot_state_publisher's is what keeps RViz's TF lookups resolving.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = BaseTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
