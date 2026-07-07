#!/usr/bin/env python3
"""Pure passthrough from /g1/position_command to the position controllers.

No PID, no gains, no effort clamping: gazebo_ros2_control's GazeboSystem
drives the "position" command interface via Joint::SetPosition(), which
kinematically sets the joint straight to the commanded angle every step
regardless of contact/gravity resistance -- i.e. "infinite torque, mass and
contact physics still apply to everything else." This replaced an earlier
effort/PID-based bridge that existed only because SetPosition() looked
broken on this robot; that turned out to be a disconnected-SDF-pose-graph
bug elsewhere (see g1_gazebo.launch.py), not a real SetPosition() limitation.

This node just re-sorts /g1/position_command (sensor_msgs/JointState, partial
updates fine) into the three groups' fixed joint order and republishes as
std_msgs/Float64MultiArray on each <controller>/commands topic at 200 Hz.
Joints that have never been commanded hold their last known /joint_states
position (not 0), so nothing snaps on startup before a command arrives.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

BODY_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "neck_yaw_joint", "neck_pitch_joint",
]
_FINGER_JOINTS = ["mcp_flex", "pip", "dip"]
_THUMB_JOINTS = ["thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip"]
LEFT_HAND_JOINTS = (
    [f"left_{finger}_{joint}" for finger in ("index", "middle", "ring", "pinky")
     for joint in _FINGER_JOINTS]
    + [f"left_{joint}" for joint in _THUMB_JOINTS]
)
RIGHT_HAND_JOINTS = [j.replace("left_", "right_", 1) for j in LEFT_HAND_JOINTS]

GROUPS = {
    "body_controller": BODY_JOINTS,
    "left_hand_controller": LEFT_HAND_JOINTS,
    "right_hand_controller": RIGHT_HAND_JOINTS,
}


class PositionBridge(Node):
    def __init__(self):
        super().__init__("g1_position_bridge")
        self._pos = {}
        self._target = {}
        self._pubs = {
            name: self.create_publisher(Float64MultiArray, f"/{name}/commands", 10)
            for name in GROUPS
        }
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(JointState, "/g1/position_command", self._on_command, 10)
        self.create_timer(1.0 / 200.0, self._on_timer)
        self.get_logger().info(
            f"Position bridge up: {sum(len(j) for j in GROUPS.values())} joints across "
            f"{len(GROUPS)} groups, listening on /g1/position_command")

    def _on_state(self, msg):
        for i, name in enumerate(msg.name):
            self._pos[name] = msg.position[i]
            self._target.setdefault(name, self._pos[name])

    def _on_command(self, msg):
        for i, name in enumerate(msg.name):
            self._target[name] = msg.position[i]

    def _on_timer(self):
        for name, joints in GROUPS.items():
            out = Float64MultiArray()
            out.data = [self._target.get(j, 0.0) for j in joints]
            self._pubs[name].publish(out)


def main():
    rclpy.init()
    node = PositionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
