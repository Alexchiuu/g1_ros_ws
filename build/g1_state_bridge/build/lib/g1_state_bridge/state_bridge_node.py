import struct
import threading
import time
import traceback

import rclpy
import zmq
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

# Index order matches unitree_hg LowState.motor_state[0:29] for the G1
# 7-DOF-arm variant (G1Arm7JointIndex in the Unitree SDK), which lines up
# 1:1 with the revolute joints in g1_tether.urdf.
JOINT_NAMES = [
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
]

# These hands are third-party "Aero" hands wired to the G1's onboard Jetson
# over USB-serial, not Unitree Dex3 -- there is no unitree_hg DDS telemetry
# for them. The vendor's aero_hand_relay.py script (run on the Jetson) reads
# the two hands and republishes over a ZMQ PUB socket as a flat "full16"
# array per hand, in this exact order (see compact7_deg_to_full16_rad in
# aero_hand_relay.py): thumb has 4 slots (abd, flex, mcp, ip-coupled-to-mcp),
# then index/middle/ring/pinky each get 3 identical slots (mcp_flex, pip,
# dip) because a single motor drives that whole finger as one rigid unit.
AERO_FULL16_JOINT_SUFFIXES = [
    "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_pip", "pinky_dip",
]
AERO_NUM_HANDS = 2
AERO_JOINTS_PER_HAND = len(AERO_FULL16_JOINT_SUFFIXES)
AERO_TOTAL_FLOATS = AERO_NUM_HANDS * AERO_JOINTS_PER_HAND
# valid(bool) + seq(uint64) + timestamp(double) + q[32](double) + dq[32](double)
AERO_STATE_FMT = f"<BQd{AERO_TOTAL_FLOATS * 2}d"
AERO_STATE_NBYTES = struct.calcsize(AERO_STATE_FMT)

_FINGER_JOINTS = ["mcp_flex", "pip", "dip"]
_THUMB_JOINTS = ["thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip"]
HAND_JOINT_NAMES = [
    f"{side}_{finger}_{joint}"
    for side in ("left", "right")
    for finger in ("index", "middle", "ring", "pinky")
    for joint in _FINGER_JOINTS
] + [
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in _THUMB_JOINTS
]


class G1StateBridge(Node):
    def __init__(self):
        super().__init__("g1_state_bridge")
        self.declare_parameter("lowstate_topic", "lowstate")
        topic = self.get_parameter("lowstate_topic").value

        self.declare_parameter("aero_hand_host", "192.168.123.164")
        self.declare_parameter("aero_hand_state_port", 5556)
        hand_host = self.get_parameter("aero_hand_host").value
        hand_port = self.get_parameter("aero_hand_state_port").value

        self._hand_q = {name: 0.0 for name in HAND_JOINT_NAMES}
        self._hand_q_lock = threading.Lock()

        self.pub_ = self.create_publisher(JointState, "joint_states", 10)
        self.sub_ = self.create_subscription(LowState, topic, self.on_low_state, 10)

        self._hand_stop = threading.Event()
        self._hand_thread = threading.Thread(
            target=self._aero_hand_loop, args=(hand_host, hand_port), daemon=True)
        self._hand_thread.start()

        self.get_logger().info(
            f"Bridging {topic} -> joint_states for {len(JOINT_NAMES)} body joints "
            f"+ tcp://{hand_host}:{hand_port} (aero_hand_relay) for "
            f"{AERO_JOINTS_PER_HAND} hand joints/side")

    def destroy_node(self):
        self._hand_stop.set()
        self._hand_thread.join(timeout=1.0)
        super().destroy_node()

    def _aero_hand_loop(self, host: str, port: int):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{host}:{port}")
        self.get_logger().info(f"aero hand ZMQ SUB thread connecting to tcp://{host}:{port}")
        n_msgs = 0
        last_report = time.monotonic()
        try:
            while not self._hand_stop.is_set():
                try:
                    payload = sock.recv()
                except zmq.Again:
                    continue
                except Exception:
                    self.get_logger().error(
                        "aero hand ZMQ recv() failed:\n" + traceback.format_exc())
                    continue
                try:
                    if len(payload) != AERO_STATE_NBYTES:
                        self.get_logger().warn(
                            f"aero_hand_relay state size mismatch: "
                            f"expected {AERO_STATE_NBYTES} got {len(payload)}")
                        continue
                    fields = struct.unpack(AERO_STATE_FMT, payload)
                    q = fields[3:3 + AERO_TOTAL_FLOATS]
                    with self._hand_q_lock:
                        for side, hand_q in zip(("left", "right"),
                                                 (q[:AERO_JOINTS_PER_HAND], q[AERO_JOINTS_PER_HAND:])):
                            for suffix, value in zip(AERO_FULL16_JOINT_SUFFIXES, hand_q):
                                self._hand_q[f"{side}_{suffix}"] = value
                    n_msgs += 1
                except Exception:
                    self.get_logger().error(
                        "aero hand ZMQ frame processing failed:\n" + traceback.format_exc())
                    continue
                now = time.monotonic()
                if now - last_report > 2.0:
                    self.get_logger().info(f"aero hand ZMQ thread: {n_msgs} frames processed so far")
                    last_report = now
        finally:
            self.get_logger().warn("aero hand ZMQ SUB thread exiting")
            sock.close()

    def on_low_state(self, msg: LowState):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES + HAND_JOINT_NAMES
        with self._hand_q_lock:
            hand_positions = [self._hand_q[name] for name in HAND_JOINT_NAMES]
        js.position = ([msg.motor_state[i].q for i in range(len(JOINT_NAMES))]
                        + hand_positions)
        js.velocity = [msg.motor_state[i].dq for i in range(len(JOINT_NAMES))] + [0.0] * len(HAND_JOINT_NAMES)
        js.effort = [msg.motor_state[i].tau_est for i in range(len(JOINT_NAMES))] + [0.0] * len(HAND_JOINT_NAMES)
        self.pub_.publish(js)


def main():
    rclpy.init()
    node = G1StateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
