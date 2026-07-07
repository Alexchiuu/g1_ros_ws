"""Decode the G1's ZED Mini H.264 stream and publish it to ROS 2.

calex/deploy/zed.py opens the ZED on the robot with depth disabled and just
enables the SDK's built-in H.264 network streaming (see that file's
docstring). It does not talk to ROS at all. This node is the receiving end:
it connects to that stream with the ZED SDK (sl.InitParameters.set_from_stream),
computes depth locally, and republishes sensor_msgs/Image + PointCloud2 so
RViz can show the color feed and point cloud.
"""
import numpy as np
import pyzed.sl as sl
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField

DEPTH_MODES = {
    'NONE': sl.DEPTH_MODE.NONE,
    'PERFORMANCE': sl.DEPTH_MODE.PERFORMANCE,
    'QUALITY': sl.DEPTH_MODE.QUALITY,
    'ULTRA': sl.DEPTH_MODE.ULTRA,
    'NEURAL': sl.DEPTH_MODE.NEURAL,
}

POINT_CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class ZedStreamNode(Node):

    def __init__(self):
        super().__init__('zed_stream_node')

        self.declare_parameter('stream_ip', '192.168.123.164')
        self.declare_parameter('stream_port', 30000)
        self.declare_parameter('frame_id', 'zed_camera_frame')
        self.declare_parameter('depth_mode', 'ULTRA')

        self._stream_ip = self.get_parameter('stream_ip').value
        self._stream_port = self.get_parameter('stream_port').value
        self._frame_id = self.get_parameter('frame_id').value
        depth_mode_name = self.get_parameter('depth_mode').value.upper()
        if depth_mode_name not in DEPTH_MODES:
            self.get_logger().warn(
                f"Unknown depth_mode '{depth_mode_name}', falling back to ULTRA")
            depth_mode_name = 'ULTRA'
        self._depth_mode = DEPTH_MODES[depth_mode_name]

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._image_pub = self.create_publisher(Image, 'zed/rgb/image_raw', sensor_qos)
        self._cloud_pub = self.create_publisher(PointCloud2, 'zed/points', sensor_qos)

        self._bridge = CvBridge()
        self._cam = sl.Camera()
        self._runtime = sl.RuntimeParameters()
        self._image_mat = sl.Mat()
        self._cloud_mat = sl.Mat()
        self._opened = False

    def _try_open(self) -> bool:
        init = sl.InitParameters()
        init.set_from_stream(self._stream_ip, self._stream_port)
        init.depth_mode = self._depth_mode
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        status = self._cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(
                f'Waiting for ZED stream at {self._stream_ip}:{self._stream_port} '
                f'({status}). Is calex/deploy/run.sh running on the robot?',
                throttle_duration_sec=5.0)
            return False
        self.get_logger().info(
            f'Connected to ZED stream at {self._stream_ip}:{self._stream_port}')
        return True

    def spin(self):
        while rclpy.ok():
            if not self._opened:
                self._opened = self._try_open()
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            status = self._cam.grab(self._runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f'grab() failed: {status}, reconnecting',
                                        throttle_duration_sec=5.0)
                self._cam.close()
                self._opened = False
                continue

            stamp = self.get_clock().now().to_msg()

            self._cam.retrieve_image(self._image_mat, sl.VIEW.LEFT)
            image_msg = self._bridge.cv2_to_imgmsg(self._image_mat.get_data(), encoding='bgra8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self._frame_id
            self._image_pub.publish(image_msg)

            self._cam.retrieve_measure(self._cloud_mat, sl.MEASURE.XYZRGBA)
            self._cloud_pub.publish(self._make_cloud_msg(self._cloud_mat.get_data(), stamp))

            rclpy.spin_once(self, timeout_sec=0.0)

        if self._opened:
            self._cam.close()

    def _make_cloud_msg(self, xyzrgba: np.ndarray, stamp) -> PointCloud2:
        height, width, _ = xyzrgba.shape
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = height
        msg.width = width
        msg.fields = POINT_CLOUD_FIELDS
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * width
        msg.is_dense = False
        msg.data = np.ascontiguousarray(xyzrgba, dtype=np.float32).tobytes()
        return msg


def main():
    rclpy.init()
    node = ZedStreamNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
