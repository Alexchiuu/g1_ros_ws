"""Launch the ZED stream decoder plus a static TF for its mount point.

The camera rides on neck_link (see g1_description/urdf/g1_tether.urdf).
neck_link's axes are rotated +90 deg about X from head_mocap/torso (per the
fixed_neck_link joint), so its local convention is X-forward, Y-up, Z-right
rather than the usual X-forward, Y-left, Z-up. The ZED point cloud is
published in the latter (REP-103 sensor-body) convention, so the mount TF
needs a matching -90 deg roll to reconcile the two -- confirmed visually by
checking that the point cloud (room geometry) lines up with the RGB image
and sits in front of the head rather than floating off at an angle.
Translation is still an identity placeholder -- adjust camera_xyz below once
the real offset from neck_link's origin is measured.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stream_ip = LaunchConfiguration('stream_ip')
    stream_port = LaunchConfiguration('stream_port')
    frame_id = LaunchConfiguration('frame_id')
    depth_mode = LaunchConfiguration('depth_mode')
    camera_xyz = [LaunchConfiguration('camera_x'), LaunchConfiguration('camera_y'),
                  LaunchConfiguration('camera_z')]
    camera_ypr = [LaunchConfiguration('camera_yaw'), LaunchConfiguration('camera_pitch'),
                  LaunchConfiguration('camera_roll')]

    return LaunchDescription([
        DeclareLaunchArgument('stream_ip', default_value='192.168.123.164'),
        DeclareLaunchArgument('stream_port', default_value='30000'),
        DeclareLaunchArgument('frame_id', default_value='zed_camera_frame'),
        DeclareLaunchArgument('depth_mode', default_value='ULTRA'),
        DeclareLaunchArgument('camera_x', default_value='0'),
        DeclareLaunchArgument('camera_y', default_value='0'),
        DeclareLaunchArgument('camera_z', default_value='0'),
        DeclareLaunchArgument('camera_yaw', default_value='0'),
        DeclareLaunchArgument('camera_pitch', default_value='0'),
        DeclareLaunchArgument('camera_roll', default_value='-1.5707963'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_camera_mount_tf',
            arguments=camera_xyz + camera_ypr + ['neck_link', 'zed_camera_frame'],
        ),

        Node(
            package='g1_zed_bridge',
            executable='zed_stream_node',
            name='zed_stream_node',
            output='screen',
            parameters=[{
                'stream_ip': stream_ip,
                'stream_port': stream_port,
                'frame_id': frame_id,
                'depth_mode': depth_mode,
            }],
        ),
    ])
