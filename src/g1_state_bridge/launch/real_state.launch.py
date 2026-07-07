"""Launch RViz2 + robot_state_publisher fed by the real G1's DDS state."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory("g1_description")
    urdf_path = Path(desc_pkg) / "urdf" / "g1_tether.urdf"
    robot_description = urdf_path.read_text()
    zed_bridge_pkg = get_package_share_directory("g1_zed_bridge")

    return LaunchDescription([
        # Raw Dynamixel ticks treated as the neck's zero pose -- defaults to
        # wherever neck.py's calibration lock left it as of 2026-07-06 (the
        # same pose the zed_camera_frame mount TF was measured against), so
        # out of the box RViz's neck + camera line up. Re-lock the neck to a
        # new pose with neck.py and update these to match (read the raw
        # ticks it prints, e.g. "ID1: 2023").
        DeclareLaunchArgument("neck_yaw_zero_ticks", default_value="2023"),
        DeclareLaunchArgument("neck_pitch_zero_ticks", default_value="3688"),
        # Confirmed 2026-07-06 by physically moving the neck: the raw-tick
        # -> radians mapping guessed when the joints were added had both
        # axes backwards relative to what RViz should show, so both signs
        # are flipped from the physical hardware's native tick direction.
        DeclareLaunchArgument("neck_yaw_sign", default_value="-1"),
        DeclareLaunchArgument("neck_pitch_sign", default_value="-1"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="g1_state_bridge",
            executable="state_bridge_node",
            name="g1_state_bridge",
            output="screen",
            parameters=[{
                "neck_yaw_zero_ticks": LaunchConfiguration("neck_yaw_zero_ticks"),
                "neck_pitch_zero_ticks": LaunchConfiguration("neck_pitch_zero_ticks"),
                "neck_yaw_sign": LaunchConfiguration("neck_yaw_sign"),
                "neck_pitch_sign": LaunchConfiguration("neck_pitch_sign"),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(zed_bridge_pkg) / "launch" / "zed_bridge.launch.py")
            ),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(Path(desc_pkg) / "rviz" / "display.rviz")],
        ),
    ])
