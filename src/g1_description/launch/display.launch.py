"""Launch RViz2 with robot_state_publisher for the G1 tether description."""

from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("g1_description")

    use_xacro = LaunchConfiguration("use_xacro", default="false")

    # Process xacro → URDF string at launch time when use_xacro is true.
    # For simplicity we default to the pre-generated URDF.
    urdf_path = Path(pkg) / "urdf" / "g1_tether.urdf"
    robot_description = urdf_path.read_text()

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_xacro", default_value="false",
            description="Set true to process the .urdf.xacro file at launch"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
