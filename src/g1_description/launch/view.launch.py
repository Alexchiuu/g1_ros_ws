"""Unified RViz viewer: real robot or IsaacGym sim, picked with source:=.

    ros2 launch g1_description view.launch.py source:=sim
    ros2 launch g1_description view.launch.py source:=real

Prefer script/view.sh over calling this directly -- it also sets up the
CycloneDDS/RMW env source:=real needs (and that source:=sim must NOT have:
it pins ROS2 discovery to a specific network interface for talking to the
real robot).

source:=real delegates entirely to g1_state_bridge's own launch file
(robot_state_publisher + the real DDS state bridge + ZED bridge + rviz2 on
display.rviz, Fixed Frame `pelvis` -- the real robot has no global-pose
source to put in `world`).

source:=sim brings up robot_state_publisher + rviz_bridge.py (UDP JSON from
stand_g1.py's IsaacGym process -> JointState + a world->pelvis TF, see that
script's docstring for why it's UDP rather than an in-process rclpy import)
+ rviz2 on isaacgym_live.rviz, Fixed Frame `world` (IsaacGym gives
ground-truth pelvis pose, unlike the real robot).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("g1_description")
    state_bridge_pkg = get_package_share_directory("g1_state_bridge")

    # g1_isaacgym isn't a colcon package (it runs against the dexman_isaacgym
    # conda env, not ROS's Python), so it's never under install/ -- locate it
    # via the workspace root instead of get_package_share_directory(). The
    # default only works when this launch file is run from source
    # (src/g1_description/launch/view.launch.py); script/view.sh passes the
    # real workspace root explicitly so it also works from install/.
    default_ws_root = str(Path(__file__).resolve().parents[3])

    source = LaunchConfiguration("source")
    ws_root = LaunchConfiguration("ws_root")
    port = LaunchConfiguration("port")
    is_real = PythonExpression(["'", source, "' == 'real'"])
    is_sim = PythonExpression(["'", source, "' == 'sim'"])

    robot_description = (Path(pkg) / "urdf" / "g1_tether.urdf").read_text()

    return LaunchDescription([
        DeclareLaunchArgument(
            "source", default_value="sim",
            description="'sim' (IsaacGym, via UDP) or 'real' (robot DDS state)"),
        DeclareLaunchArgument(
            "port", default_value="5555",
            description="UDP port stand_g1.py --ros_bridge_port streams to (source:=sim only)"),
        DeclareLaunchArgument(
            "ws_root", default_value=default_ws_root,
            description="workspace root, to locate g1_isaacgym (source:=sim only, not a colcon package)"),
        # Forwarded to real_state.launch.py for source:=real; see that
        # file's header for what these mean. Harmless when source:=sim.
        DeclareLaunchArgument("neck_yaw_zero_ticks", default_value="2023"),
        DeclareLaunchArgument("neck_pitch_zero_ticks", default_value="3688"),
        DeclareLaunchArgument("neck_yaw_sign", default_value="-1"),
        DeclareLaunchArgument("neck_pitch_sign", default_value="-1"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(state_bridge_pkg) / "launch" / "real_state.launch.py")),
            launch_arguments={
                "neck_yaw_zero_ticks": LaunchConfiguration("neck_yaw_zero_ticks"),
                "neck_pitch_zero_ticks": LaunchConfiguration("neck_pitch_zero_ticks"),
                "neck_yaw_sign": LaunchConfiguration("neck_yaw_sign"),
                "neck_pitch_sign": LaunchConfiguration("neck_pitch_sign"),
            }.items(),
            condition=IfCondition(is_real),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(is_sim),
        ),
        ExecuteProcess(
            cmd=["python3",
                 PathJoinSubstitution([ws_root, "src", "g1_isaacgym", "scripts", "rviz_bridge.py"]),
                 "--port", port],
            output="screen",
            condition=IfCondition(is_sim),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(Path(pkg) / "rviz" / "isaacgym_live.rviz")],
            condition=IfCondition(is_sim),
        ),
    ])
