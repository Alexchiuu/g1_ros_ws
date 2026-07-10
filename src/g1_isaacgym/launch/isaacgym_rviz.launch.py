"""RViz fed by stand_g1.py's live IsaacGym sim (see rviz_bridge.py).

g1_isaacgym isn't a colcon package, so launch this by filesystem path rather
than package name:

    ros2 launch src/g1_isaacgym/launch/isaacgym_rviz.launch.py

Then, separately, in the dexman_isaacgym conda env:

    conda activate dexman_isaacgym
    python src/g1_isaacgym/scripts/stand_g1.py --viewer --ros_bridge_port 5555
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

THIS_DIR = Path(__file__).resolve().parent


def generate_launch_description():
    pkg = get_package_share_directory("g1_description")

    # Full (non-stripped) URDF: robot_state_publisher downgrades the
    # world/floating_base_joint to an identity-fixed world->pelvis TF (it
    # can't represent a 6-DOF floating joint), which rviz_bridge.py's TF
    # broadcast then overrides with the sim's real, moving pelvis pose --
    # same split g1_gazebo's base_tf_broadcaster.py used to do.
    robot_description = (Path(pkg) / "urdf" / "g1_tether.urdf").read_text()

    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="5555",
            description="UDP port stand_g1.py --ros_bridge_port streams to"),

        ExecuteProcess(
            cmd=["python3", str(THIS_DIR.parent / "scripts" / "rviz_bridge.py"), "--port", port],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            # isaacgym_live.rviz is display.rviz with Fixed Frame changed to
            # "world" (display.rviz uses "pelvis", fine for the static
            # description viewer but it'd hide the live sim's floating-base
            # motion by re-centering on the moving pelvis every frame).
            arguments=["-d", str(THIS_DIR.parent / "rviz" / "isaacgym_live.rviz")],
        ),
    ])
