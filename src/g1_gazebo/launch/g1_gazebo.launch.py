"""Spawn the G1 tether robot in Gazebo Classic with ros2_control.

Loads g1_tether.gazebo.xacro (the sim overlay on top of g1_description's
plain URDF -- see that file's header comment), starts Gazebo with a simple
ground-plane world, spawns the robot, then brings up joint_state_broadcaster,
three position_controllers/JointGroupPositionController forwarders (body,
left_hand, right_hand -- the same body/hand split g1_state_bridge uses on the
real robot), position_bridge.py (re-sorts /g1/position_command into those
three groups), and overlap_monitor.py (warns in the log when the robot's own
links interpenetrate, since position control doesn't prevent that itself --
see each script's header comment).
"""

import os
import subprocess
import tempfile
from xml.dom import minidom

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_g1_gazebo = get_package_share_directory("g1_gazebo")
    pkg_g1_description = get_package_share_directory("g1_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")

    # gzclient resolves the URDF's package://g1_description/meshes/... visual
    # mesh URIs as model://g1_description/meshes/... against GAZEBO_MODEL_PATH
    # (one level above the "g1_description" dir). sourcing install/setup.bash
    # does NOT set this, so without it gzclient can't find any mesh, spams
    # "No mesh specified", and the GUI never finishes loading (looks hung).
    # gzserver never hits this because physics only needs the URDF's
    # primitive collision shapes, not the visual meshes.
    set_gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.path.dirname(pkg_g1_description) + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
    )

    xacro_path = os.path.join(pkg_g1_description, "urdf", "g1_tether.gazebo.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    # The robot spawned into Gazebo must NOT contain the URDF's world link +
    # floating_base_joint. The URDF->SDF converter drops that joint (SDF can't
    # joint to a link that isn't in the model) but still emits pelvis's pose as
    # relative_to='floating_base_joint' -- a dangling reference that disconnects
    # the entire SDF pose graph (gzserver logs "PoseRelativeToGraph ... [pelvis]
    # is disconnected"). Physics builds the joint tree separately and stays
    # correct, but the visual poses gzclient receives are resolved through that
    # broken graph, fail, and fall back to raw local poses -- every mesh renders
    # collapsed at the model origin. Stripping world/floating_base_joint before
    # conversion keeps the graph rooted at pelvis (free-floating by default),
    # which is what we want in sim anyway.
    spawn_doc = minidom.parseString(robot_description)
    robot_el = spawn_doc.documentElement
    for link in robot_el.getElementsByTagName("link"):
        if link.getAttribute("name") == "world":
            robot_el.removeChild(link)
    for joint in robot_el.getElementsByTagName("joint"):
        if joint.getAttribute("name") == "floating_base_joint":
            robot_el.removeChild(joint)
    stripped_description = spawn_doc.toxml()
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as urdf_file:
        urdf_file.write(stripped_description)
        urdf_path = urdf_file.name
    sdf_text = subprocess.run(
        ["gz", "sdf", "-p", urdf_path], capture_output=True, check=True, text=True
    ).stdout
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as sdf_file:
        sdf_file.write(sdf_text)
        sdf_path = sdf_file.name

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    # robot_state_publisher also gets the STRIPPED description: with the world
    # link present it would publish world->pelvis as a fixed identity TF
    # (floating joints are downgraded to fixed), pinning the robot upright in
    # RViz and conflicting with the ground-truth world->pelvis TF that
    # base_tf_broadcaster.py publishes from the p3d plugin's /g1/base_pose.
    # use_sim_time is deliberately NOT set here (or on rviz/base_tf_broadcaster
    # below): gazebo_ros's /clock publisher has a publisher and subscribers
    # but silently delivers zero messages in this setup (a QoS mismatch, most
    # likely), so any node with use_sim_time:=true gets its clock stuck at
    # t=0 forever while everything else advances on the wall clock -- the
    # resulting cross-node timestamp mismatch is what made RViz report "No
    # transform from [X]" for large parts of the robot. Physics already runs
    # at ~real_time_factor 1.0, so plain wall-clock timestamps everywhere
    # sidesteps the bug with no accuracy cost.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": stripped_description}],
    )

    base_tf_broadcaster = Node(
        package="g1_gazebo",
        executable="base_tf_broadcaster.py",
        name="g1_base_tf_broadcaster",
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", sdf_path, "-entity", "g1", "-z", spawn_z],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["body_controller"],
        output="screen",
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["left_hand_controller"],
        output="screen",
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["right_hand_controller"],
        output="screen",
    )

    position_bridge = Node(
        package="g1_gazebo",
        executable="position_bridge.py",
        name="g1_position_bridge",
        output="screen",
    )

    overlap_monitor = Node(
        package="g1_gazebo",
        executable="overlap_monitor.py",
        name="g1_overlap_monitor",
        output="screen",
    )

    # Sim-side RViz. sim.rviz is display.rviz with fixed frame "world" so the
    # floating-base attitude (from base_tf_broadcaster) is visible -- don't
    # confuse this window with a real-robot RViz on another ROS domain.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_sim",
        output="screen",
        arguments=["-d", os.path.join(pkg_g1_gazebo, "rviz", "sim.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Slider GUI for commanding the sim. joint_state_publisher_gui's output
    # (sensor_msgs/JointState on "joint_states") is remapped to
    # /g1/position_command: in sim, Gazebo owns /joint_states (via
    # joint_state_broadcaster), so the sliders must be commands, not state --
    # position_bridge re-sorts them into each controller's commands topic.
    command_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="g1_command_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "/g1/position_command")],
        condition=IfCondition(LaunchConfiguration("command_gui")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_g1_gazebo, "worlds", "g1_world.world"),
            description="Path to the Gazebo world file to load"),
        DeclareLaunchArgument(
            "spawn_z", default_value="0.85",
            description="Spawn height (m); leg joints rest ~0.75m above the foot sole at zero pose"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start a sim-fed RViz alongside Gazebo"),
        DeclareLaunchArgument(
            "command_gui", default_value="true",
            description="Start the joint slider GUI (publishes /g1/position_command)"),

        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        base_tf_broadcaster,
        spawn_entity,

        # Controllers can only be spawned once gazebo_ros2_control's
        # controller_manager exists in the running gzserver, which happens
        # when spawn_entity loads the robot's <ros2_control> plugin.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    body_controller_spawner,
                    left_hand_controller_spawner,
                    right_hand_controller_spawner,
                    position_bridge,
                    overlap_monitor,
                    rviz,
                    command_gui,
                ],
            )
        ),
    ])
