#!/usr/bin/env bash
# Launch the neck controller GUI -- yaw/pitch sliders for direct, immediate
# control of the physical neck via neck_server.py's ZMQ command socket on
# the robot.
#
# Requires: neck_server.py running on the robot (started by run.sh's neck
# pane -- NOT the old neck.py, which locks/releases torque instead of
# tracking a live goal).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/foxy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 run g1_state_bridge neck_controller_gui
