#!/usr/bin/env bash
# Launch the Aero hand controller GUI -- 7 sliders/hand (one per real motor)
# for direct, immediate control of the physical hands via aero_hand_relay's
# ZMQ command socket on the robot.
#
# Requires: aero_hand_relay.py already running on the robot
# (unitree@192.168.123.164:~/khtu/aero_hands).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/foxy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 run g1_state_bridge hand_controller_gui
