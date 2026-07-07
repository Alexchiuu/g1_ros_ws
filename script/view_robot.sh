#!/usr/bin/env bash
# Launch RViz2 fed by the real G1's live state: robot_state_publisher +
# g1_state_bridge (body /lowstate + Aero hand ZMQ relay) + rviz2.
#
# Requires: aero_hand_relay.py already running on the robot
# (unitree@192.168.123.164:~/khtu/aero_hands) for live hand data -- body
# joints will show regardless.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/foxy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx001122683161" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

exec ros2 launch g1_state_bridge real_state.launch.py
