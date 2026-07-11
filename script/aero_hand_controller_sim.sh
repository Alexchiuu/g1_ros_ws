#!/usr/bin/env bash
# Launch the Aero hand controller GUI against the IsaacGym sim instead of
# the real robot -- the exact same GUI (g1_state_bridge's hand_controller_gui),
# just pointed at 127.0.0.1. Works because that GUI talks to whatever's
# listening on a documented ZMQ wire protocol, and stand_g1.py --teleop_gui
# implements that same protocol (see ZmqTeleopLink in that script).
#
# Requires stand_g1.py running with --teleop_gui, e.g.:
#   ./script/run_isaacgym.sh --viewer --teleop_gui
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/foxy/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 run g1_state_bridge hand_controller_gui --host 127.0.0.1
