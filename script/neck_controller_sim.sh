#!/usr/bin/env bash
# Launch the neck controller GUI against the IsaacGym sim instead of the
# real robot -- the exact same GUI (g1_state_bridge's neck_controller_gui),
# just pointed at 127.0.0.1 with zero-ticks/sign reset to the sim's own
# convention (there's no real encoder here to calibrate against -- see
# stand_g1.py's --teleop_gui docstring).
#
# Requires stand_g1.py running with --teleop_gui, e.g.:
#   ./script/run_isaacgym.sh --viewer --teleop_gui
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/foxy/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 run g1_state_bridge neck_controller_gui --host 127.0.0.1 \
    --yaw-zero-ticks 0 --pitch-zero-ticks 0 --yaw-sign 1 --pitch-sign 1
