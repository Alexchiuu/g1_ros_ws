#!/usr/bin/env bash
# Bring up RViz for either the real robot or the IsaacGym sim:
#
#   ./script/view.sh sim                    # RViz fed by stand_g1.py over UDP
#   ./script/view.sh sim port:=5556         # non-default UDP port
#   ./script/view.sh real                   # RViz fed by the real robot's DDS state
#
# For sim, run the sim itself separately (dexman_isaacgym conda env):
#   ./script/run_isaacgym.sh --viewer --ros_bridge_port 5555
#
# Thin wrapper around `ros2 launch g1_description view.launch.py source:=...`
# that also sets up the real robot's CycloneDDS/RMW env for source=real --
# and deliberately does NOT for source=sim, since that env pins ROS2
# discovery to a specific network interface for talking to real hardware.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

SOURCE="${1:-sim}"
if [[ "$SOURCE" != "real" && "$SOURCE" != "sim" ]]; then
  echo "usage: $0 [real|sim] [extra ros2 launch args, e.g. port:=5556]" >&2
  exit 1
fi
shift || true

source /opt/ros/foxy/setup.bash
source "$WS_DIR/install/setup.bash"

if [[ "$SOURCE" == "real" ]]; then
  source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx001122683161" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
fi

exec ros2 launch g1_description view.launch.py "source:=$SOURCE" "ws_root:=$WS_DIR" "$@"
