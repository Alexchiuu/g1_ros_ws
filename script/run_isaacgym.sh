#!/usr/bin/env bash
# Run the G1 IsaacGym standing sim (src/g1_isaacgym/scripts/stand_g1.py):
# activates the dexman_isaacgym conda env for you and forwards all args.
#
# Examples:
#   ./script/run_isaacgym.sh                                    # headless, 12s
#   ./script/run_isaacgym.sh --viewer --duration 120             # interactive window
#   ./script/run_isaacgym.sh --headless --video /tmp/g1_stand.mp4
#   ./script/run_isaacgym.sh --viewer --ros_bridge_port 5555     # + feed RViz
#
# For the last one, also bring up RViz separately (system ROS, not this
# conda env): ./script/view.sh sim
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate dexman_isaacgym

cd "$WS_DIR"
exec python src/g1_isaacgym/scripts/stand_g1.py "$@"
