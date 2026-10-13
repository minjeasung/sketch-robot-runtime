#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/construct_ros_env.sh"
python3 "$SCRIPT_DIR/wait_construct_sim.py"
exec ros2 launch "$SCRIPT_DIR/construct_moveit.launch.py" "$@"
