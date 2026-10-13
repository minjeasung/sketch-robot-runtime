#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/construct_ros_env.sh"
export ROS_DOMAIN_ID=105
cd "$ROOT"
export SKETCH_PROFILE=isaac_construct
export SKETCH_MODEL_ID="${SKETCH_MODEL_ID:-construct_left}"
export SKETCH_PROCESS_MODE=spray
exec "$ROOT/.venv-api/bin/python" -m sketch_control.system_api --workspace "$ROOT" "$@"
