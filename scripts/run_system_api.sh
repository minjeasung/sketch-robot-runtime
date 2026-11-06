#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ "${1:-}" == "--construct" ]]; then
    shift
    exec bash "$SCRIPT_DIR/run_construct_system.sh" "$@"
fi
RB20_PAINT_API=false
if [[ "${1:-}" == "--rb20-paint" ]]; then
    shift
    RB20_PAINT_API=true
fi
if [[ -f "$ROBOT_WORKSPACE/config/sketch_runtime.env" ]]; then
    set -a
    source "$ROBOT_WORKSPACE/config/sketch_runtime.env"
    set +a
fi
if [[ "$RB20_PAINT_API" == true ]]; then
    source "$ROBOT_WORKSPACE/scripts/rb20_painting_ros_env.sh"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-106}"
    export SKETCH_PROFILE=isaac_rb20_paint
    export SKETCH_MODEL_ID=rb20_1900es
    export SKETCH_PROCESS_MODE=paint
    export SKETCH_CAMERA_BACKEND=native
    export SKETCH_LAUNCH_ZED_DRIVER=false SKETCH_LAUNCH_D405_DRIVER=false SKETCH_LAUNCH_RVIZ=false
    export SKETCH_OUTPOST_ZED_HW_ID= SKETCH_OUTPOST_ZED_SERIAL= SKETCH_OUTPOST_D405_HW_ID= SKETCH_OUTPOST_D405_SERIAL=
    export SKETCH_SPRAY_EOAT_PROFILE=
else
    source "$ROBOT_WORKSPACE/scripts/ros_env.sh"
fi
if [[ ! -x "$ROBOT_WORKSPACE/.venv-api/bin/python" ]]; then
    echo "Run scripts/setup_system_api.sh first" >&2
    exit 1
fi
export PYTHONPATH="$ROBOT_WORKSPACE/src/sketch_control${PYTHONPATH:+:$PYTHONPATH}"
exec "$ROBOT_WORKSPACE/.venv-api/bin/python" -m sketch_control.system_api --workspace "$ROBOT_WORKSPACE" "$@"
