#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/construct_ros_env.sh"
MODEL="${1:-construct_left}"
case "$MODEL" in construct_left|construct_right) ;; *) echo 'Unknown Construct arm' >&2; exit 1;; esac
CAMERA_ID="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["hw_id"])' "$ROOT/.runtime/construct/zed-camera.json")"
exec ros2 launch "$ROOT/scripts/construct_perception.launch.py" \
    "model_id:=$MODEL" "process_mode:=${2:-spray}" "weld_input_provider:=${WELD_INPUT_PROVIDER:-fixture}" "outpost_zed_hw_id:=$CAMERA_ID" \
    "spray_eoat_profile:=$ROOT/isaac_assets/${MODEL}_eoat.json" \
    spray_tool_axis:=+z spray_standoff_m:=0.3 spray_footprint_width_m:=0.05
