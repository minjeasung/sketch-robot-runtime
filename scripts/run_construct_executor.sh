#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/construct_ros_env.sh"
MODEL="${1:-construct_left}"
case "$MODEL" in construct_left|construct_right) ;; *) echo 'Unknown Construct arm' >&2; exit 1;; esac
exec ros2 launch "$ROOT/scripts/construct_executor.launch.py" "model_id:=$MODEL" "process_mode:=${2:-spray}" "weld_input_provider:=${WELD_INPUT_PROVIDER:-fixture}"
