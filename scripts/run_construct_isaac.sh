#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/isaac_env.sh"
[[ -f "$SKETCH_WORKSPACE/isaac_assets/construct.usd" ]] || { echo 'Run bash scripts/setup_construct_isaac.sh first.' >&2; exit 1; }
cd "$SKETCH_WORKSPACE"
if [[ "${CONSTRUCT_ZED:-0}" == 1 ]]; then
    mkdir -p "$SKETCH_WORKSPACE/.runtime/construct"
    export CONSTRUCT_ZED_TIMING_FILE="$SKETCH_WORKSPACE/.runtime/construct/zed-timing.json"
fi
exec "$ISAACSIM_VENV/bin/python" -u "$SCRIPT_DIR/construct_app.py" "$@"
