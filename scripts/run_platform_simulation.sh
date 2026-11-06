#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/isaac_env.sh"
source "$SCRIPT_DIR/rb20_painting_dds_env.sh"
export OMNI_KIT_ACCEPT_EULA=YES
OVERLAY="$SKETCH_WORKSPACE/.runtime/platform/install"
export PYTHONPATH="$OVERLAY/isaac_painting_msgs/lib/python3.12/site-packages:$SKETCH_WORKSPACE/src/sketch_control:$SKETCH_WORKSPACE/src/rbpodo_painting_control:$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$OVERLAY/isaac_painting_msgs/lib:$LD_LIBRARY_PATH"
exec "$ISAACSIM_VENV/bin/python" -u "$SCRIPT_DIR/platform_simulation_app.py" "$@"
