#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-106}"
source "$SCRIPT_DIR/isaac_env.sh"
export ROS_DOMAIN_ID=106
# ROS LOCALHOST discovery overrides Fast DDS builtins and enables SHM even
# when FASTDDS_BUILTIN_TRANSPORTS requests UDP. Use an explicit loopback profile.
source "$SCRIPT_DIR/rb20_painting_dds_env.sh"
export OMNI_KIT_ACCEPT_EULA=YES
ASSETS="$ROOT/.runtime/rb20_painting/assets"
[[ -f "$ASSETS/manifest.json" ]] || { echo 'Prepare RB20 assets first.' >&2; exit 1; }
OVERLAY="$ROOT/.runtime/rb20_painting/install"
[[ -f "$OVERLAY/isaac_painting_msgs/share/isaac_painting_msgs/local_setup.bash" ]] || { echo 'Run scripts/build_rb20_painting_sim.sh first.' >&2; exit 1; }
# Use Isaac's bundled Jazzy rclpy and native bridge libraries, plus generated messages.
export PYTHONPATH="$OVERLAY/isaac_painting_msgs/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$OVERLAY/isaac_painting_msgs/lib:$LD_LIBRARY_PATH"
cd "$ROOT"
if ! "$ISAACSIM_VENV/bin/python" "$SCRIPT_DIR/prepare_rb20_painting_assets.py" --verify > /dev/null 2>&1; then
    "$ISAACSIM_VENV/bin/python" "$SCRIPT_DIR/isaac_entrypoint.py" isaacsim.exp.base.python --no-window --exec "$SCRIPT_DIR/import_rb20_painting_usd.py"
fi
exec "$ISAACSIM_VENV/bin/python" -u "$SCRIPT_DIR/rb20_painting_app.py" "$@"
