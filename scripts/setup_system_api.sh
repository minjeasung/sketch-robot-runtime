#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
/usr/bin/python3 -m venv --system-site-packages "$ROBOT_WORKSPACE/.venv-api"
"$ROBOT_WORKSPACE/.venv-api/bin/python" -m pip install -r "$ROBOT_WORKSPACE/config/system_api.requirements.txt"
echo "Ready. Start with: $ROBOT_WORKSPACE/scripts/run_system_api.sh"
