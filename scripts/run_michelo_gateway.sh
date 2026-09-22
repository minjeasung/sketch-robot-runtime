#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$ROBOT_WORKSPACE/config/sketch_runtime.env" ]]; then
    set -a
    source "$ROBOT_WORKSPACE/config/sketch_runtime.env"
    set +a
fi
if [[ ! -x "$ROBOT_WORKSPACE/.venv-api/bin/python" ]]; then
    echo "Run scripts/setup_system_api.sh first" >&2
    exit 1
fi
export PYTHONPATH="$ROBOT_WORKSPACE/src/sketch_control${PYTHONPATH:+:$PYTHONPATH}"
exec "$ROBOT_WORKSPACE/.venv-api/bin/python" -m sketch_control.michelo_gateway "$@"
