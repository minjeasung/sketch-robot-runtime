#!/usr/bin/env bash
set -eo pipefail
SKETCH_CHECK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$SKETCH_CHECK_ROOT/config/sketch_runtime.env" ]]; then
    set -a; source "$SKETCH_CHECK_ROOT/config/sketch_runtime.env"; set +a
fi
source "$SKETCH_CHECK_ROOT/scripts/ros_env.sh"
exec "$SKETCH_CHECK_ROOT/.venv-api/bin/python" "$SKETCH_CHECK_ROOT/scripts/check_runtime.py" "$@"
