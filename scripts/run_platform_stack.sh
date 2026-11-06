#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform_ros_env.sh"
exec /usr/bin/python3 -u "$SCRIPT_DIR/platform_stack.py" "$@"
