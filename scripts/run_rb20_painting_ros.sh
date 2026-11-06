#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPONENT="${1:-all}"
case "$COMPONENT" in
  robot_control|perception|force_pipeline|executor|all) ;;
  *) echo "Invalid RB20 painting component: $COMPONENT" >&2; exit 2 ;;
esac
if (($#)); then shift; fi
source "$ROOT/scripts/rb20_painting_ros_env.sh"
export SKETCH_WORKSPACE="$ROOT"
exec ros2 launch sketch_control rb20_painting_isaac.launch.py \
  "component:=$COMPONENT" \
  "assets:=$ROOT/.runtime/rb20_painting/assets" \
  "readiness_file:=$ROOT/.runtime/rb20_painting/managed-ready.json" "$@"
