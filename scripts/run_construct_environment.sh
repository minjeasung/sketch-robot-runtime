#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/construct_ros_env.sh"
OPTIONS=()
[[ "${CONSTRUCT_LOOK_AT_BEAM:-1}" == 1 ]] && OPTIONS+=(--look-at-beam)
exec python3 "$ROOT/scripts/publish_construct_environment.py" "${OPTIONS[@]}" "$@"
