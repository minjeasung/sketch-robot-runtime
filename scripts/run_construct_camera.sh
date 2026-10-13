#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/construct_zed_env.sh"
OUTPOST="${CONSTRUCT_OUTPOST_ROOT:-$ROOT/../official-outpost}"
export OUTPOST_BIND=127.0.0.1
export OUTPOST_PORT=8100
export OUTPOST_IPC_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/construct-outpost"
mkdir -p -m 700 "$OUTPOST_IPC_DIR"
cd "$OUTPOST"
exec "$OUTPOST/.venv/bin/michelo-outpost" "$@"
