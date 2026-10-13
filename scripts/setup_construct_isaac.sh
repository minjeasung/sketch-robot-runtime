#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"
python3 scripts/prepare_construct_assets.py
source "$SCRIPT_DIR/isaac_env.sh"
exec "$ISAACSIM_VENV/bin/python" "$SCRIPT_DIR/isaac_entrypoint.py" isaacsim.exp.base.python --no-window --exec "$SCRIPT_DIR/import_construct_usd.py"
