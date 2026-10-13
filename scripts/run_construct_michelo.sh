#!/usr/bin/env bash
# Run the delivered official backend; profile data stays outside its source tree.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND="${CONSTRUCT_MICHELO_ROOT:-$ROOT/../official-michelo-builder/backend}"
unset PYTHONPATH LD_LIBRARY_PATH
export MICHELO_DB_ROOT="$ROOT/.runtime/construct/michelo-profiles"
export MICHELO_DEV_MODE=1
export PYTHONNOUSERSITE=1
# Camera relay needs no GPU; reserve it for Isaac/Outpost. Opt in explicitly for
# Michelo's separate GPU vision workloads and their automatic provisioning.
export CUDA_VISIBLE_DEVICES="${CONSTRUCT_MICHELO_CUDA_DEVICES:-}"
[[ -f "$MICHELO_DB_ROOT/db_init/hw_interface/hw_interfaces.json" ]] || {
    echo 'Missing local profile. Run scripts/setup_construct_official.py first.' >&2
    exit 1
}
cd "$BACKEND"
exec "$BACKEND/.venv/bin/test-camera" --no-browser --host 127.0.0.1 --port 8101 "$@"
