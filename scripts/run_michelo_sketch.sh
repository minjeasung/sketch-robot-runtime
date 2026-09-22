#!/usr/bin/env bash
# One terminal, two web services. Camera/robot processes are never auto-started.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_API_PID=
SKETCH_GATEWAY_PID=
cleanup() {
    trap - EXIT INT TERM
    for pid in "$SKETCH_API_PID" "$SKETCH_GATEWAY_PID"; do
        if [[ -n "$pid" ]]; then kill -TERM "$pid" 2>/dev/null || true; fi
    done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash "$SCRIPT_DIR/run_system_api.sh" &
SKETCH_API_PID=$!
bash "$SCRIPT_DIR/run_michelo_gateway.sh" &
SKETCH_GATEWAY_PID=$!
echo 'Michelo + Sketch web services started. Default console: http://127.0.0.1:8101/console/'
echo 'Default system manager: http://127.0.0.1:8081/ — no robot motion is started automatically.'
wait -n "$SKETCH_API_PID" "$SKETCH_GATEWAY_PID"
