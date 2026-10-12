#!/usr/bin/env bash
# Deliberately offline: no ROS environment sourcing, services or robot control.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

for required in git node python3; do
  if ! command -v "$required" >/dev/null 2>&1; then
    echo "Missing prerequisite: $required" >&2
    exit 2
  fi
done

if ! python3 -c 'import pytest' >/dev/null 2>&1; then
  echo "Missing Python prerequisite: pytest (install it in your test environment)" >&2
  exit 2
fi

echo "== Offline plane-version tests =="
python3 -m pytest -q scripts/tests/test_plane_versions.py

echo "== Offline web UI tests =="
node --test web/test/*.test.js

echo "Offline subset finished. ROS, safety integration and physical robot were NOT tested."

