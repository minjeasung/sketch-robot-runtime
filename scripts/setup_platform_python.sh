#!/usr/bin/env bash
# Install only platform-specific pure Python dependencies for system ROS Python.
set -eo pipefail
PLATFORM_PYTHON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"$PLATFORM_PYTHON_ROOT/.venv-api/bin/python" -m pip install --no-deps --upgrade \
    --target "$PLATFORM_PYTHON_ROOT/.runtime/platform/python" \
    -r "$PLATFORM_PYTHON_ROOT/config/platform_ros.requirements.txt"
PYTHONPATH="$PLATFORM_PYTHON_ROOT/.runtime/platform/python" PYTHONNOUSERSITE=1 \
    /usr/bin/python3 -c 'import numpy, scipy, trimesh, collada; print("ROS mesh decoder:", trimesh.__version__, "COLLADA:", collada.__version__)'
