#!/usr/bin/env bash
# Account-local SDK and TensorRT; no system SDK/udev/driver changes.
CONSTRUCT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ZED_DIR="${CONSTRUCT_ZED_SDK:-$CONSTRUCT_ROOT/../zed-sdk-5.3}"
export ZED_SDK_RESOURCES_DIR="$ZED_DIR"
export LC_ALL=C
export LD_LIBRARY_PATH="$ZED_DIR/lib:$ZED_DIR/dependencies/ai_dependencies/lib:/usr/local/cuda-13.0/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
