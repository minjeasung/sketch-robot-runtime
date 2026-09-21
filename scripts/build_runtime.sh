#!/usr/bin/env bash
# Isolated build: never source rb10_ws or ros2_ws from the operator's home.
set -eo pipefail
SKETCH_BUILD_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SKETCH_BUILD_JOBS="${SKETCH_BUILD_JOBS:-2}"
SKETCH_BUILD_WITH_ZED=true
if [[ "${1:-}" == --without-zed ]]; then SKETCH_BUILD_WITH_ZED=false; shift; fi
if [[ $# -ne 0 ]]; then echo 'Usage: build_runtime.sh [--without-zed]' >&2; exit 2; fi
if [[ ! "$SKETCH_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then echo 'SKETCH_BUILD_JOBS must be a positive integer' >&2; exit 2; fi
cd "$SKETCH_BUILD_ROOT"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then echo 'Install ROS 2 Jazzy first; see docs/PORTABLE_INSTALL.md' >&2; exit 1; fi
if [[ "$SKETCH_BUILD_WITH_ZED" == true && ! -f /usr/local/zed/zed-config.cmake ]]; then
    echo 'Install ZED SDK 5.3 with its matching CUDA toolkit first, or use --without-zed for fake hardware.' >&2
    exit 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
export CMAKE_BUILD_PARALLEL_LEVEL="$SKETCH_BUILD_JOBS"
SKETCH_FETCH_ARGS=()
if [[ "$SKETCH_BUILD_WITH_ZED" == false ]]; then SKETCH_FETCH_ARGS+=(--without-zed); fi
python3 scripts/fetch_runtime_sources.py "${SKETCH_FETCH_ARGS[@]}"
cmake -S .runtime/sources/rbpodo -B .runtime/sdk/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_EIGEN_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DCMAKE_INSTALL_PREFIX="$SKETCH_BUILD_ROOT/.runtime/sdk/install"
cmake --build .runtime/sdk/build --parallel "$SKETCH_BUILD_JOBS"
cmake --install .runtime/sdk/build
export CMAKE_PREFIX_PATH="$SKETCH_BUILD_ROOT/.runtime/sdk/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
SKETCH_BASE_PATHS=(src)
if [[ "$SKETCH_BUILD_WITH_ZED" == true ]]; then SKETCH_BASE_PATHS+=(.runtime/sources/zed-ros2-wrapper); fi
colcon --log-base .runtime/ros2/log build --symlink-install \
  --build-base .runtime/ros2/build --install-base .runtime/ros2/install \
  --base-paths "${SKETCH_BASE_PATHS[@]}" --parallel-workers "$SKETCH_BUILD_JOBS" \
  --allow-overriding admittance_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
printf '%s\n' "$SKETCH_BUILD_WITH_ZED" > .runtime/with_zed
printf '%s\n' "$SKETCH_BUILD_ROOT" > .runtime/READY
printf 'Portable build complete: %s\n' "$SKETCH_BUILD_ROOT"
