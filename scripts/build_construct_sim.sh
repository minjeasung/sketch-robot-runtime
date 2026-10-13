#!/usr/bin/env bash
# Build the original sketch runtime with Construct data and simulation topic hardware.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
PLUGIN_DIR="$ROOT/.runtime/sources/topic_based_ros2_control"
PLUGIN_COMMIT=6bd8d55e1c4ad3188770fe5c8b93b942bcede4a2
if [[ ! -d "$PLUGIN_DIR/.git" ]]; then
    mkdir -p "$ROOT/.runtime/sources"
    git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git "$PLUGIN_DIR"
    git -C "$PLUGIN_DIR" checkout "$PLUGIN_COMMIT"
fi
[[ "$(git -C "$PLUGIN_DIR" rev-parse HEAD)" == "$PLUGIN_COMMIT" ]] || { echo 'Unexpected topic hardware version' >&2; exit 1; }
colcon --log-base .runtime/construct/log build --symlink-install \
    --build-base .runtime/construct/build --install-base .runtime/construct/install \
    --base-paths src/construct_description src/construct_moveit_config src/sketch_control src/rbpodo_painting_control src/rbpodo_ros2/rbpodo_msgs "$PLUGIN_DIR" \
    --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
python3 scripts/prepare_construct_assets.py
"$ROOT/.venv-api/bin/python" -m pip install --no-deps --target "$ROOT/.runtime/python" 'trimesh==4.12.2'
PYTHONPATH="$ROOT/.runtime/python${PYTHONPATH:+:$PYTHONPATH}" python3 scripts/prepare_construct_runtime.py
