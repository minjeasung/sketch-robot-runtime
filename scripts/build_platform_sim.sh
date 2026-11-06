#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
bash "$ROOT/scripts/setup_platform_python.sh"
PLUGIN_DIR="$ROOT/.runtime/sources/topic_based_ros2_control"
PLUGIN_COMMIT=6bd8d55e1c4ad3188770fe5c8b93b942bcede4a2
if [[ ! -d "$PLUGIN_DIR/.git" ]]; then
    mkdir -p "$ROOT/.runtime/sources"
    git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git "$PLUGIN_DIR"
    git -C "$PLUGIN_DIR" checkout "$PLUGIN_COMMIT"
fi
[[ "$(git -C "$PLUGIN_DIR" rev-parse HEAD)" == "$PLUGIN_COMMIT" ]] || exit 1
colcon --log-base .runtime/platform/log build --symlink-install \
    --build-base .runtime/platform/build --install-base .runtime/platform/install \
    --base-paths src/construct_description src/construct_moveit_config src/sketch_control \
    src/rbpodo_painting_control src/rbpodo_ros2/rbpodo_description src/rbpodo_ros2/rbpodo_bringup src/rbpodo_ros2/rbpodo_moveit_config src/rbpodo_ros2/rbpodo_msgs src/admittance_controller src/eoat_description src/vendor/realsense-ros/realsense2_description \
    src/isaac_painting_msgs src/isaac_painting_hardware "$PLUGIN_DIR" \
    --parallel-workers 2 --allow-overriding admittance_controller --packages-up-to admittance_controller sketch_control rbpodo_painting_control construct_moveit_config rbpodo_moveit_config eoat_description isaac_painting_hardware topic_based_ros2_control \
    --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
