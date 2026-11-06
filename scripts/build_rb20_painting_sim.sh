#!/usr/bin/env bash
set -eo pipefail
RB20_PAINTING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$RB20_PAINTING_ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
bash scripts/install_rb20_painting_deps.sh
export PYTHONNOUSERSITE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${SKETCH_BUILD_JOBS:-2}"
colcon --log-base .runtime/rb20_painting/log build --symlink-install \
  --build-base .runtime/rb20_painting/build --install-base .runtime/rb20_painting/install \
  --base-paths src/isaac_painting_msgs src/isaac_painting_hardware src/admittance_controller \
    src/eoat_description src/sketch_control src/rbpodo_painting_control \
    src/rbpodo_ros2/rbpodo_description src/rbpodo_ros2/rbpodo_bringup \
    src/rbpodo_ros2/rbpodo_moveit_config src/rbpodo_ros2/rbpodo_msgs \
  --parallel-workers "${SKETCH_BUILD_JOBS:-2}" --allow-overriding admittance_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source scripts/rb20_painting_ros_env.sh
python3 scripts/prepare_rb20_painting_assets.py
