#!/usr/bin/env bash
# Ubuntu/ROS and vendor GPU SDK prerequisites are checked, never silently replaced.
set -eo pipefail
SKETCH_INSTALL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SKETCH_INSTALL_APT=false
SKETCH_INSTALL_WITH_ZED=true
for SKETCH_ARG in "$@"; do
    case "$SKETCH_ARG" in
        --install-deps) SKETCH_INSTALL_APT=true ;;
        --without-zed) SKETCH_INSTALL_WITH_ZED=false ;;
        --help) echo 'Usage: install_runtime.sh [--install-deps] [--without-zed]'; exit 0 ;;
        *) echo "Unknown option: $SKETCH_ARG" >&2; exit 2 ;;
    esac
done
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || "$(uname -m)" != x86_64 ]]; then
    echo 'This installer targets Ubuntu 24.04 x86_64 / ROS 2 Jazzy. See docs/PORTABLE_INSTALL.md.' >&2; exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'Install ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2; exit 1
fi
cd "$SKETCH_INSTALL_ROOT"
SKETCH_INSTALL_ARGS=()
if [[ "$SKETCH_INSTALL_WITH_ZED" == false ]]; then SKETCH_INSTALL_ARGS+=(--without-zed); fi
if [[ "$SKETCH_INSTALL_APT" == true ]]; then
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git python3-dev python3-venv \
      python3-colcon-common-extensions python3-rosdep python3-opencv python3-scipy \
      python3-numpy python3-pil python3-pil.imagetk libeigen3-dev libboost-all-dev \
      ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-kinematics-interface-kdl ros-jazzy-realsense2-camera ros-jazzy-cv-bridge \
      ros-jazzy-rosbridge-server ros-jazzy-tf-transformations
fi
python3 scripts/fetch_runtime_sources.py "${SKETCH_INSTALL_ARGS[@]}"
if [[ "$SKETCH_INSTALL_APT" == true ]]; then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
    rosdep update --rosdistro jazzy
    SKETCH_ROSDEP_PATHS=(src)
    if [[ "$SKETCH_INSTALL_WITH_ZED" == true ]]; then SKETCH_ROSDEP_PATHS+=(.runtime/sources/zed-ros2-wrapper); fi
    rosdep install --from-paths "${SKETCH_ROSDEP_PATHS[@]}" --ignore-src --rosdistro jazzy \
      --skip-keys 'rbpodo python3-pil.imagetk zed_wrapper' -y
fi
bash scripts/setup_system_api.sh
bash scripts/build_runtime.sh "${SKETCH_INSTALL_ARGS[@]}"
# Never overwrite an existing machine profile or credentials.
if [[ ! -e config/sketch_runtime.env ]]; then
    cp config/sketch_runtime.env.example config/sketch_runtime.env
    if [[ "$SKETCH_INSTALL_WITH_ZED" == false ]]; then
        printf '\nSKETCH_PROFILE=fake\nSKETCH_LAUNCH_ZED_DRIVER=false\nSKETCH_LAUNCH_D405_DRIVER=false\nSKETCH_LAUNCH_RVIZ=false\n' >> config/sketch_runtime.env
    fi
fi
bash scripts/check_runtime.sh
