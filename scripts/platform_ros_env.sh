#!/usr/bin/env bash
PLATFORM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
source "$PLATFORM_ROOT/.runtime/platform/install/local_setup.bash"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-105}"
export SKETCH_WORKSPACE="$PLATFORM_ROOT"
export PYTHONPATH="$PLATFORM_ROOT/src/sketch_control:$PLATFORM_ROOT/src/rbpodo_painting_control:$PLATFORM_ROOT/scripts:$PLATFORM_ROOT/.runtime/platform/python${PYTHONPATH:+:$PYTHONPATH}"
source "$PLATFORM_ROOT/scripts/rb20_painting_dds_env.sh"
