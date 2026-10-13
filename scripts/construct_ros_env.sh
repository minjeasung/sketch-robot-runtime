#!/usr/bin/env bash
CONSTRUCT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
source "$CONSTRUCT_ROOT/.runtime/construct/install/local_setup.bash"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-105}"
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export SKETCH_WORKSPACE="$CONSTRUCT_ROOT"
export PYTHONPATH="$CONSTRUCT_ROOT/.runtime/python:$CONSTRUCT_ROOT/src/sketch_control:$CONSTRUCT_ROOT/src/rbpodo_painting_control${PYTHONPATH:+:$PYTHONPATH}"
