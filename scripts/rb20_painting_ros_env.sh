#!/usr/bin/env bash
# Source in a task process only. No robot-PC underlays are loaded.
RB20_PAINTING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RB20_PAINTING_RESTORE_NOUNSET=0
case $- in *u*) RB20_PAINTING_RESTORE_NOUNSET=1; set +u ;; esac
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
# Optional local extraction of the official Jazzy TRAC-IK packages. This keeps
# the runtime reproducible on workstations without root package permissions.
RB20_PAINTING_DEPS="$RB20_PAINTING_ROOT/.runtime/rb20_painting/deps/opt/ros/jazzy"
if [[ -d "$RB20_PAINTING_DEPS/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="$RB20_PAINTING_DEPS${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$RB20_PAINTING_DEPS/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ ! -f "$RB20_PAINTING_ROOT/.runtime/rb20_painting/install/local_setup.bash" ]]; then
    echo 'Run bash scripts/build_rb20_painting_sim.sh first.' >&2
    return 1
fi
source "$RB20_PAINTING_ROOT/.runtime/rb20_painting/install/local_setup.bash"
export SKETCH_WORKSPACE="$RB20_PAINTING_ROOT"
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=106
source "$RB20_PAINTING_ROOT/scripts/rb20_painting_dds_env.sh"
export PYTHONPATH="$RB20_PAINTING_ROOT/src/sketch_control:$RB20_PAINTING_ROOT/src/rbpodo_painting_control:$RB20_PAINTING_ROOT/.runtime/python${PYTHONPATH:+:$PYTHONPATH}"
if [[ "$RB20_PAINTING_RESTORE_NOUNSET" == 1 ]]; then set -u; fi
