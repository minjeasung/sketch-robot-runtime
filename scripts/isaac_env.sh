#!/usr/bin/env bash
# Source only inside an Isaac launcher, not in ~/.bashrc.
SKETCH_ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ISAACSIM_VENV="${ISAACSIM_VENV:-$SKETCH_ISAAC_ROOT/../.venvs/isaacsim-6.1}"
if [[ ! -x "$ISAACSIM_VENV/bin/isaacsim" ]]; then
    echo "Isaac Sim 6.1 missing: $ISAACSIM_VENV. See docs/ISAAC_SIM.md" >&2
    return 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-105}"
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SKETCH_ISAAC_SITE="$("$ISAACSIM_VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export LD_LIBRARY_PATH="$SKETCH_ISAAC_SITE/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
export SKETCH_WORKSPACE="$SKETCH_ISAAC_ROOT"
export PATH="$ISAACSIM_VENV/bin:$PATH"
