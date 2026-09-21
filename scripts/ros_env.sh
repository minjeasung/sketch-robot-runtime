#!/usr/bin/env bash
# Source this file from Bash. Portable installations use only their own overlay.
export SKETCH_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$SKETCH_WORKSPACE/.runtime/READY" ]]; then
    if [[ "$(cat "$SKETCH_WORKSPACE/.runtime/READY")" != "$SKETCH_WORKSPACE" ]]; then
        echo 'The compiled installation moved. Rebuild with scripts/build_runtime.sh.' >&2
        return 1
    fi
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
    source /opt/ros/jazzy/setup.bash
    export CMAKE_PREFIX_PATH="$SKETCH_WORKSPACE/.runtime/sdk/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$SKETCH_WORKSPACE/.runtime/sdk/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    source "$SKETCH_WORKSPACE/.runtime/ros2/install/local_setup.bash"
    export SKETCH_RUNTIME_LAYOUT=portable
else
    # Existing operating PC: preserve its current underlays until a portable
    # installation has completed successfully.
    source /opt/ros/jazzy/setup.bash
    for SKETCH_UNDERLAY in "$HOME/ros2_ws/install/setup.bash" "$HOME/rb10_ws/install/setup.bash"; do
        if [[ -f "$SKETCH_UNDERLAY" ]]; then source "$SKETCH_UNDERLAY"; fi
    done
    if [[ ! -f "$SKETCH_WORKSPACE/install/setup.bash" ]]; then
        echo 'Run scripts/install_runtime.sh first.' >&2; return 1
    fi
    source "$SKETCH_WORKSPACE/install/setup.bash"
    export SKETCH_RUNTIME_LAYOUT=legacy
fi
export PYTHONNOUSERSITE=1
export PYTHONPATH="$SKETCH_WORKSPACE/src/sketch_control${PYTHONPATH:+:$PYTHONPATH}"
