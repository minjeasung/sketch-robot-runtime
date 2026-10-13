#!/usr/bin/env bash
# Ubuntu24 / Python3.11 / CUDA13, matching the official Outpost lockfile.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="$ROOT/../zed-sdk-5.3"
OUTPOST="$ROOT/../official-outpost"
EXT="$ROOT/../zed-isaac-sim"
DOWNLOAD="$HOME/Downloads/zed-sdk"
mkdir -p "$DOWNLOAD" "$ROOT/logs" "$ROOT/.runtime"
INSTALLER="$DOWNLOAD/ZED_SDK_Ubuntu24_cuda13_v5.3.run"
if [[ ! -f "$SDK/lib/libsl_zed.so" ]]; then
    if [[ ! -f "$INSTALLER" ]]; then
        curl -fL --retry 3 -o "$INSTALLER.part" https://download.stereolabs.com/zedsdk/5.3/cu13/ubuntu24
        mv "$INSTALLER.part" "$INSTALLER"
    fi
    bash "$INSTALLER" --quiet --noexec --target "$SDK"
fi
[[ -f "$OUTPOST/pyproject.toml" ]] || { echo "Official Outpost source missing: $OUTPOST" >&2; exit 1; }
PATCH="$ROOT/patches/outpost-zed-stream.patch"
if ! git -C "$OUTPOST" apply --reverse --check "$PATCH" 2>/dev/null; then
    git -C "$OUTPOST" apply --check "$PATCH"
    git -C "$OUTPOST" apply "$PATCH"
fi
(cd "$OUTPOST" && "$ROOT/../.venvs/uv-tools/bin/uv" sync --locked --extra zed)
# The delivered lock omits this dependency from the Linux Open3D wheel metadata.
# Keep the original lock/source intact; apply the local environment supplement.
"$ROOT/../.venvs/uv-tools/bin/uv" pip install --python "$OUTPOST/.venv/bin/python" 'ipywidgets==8.1.9'
if [[ ! -d "$EXT" ]]; then
    git clone https://github.com/stereolabs/zed-isaac-sim.git "$EXT"
    git -C "$EXT" checkout 0164268ca123fa4549fc9d2062c1fd55cf7996dc
fi
if ! git -C "$EXT" apply --reverse --check "$ROOT/patches/zed-isaac-capture-time.patch" 2>/dev/null; then
    git -C "$EXT" apply --check "$ROOT/patches/zed-isaac-capture-time.patch"
    git -C "$EXT" apply "$ROOT/patches/zed-isaac-capture-time.patch"
    (cd "$EXT" && bash build.sh)
fi
if [[ ! -f "$EXT/exts/sl.sensor.camera/bin/libsl.sensor.camera.plugin.so" ]]; then
    (cd "$EXT" && bash build.sh)
fi
source "$ROOT/scripts/construct_zed_env.sh"
"$OUTPOST/.venv/bin/python" -c 'import pyzed.sl as sl; print("ZED SDK",sl.Camera.get_sdk_version())'
