#!/usr/bin/env bash
# Use official ROS Debian packages without modifying the system installation.
set -eo pipefail
RB20_PAINTING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RB20_PAINTING_DEPS="$RB20_PAINTING_ROOT/.runtime/rb20_painting/deps"
if [[ -f /opt/ros/jazzy/share/ament_index/resource_index/packages/trac_ik_kinematics_plugin ]]; then
    exit 0
fi
if [[ -f "$RB20_PAINTING_DEPS/opt/ros/jazzy/lib/libtrac_ik_kinematics_plugin.so" && -f "$RB20_PAINTING_DEPS/opt/ros/jazzy/lib/libtrac_ik.so" ]]; then
    exit 0
fi
command -v apt-get >/dev/null
command -v dpkg-deb >/dev/null
mkdir -p "$RB20_PAINTING_ROOT/.runtime/rb20_painting/debs" "$RB20_PAINTING_DEPS"
cd "$RB20_PAINTING_ROOT/.runtime/rb20_painting/debs"
apt-get download ros-jazzy-trac-ik-kinematics-plugin ros-jazzy-trac-ik-lib
for package in ros-jazzy-trac-ik-kinematics-plugin_*.deb ros-jazzy-trac-ik-lib_*.deb; do
    dpkg-deb -x "$package" "$RB20_PAINTING_DEPS"
done
sha256sum ./*.deb > packages.sha256
