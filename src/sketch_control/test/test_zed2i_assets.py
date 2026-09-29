"""Static contracts for the real ZED 2i model; no camera or ROS runtime required."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src" / "sketch_control"


def test_zed2i_description_is_sdk_free_runtime_dependency():
    package = ET.parse(PACKAGE / "package.xml").getroot()
    exec_deps = {e.text for e in package.findall("exec_depend")}
    assert "zed_description" in exec_deps
    install = (ROOT / "scripts" / "install_runtime.sh").read_text()
    assert "ros-jazzy-zed-description" in install


def test_moveit_uses_official_zed2i_mesh_in_calibrated_camera_frame():
    cfg = yaml.safe_load((PACKAGE / "config" / "objects.yaml").read_text())
    camera = next(obj for obj in cfg["objects"] if obj["name"] == "zed_camera")
    assert camera["moveit_shape"] == "mesh"
    assert camera["mesh_resource"] == "package://zed_description/meshes/zed2i.stl"
    assert camera["mesh_frame"] == "zed_left_camera_frame"
    assert camera["mesh_position"] == [0.0, -0.06, 0.0]
    assert camera["mesh_orientation"] == [0.0, 0.0, 0.0, 1.0]
    # Retain a conservative box only as filtering/fallback geometry.
    assert camera["shape"] == "box"
    assert len(camera["size"]) == 3


def test_isaac_sim_uses_zed2i_not_zedx_asset():
    script = (PACKAGE / "sketch_control" / "isaac_sim_rb10.py").read_text()
    assert "ZED_2i.usdc" in script
    assert "/base_link/ZED_2i/CameraLeft" in script
    assert "/base_link/ZED_2i/CameraRight" in script
    assert "/base_link/ZED_2i/Imu_Sensor" in script
    assert "ZED_X.usdc" not in script
    assert "/base_link/ZED_X/" not in script
