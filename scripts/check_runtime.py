#!/usr/bin/env python3
"""Read-only installation checks. Never initialize ROS or connect to hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real", action="store_true", help="also require camera packages and calibration files")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    checks = []
    def check(name, ok, detail=""):
        checks.append({"name": name, "ok": bool(ok), "detail": str(detail)})
    check("Python 3.12", sys.version_info[:2] == (3, 12), platform.python_version())
    check("ROS Jazzy", os.environ.get("ROS_DISTRO") == "jazzy", os.environ.get("ROS_DISTRO", "not sourced"))
    for module in ("fastapi", "uvicorn", "rclpy", "numpy", "scipy", "cv2", "yaml"):
        check("module " + module, importlib.util.find_spec(module) is not None)
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    packages = ["sketch_control", "rbpodo_painting_control", "admittance_controller", "rbpodo_hardware",
                "rbpodo_description", "eoat_description", "realsense2_description", "rosbridge_server"]
    if args.real:
        packages += ["zed_wrapper", "realsense2_camera"]
    layout = os.environ.get("SKETCH_RUNTIME_LAYOUT", "unknown")
    for package in packages:
        try:
            prefix = Path(get_package_prefix(package))
            owned = package in ("sketch_control", "admittance_controller", "rbpodo_hardware")
            check("package " + package, not (layout == "portable" and owned) or prefix.is_relative_to(root / ".runtime"), prefix)
        except PackageNotFoundError:
            check("package " + package, False, "not installed")
    if args.real:
        for name in ("zed_d405_apriltag_calibration.json", "d405_eyeinhand_charuco_calibration.json"):
            try:
                value = json.loads((root / name).read_text())
                check("calibration " + name, isinstance(value, dict) and bool(value), root / name)
            except (OSError, ValueError) as exc:
                check("calibration " + name, False, str(exc))
    print("Runtime layout:", layout)
    if layout != "portable":
        print("NOTE: existing-PC environment; other home workspaces may still be required.")
    for item in checks:
        print(("OK   " if item["ok"] else "FAIL ") + item["name"] + (": " + item["detail"] if item["detail"] else ""))
    print("Checks cover installation only, not robot/camera readiness or calibration accuracy.")
    return 0 if all(item["ok"] for item in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
