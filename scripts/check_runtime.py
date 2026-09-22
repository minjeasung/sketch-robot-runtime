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
    parser.add_argument('--camera-backend', choices=('native', 'outpost'),
                        default=os.environ.get('SKETCH_CAMERA_BACKEND', 'outpost'))
    parser.add_argument("--model-id", default=os.environ.get("SKETCH_MODEL_ID", "rb10_1300e_u"),
                        choices=("rb10_1300e_u", "rb20_1900es"))
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
    if args.real and args.camera_backend == 'native':
        packages += ["zed_wrapper", "realsense2_camera"]
    if args.camera_backend == 'outpost':
        check('Outpost raw IPC Python dependency', importlib.util.find_spec('zmq') is not None)
    layout = os.environ.get("SKETCH_RUNTIME_LAYOUT", "unknown")
    for package in packages:
        try:
            prefix = Path(get_package_prefix(package))
            owned = package in ("sketch_control", "admittance_controller", "rbpodo_hardware")
            check("package " + package, not (layout == "portable" and owned) or prefix.is_relative_to(root / ".runtime"), prefix)
        except PackageNotFoundError:
            check("package " + package, False, "not installed")
    if args.real:
        from sketch_control.robot_models import model_calibration_files, validate_calibration_files
        paths = model_calibration_files(root, args.model_id)
        for name, filename in paths.items():
            try:
                value = json.loads(Path(filename).read_text())
                check("calibration " + name, isinstance(value, dict) and bool(value), filename)
            except (OSError, ValueError) as exc:
                check("calibration " + name, False, str(exc))
        if args.model_id != "rb10_1300e_u":
            try:
                validate_calibration_files(paths)
                check("robot-specific camera transforms", True)
            except ValueError as exc:
                check("robot-specific camera transforms", False, exc)
    try:
        share = Path(get_package_prefix('rbpodo_description'))/'share/rbpodo_description'
        files = [share/'robots'/f'{args.model_id}.urdf.xacro']
        files += [share/'meshes'/args.model_id/kind/f'link{i}.{ext}'
                  for kind,ext in [('visual','dae'),('collision','stl')] for i in range(7)]
        check('arm model '+args.model_id, all(p.is_file() for p in files), 'URDF + 14 meshes')
    except PackageNotFoundError:
        check('arm model '+args.model_id, False, 'rbpodo_description not installed')
    print("Runtime layout:", layout)
    if layout != "portable":
        print("NOTE: existing-PC environment; other home workspaces may still be required.")
    for item in checks:
        print(("OK   " if item["ok"] else "FAIL ") + item["name"] + (": " + item["detail"] if item["detail"] else ""))
    print("Checks cover installation only, not robot/camera readiness or calibration accuracy.")
    return 0 if all(item["ok"] for item in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
