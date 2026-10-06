"""Humble entry points. Importing this module never starts ROS or hardware."""
import argparse
from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import threading

from .config import AddonConfig, TARGET_PLATFORM, check_platform, runtime_environment, owned_path
from .compatibility import check_upstream
from .runtime import Runtime, repository_root


def bootstrap(config):
    root = repository_root()
    os.environ.update(runtime_environment(config))
    sys.dont_write_bytecode = True
    modules = [root/'src/sketch_control', root/'src/rbpodo_painting_control',
               Path(config.upstream_root)/'linux/control', Path(config.upstream_root)/'interfaces/python',
               Path(config.upstream_root)/'linux/gateway',
               Path(config.upstream_root)/'linux/drivers/ros2_ws/src/rb10_realsense_moveit/scripts']
    sys.path[:0] = [str(p) for p in modules if p.is_dir()]
    upstream_packages = Path(config.upstream_root)/'.venv/lib/python3.10/site-packages'
    if upstream_packages.is_dir():
        sys.path.append(str(upstream_packages))
    os.environ['AMENT_PREFIX_PATH'] = str(root)+os.pathsep+os.environ.get('AMENT_PREFIX_PATH', '')


def doctor(config):
    checks = {'upstream': check_upstream(config.upstream_root)}
    checks['platform'] = dict(system=platform.system(), machine=platform.machine(),
                              python=platform.python_version(), ros=os.environ.get('ROS_DISTRO'),
                              target=TARGET_PLATFORM)
    try:
        release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
        check_platform(release.get('VERSION_ID', '').strip('"'), os.environ.get('ROS_DISTRO'), sys.version_info)
        checks['platform']['supported'] = platform.system() == 'Linux'
    except (OSError, ValueError):
        checks['platform']['supported'] = False
    checks['dependencies'] = {name: importlib.util.find_spec(name) is not None for name in
        ('rclpy', 'moveit_msgs', 'controller_manager_msgs', 'cv2', 'scipy', 'numpy', 'shapely', 'fastapi', 'trimesh', 'rb20_spray')}
    checks['calibration_present'] = Path(config.calibration_file).is_file()
    # GPU libraries belong to the existing camera environment, which may be
    # a separate process/container. Never claim that target metadata detects it.
    checks['camera_environment'] = dict(cuda_target=TARGET_PLATFORM['cuda'],
        zed_sdk_target=TARGET_PLATFORM['zed_sdk'], verified=False, managed_by_addon=False)
    checks['ok'] = bool(checks['upstream']['compatible'] and checks['platform']['supported']
                        and checks['calibration_present'] and all(checks['dependencies'].values()))
    checks['hardware_verified'] = False
    return checks


def run_core(config, component, profile):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    modules = {'projector': ('sketch_control.wall_projector_node', 'WallProjectorNode'),
               'generator': ('sketch_control.sketch_to_waypoints_node', 'SketchToWaypointsNode'),
               'preview': ('sketch_control.zed_preview_node', 'ZedPreviewNode')}
    import importlib
    name, cls = modules[component]
    args = ['--ros-args', '-r', '__node:=snucem_sketch_'+component,
            '-r', '/zed/zed_node/rgb/color/rect/image:='+config.image_topic,
            '-r', '/zed/zed_node/rgb/color/rect/camera_info:='+config.camera_info_topic,
            '-p', 'process_mode:=spray']
    if component == 'generator':
        args += ['-p', 'model_id:='+config.model_id, '-p', 'spray_tool_axis:=+z',
                 '-p', 'spray_eoat_profile:='+profile,
                 '-p', 'dry_run:='+str(config.profile in ('preview', 'dry_run')).lower(),
                 '-p', 'real_painting_enabled:='+str(config.profile != 'preview').lower()]
    rclpy.init(args=args)
    module = importlib.import_module(name)
    if component == 'generator':
        # This dedicated child uses the upstream planning frame directly.
        # Do not require or fabricate an Isaac-specific World transform.
        module.WORLD_FRAME = 'link0'
    node = getattr(module, cls)()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def serve(config, config_path):
    import fcntl
    import uvicorn
    from .api import create_app
    state = Path(config.state_root)
    state.mkdir(parents=True, exist_ok=True)
    lock = owned_path(state, 'service.lock').open('a+')
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    marker = owned_path(state, 'service-active.json')
    marker.write_text(json.dumps(dict(pid=os.getpid())), encoding='utf-8')
    runtime = Runtime(config, config_path)
    done = threading.Event()
    def watchdog():
        while not done.wait(.5):
            with runtime.lock:
                try:
                    runtime.require_children_alive()
                except RuntimeError as exc:
                    runtime.error = str(exc)
                    runtime.shutdown()
    thread = threading.Thread(target=watchdog, daemon=True)
    thread.start()
    try:
        uvicorn.run(create_app(config, runtime, repository_root()/'web'), host=config.api_host, port=config.api_port)
    finally:
        done.set()
        runtime.shutdown()
        marker.unlink(missing_ok=True)
        lock.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only SNUCEM_Robot_22.04 Spray integration')
    parser.add_argument('--config', required=True, type=Path)
    commands = parser.add_subparsers(dest='command', required=True)
    for command in ('doctor', 'serve', 'perception', 'bridge', 'model', 'projector', 'generator', 'preview', 'executor'):
        sub = commands.add_parser(command)
        if command in ('generator', 'executor'):
            sub.add_argument('--model-profile', required=True)
    install = commands.add_parser('install')
    install.add_argument('--bundle', type=Path, required=True)
    install.add_argument('--setup-env', action='store_true')
    args = parser.parse_args(argv)
    config = AddonConfig.load(args.config)
    if args.command == 'install':
        from .install import install_bundle, activate_version
        if args.setup_env:
            release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
            check_platform(release.get('VERSION_ID', '').strip('"'), os.environ.get('ROS_DISTRO'), sys.version_info)
        result = install_bundle(args.bundle, config, activate=False)
        if args.setup_env:
            venv = owned_path(config.install_root, 'envs/'+result['version'])
            subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages', str(venv)], check=True,
                           env=runtime_environment(config))
            subprocess.run([str(venv/'bin/python'), '-m', 'pip', 'install', '--only-binary=:all:', '-r',
                str(Path(result['version_root'])/'addons/snucem_spray/requirements-humble.txt')], check=True,
                env=runtime_environment(config))
        activate_version(config, result['version'], Path(result['version_root']))
        result['activated'] = True
        print(json.dumps(result, indent=2))
        return
    bootstrap(config)
    if args.command == 'doctor':
        report = doctor(config)
        print(json.dumps(report, indent=2))
        raise SystemExit(0 if report['ok'] else 1)
    Runtime(config, args.config).preflight()
    if args.command == 'serve':
        return serve(config, args.config.resolve())
    if args.command in ('projector', 'generator', 'preview'):
        return run_core(config, args.command, getattr(args, 'model_profile', ''))
    if args.command == 'executor':
        from .execution import run
        return run(config, args.model_profile)
    from importlib import import_module
    module_name = {'perception':'perception', 'bridge':'plane_bridge', 'model':'model_node'}[args.command]
    return import_module('snucem_spray_addon.'+module_name).run(config)
