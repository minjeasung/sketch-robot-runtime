"""Allow-listed ROS process lifecycle, separate from robot motion control."""
from __future__ import annotations
import asyncio
from collections import deque
from contextlib import asynccontextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import socket
import time
import uuid
from .robot_models import (DEFAULT_MODEL, MODEL_LABELS, validate_model,
                           model_calibration_files, validate_calibration_files,
                           validate_process_mode, process_cameras)
from .outpost_camera import validate_origin, camera_status


class SupervisorError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


@dataclass(frozen=True)
class ProcessSpec:
    name: str
    description: str
    dependencies: tuple[str, ...]
    command: tuple[str, ...]
    nodes: tuple[str, ...] = ()


def _rosbridge_port(profile):
    if profile != 'isaac_rb20_paint':
        return 9090
    try:
        port = int(os.environ.get('SKETCH_ROSBRIDGE_PORT', '9090'))
        if not 1 <= port <= 65535:
            raise ValueError()
        return port
    except ValueError:
        raise SupervisorError('SKETCH_ROSBRIDGE_PORT must be a TCP port from 1 to 65535', 400) from None


def build_specs(workspace, options=None):
    options = dict(options or {})
    allowed = {"profile", "process_mode", "robot_ip", "model_id", "launch_rviz", "launch_zed_driver", "launch_d405_driver",
               "camera_backend", "outpost_http", "outpost_zed_hw_id", "outpost_zed_serial",
               "outpost_d405_hw_id", "outpost_d405_serial", "spray_eoat_profile"}
    if set(options) - allowed:
        raise SupervisorError("Unknown configuration options", 400)
    eoat_profile = options.setdefault('spray_eoat_profile', '')
    if (not isinstance(eoat_profile, str) or len(eoat_profile) > 4096
            or any(ord(c) < 32 or 127 <= ord(c) <= 159 for c in eoat_profile)):
        raise SupervisorError('spray_eoat_profile must be a path string of at most 4096 characters without control characters', 400)
    if eoat_profile:
        # Resolve against the workspace without requiring the profile to exist.
        eoat_profile = os.path.abspath(os.path.join(workspace, os.path.expanduser(eoat_profile)))
        if len(eoat_profile) > 4096:
            raise SupervisorError('spray_eoat_profile normalized path exceeds 4096 characters', 400)
        options['spray_eoat_profile'] = eoat_profile
    profile = options.setdefault("profile", "dry_run")
    if profile not in ("dry_run", "work", "fake", "spray_motion_test", "zed_preview", "isaac_construct", "isaac_rb20_paint"):
        raise SupervisorError("profile must be dry_run, work, fake, spray_motion_test, zed_preview, isaac_construct or isaac_rb20_paint", 400)
    try:
        process_mode = validate_process_mode(options.setdefault(
            'process_mode', 'spray' if profile in ('spray_motion_test', 'zed_preview', 'isaac_construct') else 'paint'))
    except ValueError as exc:
        raise SupervisorError(str(exc), 400) from None
    if profile == 'isaac_rb20_paint' and process_mode != 'paint':
        raise SupervisorError('isaac_rb20_paint requires process_mode=paint', 400)
    if process_mode == 'weld' and profile != 'isaac_construct':
        raise SupervisorError('Weld hardware driver is unavailable; use Isaac Construct', 400)
    if profile == 'isaac_construct' and process_mode not in ('spray', 'weld'):
        raise SupervisorError('Construct requires process_mode=spray or weld', 400)
    if profile in ('spray_motion_test', 'zed_preview') and process_mode != 'spray':
        raise SupervisorError(profile + ' requires process_mode=spray', 400)
    robot_ip = options.setdefault("robot_ip", "10.0.2.7")
    construct_models = ('construct_left', 'construct_right')
    model_id = options.setdefault('model_id', 'construct_left' if profile == 'isaac_construct' else
                                  'rb20_1900es' if profile == 'isaac_rb20_paint' else DEFAULT_MODEL)
    if profile == 'isaac_rb20_paint' and model_id != 'rb20_1900es':
        raise SupervisorError('isaac_rb20_paint requires model_id=rb20_1900es', 400)
    if profile == 'isaac_construct' and model_id not in construct_models:
        raise SupervisorError('isaac_construct requires model_id=construct_left or construct_right', 400)
    if model_id in construct_models and profile != 'isaac_construct':
        raise SupervisorError('Construct models require profile=isaac_construct; no physical driver is configured', 400)
    try:
        validate_model(model_id)
    except ValueError as exc:
        raise SupervisorError(str(exc), 400) from None
    try:
        if not isinstance(robot_ip, str):
            raise ValueError()
        ipaddress.IPv4Address(robot_ip)
    except ValueError:
        raise SupervisorError("robot_ip must be an IPv4 address", 400) from None
    backend = options.setdefault('camera_backend', 'native' if profile == 'isaac_rb20_paint' else 'outpost')
    if profile == 'isaac_rb20_paint' and backend != 'native':
        raise SupervisorError('isaac_rb20_paint requires camera_backend=native', 400)
    if backend not in ('outpost', 'native'):
        raise SupervisorError('camera_backend must be outpost or native', 400)
    try:
        options['outpost_http'] = validate_origin(options.get('outpost_http', 'http://127.0.0.1:8100'))
    except (ValueError, TypeError) as exc:
        raise SupervisorError(str(exc), 400) from None
    if profile == 'isaac_construct':
        if backend != 'outpost':
            raise SupervisorError('isaac_construct requires camera_backend=outpost', 400)
        if options['outpost_http'] != 'http://127.0.0.1:8100':
            raise SupervisorError('isaac_construct requires outpost_http=http://127.0.0.1:8100', 400)
        serial = options.get('outpost_zed_serial') or 'stream://127.0.0.1:30000'
        if serial != 'stream://127.0.0.1:30000':
            raise SupervisorError('isaac_construct requires outpost_zed_serial=stream://127.0.0.1:30000', 400)
        options['outpost_zed_serial'] = serial
        if not options.get('outpost_zed_hw_id'):
            try:
                camera = json.loads((Path(workspace) / '.runtime/construct/zed-camera.json').read_text())
                options['outpost_zed_hw_id'] = camera['hw_id']
            except (OSError, ValueError, KeyError, TypeError):
                options['outpost_zed_hw_id'] = ''
        options['spray_eoat_profile'] = str(Path(workspace).resolve() / 'isaac_assets' / (model_id + '_eoat.json'))
    for key in ('outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial'):
        value = options.setdefault(key, '')
        if profile == 'isaac_rb20_paint' and value:
            raise SupervisorError('isaac_rb20_paint uses rendered cameras; physical camera identifiers must be empty', 400)
        if profile == 'isaac_construct' and key == 'outpost_zed_serial' and value == 'stream://127.0.0.1:30000':
            continue
        if not isinstance(value, str) or len(value) > 128 or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in value):
            raise SupervisorError(key + ' must be a camera identifier', 400)
    for key in ("launch_rviz", "launch_zed_driver", "launch_d405_driver"):
        options.setdefault(key, profile not in ("fake", "isaac_construct", "isaac_rb20_paint") and (key == 'launch_rviz' or backend == 'native'))
        if type(options[key]) is not bool:
            raise SupervisorError(f"{key} must be a JSON boolean", 400)
    if profile == 'isaac_rb20_paint' and (options['launch_zed_driver'] or options['launch_d405_driver']):
        raise SupervisorError('isaac_rb20_paint disables physical camera drivers', 400)
    if profile in ('isaac_construct', 'isaac_rb20_paint'):
        for key in ('launch_rviz', 'launch_zed_driver', 'launch_d405_driver'):
            options[key] = False
    if process_mode != 'paint':
        options['launch_d405_driver'] = False
    if backend == 'outpost' and (options['launch_zed_driver'] or options['launch_d405_driver']):
        raise SupervisorError('Outpost owns cameras; native camera drivers must be disabled', 400)
    if profile == 'isaac_rb20_paint':
        port = _rosbridge_port(profile)
        groups = (
            ('simulation', 'RB20 painting · Isaac Sim · measured sensors', (), ()),
            ('robot_control', 'RB20 · MoveIt · simulated controllers', ('simulation',), ('controller_manager', 'move_group')),
            ('perception', 'Rendered ZED · D405 · sketch perception', ('robot_control',),
             ('target_selector', 'wall_projector', 'sketch_to_waypoints', 'd405_surface_refiner')),
            ('force_pipeline', 'Wrench reference · force monitor', ('robot_control',), ('painting_force_monitor',)),
            ('executor', 'Original guarded painting executor', ('robot_control', 'perception', 'force_pipeline'), ('moveit_executor',)),
        )
        specs = [ProcessSpec(name, description, dependencies,
                 ('bash', 'scripts/run_rb20_painting_isaac.sh') if name == 'simulation' else
                 ('bash', 'scripts/run_rb20_painting_ros.sh', name), nodes)
                 for name, description, dependencies, nodes in groups]
        specs.append(ProcessSpec('rosbridge', f'Browser ROS WebSocket ({port})', (),
                     ('ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
                      '--ros-args', '-r', '__node:=painting_rosbridge_websocket',
                      '-p', 'max_message_size:=10000000', '-p', f'port:={port}', '-p', 'use_sim_time:=true'),
                     ('painting_rosbridge_websocket', 'rosbridge_websocket')))
        return options, specs
    if profile == 'isaac_construct':
        return options, [
            ProcessSpec('simulation', 'Construct · Isaac Sim · MoveIt · controllers', (),
                        ('/usr/bin/python3', '-u', 'scripts/run_construct_demo.py', '--managed-runtime') +
                        (('--process-mode', 'weld', '--model-id', model_id) if process_mode == 'weld' else ()),
                        ('controller_manager', 'move_group')),
            ProcessSpec('perception', 'ZED · Outpost · sketch perception', ('simulation',),
                        ('bash', 'scripts/run_construct_perception.sh', model_id) + (('weld',) if process_mode == 'weld' else ()),
                        ('weld_input',) if process_mode == 'weld' else ('target_selector', 'wall_projector', 'sketch_to_waypoints')),
            ProcessSpec('executor', 'Sketch path executor · Construct', ('simulation', 'perception'),
                        ('bash', 'scripts/run_construct_executor.sh', model_id) + (('weld',) if process_mode == 'weld' else ()), ('moveit_executor',)),
            ProcessSpec('rosbridge', 'Browser ROS WebSocket (9090)', (),
                        ('ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
                         '--ros-args', '-r', '__node:=painting_rosbridge_websocket',
                         '-p', 'max_message_size:=10000000'),
                        ('painting_rosbridge_websocket', 'rosbridge_websocket')),
        ]
    if profile == 'zed_preview':
        if backend != 'outpost':
            raise SupervisorError('zed_preview requires camera_backend=outpost', 400)
        options['launch_rviz'] = False
        calibration = model_calibration_files(workspace, options['model_id'], 'spray')
        flags = {key: options[key] for key in
                 ('model_id', 'outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial', 'spray_eoat_profile')}
        flags['zed_calibration_file'] = calibration['zed_calibration_file']
        command = ('ros2', 'launch', 'sketch_control', 'zed_preview.launch.py') + tuple(
            f'{key}:={value}' for key, value in flags.items())
        return options, [
            ProcessSpec('perception', 'ZED · 대상/작업영역 · 경로 미리보기', (), command,
                        ('target_selector', 'wall_projector', 'sketch_to_waypoints',
                         'zed_preview')),
            ProcessSpec('rosbridge', 'Browser ROS WebSocket (9090)', (),
                        ('ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
                         '--ros-args', '-r', '__node:=painting_rosbridge_websocket',
                         '-p', 'max_message_size:=10000000'),
                        ('painting_rosbridge_websocket', 'rosbridge_websocket')),
        ]
    common = dict(options)
    common.pop("profile")
    if profile == 'fake':
        common['camera_backend'] = 'native'
    common.update(model_calibration_files(workspace, options['model_id'], process_mode))
    common.update(use_fake_hardware=profile == "fake", use_isaac_sim=False, use_sim_time=False,
                  real_painting_enabled=profile in ("work", "spray_motion_test"),
                  dry_run=profile not in ("work", "spray_motion_test"),
                  painting_force_enabled=profile == "work" and process_mode == 'paint',
                  spray_motion_test=profile == "spray_motion_test")
    perception_nodes = ('target_selector', 'wall_projector', 'sketch_to_waypoints')
    if process_mode == 'paint':
        perception_nodes += ('d405_surface_refiner',)
    groups = (
        ("robot_control", MODEL_LABELS[options['model_id']] + " · MoveIt · controllers · RViz", (), ("controller_manager", "move_group")),
        ("perception", ("ZED · D405" if process_mode == 'paint' else "ZED") +
         " · calibration · sketch perception", ("robot_control",), perception_nodes),
        ("force_pipeline", "Wrench reference · force monitor", ("robot_control",), ("painting_force_monitor",)),
        ("executor", "Sketch path executor · flight recorder", ("robot_control", "perception", "force_pipeline"), ("moveit_executor",)),
        ("rosbridge", "Browser ROS WebSocket (9090)", (), ("painting_rosbridge_websocket", "rosbridge_websocket")),
    )
    specs = []
    for name, description, dependencies, nodes in groups:
        flags = dict(common)
        flags.update({f"launch_{key}": key == name for key in
                      ("robot_control", "perception", "force_pipeline", "executor", "rosbridge")})
        flags["enable_interlock_flight_recorder"] = name == "executor"
        command = ("ros2", "launch", "sketch_control", "rb10_painting_system.launch.py") + tuple(
            f"{key}:={str(value).lower() if type(value) is bool else value}" for key, value in flags.items())
        specs.append(ProcessSpec(name, description, dependencies, command, nodes))
    return options, specs


def utcnow():
    return datetime.now(timezone.utc).isoformat()


class Supervisor:
    def __init__(self, workspace, monitor, options=None, *, specs=None, grace=1.0, stop_timeout=8.0,
                 readiness_timeout=300.0):
        self.workspace, self.monitor = Path(workspace), monitor
        self.options, built = build_specs(workspace, options)
        self.grace, self.stop_timeout = grace, stop_timeout
        self.readiness_timeout = min(300.0, max(0.0, readiness_timeout))
        self.log_dir = self.workspace / "logs/system_api"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._simulation_identity = None
        self.created = time.monotonic()
        self.lock = asyncio.Lock()
        self._startup_abort = asyncio.Event()
        self._shutdown_requests = 0
        self._legacy_starting = False
        self.event_list = deque(maxlen=500)
        self.monitor_task = None
        self._set_specs(built if specs is None else specs)
        from .runtime_apply import RuntimeApplyManager
        from .runtime_selection import legacy_to_selection
        self.runtime_manager = RuntimeApplyManager(self.workspace / '.runtime/platform',
            self._build_runtime_assets, self._shutdown_for_runtime, self._start_runtime_bundle,
            busy=self._runtime_busy, initial=legacy_to_selection(self.options))
        restored=self.runtime_manager.state().get('legacy_configuration')
        if restored and specs is None:
            self.options,restored_specs=build_specs(self.workspace,restored)
            self._set_specs(restored_specs)

    async def _shutdown_for_runtime(self):
        return await self.shutdown(_runtime_internal=True)

    def _runtime_busy(self):
        snapshot = self.monitor.snapshot()
        return bool(self._legacy_starting or snapshot.get('motion_busy') or any(
            (snapshot.get(source) or {}).get('running') or (snapshot.get(source) or {}).get('stopping')
            for source in ('readiness', 'execution', 'welding')))

    def _build_runtime_assets(self, selection, directory):
        import sys
        scripts = str(self.workspace / 'scripts')
        if scripts not in sys.path: sys.path.insert(0, scripts)
        from prepare_platform_assets import generate_assets
        return generate_assets(self.workspace, selection, directory)

    async def _start_runtime_bundle(self, bundle):
        from .platform_runtime import build_platform_specs
        self._runtime_bundle = bundle
        selection = bundle['selection']
        arm = bundle['arms'][selection['active_arm']]
        async with self.lock:
            self.options.update(profile='platform', model_id=arm['model_id'], process_mode=selection['process'],
                camera_backend='native' if selection['execution_mode']=='simulation' else 'outpost',
                launch_zed_driver=False, launch_d405_driver=False)
            self._set_specs(build_platform_specs(self.workspace,bundle))
            await self._start('simulation')
        return {'ready':True, 'asset_revision':bundle['revision']}

    def _platform_ready_marker(self,process):
        try:
            bundle=self._runtime_bundle
            marker=json.loads((Path(bundle['directory'])/'managed-ready.json').read_text())
            now=time.monotonic();updated=marker['updated_monotonic'];started=self.records['simulation']['started']
            if (process is not None and process.returncode is None and marker.get('pid')==process.pid
                    and marker.get('asset_revision')==bundle['revision']
                    and marker.get('runtime_revision')==bundle['runtime_revision']
                    and type(updated) in (float,int) and math.isfinite(updated)
                    and started is not None and started<=updated<=now and now-updated<2
                    and type(marker.get('step_index')) is int and marker['step_index']>0
                    and marker.get('ready') is True):return marker
        except (OSError,ValueError,KeyError,TypeError,AttributeError):pass
        return None

    def _platform_startup_error(self,process):
        """Accept a terminal diagnostic only from this owned launch/revision."""
        try:
            bundle=self._runtime_bundle
            error=json.loads((Path(bundle['directory'])/'startup-error.json').read_text())
            started=self.records['simulation']['started'];updated=error.get('updated_monotonic')
            if (process is not None and type(error.get('pid')) is int and error['pid']==process.pid
                    and error.get('asset_revision')==bundle['revision']
                    and error.get('runtime_revision')==bundle['runtime_revision']
                    and isinstance(error.get('session_id'),str) and error['session_id']
                    and type(updated) in (int,float) and math.isfinite(updated)
                    and started is not None and started<=updated<=time.monotonic()
                    and isinstance(error.get('error'),str) and error['error'].strip()):
                return error['error'][:4096]
        except (OSError,ValueError,KeyError,TypeError,AttributeError):pass
        return None

    async def _wait_platform_ready(self, process):
        deadline=time.monotonic()+self.readiness_timeout
        previous=None
        while time.monotonic()<deadline:
            if self._startup_abort.is_set():raise SupervisorError('Startup cancelled by shutdown')
            failure=self._platform_startup_error(process)
            if failure:raise SupervisorError(failure)
            if process.returncode is not None:raise SupervisorError('Platform runtime exited; inspect logs')
            marker=self._platform_ready_marker(process)
            if marker is not None:
                if previous is not None and marker['step_index']>previous:return
                previous=marker['step_index']
            else:previous=None
            await asyncio.sleep(.1)
        raise SupervisorError(self._platform_startup_error(process) or 'Platform readiness timed out; inspect logs')

    def _set_specs(self, specs):
        seen = set()
        for spec in specs:
            if spec.name in seen or not set(spec.dependencies) <= seen:
                raise ValueError("Process registry must be unique and in dependency order")
            seen.add(spec.name)
        self.records = {s.name: {"spec": s, "process": None, "state": "STOPPED", "error": None,
            "last_pid": None, "last_exit_code": None, "last_started_at": None,
            "last_stopped_at": None, "start_count": 0, "started": None} for s in specs}

    def _record(self, name):
        if name not in self.records:
            raise SupervisorError("Unknown process: " + name, 404)
        return self.records[name]

    def _event(self, kind, name, message):
        self.event_list.append({"timestamp": utcnow(), "kind": kind, "process": name, "message": message})

    def snapshot(self, name):
        r = self._record(name)
        p = r["process"]
        alive = p is not None and p.returncode is None
        state = r["state"]
        if p is not None and not alive and state in ("STARTING", "RUNNING"):
            state = "FAILED" if p.returncode else "EXITED"
        return {"name": name, "description": r["spec"].description, "state": state,
                "configured": True, "configuration_error": None,
                "dependencies": list(r["spec"].dependencies), "pid": p.pid if alive else None,
                "last_pid": r["last_pid"], "last_exit_code": p.returncode if p else r["last_exit_code"],
                "last_started_at": r["last_started_at"], "last_stopped_at": r["last_stopped_at"],
                "uptime_sec": round(time.monotonic() - r["started"], 2) if alive else None,
                "start_count": r["start_count"], "log_path": str(self.log_dir / (name + ".log")),
                "error": r["error"], "command": list(r["spec"].command)}

    def runtime_state(self):
        if self.runtime_manager.state()['phase']=='ready':
            if not all(self.snapshot(name)['state']=='RUNNING' for name in self.records):
                self.runtime_manager.mark_unready('Applied runtime processes are no longer running; apply again to verify readiness.')
            elif self.options['profile']=='platform' and not self._platform_ready_marker(self.records.get('simulation',{}).get('process')):
                self.runtime_manager.mark_unready('Applied runtime readiness heartbeat expired or identity changed; apply again to verify readiness.')
        return self.runtime_manager.state()

    def status(self):
        processes = [self.snapshot(name) for name in self.records]
        runtime = {"simulation": self.options['profile'] in ('isaac_construct', 'isaac_rb20_paint', 'platform', 'fake'),
                   "spray_standoff_m": .3 if self.options['profile'] == 'isaac_construct' else .5}
        if self.options['profile'] == 'isaac_rb20_paint':
            runtime.update(self._simulation_identity or {})
            runtime['rosbridge_port'] = _rosbridge_port(self.options['profile'])
        return {"supervisor": "running", "uptime_sec": round(time.monotonic() - self.created, 2),
                "monitor_interval_sec": 0.5, "system_prepared": all(p["state"] == "RUNNING" for p in processes),
                "teleoperation_active": False,
                "degraded": any(p["state"] in ("FAILED", "EXITED") for p in processes),
                "processes": processes, "configuration": self.options,
                "runtime": runtime, "runtime_configuration": self.runtime_state(),
                "ros": self.monitor.snapshot(), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0")}

    def connection_config(self):
        return {'rosbridge_port': _rosbridge_port(self.options['profile']),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                'simulation': self.options['profile'] in ('isaac_rb20_paint', 'isaac_construct', 'platform', 'fake')}

    async def configure(self, options):
        async with self.lock:
            options=dict(options)
            expected_revision=options.pop('expected_revision',None)
            self.runtime_manager.check_legacy_configuration(expected_revision)
            if any(r["process"] is not None for r in self.records.values()):
                raise SupervisorError("Shutdown the system before changing configuration")
            self._validate_rb20_domain(options.get('profile'))
            options.setdefault('spray_eoat_profile', self.options['spray_eoat_profile'])
            options, specs = build_specs(self.workspace, options)
            self.runtime_manager.adopt_legacy(options,expected_revision)
            self.options = options
            self._set_specs(specs)
            return options

    def _validate_model_calibration(self):
        if (self.options['model_id'] != DEFAULT_MODEL
                and self.options['profile'] not in ('fake', 'isaac_construct', 'isaac_rb20_paint')):
            try:
                mode = self.options['process_mode']
                validate_calibration_files(
                    model_calibration_files(self.workspace, self.options['model_id'], mode), mode)
            except ValueError as exc:
                raise SupervisorError(str(exc)) from None

    def _validate_cameras(self):
        if self.options['camera_backend'] != 'outpost' or self.options['profile'] == 'fake':
            return
        if self.options['profile'] == 'isaac_construct':
            # The managed demo may register a new hardware ID during startup.
            self.options, _ = build_specs(self.workspace, {**self.options, 'outpost_zed_hw_id': ''})
        for camera, kind in process_cameras(self.options['process_mode']):
            try:
                camera_status(self.options['outpost_http'], self.options[f'outpost_{camera}_hw_id'],
                              self.options[f'outpost_{camera}_serial'], kind)
            except (ValueError, OSError, KeyError, TypeError) as exc:
                raise SupervisorError('Outpost: ' + str(exc)) from None

    @staticmethod
    def _validate_rb20_domain(profile):
        if profile == 'isaac_rb20_paint' and os.environ.get('ROS_DOMAIN_ID', '0') != '106':
            raise SupervisorError('RB20 painting needs its dedicated ROS domain 106 monitor; restart the API with bash scripts/run_system_api.sh --rb20-paint')

    def preflight(self, name):
        self._validate_rb20_domain(self.options['profile'])
        r = self._record(name)
        if name == 'perception':
            self._validate_model_calibration()
            self._validate_cameras()
        graph = self.monitor.snapshot()
        if not graph["graph_fresh"]:
            raise SupervisorError("ROS discovery is not ready; retry shortly")
        conflicts = set(r["spec"].nodes)
        if self.options['profile'] == 'zed_preview':
            conflicts.add('moveit_executor')
        if name == "perception":
            if self.options['camera_backend'] == 'outpost':
                # Reject both the current split bridge nodes and the legacy
                # single-process name to prevent duplicate Outpost consumers.
                conflicts.update({
                    'sketch_outpost_zed_bridge',
                    'sketch_outpost_bridge',
                })
                if self.options['process_mode'] == 'paint':
                    conflicts.add('sketch_outpost_d405_bridge')
            if self.options["launch_zed_driver"]:
                conflicts.add("zed_node")
            if self.options["launch_d405_driver"]:
                conflicts.add("d405")
        found = [n for n in graph["nodes"] if n.rsplit("/", 1)[-1] in conflicts]
        if found:
            raise SupervisorError("Already running outside this supervisor: " + ", ".join(found))
        if name == "rosbridge":
            with socket.socket() as sock:
                sock.settimeout(0.2)
                port = _rosbridge_port(self.options['profile'])
                if sock.connect_ex(("127.0.0.1", port)) == 0:
                    raise SupervisorError(f'Port {port} is already occupied')

    def _rb20_ready_marker(self, process):
        """Accept only this launch's live measured inputs and generated scene."""
        try:
            marker = json.loads((self.workspace / '.runtime/rb20_painting/managed-ready.json').read_text())
            manifest = json.loads((self.workspace / '.runtime/rb20_painting/assets/manifest.json').read_text())
            identity = self._simulation_identity
            now = time.monotonic()
            updated = marker.get('updated_monotonic')
            step = marker.get('step_index')
            started = self.records['simulation']['started']
            if (not isinstance(marker, dict) or not isinstance(manifest, dict) or not identity
                    or type(marker.get('pid')) is not int or marker['pid'] != process.pid
                    or marker.get('session_id') != identity['session_id']
                    or not isinstance(manifest.get('scene_revision'), str) or not manifest['scene_revision']
                    or marker.get('scene_revision') != manifest['scene_revision']
                    or (identity.get('scene_revision') and marker.get('scene_revision') != identity['scene_revision'])
                    or marker.get('model_id') != 'rb20_1900es' or manifest.get('model_id') != 'rb20_1900es'
                    or type(step) is not int or step < 1
                    or type(updated) not in (int, float) or not math.isfinite(updated)
                    or started is None or not started <= updated <= now or now - updated > 2.0
                    or any(marker.get(key) is not True for key in ('joints_valid', 'wrench_valid', 'cameras_ready'))):
                return None
            return marker
        except (OSError, ValueError, TypeError, AttributeError):
            return None

    def _rb20_child_environment(self, name):
        env = dict(os.environ)
        # Jazzy LOCALHOST discovery reintroduces SHM even with UDP selected.
        # Match rb20_painting_dds_env.sh for direct children such as rosbridge;
        # the XML itself confines transport and discovery to 127.0.0.1.
        env.pop('ROS_LOCALHOST_ONLY', None)
        dds_profile = str(self.workspace / 'config/rb20_painting_fastdds.xml')
        env.update(ROS_DOMAIN_ID=env.get('ROS_DOMAIN_ID', '106'),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',
                   RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                   RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',
                   FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
                   FASTDDS_DEFAULT_PROFILES_FILE=dds_profile,
                   FASTRTPS_DEFAULT_PROFILES_FILE=dds_profile)
        if name == 'simulation':
            self._simulation_identity = {'session_id': uuid.uuid4().hex}
        if self._simulation_identity:
            env['RB20_PAINTING_SESSION_ID'] = self._simulation_identity['session_id']
            if self._simulation_identity.get('scene_revision'):
                env['RB20_PAINTING_SCENE_REVISION'] = self._simulation_identity['scene_revision']
        return env

    async def _wait_simulation_ready(self, process):
        path = self.workspace / '.runtime/construct/managed-ready.json'
        deadline = time.monotonic() + self.readiness_timeout
        previous_step = None
        rb20 = self.options['profile'] == 'isaac_rb20_paint'
        while True:
            if self._startup_abort.is_set():
                raise SupervisorError('Startup cancelled by shutdown')
            if process.returncode is not None:
                raise SupervisorError(f"simulation exited during startup ({process.returncode}); inspect logs")
            if rb20:
                marker = self._rb20_ready_marker(process)
                if marker is not None:
                    if previous_step is not None and marker['step_index'] > previous_step:
                        self._simulation_identity['scene_revision'] = marker['scene_revision']
                        return
                    previous_step = marker['step_index']
                else:
                    previous_step = None
            else:
                try:
                    marker = json.loads(path.read_text())
                    if isinstance(marker, dict) and type(marker.get('pid')) is int and marker['pid'] == process.pid:
                        return
                except (OSError, ValueError):
                    pass
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SupervisorError('simulation readiness timed out; inspect logs')
            await asyncio.sleep(min(.1, remaining))

    async def _start(self, name):
        if self._startup_abort.is_set():
            raise SupervisorError('Startup cancelled by shutdown')
        r = self._record(name)
        if self.snapshot(name)["state"] == "RUNNING":
            return self.snapshot(name)
        if r["process"] is not None:
            raise SupervisorError("Stop previous process before restarting: " + name)
        missing = [d for d in r["spec"].dependencies if self.snapshot(d)["state"] != "RUNNING"]
        if missing:
            raise SupervisorError("Start dependencies first: " + ", ".join(missing))
        self.preflight(name)
        path = self.log_dir / (name + ".log")
        if path.exists() and path.stat().st_size > 10_000_000:
            path.replace(path.with_suffix(".previous.log"))
        r["state"], r["error"] = "STARTING", None
        self._event("starting", name, "Starting registered command")
        try:
            child_env = self._rb20_child_environment(name) if self.options['profile'] == 'isaac_rb20_paint' else None
            with path.open("ab", buffering=0) as log:
                p = await asyncio.create_subprocess_exec(*r["spec"].command,
                    cwd=self.workspace, stdin=asyncio.subprocess.DEVNULL,
                    stdout=log, stderr=asyncio.subprocess.STDOUT, start_new_session=True, env=child_env)
            r.update(process=p, last_pid=p.pid, started=time.monotonic(), last_started_at=utcnow(),
                     last_stopped_at=None, last_exit_code=None, start_count=r["start_count"] + 1)
            if name == 'simulation' and self.options['profile'] == 'platform':
                await self._wait_platform_ready(p)
            elif name == 'simulation' and self.options['profile'] in ('isaac_construct', 'isaac_rb20_paint'):
                await self._wait_simulation_ready(p)
            else:
                await asyncio.sleep(self.grace)
            if self._startup_abort.is_set():
                raise SupervisorError('Startup cancelled by shutdown')
            if p.returncode is not None:
                raise SupervisorError(f"{name} exited during startup ({p.returncode}); inspect logs")
            r["state"] = "RUNNING"
            self._event("started", name, "Process passed startup liveness check")
            return self.snapshot(name)
        except asyncio.CancelledError:
            await self._stop(name)
            raise
        except (OSError, SupervisorError) as exc:
            await self._stop(name)
            r["state"], r["error"] = "FAILED", str(exc)
            raise SupervisorError(str(exc)) from exc

    @asynccontextmanager
    async def _legacy_start_operation(self):
        self.runtime_manager.check_legacy_start()
        async with self.lock:
            self.runtime_manager.check_legacy_start()
            self._legacy_starting=True
            try:yield
            finally:self._legacy_starting=False

    async def start(self, name):
        self._record(name)
        async with self._legacy_start_operation():
            return await self._start(name)

    def _depends_on(self, candidate, dependency):
        deps = self._record(candidate)["spec"].dependencies
        return dependency in deps or any(self._depends_on(d, dependency) for d in deps)

    def _dependents(self, name):
        return [n for n, r in self.records.items() if r["process"] is not None and self._depends_on(n, name)]

    @staticmethod
    def _group_alive(pid):
        try:
            os.killpg(pid, 0)
            return True
        except ProcessLookupError:
            return False

    async def _stop(self, name):
        r = self._record(name)
        p = r["process"]
        if p is None:
            r["state"] = "STOPPED"
            return self.snapshot(name)
        r["state"] = "STOPPING"
        if name == "executor" and p.returncode is None:
            try:
                for _ in range(3):
                    self.monitor.request_abort()
                    await asyncio.sleep(0.15)
            except Exception as exc:
                self._event("abort_error", name, str(exc))
        interrupt_timeout = max(40.0, self.stop_timeout) if (
            name == 'simulation' and self.options['profile'] in ('isaac_construct', 'isaac_rb20_paint', 'platform')) else self.stop_timeout
        for sig, timeout in ((signal.SIGINT, interrupt_timeout), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            until = time.monotonic() + timeout
            while self._group_alive(p.pid) and time.monotonic() < until:
                await asyncio.sleep(0.05)
            if not self._group_alive(p.pid):
                break
        if self._group_alive(p.pid):
            r["state"], r["error"] = "FAILED", "Process group did not exit"
            raise SupervisorError(name + ": process group did not exit")
        await p.wait()
        r.update(process=None, state="STOPPED", last_exit_code=p.returncode, last_stopped_at=utcnow())
        self._event("stopped", name, "Owned process group stopped")
        return self.snapshot(name)

    async def stop(self, name, cascade=False):
        self._record(name)
        self.runtime_manager.check_process_stop()
        async with self.lock:
            self.runtime_manager.check_process_stop()
            self._record(name)
            dependents = self._dependents(name)
            if dependents and not cascade:
                raise SupervisorError("Active dependents; use cascade=true: " + ", ".join(dependents))
            for n in reversed(dependents):
                await self._stop(n)
            return await self._stop(name)

    async def restart(self, name, cascade=False):
        self._record(name)
        async with self._legacy_start_operation():
            self._record(name)
            dependents = self._dependents(name)
            if dependents and not cascade:
                raise SupervisorError("Active dependents; use cascade=true")
            for n in reversed(dependents):
                await self._stop(n)
            await self._stop(name)
            return await self._start(name)

    async def prepare(self):
        async with self._legacy_start_operation():
            self._validate_model_calibration()
            if self.options['profile'] != 'isaac_construct':
                self._validate_cameras()
            started = []
            try:
                for name in self.records:
                    was_running = self.snapshot(name)["state"] == "RUNNING"
                    await self._start(name)
                    if not was_running:
                        started.append(name)
            except (Exception, asyncio.CancelledError):
                for name in reversed(started):
                    try:
                        await self._stop(name)
                    except SupervisorError as exc:
                        self._event("rollback_error", name, str(exc))
                raise
            return {"started": started, "processes": self.status()["processes"],
                    "note": "RUNNING is process liveness. Check ros.readiness before work."}

    async def shutdown(self, *, _runtime_internal=False):
        if not _runtime_internal:self.runtime_manager.request_stop()
        # Wake startup before waiting for its lifecycle lock. Queued starts must
        # observe this intent until every pending shutdown has acquired the lock.
        self._shutdown_requests += 1
        self._startup_abort.set()
        try:
            async with self.lock:
                stopped, errors = [], []
                owned_simulator = (self.options['profile'] in ('platform', 'isaac_construct', 'isaac_rb20_paint')
                    and self.records.get('simulation', {}).get('process') is not None)
                for name in reversed(self.records):
                    try:
                        await self._stop(name)
                        stopped.append(name)
                    except SupervisorError as exc:
                        errors.append(str(exc))
                if errors:
                    raise SupervisorError("; ".join(errors), 503)
                if owned_simulator and hasattr(self.monitor, 'clear_motion_observations'):
                    self.monitor.clear_motion_observations()
                self.runtime_manager.mark_unready()
                return {"system": "stopped", "stopped": stopped}
        finally:
            self._shutdown_requests -= 1
            if not self._shutdown_requests:
                self._startup_abort.clear()

    def logs(self, name, lines=100):
        self._record(name)
        path = self.log_dir / (name + ".log")
        data = b""
        if path.exists():
            with path.open("rb") as f:
                f.seek(max(0, path.stat().st_size - 256_000))
                data = f.read(256_000)
        return {"name": name, "log_path": str(path), "lines": data.decode(errors="replace").splitlines()[-lines:]}

    async def _watch(self):
        while True:
            await asyncio.sleep(0.5)
            async with self.lock:
                for name, r in self.records.items():
                    p = r["process"]
                    if p is not None and p.returncode is not None and r["state"] == "RUNNING":
                        self._event("exited", name, f"Unexpected exit: {p.returncode}")
                        for dependent in reversed(self._dependents(name)):
                            try:
                                await self._stop(dependent)
                            except SupervisorError as exc:
                                self._event("shutdown_error", dependent, str(exc))
                        try:
                            await self._stop(name)
                        except SupervisorError as exc:
                            self._event("shutdown_error", name, str(exc))
                        r["state"], r["error"] = "FAILED", "Unexpected process exit; inspect logs"

    async def open(self):
        self.monitor_task = asyncio.create_task(self._watch())

    async def close(self):
        if self.monitor_task:
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except asyncio.CancelledError:
                pass
        await self.shutdown()
