"""Allow-listed ROS process lifecycle, separate from robot motion control."""
from __future__ import annotations
import asyncio
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import ipaddress
import os
from pathlib import Path
import signal
import socket
import time
from .robot_models import (DEFAULT_MODEL, MODEL_LABELS, validate_model,
                           model_calibration_files, validate_calibration_files)
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


def build_specs(workspace, options=None):
    options = dict(options or {})
    allowed = {"profile", "robot_ip", "model_id", "launch_rviz", "launch_zed_driver", "launch_d405_driver",
               "camera_backend", "outpost_http", "outpost_zed_hw_id", "outpost_zed_serial",
               "outpost_d405_hw_id", "outpost_d405_serial"}
    if set(options) - allowed:
        raise SupervisorError("Unknown configuration options", 400)
    profile = options.setdefault("profile", "dry_run")
    if profile not in ("dry_run", "work", "fake", "spray_motion_test"):
        raise SupervisorError("profile must be dry_run, work, fake or spray_motion_test", 400)
    robot_ip = options.setdefault("robot_ip", "10.0.2.7")
    try:
        validate_model(options.setdefault("model_id", DEFAULT_MODEL))
    except ValueError as exc:
        raise SupervisorError(str(exc), 400) from None
    try:
        if not isinstance(robot_ip, str):
            raise ValueError()
        ipaddress.IPv4Address(robot_ip)
    except ValueError:
        raise SupervisorError("robot_ip must be an IPv4 address", 400) from None
    backend = options.setdefault('camera_backend', 'outpost')
    if backend not in ('outpost', 'native'):
        raise SupervisorError('camera_backend must be outpost or native', 400)
    try:
        options['outpost_http'] = validate_origin(options.get('outpost_http', 'http://127.0.0.1:8100'))
    except (ValueError, TypeError) as exc:
        raise SupervisorError(str(exc), 400) from None
    for key in ('outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial'):
        value = options.setdefault(key, '')
        if not isinstance(value, str) or len(value) > 128 or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in value):
            raise SupervisorError(key + ' must be a camera identifier', 400)
    for key in ("launch_rviz", "launch_zed_driver", "launch_d405_driver"):
        options.setdefault(key, profile != "fake" and (key == 'launch_rviz' or backend == 'native'))
        if type(options[key]) is not bool:
            raise SupervisorError(f"{key} must be a JSON boolean", 400)
    if backend == 'outpost' and (options['launch_zed_driver'] or options['launch_d405_driver']):
        raise SupervisorError('Outpost owns cameras; native camera drivers must be disabled', 400)
    common = dict(options)
    common.pop("profile")
    if profile == 'fake':
        common['camera_backend'] = 'native'
    common.update(model_calibration_files(workspace, options['model_id']))
    common.update(use_fake_hardware=profile == "fake", use_isaac_sim=False, use_sim_time=False,
                  real_painting_enabled=profile in ("work", "spray_motion_test"),
                  dry_run=profile not in ("work", "spray_motion_test"),
                  painting_force_enabled=profile == "work",
                  spray_motion_test=profile == "spray_motion_test")
    groups = (
        ("robot_control", MODEL_LABELS[options['model_id']] + " · MoveIt · controllers · RViz", (), ("controller_manager", "move_group")),
        ("perception", "ZED · D405 · calibration · sketch perception", ("robot_control",), ("target_selector", "d405_surface_refiner")),
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
    def __init__(self, workspace, monitor, options=None, *, specs=None, grace=1.0, stop_timeout=8.0):
        self.workspace, self.monitor = Path(workspace), monitor
        self.options, built = build_specs(workspace, options)
        self.grace, self.stop_timeout = grace, stop_timeout
        self.log_dir = self.workspace / "logs/system_api"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.created = time.monotonic()
        self.lock = asyncio.Lock()
        self.event_list = deque(maxlen=500)
        self.monitor_task = None
        self._set_specs(built if specs is None else specs)

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

    def status(self):
        processes = [self.snapshot(name) for name in self.records]
        return {"supervisor": "running", "uptime_sec": round(time.monotonic() - self.created, 2),
                "monitor_interval_sec": 0.5, "system_prepared": all(p["state"] == "RUNNING" for p in processes),
                "teleoperation_active": False,
                "degraded": any(p["state"] in ("FAILED", "EXITED") for p in processes),
                "processes": processes, "configuration": self.options,
                "ros": self.monitor.snapshot(), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0")}

    async def configure(self, options):
        async with self.lock:
            if any(r["process"] is not None for r in self.records.values()):
                raise SupervisorError("Shutdown the system before changing configuration")
            options, specs = build_specs(self.workspace, options)
            self.options = options
            self._set_specs(specs)
            return options

    def _validate_model_calibration(self):
        if (self.options['model_id'] != DEFAULT_MODEL
                and self.options['profile'] != 'fake'):
            try:
                validate_calibration_files(model_calibration_files(self.workspace, self.options['model_id']))
            except ValueError as exc:
                raise SupervisorError(str(exc)) from None

    def _validate_cameras(self):
        if self.options['camera_backend'] != 'outpost' or self.options['profile'] == 'fake':
            return
        for camera, kind in (('zed', 'zed'), ('d405', 'realsense')):
            try:
                camera_status(self.options['outpost_http'], self.options[f'outpost_{camera}_hw_id'],
                              self.options[f'outpost_{camera}_serial'], kind)
            except (ValueError, OSError, KeyError, TypeError) as exc:
                raise SupervisorError('Outpost: ' + str(exc)) from None

    def preflight(self, name):
        r = self._record(name)
        if name == 'perception':
            self._validate_model_calibration()
            self._validate_cameras()
        graph = self.monitor.snapshot()
        if not graph["graph_fresh"]:
            raise SupervisorError("ROS discovery is not ready; retry shortly")
        conflicts = set(r["spec"].nodes)
        if name == "perception":
            if self.options['camera_backend'] == 'outpost':
                conflicts.add('sketch_outpost_bridge')
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
                if sock.connect_ex(("127.0.0.1", 9090)) == 0:
                    raise SupervisorError("Port 9090 is already occupied")

    async def _start(self, name):
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
            with path.open("ab", buffering=0) as log:
                p = await asyncio.create_subprocess_exec(*r["spec"].command,
                    cwd=self.workspace, stdin=asyncio.subprocess.DEVNULL,
                    stdout=log, stderr=asyncio.subprocess.STDOUT, start_new_session=True)
            r.update(process=p, last_pid=p.pid, started=time.monotonic(), last_started_at=utcnow(),
                     last_stopped_at=None, last_exit_code=None, start_count=r["start_count"] + 1)
            await asyncio.sleep(self.grace)
            if p.returncode is not None:
                raise SupervisorError(f"{name} exited during startup ({p.returncode}); inspect logs")
            r["state"] = "RUNNING"
            self._event("started", name, "Process passed startup liveness check")
            return self.snapshot(name)
        except (OSError, SupervisorError) as exc:
            await self._stop(name)
            r["state"], r["error"] = "FAILED", str(exc)
            raise SupervisorError(str(exc)) from exc

    async def start(self, name):
        async with self.lock:
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
        for sig, timeout in ((signal.SIGINT, self.stop_timeout), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
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
        async with self.lock:
            self._record(name)
            dependents = self._dependents(name)
            if dependents and not cascade:
                raise SupervisorError("Active dependents; use cascade=true: " + ", ".join(dependents))
            for n in reversed(dependents):
                await self._stop(n)
            return await self._stop(name)

    async def restart(self, name, cascade=False):
        async with self.lock:
            self._record(name)
            dependents = self._dependents(name)
            if dependents and not cascade:
                raise SupervisorError("Active dependents; use cascade=true")
            for n in reversed(dependents):
                await self._stop(n)
            await self._stop(name)
            return await self._start(name)

    async def prepare(self):
        async with self.lock:
            self._validate_model_calibration()
            self._validate_cameras()
            started = []
            try:
                for name in self.records:
                    was_running = self.snapshot(name)["state"] == "RUNNING"
                    await self._start(name)
                    if not was_running:
                        started.append(name)
            except Exception:
                for name in reversed(started):
                    try:
                        await self._stop(name)
                    except SupervisorError as exc:
                        self._event("rollback_error", name, str(exc))
                raise
            return {"started": started, "processes": self.status()["processes"],
                    "note": "RUNNING is process liveness. Check ros.readiness before work."}

    async def shutdown(self):
        async with self.lock:
            stopped, errors = [], []
            for name in reversed(self.records):
                try:
                    await self._stop(name)
                    stopped.append(name)
                except SupervisorError as exc:
                    errors.append(str(exc))
            if errors:
                raise SupervisorError("; ".join(errors), 503)
            return {"system": "stopped", "stopped": stopped}

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
