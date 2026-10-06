"""Manage only children created by this add-on, never the external stack."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from .config import check_platform, runtime_environment, validate_paths, owned_path
from .compatibility import require_upstream


def repository_root():
    return Path(__file__).resolve().parents[3]


def component_commands(config, config_path, profile_path=''):
    base = [sys.executable, '-B', '-m', 'snucem_spray_addon', '--config', str(config_path)]
    names = ('perception', 'bridge', 'model', 'projector', 'generator',
             'preview' if config.profile == 'preview' else 'executor')
    result = {name: base+[name]+(['--model-profile', profile_path] if name in ('generator', 'executor') else []) for name in names}
    if config.own_rosbridge:
        result['rosbridge'] = [sys.executable, '-B', '-c', 'from ros2cli.cli import main; main()',
            'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']
    return result


class Runtime:
    def __init__(self, config, config_path):
        self.config, self.config_path = config, Path(config_path).resolve()
        self.children, self.logs = {}, {}
        self.lock = threading.RLock()
        self.error = ''

    def preflight(self):
        release = {}
        for line in Path('/etc/os-release').read_text().splitlines():
            key, _, value = line.partition('=')
            release[key] = value.strip('"')
        check_platform(release.get('VERSION_ID'), os.environ.get('ROS_DISTRO'), sys.version_info)
        validate_paths(self.config.upstream_root, self.config.install_root, self.config.state_root)
        require_upstream(self.config.upstream_root)
        if not Path(self.config.calibration_file).is_file():
            raise ValueError('active calibration_file required')
        import rclpy  # noqa: F401
        from moveit_msgs.srv import GetCartesianPath  # noqa: F401

    def environment(self):
        root = repository_root()
        up = Path(self.config.upstream_root)
        env = runtime_environment(self.config)
        modules = [root/'addons/snucem_spray', root/'src/sketch_control', root/'src/rbpodo_painting_control',
                   up/'linux/control', up/'interfaces/python', up/'linux/gateway',
                   up/'linux/drivers/ros2_ws/src/rb10_realsense_moveit/scripts']
        env['PYTHONPATH'] = os.pathsep.join(map(str, modules))+os.pathsep+env.get('PYTHONPATH', '')
        env['AMENT_PREFIX_PATH'] = str(root)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
        return env

    def start(self, name, profile=''):
        with self.lock:
            commands = component_commands(self.config, self.config_path, profile)
            if name not in commands:
                raise ValueError('not an owned component: '+name)
            if name in self.children and self.children[name].poll() is None:
                return
            state = Path(self.config.state_root)
            validate_paths(self.config.upstream_root, self.config.install_root, state)
            owned_path(state, 'logs').mkdir(parents=True, exist_ok=True)
            log = owned_path(state, 'logs/'+name+'.log').open('ab')
            try:
                child = subprocess.Popen(commands[name], cwd=state, env=self.environment(),
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            except Exception:
                log.close()
                raise
            self.children[name], self.logs[name] = child, log

    def wait_model(self):
        path = Path(self.config.state_root)/'model-status.json'
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            self.require_children_alive()
            try:
                status = json.loads(path.read_text())
                if status.get('fingerprint') and 0 <= time.time()-status['written_at'] <= 1:
                    profile = Path(status['profile']).resolve()
                    if not profile.is_relative_to(Path(self.config.state_root).resolve()/'models'):
                        raise ValueError('model profile outside owned state')
                    return str(profile)
            except (OSError, ValueError, KeyError):
                pass
            time.sleep(.1)
        raise RuntimeError('live model unavailable; inspect model.log and external stack/ROS domain')

    def require_children_alive(self):
        for name, child in self.children.items():
            if child.poll() is not None:
                raise RuntimeError(name+' exited; inspect its owned log')

    def prepare(self):
        with self.lock:
            self.preflight()
            if self.children:
                self.require_children_alive()
                return self.status()
            created = []
            try:
                for name in ('model',):
                    self.start(name)
                    created.append(name)
                profile = self.wait_model()
                for name in ('bridge', 'perception', 'projector', 'generator',
                             'preview' if self.config.profile == 'preview' else 'executor'):
                    self.start(name, profile)
                    created.append(name)
                if self.config.own_rosbridge:
                    self.start('rosbridge')
                    created.append('rosbridge')
                time.sleep(.5)
                self.require_children_alive()
                self.error = ''
                return self.status()
            except Exception as exc:
                self.error = str(exc)
                for name in reversed(created):
                    self.stop(name)
                raise

    def stop(self, name):
        with self.lock:
            child = self.children.pop(name, None)
            if child is not None and child.poll() is None:
                # Each child has a new session. Only its own process group is touched.
                if os.name == 'posix':
                    os.killpg(child.pid, signal.SIGINT)
                else:
                    child.terminate()
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    if os.name == 'posix':
                        os.killpg(child.pid, signal.SIGKILL)
                    else:
                        child.kill()
                    child.wait(timeout=2)
            log = self.logs.pop(name, None)
            if log:
                log.close()

    def shutdown(self):
        with self.lock:
            for name in ('executor', 'preview', 'generator', 'projector', 'perception', 'bridge', 'model', 'rosbridge'):
                self.stop(name)
            return self.status()

    def status(self):
        return dict(owned={name: dict(pid=child.pid, running=child.poll() is None) for name, child in self.children.items()},
                    external=dict(stack='external; never started or stopped here',
                                  rosbridge='owned' if self.config.own_rosbridge else 'external'), error=self.error)
