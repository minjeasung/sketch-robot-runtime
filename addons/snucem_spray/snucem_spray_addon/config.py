"""Validated configuration and an explicit owned-write boundary."""
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
from urllib.parse import urlparse


def validate_paths(upstream_root, install_root, state_root):
    roots = tuple(Path(p).expanduser().resolve() for p in (upstream_root, install_root, state_root))
    upstream, install, state = roots
    if not upstream.is_dir():
        raise ValueError('upstream_root must be an existing installation')
    for owned in (install, state):
        if owned == upstream or upstream in owned.parents or owned in upstream.parents:
            raise ValueError('owned write roots must not overlap upstream')
    if install == state or state in install.parents or install in state.parents:
        raise ValueError('install and state roots must not overlap')
    return roots


def check_platform(ubuntu, ros_distro, python_version):
    if ubuntu != '22.04' or ros_distro != 'humble' or tuple(python_version[:2]) != (3, 10):
        raise ValueError('requires Ubuntu 22.04, ROS 2 Humble and system Python 3.10')


def owned_path(root, relative):
    """Resolve each write destination before use, including existing child links.

    Owned directories must be writable only by the operator account; this is not
    a sandbox against a concurrent process replacing paths after validation.
    """
    root = Path(root).resolve()
    relative = Path(relative)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('write path must stay inside its owned root')
    path = root/relative
    if not path.resolve().is_relative_to(root):
        raise ValueError('write path escapes its owned root')
    if path.is_file() and path.stat().st_nlink > 1:
        raise ValueError('owned write files must not have hard links')
    for part in [path, *path.parents]:
        if part == root:
            break
        if part.is_symlink():
            raise ValueError('owned write paths must not contain symlinks')
    return path


@dataclass(frozen=True)
class AddonConfig:
    upstream_root: str
    install_root: str
    state_root: str
    profile: str = 'preview'
    ros_domain_id: int = 0
    rosbridge_url: str = 'ws://127.0.0.1:9090'
    image_topic: str = '/zed/zed_node/left/color/rect/image'
    camera_info_topic: str = '/zed/zed_node/left/color/rect/camera_info'
    points_topic: str = '/rb/spray/zed/points'
    model_id: str = 'rb20_1900es'
    api_host: str = '127.0.0.1'
    api_port: int = 8081
    api_token: str = ''
    perception_args: tuple = ()
    calibration_file: str = ''
    own_rosbridge: bool = False

    def __post_init__(self):
        roots = validate_paths(self.upstream_root, self.install_root, self.state_root)
        for name, value in zip(('upstream_root', 'install_root', 'state_root'), roots):
            object.__setattr__(self, name, str(value))
        if self.profile not in ('preview', 'dry_run', 'motion_test', 'spray'):
            raise ValueError('invalid Spray-only profile')
        if self.model_id not in ('rb10_1300e_u', 'rb20_1900es'):
            raise ValueError('unsupported robot model')
        if type(self.ros_domain_id) is not int or not 0 <= self.ros_domain_id <= 232:
            raise ValueError('invalid ROS domain')
        if type(self.api_port) is not int or not 1024 <= self.api_port <= 65535:
            raise ValueError('invalid API port')
        if type(self.own_rosbridge) is not bool:
            raise ValueError('own_rosbridge must be boolean')
        url = urlparse(self.rosbridge_url)
        if url.scheme not in ('ws', 'wss') or not url.hostname or url.username or url.password:
            raise ValueError('invalid rosbridge URL')
        for topic in (self.image_topic, self.camera_info_topic, self.points_topic):
            if not topic.startswith('/') or any(c.isspace() for c in topic):
                raise ValueError('invalid ROS topic')
        if self.api_host not in ('127.0.0.1', 'localhost', '::1') and len(self.api_token) < 24:
            raise ValueError('remote API requires a token of at least 24 characters')
        if not isinstance(self.perception_args, (list, tuple)) or not all(isinstance(v, str) for v in self.perception_args):
            raise ValueError('perception_args must be a list of arguments')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding='utf-8')))

    def public(self):
        return {k: v for k, v in asdict(self).items() if k != 'api_token'}


def runtime_environment(config, environ=None):
    validate_paths(config.upstream_root, config.install_root, config.state_root)
    env = dict(os.environ if environ is None else environ)
    state = Path(config.state_root)
    env.update(PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID=str(config.ros_domain_id),
               ROS_LOG_DIR=str(owned_path(state, 'logs')), ROS_HOME=str(owned_path(state, 'ros')),
               XDG_CACHE_HOME=str(owned_path(state, 'cache')), MPLCONFIGDIR=str(owned_path(state, 'cache/matplotlib')),
               PYTHONPYCACHEPREFIX=str(owned_path(state, 'cache/pycache')), PYTHONNOUSERSITE='1',
               PIP_CACHE_DIR=str(owned_path(state, 'cache/pip')))
    return env
