import ast
import importlib
import json
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'addons/snucem_spray'))
sys.path.insert(0, str(ROOT / 'src/sketch_control'))
sys.path.insert(0, str(ROOT / 'src/rbpodo_painting_control'))


def module(name):
    assert (ROOT / 'addons/snucem_spray/snucem_spray_addon' / (name + '.py')).exists(), name + ' implementation missing'
    return importlib.import_module('snucem_spray_addon.' + name)


def test_owned_roots_cannot_overlap_upstream(tmp_path):
    paths = module('config')
    upstream = tmp_path / 'upstream'
    upstream.mkdir()
    for install, state in [(upstream, tmp_path/'state'), (upstream/'addon', tmp_path/'state'), (tmp_path, tmp_path/'state')]:
        with pytest.raises(ValueError, match='overlap'):
            paths.validate_paths(upstream, install, state)
    paths.validate_paths(upstream, tmp_path/'addon', tmp_path/'state')


def test_only_humble_spray_profiles_and_owned_environment(tmp_path):
    m = module('config')
    upstream = tmp_path/'upstream'
    upstream.mkdir()
    cfg = m.AddonConfig(str(upstream), str(tmp_path/'addon'), str(tmp_path/'state'))
    env = m.runtime_environment(cfg, {})
    assert env['PYTHONDONTWRITEBYTECODE'] == '1'
    assert Path(env['ROS_LOG_DIR']).is_relative_to(tmp_path/'state')
    assert cfg.profile == 'preview'
    with pytest.raises(ValueError):
        m.AddonConfig(str(upstream), str(tmp_path/'addon'), str(tmp_path/'state'), profile='paint')
    with pytest.raises(ValueError):
        m.check_platform('24.04', 'jazzy', (3, 12))
    m.check_platform('22.04', 'humble', (3, 10), 'aarch64')


@pytest.mark.parametrize('machine', ['aarch64', 'arm64'])
def test_arm64_humble_is_an_installable_platform(machine):
    module('config').check_platform('22.04', 'humble', (3, 10), machine)


@pytest.mark.parametrize('machine', ['x86_64', 'AMD64', 'armv7l', 'arm', ''])
def test_other_architectures_are_rejected_before_runtime_start(machine):
    with pytest.raises(ValueError, match='ARM64'):
        module('config').check_platform('22.04', 'humble', (3, 10), machine)


def test_platform_check_uses_actual_machine_by_default(monkeypatch):
    import platform
    monkeypatch.setattr(platform, 'machine', lambda: 'x86_64')
    with pytest.raises(ValueError, match='ARM64'):
        module('config').check_platform('22.04', 'humble', (3, 10))


@pytest.mark.parametrize('machine,supported', [('aarch64', True), ('x86_64', False)])
def test_doctor_reports_arm64_support_without_loading_camera_sdk(tmp_path, monkeypatch, machine, supported):
    cli = module('cli')
    upstream = tmp_path/'up'
    upstream.mkdir()
    calibration = tmp_path/'calibration.yaml'
    calibration.write_text('measured: true')
    cfg = module('config').AddonConfig(str(upstream), str(tmp_path/'install'), str(tmp_path/'state'),
                                     calibration_file=str(calibration))
    original_read = Path.read_text
    def read_text(path, *args, **kwargs):
        return 'VERSION_ID="22.04"' if path == Path('/etc/os-release') else original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(cli.platform, 'machine', lambda: machine)
    monkeypatch.setattr(cli.platform, 'system', lambda: 'Linux')
    monkeypatch.setattr(cli.sys, 'version_info', (3, 10, 12))
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    monkeypatch.setattr(cli, 'check_upstream', lambda root: {'compatible': True})
    # These are separately installed ROS/upstream dependencies, not GPU SDK imports.
    monkeypatch.setattr(cli.importlib.util, 'find_spec', lambda name: None if name in ('pyzed', 'cuda') else object())
    report = cli.doctor(cfg)
    assert report['platform']['supported'] is supported
    assert report['ok'] is supported


def test_symlink_write_alias_is_rejected(tmp_path):
    m = module('config')
    up = tmp_path/'up'
    up.mkdir()
    alias = tmp_path/'alias'
    try:
        alias.symlink_to(up, target_is_directory=True)
    except OSError:
        pytest.skip('Windows symlink privilege unavailable')
    with pytest.raises(ValueError, match='overlap'):
        m.validate_paths(up, alias/'install', tmp_path/'state')


def test_compatibility_fails_on_modified_interface(tmp_path):
    m = module('compatibility')
    report = m.check_upstream(tmp_path)
    assert not report['compatible']
    assert report['missing']
    assert m.UPSTREAM_REPOSITORY.endswith('SNUCEM_Robot_22.04')
    assert len(m.UPSTREAM_REVISION) == 40


def test_owned_child_rejects_traversal_and_escaped_resolution(tmp_path, monkeypatch):
    m = module('config')
    root = tmp_path/'state'
    root.mkdir()
    with pytest.raises(ValueError):
        m.owned_path(root, '../up/file')
    original = Path.resolve
    escaped = tmp_path/'up'
    def resolve(path, *args, **kwargs):
        if path == root/'logs':
            return escaped
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'resolve', resolve)
    with pytest.raises(ValueError):
        m.owned_path(root, 'logs')


def test_hardlink_write_alias_is_rejected(tmp_path):
    m = module('config')
    up, state = tmp_path/'up', tmp_path/'state'
    up.mkdir()
    state.mkdir()
    original = up/'original'
    original.write_text('preserve')
    try:
        os.link(original, state/'active-version.tmp')
    except OSError:
        pytest.skip('filesystem does not support hard links')
    with pytest.raises(ValueError, match='hard link'):
        m.owned_path(state, 'active-version.tmp')
    assert original.read_text() == 'preserve'


def test_python_sources_compile_as_python310():
    for path in (ROOT/'addons/snucem_spray').rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), feature_version=(3, 10))
