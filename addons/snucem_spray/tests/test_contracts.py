import ast
import importlib
import json
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
    m.check_platform('22.04', 'humble', (3, 10))


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


def test_python_sources_compile_as_python310():
    for path in (ROOT/'addons/snucem_spray').rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), feature_version=(3, 10))
