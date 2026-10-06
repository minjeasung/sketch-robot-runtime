import io
import json
from pathlib import Path
import tarfile
import pytest
from test_contracts import ROOT, module


def test_bundle_is_deterministic_small_and_contains_no_upstream_assets(tmp_path):
    m = module('packaging')
    first, manifest = m.build_bundle(ROOT, tmp_path/'one.tar.gz')
    second, _ = m.build_bundle(ROOT, tmp_path/'two.tar.gz')
    assert first.read_bytes() == second.read_bytes()
    assert first.stat().st_size <= 5_000_000
    assert manifest['unpacked_bytes'] <= 10_000_000
    paths = list(manifest['files'])
    assert 'web/js/app.js' in paths
    assert 'addons/snucem_spray/snucem_spray_addon/execution.py' in paths
    assert not any(any(part in p for part in ('vendor/', '.git/', 'meshes/', 'upstream-reference/')) for p in paths)


def test_install_preserves_config_and_upstream_and_rejects_active_upgrade(tmp_path):
    m = module('packaging')
    installer = module('install')
    bundle, manifest = m.build_bundle(ROOT, tmp_path/'bundle.tar.gz')
    up = tmp_path/'up'
    up.mkdir()
    (up/'original').write_bytes(b'unchanged')
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    state = Path(cfg.state_root)
    state.mkdir()
    config_path = state/'config.json'
    config_path.write_text('{"keep":"operator settings"}')
    result = installer.install_bundle(bundle, cfg)
    assert (Path(result['version_root'])/'web/index.html').is_file()
    assert config_path.read_text() == '{"keep":"operator settings"}'
    assert (up/'original').read_bytes() == b'unchanged'
    with pytest.raises(ValueError, match='active'):
        installer.install_bundle(bundle, cfg, service_active=True)


def test_installer_rejects_traversal_before_creating_files(tmp_path):
    installer = module('install')
    up = tmp_path/'up'
    up.mkdir()
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    archive = tmp_path/'bad.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        member = tarfile.TarInfo('../escape')
        member.size = 1
        tar.addfile(member, io.BytesIO(b'x'))
    with pytest.raises(ValueError):
        installer.install_bundle(archive, cfg)
    assert not (tmp_path/'escape').exists()


@pytest.mark.parametrize('field,value', [('architecture', 'x86_64'), ('cuda', '13.0')])
def test_installer_rejects_bundle_for_another_robot_platform(tmp_path, field, value):
    bundle, _ = module('packaging').build_bundle(ROOT, tmp_path/'bundle.tar.gz')
    contents = module('install').verified_members(bundle)
    manifest = json.loads(contents['manifest.json'])
    manifest[field] = value
    contents['manifest.json'] = json.dumps(manifest).encode()
    wrong = tmp_path/'wrong-platform.tar.gz'
    with tarfile.open(wrong, 'w:gz') as archive:
        for name, data in contents.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
    up = tmp_path/'up'
    up.mkdir()
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    with pytest.raises(ValueError, match='manifest'):
        module('install').install_bundle(wrong, cfg)
    assert not Path(cfg.install_root).exists()


def test_environment_setup_failure_does_not_activate_new_version(tmp_path, monkeypatch):
    from dataclasses import asdict
    import subprocess
    cli = module('cli')
    bundle, _ = module('packaging').build_bundle(ROOT, tmp_path/'bundle.tar.gz')
    up = tmp_path/'up'
    up.mkdir()
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    state = Path(cfg.state_root)
    state.mkdir()
    active = state/'active-version.json'
    active.write_text('{"version":"previous"}')
    config_file = state/'config.json'
    config_file.write_text(json.dumps(asdict(cfg)), encoding='utf-8')
    original_read = Path.read_text
    def read_text(path, *args, **kwargs):
        return 'VERSION_ID="22.04"' if path == Path('/etc/os-release') else original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(cli, 'check_platform', lambda *args: None)
    def setup_failure(*args, **kwargs):
        raise subprocess.CalledProcessError(1, args[0])
    monkeypatch.setattr(cli.subprocess, 'run', setup_failure)
    with pytest.raises(subprocess.CalledProcessError):
        cli.main(['--config', str(config_file), 'install', '--bundle', str(bundle), '--setup-env'])
    assert json.loads(active.read_text()) == {'version': 'previous'}


def test_environment_install_rejects_wrong_cpu_before_writing(tmp_path, monkeypatch):
    from dataclasses import asdict
    cli = module('cli')
    bundle, _ = module('packaging').build_bundle(ROOT, tmp_path/'bundle.tar.gz')
    up = tmp_path/'up'
    up.mkdir()
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    config_file = tmp_path/'config.json'
    config_file.write_text(json.dumps(asdict(cfg)), encoding='utf-8')
    original_read = Path.read_text
    def read_text(path, *args, **kwargs):
        return 'VERSION_ID="22.04"' if path == Path('/etc/os-release') else original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(cli.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(cli.sys, 'version_info', (3, 10, 12))
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    with pytest.raises(ValueError, match='ARM64'):
        cli.main(['--config', str(config_file), 'install', '--bundle', str(bundle), '--setup-env'])
    assert not Path(cfg.install_root).exists()
    assert not Path(cfg.state_root).exists()
