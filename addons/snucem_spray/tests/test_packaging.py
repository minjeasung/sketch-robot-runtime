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
