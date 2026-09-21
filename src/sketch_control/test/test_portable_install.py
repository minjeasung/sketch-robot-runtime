"""Distribution excludes local credentials and includes current source edits."""
import importlib.util
import json
from pathlib import Path
import tarfile

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('export_runtime', ROOT/'scripts/export_runtime.py')
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


def test_bundle_includes_source_but_not_credentials_or_builds(tmp_path):
    for name in ('src/pkg/code.py', 'src/pkg/__pycache__/code.pyc', 'src/pkg/code.py.bak',
                 'config/sketch_runtime.env', 'config/sketch_runtime.env.example',
                 'config/device.local.json', 'build/lib.so', '.venv-api/token',
                 'web/index.html', 'scripts/run.sh', 'docs/README.md', 'README.md',
                 'zed_d405_apriltag_calibration.json'):
        p=tmp_path/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('current contents')
    output=tmp_path/'release.tar.gz'
    exporter.export(tmp_path,output)
    with tarfile.open(output) as archive:
        names=set(archive.getnames())
        assert 'sketch_robot_ws/src/pkg/code.py' in names
        assert 'sketch_robot_ws/config/sketch_runtime.env.example' in names
        assert not any(n.endswith('.env') or '.local.' in n or '.bak' in n or '__pycache__' in n for n in names)
        assert 'sketch_robot_ws/zed_d405_apriltag_calibration.json' not in names
        manifest=json.load(archive.extractfile('sketch_robot_ws/BUNDLE_MANIFEST.json'))
        assert manifest['kind']=='software-source'
    assert output.with_name(output.name+'.sha256').exists()
    with pytest.raises(FileExistsError):
        exporter.export(tmp_path,output)


def test_calibration_export_has_only_whitelist(tmp_path):
    for name in (*exporter.CALIBRATION_FILES, 'secret.env'):
        (tmp_path/name).write_text('{}')
    output=tmp_path/'calibration.tar.gz'
    exporter.export(tmp_path,output,True)
    with tarfile.open(output) as archive:
        assert set(archive.getnames())=={'sketch_robot_ws/'+n for n in (*exporter.CALIBRATION_FILES, 'CALIBRATION_MANIFEST.json')}


def test_export_rejects_symlinks(tmp_path):
    (tmp_path/'src').mkdir()
    (tmp_path/'src/external').symlink_to('/etc/passwd')
    with pytest.raises(ValueError, match='symbolic link'):
        exporter.export(tmp_path,tmp_path/'release.tar.gz')


def test_calibration_uses_launcher_workspace(monkeypatch,tmp_path):
    monkeypatch.setenv('SKETCH_WORKSPACE',str(tmp_path))
    for name in ('rb10_perception_sketch.launch.py', 'rb10_real_perception_sketch.launch.py'):
        module_spec=importlib.util.spec_from_file_location('portable_launch',ROOT/'src/sketch_control/launch'/name)
        module=importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
        assert Path(module.DEFAULT_D405_CALIBRATION_FILE).parent==tmp_path
