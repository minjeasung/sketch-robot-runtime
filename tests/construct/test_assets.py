import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('prepare', ROOT / 'scripts/prepare_construct_assets.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def test_assets_keep_model_and_simulation_contract(tmp_path):
    manifest = module.prepare(ROOT, tmp_path)
    robot = ET.parse(tmp_path / 'construct.urdf').getroot()
    joints = [j for j in robot.findall('joint') if j.get('type') == 'revolute']
    assert len(joints) == 14
    assert set(manifest['initial_positions']) == {j.get('name') for j in joints}
    for m in robot.findall('.//mesh'):
        assert Path(m.get('filename')).is_file()
    for j in joints:
        limits = j.find('limit')
        assert float(limits.get('lower')) <= manifest['initial_positions'][j.get('name')] <= float(limits.get('upper'))
    controlled = ET.parse(tmp_path / 'construct_control.urdf').getroot()
    assert [p.text for p in controlled.findall('.//hardware/plugin')] == ['topic_based_ros2_control/TopicBasedSystem']
    assert len(controlled.findall('./ros2_control/joint')) == 14
    assert manifest['groups']['robot_head_controller'] == ['robot_head_rev_joint1', 'robot_head_rev_joint2']

def test_missing_mesh_is_rejected(tmp_path):
    import shutil
    source = tmp_path / 'workspace'
    shutil.copytree(ROOT / 'src/construct_description', source / 'src/construct_description')
    shutil.copytree(ROOT / 'src/construct_moveit_config', source / 'src/construct_moveit_config')
    next((source / 'src/construct_description/meshes_0528').glob('*.stl')).unlink()
    with pytest.raises(FileNotFoundError):
        module.prepare(source, tmp_path / 'out')

def test_exported_camera_inertia_is_repaired_only_in_generated_assets(tmp_path):
    module.prepare(ROOT,tmp_path)
    source=ET.parse(ROOT/'src/construct_description/urdf_0528/construct_robot_0528.urdf').getroot()
    generated=ET.parse(tmp_path/'construct.urdf').getroot()
    original_mass=float(source.find("link[@name='right_D435_link']/inertial/mass").get('value'))
    assert original_mass > 200
    left=source.find("link[@name='left_D435_link']/inertial")
    right=generated.find("link[@name='right_D435_link']/inertial")
    assert ET.tostring(right) == ET.tostring(left)
    assert float(right.find('mass').get('value')) < .1

def test_head_has_simulation_acceleration_limits_for_time_parameterization(tmp_path):
    import yaml
    module.prepare(ROOT,tmp_path)
    limits=yaml.safe_load((tmp_path/'moveit_joint_limits.yaml').read_text())['joint_limits']
    for name in ('robot_head_rev_joint1','robot_head_rev_joint2'):
        assert limits[name]['has_acceleration_limits'] is True
        assert limits[name]['max_acceleration'] == .5
        assert limits[name]['has_jerk_limits'] is True
        assert limits[name]['max_jerk'] == 2.0


def test_moveit_meshes_are_file_uris_for_resource_retriever(tmp_path):
    from urllib.parse import urlparse,unquote
    module.prepare(ROOT,tmp_path)
    controlled=ET.parse(tmp_path/'construct_control.urdf').getroot()
    for mesh in controlled.findall('.//mesh'):
        uri=urlparse(mesh.get('filename'))
        assert uri.scheme=='file', 'ROS resource_retriever rejects bare absolute paths'
        assert Path(unquote(uri.path)).is_file()
