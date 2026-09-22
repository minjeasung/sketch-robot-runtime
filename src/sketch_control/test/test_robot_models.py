"""Arm substitution must preserve tool geometry and the selected model contract."""
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xacro

from sketch_control.robot_models import (
    DEFAULT_MODEL, MODEL_LABELS, JOINT_NAMES, validate_model, model_srdf,
    model_joint_limits, model_calibration_files, validate_calibration_files,
)
from sketch_control.process_supervisor import build_specs, SupervisorError
from sketch_control.moveit_executor import MoveItExecutor, JOINT_LIMITS

SRC = Path(__file__).resolve().parents[2]
PACKAGE = SRC / 'sketch_control'
DESCRIPTION = SRC / 'rbpodo_ros2/rbpodo_description'
MOVEIT = SRC / 'rbpodo_ros2/rbpodo_moveit_config'


@pytest.fixture(scope='module')
def models():
    return {model: ET.fromstring(xacro.process_file(
        str(PACKAGE/'urdf/rbpodo_with_eoat.urdf.xacro'),
        mappings=dict(model_id=model, use_fake_hardware='true',
                      fake_sensor_commands='true', use_isaac_sim='false'),
    ).toxml()) for model in MODEL_LABELS}


def transform(joint):
    origin = joint.find('origin')
    result = np.eye(4)
    result[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


@pytest.mark.parametrize('model', MODEL_LABELS)
def test_selected_arm_has_complete_meshes_and_fake_hardware_is_disconnected(models, model):
    from ament_index_python.packages import get_package_share_directory
    root = models[model]
    assert [j.get('name') for j in root.findall('joint') if j.get('type')=='revolute'] == list(JOINT_NAMES)
    arm_meshes = []
    for mesh in root.findall('.//mesh'):
        url = mesh.get('filename')
        assert url.startswith('package://')
        package, relative = url.removeprefix('package://').split('/',1)
        assert (Path(get_package_share_directory(package))/relative).is_file(), url
        if package == 'rbpodo_description': arm_meshes.append(url)
    assert len(arm_meshes)==14 and all('/'+model+'/' in name for name in arm_meshes)
    hardware = root.find('ros2_control/hardware')
    assert hardware.find("param[@name='fake_mode']").text.lower()=='true'
    assert hardware.find("param[@name='fake_sensor_commands']").text.lower()=='true'


def test_rb20_arm_matches_bundled_native_model_and_tool_points_out_of_flange(models):
    root=models['rb20_1900es']
    native=ET.parse(DESCRIPTION/'robots/rb20_1900es.urdf').getroot()
    for name in JOINT_NAMES:
        actual=root.find(f"joint[@name='{name}']")
        expected=native.find(f"joint[@name='{name}']")
        np.testing.assert_allclose(transform(actual),transform(expected),atol=1e-12)
        assert actual.find('axis').get('xyz') == expected.find('axis').get('xyz')
    tcp=transform(root.find("joint[@name='ft_sensor_joint']")) @ transform(root.find("joint[@name='tcp_joint']"))
    np.testing.assert_allclose(tcp[:3,3],[0,0,.13],atol=1e-12)
    np.testing.assert_allclose(-tcp[:3,1],[0,0,1],atol=1e-12)
    np.testing.assert_allclose(tcp[:3,0],[1,0,0],atol=1e-12)
    rb10=models[DEFAULT_MODEL]
    # Everything downstream of application TCP is unchanged.
    for joint in rb10.findall('joint'):
        if joint.get('name') in JOINT_NAMES or joint.get('name') in ('ft_sensor_joint','tcp_joint'): continue
        other=root.find("joint[@name='%s']"%joint.get('name'))
        assert other is not None
        np.testing.assert_allclose(transform(other),transform(joint),atol=1e-12)


@pytest.mark.parametrize('model', MODEL_LABELS)
def test_selected_limits_and_semantic_collision_baseline_match_executor(model):
    assert model_joint_limits(model,DESCRIPTION,MOVEIT)==JOINT_LIMITS
    root=ET.fromstring(model_srdf(model,PACKAGE/'config/rbpodo_named.srdf'))
    expected={(e.get('link1'),e.get('link2')) for e in root.findall('disable_collisions')}
    assert set(MoveItExecutor._load_srdf_allowed_collision_pairs(model))==expected
    if model!='rb10_1300e_u':
        assert root.findall('group_state')==[]
        assert all(e.get('reason')!='Never' for e in root.findall('disable_collisions'))


@pytest.mark.parametrize('profile',['dry_run','work','fake','spray_motion_test'])
def test_every_process_uses_selected_rb20_and_separate_camera_calibrations(tmp_path,profile):
    options,specs=build_specs(tmp_path,dict(profile=profile,model_id='rb20_1900es'))
    assert options['model_id']=='rb20_1900es'
    for spec in specs:
        assert 'model_id:=rb20_1900es' in spec.command
        assert f'zed_calibration_file:={tmp_path}/calibration/rb20_1900es/zed_d405_apriltag_calibration.json' in spec.command


@pytest.mark.parametrize('model',['rb20','rb99','../rb20_1900es',True,None])
def test_unknown_models_are_rejected(tmp_path,model):
    with pytest.raises(ValueError):validate_model(model)
    with pytest.raises(SupervisorError):build_specs(tmp_path,dict(model_id=model))


def test_rb20_never_sends_rb10_taught_joint_presets():
    messages=[]
    node=SimpleNamespace(model_id='rb20_1900es',executing=True,
                         get_logger=lambda:SimpleNamespace(warn=messages.append))
    MoveItExecutor._start_preset_motion(node,'ready')
    MoveItExecutor.stage5_return_to_ready(node)
    assert not node.executing and len(messages)==2


def test_rb20_calibration_is_separate_and_rejects_missing_or_invalid_transform(tmp_path):
    old=model_calibration_files(tmp_path,DEFAULT_MODEL)
    paths=model_calibration_files(tmp_path,'rb20_1900es')
    assert old!=paths
    with pytest.raises(ValueError,match='calibration required'):validate_calibration_files(paths)
    for name,key in [('zed_calibration_file','T_world_zed_optical'),('d405_calibration_file','T_d405_optical_to_tcp')]:
        path=Path(paths[name]);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps({key:dict(translation=[0,0,0],rotation_xyzw=[0,0,0,1])}))
    validate_calibration_files(paths)
    Path(paths['d405_calibration_file']).write_text(json.dumps({'T_d405_optical_to_tcp':dict(translation=[0,0,0],rotation_xyzw=[0,0,0,0])}))
    with pytest.raises(ValueError,match='Invalid rigid transform'):validate_calibration_files(paths)
