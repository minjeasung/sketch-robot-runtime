import json,sys,hashlib
from pathlib import Path
from types import SimpleNamespace
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/sketch_control')]
from prepare_platform_assets import generate_urdf_assets
from sketch_control.runtime_selection import normalize_selection
from sketch_control.weld_contract import load_tool_profile
from sketch_control.weld_execution import tool_attachment,check_tool_attachment


def test_native_weld_has_no_dummy_attachment_and_checks_description(tmp_path):
    b=generate_urdf_assets(ROOT,normalize_selection({'process':'weld','active_arm':'right'}),tmp_path)
    path=tmp_path/'weld_tool.json'
    data={'profile_id':'hongik_v4','revision':b['revision'],'simulation_only':True,'models':['construct_right'],
          'tip_translation_m':[0,0,0],'tip_quaternion_xyzw':[0,0,0,1],'reference_up':[0,-1,0],'radius_m':.006,
          'collision_mode':'native_urdf','robot_description_file':b['urdf'],
          'robot_description_sha256':hashlib.sha256(Path(b['urdf']).read_bytes()).hexdigest(),
          'native_collision_links':[n for n in b['arms']['right']['tool_links'] if n.endswith('hongik_eoat_base_link')]}
    path.write_text(json.dumps(data));profile=load_tool_profile(path,'construct_right')
    assert tool_attachment(profile,'right_manipulator_ee_point') is None
    scene=SimpleNamespace(robot_state=SimpleNamespace(attached_collision_objects=[]))
    check_tool_attachment(scene,profile,'right_manipulator_ee_point')
    scene.robot_state.attached_collision_objects=[SimpleNamespace(object=SimpleNamespace(id='weld_demo_torch'))]
    with pytest.raises(ValueError):check_tool_attachment(scene,profile,'right_manipulator_ee_point')
    Path(b['urdf']).write_text('<robot/>')
    with pytest.raises(ValueError):load_tool_profile(path,'construct_right')


@pytest.fixture
def rb10_native_weld(tmp_path):
    bundle=generate_urdf_assets(ROOT,normalize_selection({'platform':'rb10','process':'weld'}),tmp_path)
    return bundle, tmp_path/'weld_tool.json'


def test_rb10_native_weld_runtime_accepts_generated_simulator(rb10_native_weld):
    from sketch_control.weld_execution import validate_weld_runtime
    bundle,path=rb10_native_weld
    profile=validate_weld_runtime('rb10_1300e_u',True,path,bundle['urdf'])
    assert profile.collision_mode=='native_urdf'
    assert tool_attachment(profile,'tcp') is None


@pytest.mark.parametrize('fault',['real_clock','no_generated','wrong_description','physical_plugin',
                                  'missing_joint','wrong_topic','detached_torch','demo_profile','wrong_model'])
def test_rb10_native_weld_runtime_fails_closed(rb10_native_weld,tmp_path,fault):
    from sketch_control.weld_execution import validate_weld_runtime
    import xml.etree.ElementTree as ET
    bundle,path=rb10_native_weld
    generated=bundle['urdf'];sim=True;model='rb10_1300e_u'
    data=json.loads(path.read_text());robot=ET.parse(generated)
    if fault=='real_clock':sim=False
    elif fault=='no_generated':generated=''
    elif fault=='wrong_description':
        other=tmp_path/'other.urdf';other.write_bytes(Path(generated).read_bytes());generated=str(other)
    elif fault=='physical_plugin':robot.find('.//hardware/plugin').text='rbpodo_hardware/RBSystem'
    elif fault=='missing_joint':
        control=robot.find('ros2_control');control.remove(control.find('joint'))
    elif fault=='wrong_topic':robot.find(".//hardware/param[@name='joint_commands_topic']").text='/physical_commands'
    elif fault=='detached_torch':
        name=data['native_collision_links'][0]
        for joint in robot.findall('joint'):
            if joint.find('child').get('link')==name:robot.getroot().remove(joint)
    elif fault=='demo_profile':data['collision_mode']='attached_demo'
    elif fault=='wrong_model':model='rb20_1900es'
    if fault in ('physical_plugin','missing_joint','wrong_topic','detached_torch'):
        robot.write(bundle['urdf']);data['robot_description_sha256']=hashlib.sha256(Path(bundle['urdf']).read_bytes()).hexdigest()
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):validate_weld_runtime(model,sim,path,generated)


def test_legacy_construct_weld_profile_remains_simulation_only():
    from sketch_control.weld_execution import validate_weld_runtime
    profile=ROOT/'src/sketch_control/config/weld_sim_profile.json'
    assert validate_weld_runtime('construct_left',True,profile).collision_mode=='attached_demo'
    with pytest.raises(ValueError):validate_weld_runtime('construct_left',False,profile)


@pytest.mark.parametrize('side',['left','right'])
def test_generated_commercial_native_weld_still_accepted(tmp_path,side):
    from sketch_control.weld_execution import validate_weld_runtime
    bundle=generate_urdf_assets(ROOT,normalize_selection({'process':'weld','active_arm':side}),tmp_path)
    profile=validate_weld_runtime('construct_'+side,True,tmp_path/'weld_tool.json',bundle['urdf'])
    assert profile.collision_mode=='native_urdf'
