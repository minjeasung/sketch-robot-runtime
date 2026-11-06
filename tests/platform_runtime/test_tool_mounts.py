import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/sketch_control')]
from sketch_control.runtime_selection import normalize_selection
from prepare_platform_assets import generate_urdf_assets
from sketch_control.eoat_catalog import load_tool, weld_tip_offset


@pytest.mark.parametrize('platform,arm', [('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('process', ['spray','weld','paint'])
def test_each_arm_process_preserves_tree_and_tcp(tmp_path,platform,arm,process):
    selection=normalize_selection(dict(platform=platform,active_arm=arm,process=process))
    bundle=generate_urdf_assets(ROOT,selection,tmp_path)
    robot=ET.parse(bundle['urdf']).getroot()
    names=[n.get('name') for n in robot.findall('link')]
    assert len(names)==len(set(names))
    joints=robot.findall('joint')
    assert len(joints)==len(names)-1
    assert all(j.find('parent').get('link') in names and j.find('child').get('link') in names for j in joints)
    assert not any('D435' in n for n in names)
    arm_info=bundle['arms'][arm]
    assert arm_info['tcp_link'] in names
    assert arm_info['tool_id']==selection.tools[arm]
    assert len(arm_info['joints'])==6
    semantic=ET.parse(bundle['srdf']).getroot()
    assert semantic.get('name')==robot.get('name')
    assert 'scripts/platform_asset_config.py' in bundle['source_hashes']
    assert all(p.get('link1') in names and p.get('link2') in names for p in semantic.findall('disable_collisions'))
    assert 'rbpodo_hardware' not in Path(bundle['urdf']).read_text()
    if process=='weld':
        assert any('hongik_eoat_base_link' in n for n in names)
        assert arm_info['tcp_basis']=='contact_tip_face_center'
    if process=='paint':
        physics=ET.parse(bundle['physics_urdf']).getroot()
        assert physics.find(f"joint[@name='{arm_info['sensor_joint']}']") is not None
        assert arm_info['contact_link'] in names


@pytest.mark.parametrize('active,process', [('left','spray'),('right','paint'),('left','weld')])
def test_inactive_arm_has_no_eoat_geometry_or_sensors(tmp_path,active,process):
    from sketch_control.eoat_catalog import fixed_descendants
    selection=normalize_selection(dict(active_arm=active,process=process))
    b=generate_urdf_assets(ROOT,selection,tmp_path)
    inactive='right' if active=='left' else 'left'
    assert b['arms'][inactive]['tool_id']=='none'
    for field in ['sensor_joint','wrench_frame','contact_link','camera_link']:
        assert b['arms'][inactive][field]==''
    for path in (b['urdf'],b['physics_urdf']):
        robot=ET.parse(path).getroot()
        for name in fixed_descendants(robot,inactive+'_manipulator_link6'):
            link=robot.find(f"link[@name='{name}']")
            assert link.find('visual') is None
            assert link.find('collision') is None
            assert link.find('inertial') is None
    assert not (tmp_path/(inactive+'_eoat.stl')).exists()
    assert not (tmp_path/(inactive+'_eoat.json')).exists()
    assert (tmp_path/(active+'_eoat.stl')).exists()


def test_weld_tip_applied_once():
    assert weld_tip_offset(None)==[0.,0.,0.]
    np.testing.assert_allclose(weld_tip_offset(.02),[0,0,.0212])
    with pytest.raises(ValueError): weld_tip_offset(-.1)
    with pytest.raises(ValueError): weld_tip_offset(float('nan'))


def test_catalog_hongik_has_sibling_branches():
    tool=load_tool('hongik_v4',ROOT)
    assert tool.tcp_basis=='contact_tip_face_center'
    assert 'hongik_eoat_base_link' in [n.get('name') for n in tool.links]
    assert 'hongik_laser_link' in [n.get('name') for n in tool.links]


def test_replacement_removes_previous_tool_links(tmp_path):
    b=generate_urdf_assets(ROOT,normalize_selection({'process':'spray','tools':{'left':'spray_v2','right':'spray_v2'}}),tmp_path)
    text=Path(b['urdf']).read_text()
    assert 'hongik_eoat_base_link' not in text
    assert 'right_D435' not in text


def test_full_bundle_requires_import_success(tmp_path):
    from prepare_platform_assets import generate_assets
    with pytest.raises(RuntimeError,match='USD'):
        generate_assets(ROOT,normalize_selection({}),tmp_path,importer=lambda urdf,output: None)
    assert not (tmp_path/'ready.json').exists()


def test_roller_mass_preserved_across_sensor_reduction(tmp_path):
    b=generate_urdf_assets(ROOT,normalize_selection({'process':'paint'}),tmp_path)
    full=ET.parse(b['urdf']).getroot(); physical=ET.parse(b['physics_urdf']).getroot()
    boundary=b['ft_boundaries']['left']
    mass=sum(float(n.find('inertial/mass').get('value')) for n in full.findall('link') if n.get('name') in boundary['downstream_links'] and n.find('inertial/mass') is not None)
    assert abs(mass-boundary['mass_kg'])<1e-10
    assert float(physical.find(f"link[@name='{boundary['child']}']/inertial/mass").get('value'))==pytest.approx(mass)
    assert not any(physical.find(f"link[@name='{n}']") is not None for n in boundary['downstream_links'] if n!=boundary['child'])
    active=next(h for h in full.findall('ros2_control') if h.find('hardware/plugin').text=='isaac_painting_hardware/IsaacPaintingSystem')
    assert [j.get('name') for j in active.findall('joint')]==b['arms']['left']['joints']


def test_rb10_roller_tcp_matches_existing_model(tmp_path):
    from prepare_rb20_painting_assets import expand_source,link_transforms
    b=generate_urdf_assets(ROOT,normalize_selection({'platform':'rb10','process':'paint'}),tmp_path)
    actual=link_transforms(ET.parse(b['urdf']).getroot())['tcp']
    expected=link_transforms(expand_source(ROOT,'rb10_1300e_u'))['tcp']
    np.testing.assert_allclose(actual,expected,atol=1e-9)


@pytest.mark.parametrize('platform,arm',[('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('process',['spray','weld','paint'])
@pytest.mark.parametrize('mode',['simulation','mixed'])
def test_generated_profiles_bind_selected_process_and_tool(tmp_path,platform,arm,process,mode):
    import yaml
    selection=normalize_selection(dict(platform=platform,active_arm=arm,process=process,execution_mode=mode))
    bundle=generate_urdf_assets(ROOT,selection,tmp_path)
    profiles=yaml.safe_load((tmp_path/'painting.yaml').read_text())
    active=bundle['arms'][arm]
    for section in profiles.values():
        values=section['ros__parameters']
        assert values['use_sim_time'] is True
        if 'process_mode' in values:assert values['process_mode']==process
    executor=profiles['moveit_executor']['ros__parameters']
    sketch=profiles['sketch_to_waypoints']['ros__parameters']
    for values in (executor,sketch):
        assert values['model_id']==active['model_id']
        assert values['spray_tool_axis']=='+z'
        assert Path(values['spray_eoat_profile'])==tmp_path/(arm+'_eoat.json')
        assert Path(values['spray_eoat_profile']).is_file()
        assert values['real_painting_enabled']==(process=='paint')
        assert values['dry_run'] is False
    assert executor['follow_joint_trajectory_action']=='/'+active['controller']+'/follow_joint_trajectory'
    assert executor['require_task_scene_ack']==(mode=='mixed')
    if process=='paint':
        assert executor['roller_contact_link']==active['contact_link']
        assert executor['roller_tool_links']==active['tool_links']
    else:
        assert 'roller_contact_link' not in executor
        assert 'roller_tool_links' not in executor
    assert profiles['wall_projector']['ros__parameters']['front_view_source']==('zed' if mode=='mixed' or process!='paint' else 'd405')
    refiner=profiles['d405_surface_refiner']['ros__parameters']
    if process=='paint' and mode=='simulation':
        assert Path(refiner['calibration_file']).is_file()
    else:
        assert refiner['calibration_file']==''  # physical registration is supplied by launch, never simulated calibration
    if process=='weld':
        assert executor['weld_input_provider']==('external' if mode=='mixed' else 'fixture')
        assert Path(executor['weld_tool_profile']).is_file()
    controllers=yaml.safe_load((tmp_path/'controllers.yaml').read_text())
    constraints=controllers[active['controller']]['ros__parameters']['constraints']
    assert constraints['goal_time']==2.0
    assert constraints['stopped_velocity_tolerance']==0.01
    assert set(constraints)-{'goal_time','stopped_velocity_tolerance'}==set(active['joints'])
    for joint in active['joints']:
        assert constraints[joint]=={'goal':0.02,'trajectory':0.1}


def test_fixed_tool_assembly_collision_pairs_follow_mounted_links(tmp_path):
    b=generate_urdf_assets(ROOT,normalize_selection({'process':'paint'}),tmp_path)
    pairs={frozenset((e.get('link1'),e.get('link2'))) for e in ET.parse(b['srdf']).getroot().findall('disable_collisions')}
    assert frozenset(('left_manipulator_link6','left_roller_aft200_link')) in pairs
    assert frozenset(('left_roller_paint_eoat_no_camera_link','left_roller_paint_d405_link')) in pairs
    assert not any(any(n.startswith('right_hongik_v4_') for n in pair) for pair in pairs)
    assert not any(any(n.startswith('left_roller_') for n in p) and any(n.startswith('right_') for n in p) for p in pairs)
    assert frozenset(('left_manipulator_link1','left_roller_paint_eoat_no_camera_link')) not in pairs


def test_commercial_paint_camera_uses_front_view_without_depth_threshold_changes(tmp_path):
    import json
    selection=normalize_selection(dict(platform='commercial',active_arm='left',process='paint'))
    generate_urdf_assets(ROOT,selection,tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    np.testing.assert_allclose(np.array(scene['zed_world_optical'])[:3,3],[.3,-.85,1.45])
    assert scene['wall']['center']==[0.,-1.65,1.1]
    forward=np.array([0.,-1.62,1.1])-np.array([.3,-.85,1.45]);forward/=np.linalg.norm(forward)
    np.testing.assert_allclose(np.array(scene['zed_world_optical'])[:3,2],forward)


@pytest.mark.parametrize('platform,arm',[('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('mode,process',[('simulation','spray'),('mixed','spray'),('mixed','paint'),('mixed','weld')])
def test_dynamic_scene_registry_has_no_nominal_target_geometry(tmp_path,platform,arm,mode,process):
    import yaml
    selection=normalize_selection(dict(platform=platform,active_arm=arm,execution_mode=mode,process=process))
    generate_urdf_assets(ROOT,selection,tmp_path)
    config=yaml.safe_load((tmp_path/'objects.yaml').read_text())
    assert config['active_target_source']==('task_scene' if mode=='mixed' else 'perception')
    assert config['active_target']==('measured_task_surface' if mode=='mixed' else 'perceived_task_surface')
    assert config['objects']==[]
