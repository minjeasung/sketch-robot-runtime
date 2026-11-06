"""The controller manager must receive the same paint dynamics as the force profile."""
from pathlib import Path
import sys
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/sketch_control')]
from prepare_platform_assets import generate_urdf_assets
from sketch_control.runtime_selection import normalize_selection


@pytest.mark.parametrize('platform,side',[('commercial','left'),('commercial','right'),('rb10','single')])
def test_generated_controller_applies_paint_profile_with_selected_arm_frames(tmp_path,platform,side):
    bundle=generate_urdf_assets(ROOT,normalize_selection(dict(platform=platform,active_arm=side,process='paint')),tmp_path)
    controller=yaml.safe_load((tmp_path/'controllers.yaml').read_text())['admittance_controller']['ros__parameters']
    profile=yaml.safe_load((tmp_path/'painting.yaml').read_text())['admittance_controller']['ros__parameters']
    assert controller['admittance']==profile['admittance']
    assert controller['admittance']['selected_axes']==[False,True,False,False,False,False]
    assert controller['admittance']['damping_absolute'][1]==300.
    assert controller['ft_sensor']['filter_coefficient']==profile['ft_sensor']['filter_coefficient']==.2
    for key in ('normal_axis_index','max_normal_velocity_mps','max_normal_acceleration_mps2',
                'max_normal_trim_m','roller_balance','compliance_enabled_by_default',
                'compliance_enable_timeout_s','wrench_reference_timeout_s'):
        assert controller[key]==profile[key]
    arm=bundle['arms'][side]
    assert controller['joints']==arm['joints']
    assert controller['kinematics']['base']==arm['base_frame']
    assert controller['kinematics']['tip']==arm['tcp_link']
    assert controller['kinematics']['group_name']==arm['group']
    assert controller['control']['frame']['id']==arm['tcp_link']
    assert controller['ft_sensor']['frame']['id']==arm['wrench_frame']
    assert controller['ft_sensor']['name']=='ft_sensor'
    assert controller['gravity_compensation']['CoG']==dict(pos=[0.,0.,0.],force=0.)


@pytest.mark.parametrize('process', ['paint', 'spray', 'weld'])
def test_desired_state_interpolation_is_only_enabled_for_chained_paint(tmp_path, process):
    bundle = generate_urdf_assets(ROOT, normalize_selection(dict(
        platform='commercial', active_arm='left', process=process)), tmp_path)
    arm = bundle['arms']['left']
    controller = yaml.safe_load((tmp_path/'controllers.yaml').read_text())[arm['controller']]['ros__parameters']
    assert controller.get('interpolate_from_desired_state', False) is (process == 'paint')
    if process == 'paint':
        assert controller['command_joints'] == ['admittance_controller/' + name for name in arm['joints']]
        assert controller['open_loop_control'] is False
