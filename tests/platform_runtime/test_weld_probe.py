from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def test_target_uses_exact_live_pixel_grid_and_simulator_session():
    from check_platform_weld import target_request
    frame=dict(generation='sim-1',frame_id='zed_left_camera_frame_optical',stamp=dict(sec=12,nanosec=3),
               clock_domain='ros',width=640,height=480,valid=True,source='simulation',camera_id='zed')
    target=target_request(frame,'sim-1','executor-1','request-1')
    assert target['polygon']==[[64.,48.],[576.,48.],[576.,432.],[64.,432.]]
    assert target['stamp']==frame['stamp'] and target['server_session_id']=='executor-1'
    with pytest.raises(ValueError,match='camera identity'):target_request({**frame,'generation':'old'},'sim-1','executor-1','r')
    with pytest.raises(ValueError,match='camera identity'):target_request({**frame,'source':'physical'},'sim-1','executor-1','r')


def test_selection_uses_fixture_ids_and_refuses_unknown_or_duplicate():
    from check_platform_weld import selected_ids
    from types import SimpleNamespace as NS
    catalog=NS(seams=[NS(seam_id='cad_bead_14'),NS(seam_id='cad_bead_27')])
    assert selected_ids(catalog,[])==['cad_bead_14','cad_bead_27']
    assert selected_ids(catalog,['cad_bead_27'])==['cad_bead_27']
    for request in (['S1'],['cad_bead_14','cad_bead_14']):
        with pytest.raises(ValueError,match='selection'):selected_ids(catalog,request)


def test_live_description_rejects_changed_geometry_and_physical_plugin():
    from check_platform_weld import validate_live_description
    model='<robot name="platform"><link name="World"/><link name="tool"/><joint name="fixed" type="fixed"><parent link="World"/><child link="tool"/><origin xyz="0 0 1"/></joint><ros2_control><hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin></hardware></ros2_control></robot>'
    validate_live_description(model,model)
    with pytest.raises(ValueError,match='differs'):validate_live_description(model,model.replace('0 0 1','0 0 2'))
    with pytest.raises(ValueError,match='physical'):validate_live_description(model,model.replace('topic_based_ros2_control/TopicBasedSystem','rbpodo_hardware/System'))


def test_read_only_mode_cannot_plan_or_execute():
    from check_platform_weld import authorize_command
    with pytest.raises(ValueError,match='plan-only'):authorize_command('/welding/plan_request',False,False)
    authorize_command('/welding/plan_request',True,False)
    with pytest.raises(ValueError,match='execute'):authorize_command('/welding/execute',True,False)
    authorize_command('/welding/execute',False,True)


def test_nullable_command_ack_waits_for_exact_command_and_retains_rejection():
    from check_platform_weld import matching_command_ack
    for status in ({},{'command_ack':None},{'command_ack':{}},
                   {'command_ack':{'command_id':'previous','ok':True}}):
        assert matching_command_ack(status,'current') is None
    accepted={'command_id':'current','ok':True,'error':''}
    rejected={'command_id':'current','ok':False,'error':None}
    assert matching_command_ack({'command_ack':accepted},'current')==accepted
    assert matching_command_ack({'command_ack':rejected},'current')==rejected
