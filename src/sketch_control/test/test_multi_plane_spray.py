import copy
import json
import time
from types import SimpleNamespace
import numpy as np
import pytest
from std_msgs.msg import String, Bool
from sketch_control.multi_plane_geometry import polygon_mask, extract_planes
from sketch_control.spray_execution import SprayExecutionMixin
from sketch_control.multi_surface_execution import MultiSurfaceMixin
from rbpodo_painting_control.segment_path import (
    SegmentPathError, attach_plan_hash, parse_segment_path,
    validate_segment_path_for_real_execution, segment_waypoint_position,
    rotation_from_surface_path, build_execution_steps,
)
from test_eoat_segment_generation import _generator_stub, _publish_two_strokes, _CapturePublisher
from test_target_refine_identity import _refiner_stub, _target_pose


def test_sketch_mask_excludes_bounding_box_background():
    mask = polygon_mask([[1,1],[8,8],[2,6]], [[[0,0],[10,0],[0,10]]])
    assert mask.tolist() == [True,False,True]


def test_two_walls_have_independent_normals_and_pixel_support():
    a,b=np.meshgrid(np.linspace(-.5,.5,20),np.linspace(.2,.8,20))
    p1=np.column_stack((a.ravel(), b.ravel(), np.full(a.size,1.)))
    p2=np.column_stack((np.full(a.size,.8),a.ravel(), b.ravel()))
    pts=np.vstack((p1,p2)); pixels=np.column_stack((np.arange(len(pts)),np.arange(len(pts))%20))
    planes=extract_planes(pts,pixels,threshold=.002,iterations=400)
    assert len(planes)==2
    assert abs(np.dot(planes[0]['normal'],planes[1]['normal'])) < .01
    assert all(p['inlier_count']>=390 for p in planes)
    assert all(np.dot(p['center'],p['normal']) <= 0 for p in planes)


def spray_plan():
    node=_generator_stub(real=True); node.process_mode='spray'
    path=_publish_two_strokes(node)
    assert path is not None
    return path


def test_spray_geometry_and_stroke_transitions_match_preview():
    path=spray_plan(); validate_segment_path_for_real_execution(path)
    assert [s.mode for s in build_execution_steps(path.rows)]==[
        'SPRAY_APPROACH','SPRAY','SPRAY_TRAVEL','SPRAY','SPRAY_FINISH']
    previous=None
    for row in path.rows:
        p=np.asarray(segment_waypoint_position(path,row))
        assert np.dot(p-np.asarray(row.position),row.normal)-path.contact_geometry_offset_m == pytest.approx(.5)
        assert row.force_n==0
        rotation=rotation_from_surface_path(row.normal,row.tangent,previous)
        np.testing.assert_allclose(rotation[:,1],row.normal,atol=1e-9)
        assert abs(rotation[:,0] @ row.tangent)<1e-9
        previous=rotation[:,0]
    assert path.process_mode=='spray'


@pytest.mark.parametrize('mutation', ['force','distance','contact','mode','start','travel_endpoint'])
def test_spray_rejects_unsafe_or_cross_mode_plan(mutation):
    payload=copy.deepcopy(spray_plan().raw_payload)
    if mutation=='force':payload['rows'][1]['force_n']=2
    if mutation=='distance':payload['rows'][1]['offset_m']=.49
    if mutation=='contact':payload['rows'][1]['mode']='CONTACT_SEARCH'
    if mutation=='mode':payload['process_mode']='paint'
    if mutation=='start':payload['rows'][0]['mode']='SPRAY'
    if mutation=='travel_endpoint':payload['rows'][1]['x']+=.1
    payload=attach_plan_hash(payload)
    with pytest.raises(SegmentPathError):
        parse_segment_path(payload,default_contact_offset_m=.026,max_force_n=30,minimum_clearance_m=.01)


def test_d405_multi_mode_waits_for_arrival_before_capture():
    node=_refiner_stub();node.defer_target_capture_until_arrival=True
    node._on_target_surface(_target_pose())
    assert node._active_capture_mode is None
    node._on_target_capture(Bool(data=True))
    assert node._active_capture_mode=='target'


def gun_stub(dry=False):
    node=SprayExecutionMixin()
    node.process_mode='spray';node.dry_run=dry;node._spray_session='session'
    node._spray_seq=1;node._spray_on=False;node._spray_dispatch=False
    node._spray_edge_time=time.monotonic()-1;node._spray_status={};node._spray_status_time=0
    node.spray_command_pub=_CapturePublisher();node._active_segment_path=None
    node._compliance_active=False;node._compliance_time=time.monotonic()
    node._painting_command_enable=False;node.painting_force_enabled=False
    node.executing=False;node._active_trajectory_goal_handle=None;node._motion_abort_requested=False
    return node


def acknowledge(node,on=False):
    node._on_spray_status(String(data=json.dumps(dict(session_id=node._spray_session,
        command_id=node._spray_seq,is_on=on,ready=True,fault=''))))


def test_spray_requires_driver_and_compliance_off_without_ft():
    node=gun_stub();assert node._spray_motion_blockers()
    acknowledge(node);assert node._spray_motion_blockers()==()
    node._compliance_active=True
    assert 'SPRAY_REQUIRES_FRESH_COMPLIANCE_OFF' in node._spray_motion_blockers()


def test_gun_on_only_after_trajectory_acceptance_and_off_at_completion():
    node=gun_stub();acknowledge(node)
    node._spray_dispatch=True;node._spray_tick();assert not node._spray_on
    node._active_trajectory_goal_handle=object();node._spray_tick();assert node._spray_on
    assert json.loads(node.spray_command_pub.messages[-1].data)['lease_ms']==250
    node._active_trajectory_goal_handle=None;node._spray_tick();assert not node._spray_on


def test_dry_run_never_emits_real_gun_on():
    node=gun_stub(dry=True);node._spray_dispatch=True;node._active_trajectory_goal_handle=object()
    node._spray_tick();assert not node._spray_on


def test_stale_and_old_session_status_cannot_enable_execution():
    node=gun_stub();acknowledge(node);node._spray_status_time=time.monotonic()-1
    node._on_spray_status(String(data=json.dumps(dict(session_id='old',command_id=1,is_on=False,ready=True))))
    assert 'SPRAY_ADAPTER_STALE_OR_UNCONNECTED' in node._spray_motion_blockers()


def test_driver_loss_turns_off_and_aborts():
    node=gun_stub();node.executing=True;aborts=[];node._request_motion_abort=aborts.append
    node._spray_tick();assert aborts and not node._spray_on


def test_multi_selection_rejects_stale_catalog_before_motion():
    node=MultiSurfaceMixin();node._multi_busy=lambda:False;node._motion_abort_requested=False
    node._multi_catalog={'generation':'new','planes':[{'id':'new:1'}]}
    statuses=[];node._multi_status=lambda **kw:statuses.append(kw)
    node._multi_on_select(String(data=json.dumps({'generation':'old','ids':['new:1']})))
    assert statuses==[{'error':'INVALID_SELECTION'}]


def test_multi_measurement_ignores_other_plane_response():
    node=MultiSurfaceMixin();node._multi_current={'id':'second'};node._multi_capture_started=1
    node._multi_target_pose=_target_pose(sec=123);node._multi_refined_result=None
    record=dict(target_stamp={'sec':122,'nanosec':456000000},frame_id='zed_left_camera_frame_optical',position=[0,0,1],orientation=[0,0,0,1])
    node._multi_on_refined(String(data=json.dumps(record)))
    assert node._multi_refined_result is None
    record['target_stamp']['sec']=123
    node._multi_on_refined(String(data=json.dumps(record)))
    assert node._multi_refined_result is not None


def test_spray_reaction_and_sensor_loss_do_not_drive_force_abort_but_controller_fault_does():
    from rbpodo_painting_control.force_safety import motion_abort_required
    for reason in ['TORQUE_LIMIT','RAW_IMPACT','NORMAL_OVERFORCE','FT_STALE','FT_NONFINITE']:
        assert not motion_abort_required(reason,'IDLE',False,noncontact_spray=True)
    assert motion_abort_required('CONTROLLER_FAULT','IDLE',False,noncontact_spray=True)
    assert motion_abort_required('TORQUE_LIMIT','PAINT',True,noncontact_spray=True)
    assert motion_abort_required('TORQUE_LIMIT','IDLE',False)


def test_multi_selected_faces_are_measured_sequentially_and_preserved():
    node=MultiSurfaceMixin()
    node._multi_catalog={'generation':'g','frame_id':'camera','planes':[
        dict(id='g:1',center=[0.,0.,1.],normal=[0.,0.,-1.],corners=[[-.2,-.2,1.],[.2,-.2,1.],[.2,.2,1.],[-.2,.2,1.]]),
        dict(id='g:2',center=[.5,0.,1.],normal=[-1.,0.,0.],corners=[[.5,-.2,.8],[.5,.2,.8],[.5,.2,1.2],[.5,-.2,1.2]])]}
    node._multi_busy=lambda:False; node._motion_abort_requested=False
    node.current_joint_state=object();node._current_tcp_pose_np=lambda:object()
    node._multi_target_pub=_CapturePublisher();node._multi_status=lambda *a,**k:None
    node._reset_d405_refined_lock=lambda *a,**k:None
    node._mark_scene_dirty=lambda _:None
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:_target_pose().header.stamp))
    node.on_active_surface=lambda pose:setattr(node,'dynamic_surface_point',np.array([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]))
    node.on_work_area_corners=lambda corners:setattr(node,'dynamic_work_area_corners',corners)
    approaches=[];node._begin_d405_prescan=lambda mode:approaches.append(node._multi_current['id']) or True
    node._schedule_process_once=lambda _,cb:cb()
    activated=[];node._multi_apply_active=activated.append
    node._multi_on_select(String(data=json.dumps({'generation':'g','ids':['g:1','g:2']})))
    assert approaches==['g:1']
    node._multi_refined_result=copy.deepcopy(node._multi_target_pose);node._multi_scan_done(True)
    assert approaches==['g:1','g:2'] and list(node._multi_refined)==['g:1']
    node._multi_refined_result=copy.deepcopy(node._multi_target_pose);node._multi_scan_done(True)
    assert set(node._multi_refined)=={'g:1','g:2'} and activated==['g:2']


def test_old_work_area_invalidation_cannot_abort_owned_multi_target_scan():
    from sketch_control.moveit_executor import MoveItExecutor
    node=SimpleNamespace(_multi_current={'id':'new-face'},_multi_queue=[])
    msg=String(data=json.dumps({'selected':False,'state':'invalidated','mode':'work_area'}))
    # No old area callbacks may reset the candidate scan's plane/scene.
    MoveItExecutor.on_work_area_state(node,msg)
    MoveItExecutor.on_d405_refinement_status(node,msg)
    MoveItExecutor.on_plan_status(node,msg)
