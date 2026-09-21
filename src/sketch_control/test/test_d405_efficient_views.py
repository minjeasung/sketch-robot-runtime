"""Camera geometry, ROI identity, bounded path choice and face ordering regressions."""
import copy
import json
import time
from types import SimpleNamespace, MethodType
import numpy as np
import pytest
from std_msgs.msg import String
from sketch_control.d405_view_geometry import measurement_samples, camera_view, support_mask
from sketch_control.rotation_utils import quat_apply, quat_to_matrix, quat_from_matrix
from sketch_control.d405_scan_selection import D405ScanSelectionMixin
from sketch_control.moveit_executor import MoveItExecutor
from sketch_control.plane_lifecycle import SinglePlaneFitConfig, capture_once_and_fit_plane, validate_single_plane_result
from test_target_refine_identity import _refiner_stub, _target_pose
from test_d405_prescan_planning import _dual_ik_executor, _ik_response, _joint_state, _trajectory, _Logger


def wall(x0=-2.,x1=2.):
    return np.array([[x0,-1,1],[x1,-1,1],[x1,1,1],[x0,1,1]],float)


def test_nearest_view_is_not_wall_center_and_stays_inside_margin():
    polygon = wall()
    sample = measurement_samples([1.7,.4,.68],polygon,[0,0,-1])[0]
    np.testing.assert_allclose(sample,[1.7,.4,1.])
    edge = measurement_samples([4,4,.68],polygon,[0,0,-1])[0]
    np.testing.assert_allclose(edge,[1.92,.92,1.])
    assert support_mask([sample,edge],polygon,[0,0,-1],.08).all()


def test_inset_follows_triangle_not_bounding_rectangle():
    triangle = np.array([[0,0,1],[1,0,1],[0,1,1]],float)
    samples = measurement_samples([.9,.9,.68],triangle,[0,0,-1])
    assert support_mask(samples,triangle,[0,0,-1],.08).all()
    assert samples[0][0]+samples[0][1] < 1
    with pytest.raises(ValueError,match='too small'):
        measurement_samples([0,0,.68],wall(0,.05),[0,0,-1])


def test_calibrated_optical_axis_and_center_match_tilted_wall_both_rolls():
    mount = (np.array([.01,-.07,.062]),np.array([.017,-.692,.721,.002]))
    mount = (mount[0],mount[1]/np.linalg.norm(mount[1]))
    normal = np.array([.3,.6,-.7]);normal /= np.linalg.norm(normal)
    point = np.array([.5,.3,.8])
    for flipped in (False,True):
        p,q = camera_view(point,normal,(np.zeros(3),[0,0,0,1]),mount,flipped=flipped)
        np.testing.assert_allclose(p+quat_apply(q,mount[0]),point+.32*normal,atol=1e-9)
        np.testing.assert_allclose(quat_apply(q,quat_apply(mount[1],[0,0,1])),-normal,atol=1e-9)


def capture_request(node, point, polygon):
    return String(data=json.dumps(dict(target_stamp=node._target_stamp,
        frame_id=node.latest_target_surface.header.frame_id,sample=point,support_polygon=polygon)))


def test_off_center_capture_references_local_patch_and_keeps_target_identity():
    node = _refiner_stub();node.defer_target_capture_until_arrival=True
    node._on_target_surface(_target_pose())
    stamp = copy.deepcopy(node._target_stamp)
    node._on_target_capture_request(capture_request(node,[1.7,0,1],wall().tolist()))
    assert node._active_capture_mode == 'target'
    assert node._active_capture_reference.pose.position.x == 1.7
    assert node.latest_target_surface.pose.position.x == 0
    assert node._target_stamp == stamp
    assert node._active_capture_roi == wall().tolist()
    # A compact observed patch far from the old target center satisfies the same quality gates.
    x,y=np.meshgrid(np.linspace(1.59,1.81,25),np.linspace(-.11,.11,25))
    points=np.column_stack((x.ravel(),y.ravel(),np.ones(x.size)))
    config=SinglePlaneFitConfig()
    result=capture_once_and_fit_plane(points,[1.7,0,1],[0,0,1],config)
    assert validate_single_plane_result(result,config,0.).accepted


@pytest.mark.parametrize('mutation',['stamp','outside','off_plane','nonfinite'])
def test_invalid_capture_request_never_arms_cloud(mutation):
    node=_refiner_stub();node.defer_target_capture_until_arrival=True
    node._on_target_surface(_target_pose())
    data=json.loads(capture_request(node,[1.,0,1],wall().tolist()).data)
    if mutation=='stamp':data['target_stamp']['sec']-=1
    if mutation=='outside':data['sample']=[2.5,0,1]
    if mutation=='off_plane':data['sample']=[1.,0,1.2]
    if mutation=='nonfinite':data['sample']=[float('nan'),0,1]
    node._on_target_capture_request(String(data=json.dumps(data)))
    assert node._active_capture_mode is None


def test_real_sample_pool_checks_every_ik_before_any_planning():
    node,ik,move,finalized,_logger=_dual_ik_executor()
    node._d405_scan_candidates=[dict(name=f'sample{i}',candidate_index=i,poses=node._d405_prescan_queue,
                                   surface_points=node._d405_prescan_surface_points) for i in range(6)]
    node._request_d405_orientation_candidate_iks('D405_PRESCAN_1',node._d405_prescan_queue[0],finalized.append)
    assert len(ik.requests)==6 and all(r.ik_request.avoid_collisions for r in ik.requests)
    for i in [5,2,4,3,1]:
        ik.futures[i].resolve(_ik_response([.3+i*.01]*6))
    assert not move.goals
    ik.futures[0].resolve(_ik_response([.1]*6))
    assert len(move.goals)==1 and node._d405_selected_orientation_branch=='sample0'


def test_actual_path_travel_can_override_nearest_ik_endpoint():
    selected=[]
    node=SimpleNamespace(get_logger=lambda:_Logger(),_activate_d405_orientation_rank=lambda c,i,**kw:selected.append((i,kw)))
    context=dict(rank_index=2,ranked=[{}, {}, {}],planned_views=[((2.,1.,2.,0),0,'detour'),((.4,.2,.3,1),1,'short'),((.8,.4,.6,2),2,'other')])
    MoveItExecutor._advance_d405_view_plans(node,context)
    assert selected==[(1,{'trajectory':'short'})]


def order_node():
    node=D405ScanSelectionMixin();node._motion_abort_requested=False
    node._d405_scene_revision_confirmed=lambda revision:revision==3
    node.current_joint_state=_joint_state([0.]*6);node.current_joint_state_time=time.monotonic()
    node._stage1_candidate_joint_metrics=lambda state,seed_state:(dict(max_delta=max(state.position),l2_delta=float(np.linalg.norm(state.position)),sum_delta=sum(state.position)), '')
    node._multi_order_timer=None
    selected=[];cancelled=[]
    node._multi_start_next=lambda ordered_id:selected.append(ordered_id)
    node._multi_cancel=cancelled.append
    node.get_logger=lambda:_Logger()
    context=dict(revision=3,results={},seed=copy.deepcopy(node.current_joint_state),candidates=[dict(plane_id='far'),dict(plane_id='near')])
    node._multi_order_context=context
    return node,context,selected,cancelled


def test_face_order_uses_joint_change_not_selection_or_callback_order():
    node,context,selected,cancelled=order_node()
    node._multi_order_ik_done(SimpleNamespace(result=lambda:_ik_response([.1]*6)),context,1)
    assert not selected
    node._multi_order_ik_done(SimpleNamespace(result=lambda:_ik_response([.8]*6)),context,0)
    assert selected==['near'] and not cancelled
    node._multi_order_ik_done(SimpleNamespace(result=lambda:_ik_response([0]*6)),context,0)
    assert selected==['near']  # late callback cannot dispatch another face


def test_face_order_does_not_use_result_after_robot_moves():
    node,context,selected,cancelled=order_node()
    node._multi_order_ik_done(SimpleNamespace(result=lambda:_ik_response([.1]*6)),context,0)
    node.current_joint_state=_joint_state([.1]*6)
    node._multi_order_ik_done(SimpleNamespace(result=lambda:_ik_response([.2]*6)),context,1)
    assert not selected and cancelled==['ORDER_MEASURED_SEED_CHANGED']


def test_motionless_capture_still_requires_checked_scene_seed_and_stationarity():
    from geometry_msgs.msg import Pose
    pose=Pose();pose.orientation.w=1.
    records=[]
    node=SimpleNamespace(_d405_prescan_callback_valid=lambda t:True,_d405_scene_revision_confirmed=lambda r:True,
        _d405_plan_start_matches_measured=lambda *args:(True,''),_current_tcp_pose_np=lambda:(np.zeros(3),np.array([0,0,0,1.])),
        _d405_post_fjt_arrival_reason=lambda t:'',get_logger=lambda:_Logger(),
        _start_d405_post_fjt_verification=lambda c,t:records.append('verify'),
        execute_trajectory_direct=lambda *a,**kw:pytest.fail('must not move'))
    context=dict(token=object(),scene_revision=1,seed_state=_joint_state([0]*6))
    MoveItExecutor._dispatch_d405_orientation_trajectory(node,context,dict(candidate=dict(poses=[pose])),_trajectory([[0]*6,[0]*6]))
    assert records==['verify']
    node._d405_scene_revision_confirmed=lambda r:False
    context['finalize_cb']=records.append
    MoveItExecutor._dispatch_d405_orientation_trajectory(node,context,dict(candidate=dict(poses=[pose])),_trajectory([[0]*6,[0]*6]))
    assert records==['verify',False]


def test_executor_builds_views_using_optical_tf_instead_of_nominal_camera_center():
    from sketch_control.multi_surface_execution import MultiSurfaceMixin
    from geometry_msgs.msg import TransformStamped
    node=SimpleNamespace(_multi_current={'id':'face','corners':wall().tolist()},_multi_catalog={'frame_id':'camera'})
    tf=TransformStamped();tf.transform.rotation.w=1.
    node._lookup_transform_to_base=lambda *a,**k:tf
    mount=(np.array([.01,-.07,.06]),np.array([0,0,0,1.]))
    node._d405_mount_transform=lambda:mount
    node._current_tcp_pose_np=lambda:(np.array([1.6,.2,.62]),np.array([0,0,0,1.]))
    node._multi_current['normal']=[0,0,-1]
    node._d405_support_in_base=MethodType(MultiSurfaceMixin._d405_support_in_base,node)
    node._make_d405_scan_tcp_pose=MethodType(MoveItExecutor._make_d405_scan_tcp_pose,node)
    node.get_logger=lambda:_Logger()
    poses=MoveItExecutor._build_d405_prescan_poses(node)
    assert len(poses)==3 and len(node._d405_scan_candidates)==6
    for candidate in node._d405_scan_candidates:
        pose=candidate['poses'][0];q=pose.orientation;p=pose.position
        q=np.array([q.x,q.y,q.z,q.w]);p=np.array([p.x,p.y,p.z])
        point=candidate['surface_points'][0]
        np.testing.assert_allclose(p+quat_apply(q,mount[0]),point+[0,0,-.32],atol=1e-9)
        np.testing.assert_allclose(quat_apply(q,quat_apply(mount[1],[0,0,1])),[0,0,1],atol=1e-9)
    assert node._d405_scan_candidates[0]['surface_points'][0][0] > 1.5


def test_plan_comparison_does_not_dispatch_until_best_full_path_is_chosen():
    node,ik,move,finalized,logger=_dual_ik_executor()
    node._d405_scan_candidates=[dict(name=f'sample{i}',candidate_index=i,poses=node._d405_prescan_queue,
                                   surface_points=node._d405_prescan_surface_points) for i in range(3)]
    for name in ('_d405_orientation_plan_result','_advance_d405_view_plans','_dispatch_d405_orientation_trajectory',
                 '_d405_plan_endpoint_matches_ik','_d405_plan_start_matches_measured','_trajectory_joint_metrics','_current_positions_for_joints'):
        setattr(node,name,MethodType(getattr(MoveItExecutor,name),node))
    node._max_joint_delta=MoveItExecutor._max_joint_delta
    node._d405_prescan_trajectory_is_safe=lambda *args:True  # independently covered by existing guard tests
    node._rescale_trajectory=lambda traj,**kw:traj
    node._current_tcp_pose_np=lambda:None
    dispatched=[]
    node.execute_trajectory_direct=lambda traj,**kw:dispatched.append(traj) or True
    node._request_d405_orientation_candidate_iks('D405_PRESCAN_1',node._d405_prescan_queue[0],finalized.append)
    for i,goal in enumerate([.05,.10,.20]):ik.futures[i].resolve(_ik_response([goal]*6))
    context=node._d405_orientation_context
    plans=[_trajectory([[0]*6,[.5]*6,[.05]*6]),_trajectory([[0]*6,[.10]*6]),_trajectory([[0]*6,[.20]*6])]
    for i,trajectory in enumerate(plans):
        node.current_joint_state_time=time.monotonic()
        result=SimpleNamespace(error_code=SimpleNamespace(val=1),planned_trajectory=trajectory)
        node._d405_orientation_plan_result(SimpleNamespace(result=lambda:SimpleNamespace(result=result)),context,i)
        if i<2:assert not dispatched
    assert len(dispatched)==1
    assert list(dispatched[0].joint_trajectory.points[-1].positions)==[.10]*6
    assert node._d405_selected_orientation_branch=='sample1'


def test_capture_checks_actual_optical_ray_and_rejects_changed_mount():
    node=D405ScanSelectionMixin()
    mount=(np.zeros(3),np.array([0,0,0,1.]))
    node._d405_scan_mount=mount
    node._d405_mount_transform=lambda:mount
    node._current_tcp_pose_np=lambda:(np.array([1.5,0,.68]),np.array([0,0,0,1.]))
    node._multi_current={}
    node._d405_support_in_base=lambda p:(wall(),np.array([0,0,-1.]))
    node.get_logger=lambda:_Logger()
    np.testing.assert_allclose(node._d405_capture_sample_in_base(),[1.5,0,1.])
    node._d405_mount_transform=lambda:(np.array([.1,0,0]),np.array([0,0,0,1.]))
    assert node._d405_capture_sample_in_base() is None
