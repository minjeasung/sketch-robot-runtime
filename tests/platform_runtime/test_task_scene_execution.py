"""Measured-scene acknowledgement must gate cached plans and motion dispatch."""
import json
import time
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from sketch_control.moveit_executor import MoveItExecutor


def executor():
    node=SimpleNamespace(require_task_scene_ack=True,_task_scene_status={},_task_scene_received_at=0.,
                         process_mode='paint',active_target_name='measured_task_surface',executing=False,
                         _accepted_plan_hash='old-plan',_accepted_plan_path_id='old-path',
                         _active_segment_path=object(),_segment_path=object(),_acm_baseline_verified=True,
                         _motion_abort_requested=False,_contact_collision_allowed=False,
                         _acm_update_pending=None,_execution_snapshot={'old':True},
                         _contact_collision_baseline=None,_task_scene_revision='',
                         _abort_reasons=[],_restore=[],_task_scene_fault_latched=False)
    node._request_motion_abort=lambda reason:node._abort_reasons.append(reason)
    node._set_contact_collision_allowed=lambda value:node._restore.append(value)
    return node


def status(revision='rev1',**updates):
    return SimpleNamespace(data=json.dumps(dict(ready=True,surface_id='camera-surface-42',revision=revision,
        acks={'isaac':True,'moveit':True},error='',**updates)))


def test_task_scene_requires_both_acks_fresh_status_and_exact_contact_target():
    node=executor()
    assert hasattr(MoveItExecutor,'_task_scene_blocker'), 'Measured scene execution gate missing'
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_NOT_READY'
    MoveItExecutor.on_task_scene_status(node,status())
    assert MoveItExecutor._task_scene_blocker(node)==''
    node.active_target_name='wall'
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_TARGET_MISMATCH'
    node.active_target_name='measured_task_surface';node._task_scene_received_at=time.monotonic()-2
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_STATUS_STALE'
    MoveItExecutor.on_task_scene_status(node,SimpleNamespace(data='{"ready":true,"revision":"rev1","acks":{"isaac":true}}'))
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_NOT_READY'


def test_scene_revision_change_invalidates_old_plan_and_aborts_without_deferring_snapshot():
    node=executor()
    assert hasattr(MoveItExecutor,'on_task_scene_status'), 'Measured scene execution gate missing'
    MoveItExecutor.on_task_scene_status(node,status())
    node._accepted_plan_hash='plan1';node._accepted_plan_path_id='path1';node.executing=True
    node._contact_collision_allowed=True
    MoveItExecutor.on_task_scene_status(node,status('rev2'))
    assert node._accepted_plan_hash==node._accepted_plan_path_id==''
    assert node._segment_path is None
    assert not node._acm_baseline_verified
    assert node._abort_reasons==['TASK_SCENE_REVISION_CHANGED']
    assert node._restore==[False]


def test_unready_scene_blocks_dispatch_even_when_legacy_dry_run_is_set():
    node=executor();node.dry_run=True
    assert MoveItExecutor._motion_dispatch_inhibited_reason(node)=='TASK_SCENE_NOT_READY'
    node.require_task_scene_ack=False
    assert MoveItExecutor._motion_dispatch_inhibited_reason(node)==''


def test_status_timeout_aborts_and_activity_stays_true_until_controller_stops():
    node=executor()
    assert hasattr(MoveItExecutor,'_task_scene_tick'), 'Measured scene execution gate missing'
    published=[]
    node._task_scene_activity_pub=SimpleNamespace(publish=lambda msg:published.append(json.loads(msg.data)))
    node.executing=True
    MoveItExecutor._task_scene_tick(node)
    assert node._abort_reasons==['TASK_SCENE_NOT_READY']
    assert published[-1]['active'] is True
    node.executing=False;node._active_trajectory_goal_handle=object()
    MoveItExecutor._task_scene_tick(node)
    assert published[-1]['active'] is True
    node._active_trajectory_goal_handle=None
    MoveItExecutor._task_scene_tick(node)
    assert published[-1]['active'] is False


def test_mixed_fixed_d405_capture_never_plans_virtual_wrist_and_waits_for_fresh_refinement():
    from types import MethodType
    from std_msgs.msg import Bool
    callbacks=[];captures=[];finished=[];statuses=[]
    def unexpected_motion():
        raise AssertionError('Fixed physical camera must not follow a virtual-wrist scan')
    node=SimpleNamespace(require_task_scene_ack=True,process_mode='paint',executing=False,
        _motion_abort_requested=False,_d405_plane_accepted=False,
        dynamic_surface_source='d405_refined',dynamic_surface_source_time=0.,
        _invalidate_d405_prescan_callbacks=lambda **kwargs:None,
        _cancel_d405_prescan_timer=lambda:None,
        _build_d405_prescan_poses=unexpected_motion,
        d405_capture_pub=SimpleNamespace(publish=lambda msg:captures.append(msg.data)),
        create_timer=lambda duration,callback,**kwargs:callbacks.append(callback) or object(),
        _publish_work_area_refine_status=lambda state,**kwargs:statuses.append(state),
        _finish_d405_prescan=lambda success:finished.append(success),
        get_logger=lambda:SimpleNamespace(info=lambda *args:None,warn=lambda *args:None,error=lambda *args:None))
    assert MoveItExecutor._begin_d405_prescan(node,mode='work_area')
    assert captures==[True] and not node.executing
    assert node._d405_prescan_active
    assert not MoveItExecutor._task_scene_active(node), 'Stationary capture must allow the measured scene update'
    node._active_trajectory_goal_handle=object()
    assert MoveItExecutor._task_scene_active(node), 'Outstanding trajectory ownership must still forbid a scene update'
    node._active_trajectory_goal_handle=None
    assert statuses==['capturing_fixed']
    callbacks[0]()
    assert finished==[], 'An old refined surface cannot satisfy a new physical capture'
    node.dynamic_surface_source_time=time.monotonic();node._d405_plane_accepted=True
    callbacks[0]()
    assert finished==[True]


def test_refined_plane_requires_its_exact_scene_revision_despite_cross_topic_delivery():
    node=executor();MoveItExecutor.on_task_scene_status(node,status('initial'))
    node._d405_plane_accepted=True;node._d405_task_scene_revision='refined'
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_REFINEMENT_MISMATCH'
    MoveItExecutor.on_task_scene_status(node,status('refined'))
    assert MoveItExecutor._task_scene_blocker(node)==''
    node._d405_task_scene_revision=''
    assert MoveItExecutor._task_scene_blocker(node)=='TASK_SCENE_REFINEMENT_MISMATCH'
