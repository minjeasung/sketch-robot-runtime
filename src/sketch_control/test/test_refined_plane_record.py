"""Accepted generation and measured plane geometry are one ROS transaction."""
import copy
import json
from types import MethodType, SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sketch_control.moveit_executor import MoveItExecutor
from test_eoat_segment_generation import _generator_stub
from test_moveit_executor_fail_closed import _d405_prescan_status_executor


def accepted_status(work_area_id='work-1'):
    return dict(mode='work_area', state='accepted', accepted=True,
        work_area_id=work_area_id, plane_generation_id='plane-new',
        capture_stamp=dict(sec=12, nanosec=345), target_frame='link0',
        refined_plane=dict(frame_id='link0', stamp=dict(sec=12, nanosec=345),
            position=[1.45, -.3, 1.1], orientation=[0., 0., 0., 1.]))


def bare_pose(x=9.0, stamp=11):
    pose = PoseStamped()
    pose.header.frame_id = 'link0'
    pose.header.stamp.sec = stamp
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def executor_stub():
    node, aborts, resets, statuses = _d405_prescan_status_executor()
    node._d405_refined_lock_active = False
    node._d405_refined_generation = 0
    node._work_area_corners_signature = None
    node.dynamic_surface_source_time = 0.0
    node.dynamic_surface_source = 'zed'
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    applied = []
    def apply(pose, source):
        applied.append(pose)
        node.dynamic_surface_source = source
        node.dynamic_surface_source_time += 1.0
    node._set_active_surface = apply
    node._finish_refined_surface_update = MethodType(MoveItExecutor._finish_refined_surface_update, node)
    return node, applied


@pytest.mark.parametrize('pose_first', [True, False])
def test_executor_and_generator_accept_atomic_geometry_in_either_topic_order(pose_first):
    generator = _generator_stub(real=True, accepted=False)
    generator.latest_refined_work_area = None
    executor, applied = executor_stub()
    old = bare_pose()
    if pose_first:
        generator._on_refined_work_area(old)
        MoveItExecutor.on_refined_active_surface(executor, old)
    generator._on_d405_refinement_status(String(data=json.dumps(accepted_status())))
    MoveItExecutor.on_d405_refinement_status(executor,
        String(data=json.dumps(accepted_status('work-area-1'))))
    assert generator.latest_refined_work_area.pose.position.x == 1.45
    assert len(applied) == 1 and applied[0].pose.position.x == 1.45
    assert applied[0].header.stamp.sec == 12 and applied[0].header.stamp.nanosec == 345
    generator._on_refined_work_area(old)
    MoveItExecutor.on_refined_active_surface(executor, old)
    assert generator.latest_refined_work_area.pose.position.x == 1.45
    assert len(applied) == 1


@pytest.mark.parametrize('mutation', ['stamp', 'frame', 'nan', 'quaternion', 'work_area'])
def test_invalid_atomic_record_never_commits_geometry(mutation):
    generator = _generator_stub(real=True, accepted=False)
    generator.latest_refined_work_area = None
    executor, applied = executor_stub()
    data = accepted_status()
    if mutation == 'stamp': data['refined_plane']['stamp']['nanosec'] += 1
    elif mutation == 'frame': data['refined_plane']['frame_id'] = 'old-frame'
    elif mutation == 'nan': data['refined_plane']['position'][0] = float('nan')
    elif mutation == 'quaternion': data['refined_plane']['orientation'] = [0., 0., 0., 0.]
    elif mutation == 'work_area': data['work_area_id'] = 'old-work-area'
    generator._on_d405_refinement_status(String(data=json.dumps(data)))
    executor_data = copy.deepcopy(data)
    if mutation != 'work_area': executor_data['work_area_id'] = 'work-area-1'
    MoveItExecutor.on_d405_refinement_status(executor, String(data=json.dumps(executor_data)))
    generator._on_refined_work_area(bare_pose(1.45, 12))
    MoveItExecutor.on_refined_active_surface(executor, bare_pose(1.45, 12))
    assert generator.latest_refined_work_area is None
    assert applied == []


def test_refiner_publishes_geometry_with_accepted_identity_and_original_bare_pose(monkeypatch):
    import numpy as np
    from sketch_control import d405_surface_refiner_node as module
    from sketch_control.plane_lifecycle import PlaneLifecycle
    from sketch_control.refined_plane_record import refined_pose_from_status
    from test_target_refine_identity import _refiner_stub, _Publisher
    node = _refiner_stub()
    node._lifecycles['work_area'] = PlaneLifecycle('work_area', 'work-1')
    node._lifecycles['work_area'].invalidate('test_capture', token=1)
    node.refined_pub = _Publisher()
    node.fit_config = object()
    node._cloud_to_numpy = lambda msg: np.zeros((300, 3))
    node._select_surface_roi = lambda points, *a, **k: points
    node._set_expected_refined_corner_signature = lambda *a: None
    result = SimpleNamespace(center=np.array([1.45, -.3, 1.1]), normal=np.array([0., 0., 1.]), metrics=lambda: {'plane_shift_m_from_zed': 0.0})
    monkeypatch.setattr(module, 'capture_once_and_fit_plane', lambda *a: result)
    monkeypatch.setattr(module, 'validate_single_plane_result', lambda *a: SimpleNamespace(accepted=True))
    cloud = bare_pose(0., 12)
    cloud.header.stamp.nanosec = 345
    metadata = node._capture_metadata(cloud)
    node._evaluate_captured_cloud('work_area', cloud, bare_pose(), 'link0', None, 0., metadata)
    status = json.loads(node.status_pub.messages[-1].data)
    decoded = refined_pose_from_status(status)
    assert status['accepted'] and status['work_area_id'] == 'work-1'
    assert status['plane_generation_id'] == node._lifecycles['work_area'].plane_generation_id
    assert decoded == node.refined_pub.messages[-1]


def test_generation_change_retires_pending_identity_bound_tf_retry():
    node, applied = executor_stub()
    node._pending_surface_source = 'd405_refined'
    node._pending_surface_msg = bare_pose()
    node._d405_refined_lock_time = 0.
    node._d405_refined_lock_signature = None
    node._d405_refined_allow_corner_correction = False
    MoveItExecutor._reset_d405_refined_lock(node, 'new generation', clear_surface=False)
    assert node._pending_surface_msg is None
    assert node._pending_surface_source == ''


def refiner_with_accepted_record():
    from sketch_control.plane_lifecycle import PlaneLifecycle
    from test_target_refine_identity import _refiner_stub
    node = _refiner_stub()
    node._paint_active = False
    node._lifecycles['work_area'] = PlaneLifecycle('work_area', 'work-1')
    node._lifecycles['work_area'].invalidate('test_capture', token=1)
    node._lifecycles['work_area'].accept()
    data = accepted_status()
    node._publish_status(True, 'accepted', mode='work_area',
        metrics={'plane_shift_m_from_zed': 0.0, 'inlier_count': 300},
        **{key: data[key] for key in ('capture_stamp', 'target_frame', 'refined_plane')})
    return node, json.loads(node.status_pub.messages[-1].data)


@pytest.mark.parametrize('event', ['paint_locked', 'paint_unlocked', 'capture_ignored'])
def test_refiner_accepted_replays_keep_original_atomic_measurement(event):
    node, initial = refiner_with_accepted_record()
    node._on_painting_mode(String(data='PAINT'))
    if event == 'paint_unlocked': node._on_painting_mode(String(data='IDLE'))
    elif event == 'capture_ignored': node._start_capture('work_area', reference_plane=bare_pose())
    replay = json.loads(node.status_pub.messages[-1].data)
    assert replay['accepted']
    assert replay['plane_generation_id'] == initial['plane_generation_id']
    assert replay['refined_plane'] == initial['refined_plane']
    assert replay['capture_stamp'] == initial['capture_stamp']
    assert replay['inlier_count'] == 300


def test_refiner_never_reuses_old_geometry_for_unknown_accepted_generation():
    node, initial = refiner_with_accepted_record()
    lifecycle = node._lifecycles['work_area']
    lifecycle.invalidate('new_measurement', token=2)
    lifecycle.accept()  # An unknown acceptance lacking a measured record.
    node._publish_status(True, 'paint_locked', mode='work_area')
    replay = json.loads(node.status_pub.messages[-1].data)
    assert not replay['accepted'] and not replay['ok']
    assert 'refined_plane' not in replay


def test_generator_replay_preserves_committed_geometry_and_original_receipt_time(monkeypatch):
    from sketch_control import sketch_to_waypoints_node as module
    clock = {'now': 10.0}
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['now'])
    node = _generator_stub(real=True, accepted=False)
    data = accepted_status()
    node._on_d405_refinement_status(String(data=json.dumps(data)))
    first = node.latest_refined_work_area
    clock['now'] = 20.0
    data['state'] = 'paint_locked'
    node._on_d405_refinement_status(String(data=json.dumps(data)))
    assert node.latest_refined_work_area is first
    assert node.latest_refined_work_area_time == 10.0
    data.pop('refined_plane')  # Metadata-only replay of a known generation.
    node._on_d405_refinement_status(String(data=json.dumps(data)))
    assert node.latest_refined_work_area is first
    assert node.latest_refined_work_area_time == 10.0
    data['plane_generation_id'] = 'unknown-generation'
    node._on_d405_refinement_status(String(data=json.dumps(data)))
    assert node.latest_refined_work_area is None and not node.d405_status['accepted']


def test_same_generation_replay_cannot_replace_first_accepted_geometry():
    node, initial = refiner_with_accepted_record()
    replacement = copy.deepcopy(initial['refined_plane'])
    replacement['position'][0] += .1
    node._publish_status(True, 'capture_ignored', mode='work_area',
                         refined_plane=replacement)
    replay = json.loads(node.status_pub.messages[-1].data)
    assert replay['refined_plane'] == initial['refined_plane']
    generator = _generator_stub(real=True, accepted=False)
    data = accepted_status()
    generator._on_d405_refinement_status(String(data=json.dumps(data)))
    data['refined_plane']['position'][0] += .1
    generator._on_d405_refinement_status(String(data=json.dumps(data)))
    assert generator.latest_refined_work_area is None
    assert generator.d405_status['reason'] == 'accepted_generation_geometry_changed'
