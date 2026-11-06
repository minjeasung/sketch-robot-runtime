"""Loaded PAINT keeps the nominal trajectory separate from admittance motion."""
import copy
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from sketch_control import moveit_executor as module


@pytest.fixture
def executor(monkeypatch):
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)
    names = ['j' + str(i) for i in range(6)]
    measured = JointState(name=names + ['other_arm'], position=[.002] * 6 + [.7])
    measured.header.stamp = Time(sec=100)
    state = JointTrajectoryControllerState(joint_names=names)
    state.header.stamp = Time(sec=100)
    state.reference.positions = [.0] * 6
    state.feedback.positions = [.002] * 6
    logs, aborts, requests = [], [], []
    obj = SimpleNamespace(
        arm_joint_names=names, current_joint_state=measured, current_joint_state_time=10.,
        _paint_nominal_controller_state=state, _paint_nominal_controller_state_time=10.,
        paint_nominal_reference_topic='/platform/left/controller/controller_state',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000, to_msg=lambda: Time(sec=100))),
        get_logger=lambda: SimpleNamespace(info=logs.append),
        painting_force_enabled=True, _painting_command_enable=True,
        cartesian_client=SimpleNamespace(service_is_ready=lambda: True,
            call_async=lambda req: requests.append(req) or SimpleNamespace(add_done_callback=lambda cb: None)),
        _process_row_tcp_poses={1: Pose()}, stationary_paint_hold_s=0.,
        _process_last_tcp_pose=None, _process_step_index=0,
        _paint_entry_context=object(), _painting_command_context=object(),
        _request_motion_abort=aborts.append, paint_cartesian_planning_timeout_s=1.,
        create_timer=lambda *_args: SimpleNamespace(cancel=lambda: None),
        destroy_timer=lambda *_args: None,
        _process_cartesian_done=lambda *_args: None,
    )
    monkeypatch.setattr(module.MoveItExecutor, '_apply_contact_depth_correction', lambda *args: None)
    obj.logs, obj.aborts, obj.requests = logs, aborts, requests
    return obj


def snapshot(executor):
    result, error = module.MoveItExecutor._paint_nominal_start_snapshot(executor)
    assert not error
    return result


def trajectory(executor, value=0.):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = executor.arm_joint_names
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=[value] * 6)]
    return result


def test_loaded_paint_cartesian_seed_uses_authoritative_reference_preserving_physical_state(executor):
    original = copy.deepcopy(executor.current_joint_state)
    module.MoveItExecutor._plan_process_motion_step(executor,
        SimpleNamespace(mode='PAINT', rows=[SimpleNamespace(row_number=1)], speed_mps=.02))
    request = executor.requests[0]
    assert list(request.start_state.joint_state.position) == [0.] * 6 + [.7]
    assert request.avoid_collisions is True
    assert executor.current_joint_state == original
    context = executor._segment_cartesian_context
    assert context['seed_state'] == original
    assert context['paint_nominal_start']['seed_state'] == request.start_state.joint_state
    assert any('PAINT NOMINAL START' in line for line in executor.logs)


@pytest.mark.parametrize('mode,force,topic', [('PAINT', True, ''), ('PAINT', False, '/state'), ('RETRACT', True, '/state')])
def test_legacy_and_nonloaded_paths_keep_measured_seed(executor, mode, force, topic):
    executor.paint_nominal_reference_topic = topic
    executor.painting_force_enabled = force
    module.MoveItExecutor._plan_process_motion_step(executor,
        SimpleNamespace(mode=mode, rows=[SimpleNamespace(row_number=1)], speed_mps=.02))
    assert executor.requests[0].start_state.joint_state == executor.current_joint_state


@pytest.mark.parametrize('invalid', [
    'missing', 'receipt_stale', 'receipt_future', 'source_stale', 'source_future',
    'duplicate', 'missing_joint', 'wrong_joint', 'short_reference', 'nan_reference',
    'short_feedback', 'nan_feedback', 'feedback_disagrees', 'measured_stale',
    'measured_source_stale', 'measured_duplicate', 'measured_nan',
])
def test_invalid_nominal_or_physical_observation_aborts_before_planning(executor, invalid):
    state = executor._paint_nominal_controller_state
    if invalid == 'missing': executor._paint_nominal_controller_state = None
    elif invalid == 'receipt_stale': executor._paint_nominal_controller_state_time = 9.79
    elif invalid == 'receipt_future': executor._paint_nominal_controller_state_time = 10.01
    elif invalid == 'source_stale': state.header.stamp = Time(sec=99, nanosec=790_000_000)
    elif invalid == 'source_future': state.header.stamp = Time(sec=100, nanosec=1)
    elif invalid == 'duplicate': state.joint_names[-1] = state.joint_names[0]
    elif invalid == 'missing_joint': state.joint_names = state.joint_names[:-1]
    elif invalid == 'wrong_joint': state.joint_names[-1] = 'another_joint'
    elif invalid == 'short_reference': state.reference.positions = [0.]
    elif invalid == 'nan_reference': state.reference.positions = [float('nan')] * 6
    elif invalid == 'short_feedback': state.feedback.positions = [.002]
    elif invalid == 'nan_feedback': state.feedback.positions = [float('nan')] * 6
    elif invalid == 'feedback_disagrees': state.feedback.positions = [.1] * 6
    elif invalid == 'measured_stale': executor.current_joint_state_time = 9.79
    elif invalid == 'measured_source_stale': executor.current_joint_state.header.stamp = Time(sec=99)
    elif invalid == 'measured_duplicate': executor.current_joint_state.name[-1] = 'j0'
    elif invalid == 'measured_nan': executor.current_joint_state.position[0] = float('nan')
    module.MoveItExecutor._plan_process_motion_step(executor,
        SimpleNamespace(mode='PAINT', rows=[SimpleNamespace(row_number=1)], speed_mps=.02))
    assert executor.requests == []
    assert len(executor.aborts) == 1
    assert 'PAINT_NOMINAL' in executor.aborts[0]


def test_nominal_identity_mapping_does_not_depend_on_joint_order(executor):
    state = executor._paint_nominal_controller_state
    state.joint_names = list(reversed(state.joint_names))
    state.reference.positions = [.005, .004, .003, .002, .001, 0.]
    assert list(snapshot(executor)['seed_state'].position) == [0., .001, .002, .003, .004, .005, .7]


@pytest.mark.parametrize('invalid', ['reference_changed', 'first_point_measured', 'reference_stale', 'wrong_plan_joint', 'nan_plan'])
def test_dispatch_revalidates_nominal_continuity(executor, invalid):
    start = snapshot(executor)
    plan = trajectory(executor)
    if invalid == 'reference_changed': executor._paint_nominal_controller_state.reference.positions = [.002] * 6
    elif invalid == 'first_point_measured': plan.joint_trajectory.points[0].positions = [.002] * 6
    elif invalid == 'reference_stale': executor._paint_nominal_controller_state_time = 9.79
    elif invalid == 'wrong_plan_joint': plan.joint_trajectory.joint_names[-1] = 'another_joint'
    elif invalid == 'nan_plan': plan.joint_trajectory.points[0].positions[0] = float('nan')
    assert module.MoveItExecutor._paint_nominal_plan_start_error(executor, plan, start)


def test_nominal_dispatch_still_requires_physical_start_with_existing_tolerance(executor):
    start = snapshot(executor)
    plan = trajectory(executor)
    assert not module.MoveItExecutor._paint_nominal_plan_start_error(executor, plan, start)
    assert module.MoveItExecutor._d405_plan_start_matches_measured(executor, plan, executor.current_joint_state)[0]
    executor.current_joint_state.position[0] = .02
    assert not module.MoveItExecutor._d405_plan_start_matches_measured(executor, plan, executor.current_joint_state)[0]


@pytest.mark.parametrize('change,expected', [
    ('none', ''), ('reference', 'PAINT_NOMINAL_REFERENCE_CHANGED'),
    ('first_point', 'PAINT_NOMINAL_PLAN_START_MISMATCH'),
    ('measured_moved', 'MEASURED_SEED_MISMATCH'),
    ('measured_stale', 'PAINT_NOMINAL_MEASURED_STATE_STALE'),
    ('scene', 'planning context changed'), ('entry', 'PAINT entry context changed'),
    ('guard', 'PAINT guard lost'),
])
def test_cartesian_response_keeps_nominal_physical_and_context_gates(executor, monkeypatch, change, expected):
    module.MoveItExecutor._plan_process_motion_step(executor,
        SimpleNamespace(mode='PAINT', rows=[SimpleNamespace(row_number=1)], speed_mps=.02))
    context = executor._segment_cartesian_context
    physical_seed = copy.deepcopy(context['seed_state'])
    plan = trajectory(executor)
    dispatches, physical_checks = [], []
    executor._motion_abort_requested = False
    executor.real_painting_enabled = True
    executor.scene_confirmed = True
    executor._scene_confirmed_revision = 0
    executor._current_real_plan_blockers = lambda: ()
    executor._limit_trajectory_to_cartesian_speed = lambda result, *args: result
    executor._paint_entry_is_current = lambda entry: entry is executor._paint_entry_context
    executor._cancel_process_timer = lambda: None
    executor.execute_trajectory_direct = lambda result, **kwargs: dispatches.append((result, kwargs)) or True

    def check_physical(result, seed, **kwargs):
        physical_checks.append((copy.deepcopy(seed), kwargs))
        return module.MoveItExecutor._d405_plan_start_matches_measured(executor, result, seed, **kwargs)

    executor._d405_plan_start_matches_measured = check_physical
    monkeypatch.setattr(module.MoveItExecutor, '_force_guard_status_blockers',
                        lambda *args, **kwargs: ['lost'] if change == 'guard' else [])
    if change == 'reference': executor._paint_nominal_controller_state.reference.positions = [.002] * 6
    elif change == 'first_point': plan.joint_trajectory.points[0].positions = [.002] * 6
    elif change == 'measured_moved':
        executor.current_joint_state.position[0] = .02
        executor._paint_nominal_controller_state.feedback.positions[0] = .02
    elif change == 'measured_stale': executor.current_joint_state_time = 9.79
    elif change == 'scene': executor._scene_revision = 1
    elif change == 'entry': executor._paint_entry_context = object()
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        error_code=SimpleNamespace(val=1), fraction=1., solution=plan))
    module.MoveItExecutor._process_cartesian_done(executor, future, context['token'])
    if expected:
        assert not dispatches
        assert len(executor.aborts) == 1 and expected in executor.aborts[0]
    else:
        assert not executor.aborts
        assert len(dispatches) == 1
        assert dispatches[0][0] is plan
        assert dispatches[0][1]['requires_contact_acm'] is True
        assert physical_checks == [(physical_seed, {'max_age_s': .2})]
