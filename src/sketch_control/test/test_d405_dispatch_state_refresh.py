"""A dense D405 planning result yields for fresh state without weakening dispatch."""
import copy
from types import SimpleNamespace

import pytest
from rclpy.clock import ClockType

from sketch_control.moveit_executor import MoveItExecutor
import sketch_control.moveit_executor as executor_module
from test_d405_prescan_planning import _joint_state, _trajectory, _pose, _Logger


def _executor(monkeypatch, *, stale=True):
    now = {'wall': 10., 'ros': 4_000_000_000}
    monkeypatch.setattr(executor_module.time, 'monotonic', lambda: now['wall'])
    node = object.__new__(MoveItExecutor)
    node.get_logger = lambda: logger
    logger = _Logger()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now['ros']))
    timers, dispatched, finalized = [], [], []

    def create_timer(period, callback, **kwargs):
        timer = SimpleNamespace(period=period, callback=callback, clock=kwargs.get('clock'), cancelled=False)
        timer.cancel = lambda: setattr(timer, 'cancelled', True)
        timers.append(timer)
        return timer

    node.create_timer = create_timer
    node.destroy_timer = lambda timer: setattr(timer, 'destroyed', True)
    node._current_tcp_pose_np = lambda: None  # TF is external; this test needs a real move.
    node.execute_trajectory_direct = lambda trajectory, **kwargs: dispatched.append(trajectory) or True
    node.current_joint_state = _joint_state([0.] * 6)
    node.current_joint_state.header.stamp.sec = 4
    node.current_joint_state_time = 9.7 if stale else 10.
    node.stationary_joint_delta_rad = .001
    node.stationary_joint_velocity_rad_s = .01
    node._joint_motion_last_time = 9.
    node._motion_abort_requested = False
    node._d405_prescan_active = True
    node._d405_prescan_token = object()
    node._scene_revision = 3
    node._scene_confirmed_revision = 3
    node.scene_confirmed = True
    node._d405_selected_orientation_branch = 'sample'
    trajectory = _trajectory([[0.] * 6, [.1] * 6])
    record = {'candidate': {'poses': [_pose(1.)]}}
    context = dict(token=node._d405_prescan_token, scene_revision=3, rank_index=0,
                   seed_state=copy.deepcopy(node.current_joint_state), finalize_cb=finalized.append,
                   label='D405_PRESCAN_1')
    node._d405_orientation_context = context
    return node, context, record, trajectory, timers, dispatched, finalized, now, logger


def _fresh_state(node, now, *, stamp_ns=4_050_000_000, wall=10.05):
    now.update(wall=wall, ros=4_050_000_000)
    state = _joint_state([0.] * 6)
    state.header.stamp.sec, state.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    node.on_joint_state(state)
    return state


def test_stale_dispatch_yields_then_revalidates_same_plan_with_new_state_once(monkeypatch):
    node, context, record, trajectory, timers, sent, finalized, now, logger = _executor(monkeypatch)
    seed = copy.deepcopy(context['seed_state'])
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and sent == [] and len(timers) == 1
    timer = timers[0]
    timer.callback()
    assert sent == []
    _fresh_state(node, now)
    timer.callback()
    timer.callback()  # A queued callback from the cancelled timer is harmless.
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert sent == [trajectory] and sent[0] is trajectory
    assert context['seed_state'] == seed
    assert finalized == [] and timer.cancelled and timer.destroyed
    assert any('state refresh' in entry and 'receipt_age_s=' in entry for entry in logger.infos)


def test_refresh_deadline_is_steady_and_not_extended_by_duplicate_result(monkeypatch):
    node, context, record, trajectory, timers, sent, finalized, now, logger = _executor(monkeypatch)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and len(timers) == 1
    assert timers[0].clock.clock_type == ClockType.STEADY_TIME
    now['wall'] = 10.5  # ROS/simulator time remains paused.
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert len(timers) == 1
    now['wall'] = 11.001
    timers[0].callback()
    timers[0].callback()
    assert finalized == [False] and sent == [] and timers[0].cancelled
    assert any('STATE_REFRESH_TIMEOUT' in entry for entry in logger.errors)


@pytest.mark.parametrize('fault', ['moved', 'missing_joint', 'nonfinite', 'first_point', 'seed'])
def test_new_state_still_must_match_exact_seed_and_plan_start(monkeypatch, fault):
    node, context, record, trajectory, timers, sent, finalized, now, _ = _executor(monkeypatch)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and len(timers) == 1
    state = _fresh_state(node, now)
    if fault == 'moved': state.position[0] = .02
    elif fault == 'missing_joint': state.name.pop(); state.position.pop()
    elif fault == 'nonfinite': state.position[0] = float('nan')
    elif fault == 'first_point': trajectory.joint_trajectory.points[0].positions[0] = .02
    elif fault == 'seed': context['seed_state'].position[0] = .02
    timers[0].callback()
    assert sent == [] and finalized == [False] and timers[0].cancelled


@pytest.mark.parametrize('change', ['scene', 'token', 'abort', 'inhibit', 'context', 'rank'])
def test_refresh_never_dispatches_after_context_or_safety_change(monkeypatch, change):
    node, context, record, trajectory, timers, sent, finalized, now, _ = _executor(monkeypatch)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and len(timers) == 1
    _fresh_state(node, now)
    if change == 'scene': node._scene_revision += 1
    elif change == 'token': node._d405_prescan_token = object()
    elif change == 'abort': node._motion_abort_requested = True
    elif change == 'inhibit': node._hardware_motion_inhibited = True
    elif change == 'context': node._d405_orientation_context = {}
    elif change == 'rank': context['rank_index'] = 1
    timers[0].callback()
    timers[0].callback()
    assert sent == [] and timers[0].cancelled
    if change in ('scene', 'inhibit'):
        assert finalized == [False]
    else:
        assert finalized == []  # Old work must not finalize a newer operation.


@pytest.mark.parametrize('fault', ['same_stamp', 'old_source', 'future_source', 'old_receipt'])
def test_refresh_requires_new_receipt_and_advancing_fresh_source_stamp(monkeypatch, fault):
    node, context, record, trajectory, timers, sent, finalized, now, _ = _executor(monkeypatch)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and len(timers) == 1
    stamp = {'same_stamp': 4_000_000_000, 'old_source': 4_010_000_000,
             'future_source': 5_000_000_000, 'old_receipt': 4_050_000_000}[fault]
    _fresh_state(node, now, stamp_ns=stamp)
    if fault == 'old_source': now['ros'] = 4_500_000_000
    if fault == 'old_receipt': now['wall'] = 10.3
    timers[0].callback()
    assert sent == [] and finalized == []
    _fresh_state(node, now, stamp_ns=4_550_000_000, wall=10.55)
    now['ros'] = 4_550_000_000
    timers[0].callback()
    assert sent == [trajectory] and finalized == []


def test_new_prescan_cancels_pending_refresh_and_late_timer(monkeypatch):
    node, context, record, trajectory, timers, sent, finalized, now, _ = _executor(monkeypatch)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert finalized == [] and len(timers) == 1
    node._invalidate_d405_prescan_callbacks(clear_token=True)
    assert timers[0].cancelled and timers[0].destroyed
    _fresh_state(node, now)
    timers[0].callback()
    assert sent == [] and finalized == []


def test_fresh_state_dispatch_keeps_existing_immediate_path(monkeypatch):
    node, context, record, trajectory, timers, sent, finalized, _, _ = _executor(monkeypatch, stale=False)
    node._dispatch_d405_orientation_trajectory(context, record, trajectory)
    assert sent == [trajectory] and timers == [] and finalized == []
