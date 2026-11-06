import json

import pytest

from rbpodo_painting_control import painting_wrench_reference_node as reference
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String


class _CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _mode(value: str) -> String:
    message = String()
    message.data = value
    return message


def _enable(value: bool) -> Bool:
    message = Bool()
    message.data = value
    return message


@pytest.mark.parametrize('zero_on_abort', [True, False])
@pytest.mark.parametrize('simulated_clock', [True, False])
def test_public_motion_abort_immediately_withdraws_request_without_tf_or_clock_progress(zero_on_abort, simulated_clock):
    args = ['--ros-args', '-p', 'public_motion_abort_requires_restart:=true']
    if simulated_clock:
        args += ['-p', 'use_sim_time:=true']
    rclpy.init(args=args, domain_id=230, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = reference.PaintingWrenchReferenceNode()
        assert '/motion_abort' in [sub.topic_name for sub in node.subscriptions]
        for name in ('_ramp_complete_pub', '_ramp_status_pub', '_requested_wrench_pub',
                     '_tcp_debug_pub', '_ft_debug_pub', '_mode_pub', '_command_force_pub'):
            setattr(node, name, _CapturePublisher())
        node._enable_force = True
        node._dry_run = False
        node._mode = 'PAINT'
        node._desired_force = 20.0
        node._current_force_y = -20.0
        node._zero_on_abort = zero_on_abort

        def unavailable_tf(*args, **kwargs):
            raise AssertionError('Emergency zero must not wait for a TF lookup')
        node._tf_buffer.lookup_transform = unavailable_tf
        node._on_motion_abort(Bool(data=True))
        assert len(node._requested_wrench_pub.messages) == 1
        assert node._requested_wrench_pub.messages[-1].wrench.force.y == 0.0
        assert node._current_force_y == 0.0
        assert node._mode_pub.messages[-1].data == 'ABORT'
        node._on_abort(Bool(data=False))
        node._on_motion_abort(Bool(data=False))
        node._on_mode(String(data='PAINT'))
        node._on_enable_force(Bool(data=True))
        node._on_timer()
        assert node._requested_wrench_pub.messages[-1].wrench.force.y == 0.0
        assert node._mode_pub.messages[-1].data == 'ABORT'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_legacy_reference_recovers_after_monitor_abort_reset():
    rclpy.init(args=[], domain_id=230, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = reference.PaintingWrenchReferenceNode()
        assert '/motion_abort' not in [sub.topic_name for sub in node.subscriptions]
        node._on_abort(Bool(data=True))
        assert node._aborted
        node._on_abort(Bool(data=False))
        assert not node._aborted
        node._on_enable_force(Bool(data=True))
        node._on_mode(String(data='PAINT'))
        node._desired_force = 1.6
        assert node._target_force_y_for_mode() == -1.6
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_ramp_status_is_source_stamped_and_keeps_legacy_bool(monkeypatch):
    """Publish ordered ramp provenance while retaining the Bool API."""
    rclpy.init(
        args=[],
        domain_id=230,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    try:
        node = reference.PaintingWrenchReferenceNode()
        legacy_pub = _CapturePublisher()
        status_pub = _CapturePublisher()
        node._ramp_complete_pub = legacy_pub
        node._ramp_status_pub = status_pub
        node._requested_wrench_pub = _CapturePublisher()
        node._tcp_debug_pub = _CapturePublisher()
        node._ft_debug_pub = _CapturePublisher()
        node._mode_pub = _CapturePublisher()
        node._command_force_pub = _CapturePublisher()
        node._transform_force_tcp_to_ft = lambda force: list(force)
        node._ramp_up = 1.0
        node._ramp_down = 1.0
        node._max_force_slew_rate = 100.0
        node._desired_force = 1.6

        clock = {'now': 10.0}
        monkeypatch.setattr(
            reference.time, 'monotonic', lambda: clock['now']
        )
        node._last_time = clock['now']
        node._on_mode(_mode('RAMP_UP'))
        node._on_enable_force(_enable(True))

        clock['now'] = 10.25
        node._on_timer()
        clock['now'] = 11.0
        node._on_timer()

        clock['now'] = 12.0
        node._on_mode(_mode('RAMP_DOWN'))
        clock['now'] = 12.25
        node._on_timer()
        node._on_enable_force(_enable(False))
        clock['now'] = 12.5
        node._on_timer()

        statuses = [json.loads(message.data) for message in status_pub.messages]
        assert [message.data for message in legacy_pub.messages] == [
            False,
            True,
            False,
            True,
        ]
        assert [status['mode'] for status in statuses] == [
            'RAMP_UP',
            'RAMP_UP',
            'RAMP_DOWN',
            'RAMP_DOWN',
        ]
        assert [status['force_enable'] for status in statuses] == [
            True,
            True,
            True,
            False,
        ]
        assert [status['ramp_complete'] for status in statuses] == [
            False,
            True,
            False,
            True,
        ]
        assert [status['status_sequence'] for status in statuses] == [
            1,
            2,
            3,
            4,
        ]
        assert [status['published_monotonic_s'] for status in statuses] == [
            10.25,
            11.0,
            12.25,
            12.5,
        ]
        assert all(
            set(status)
            == {
                'force_enable',
                'mode',
                'published_monotonic_s',
                'ramp_complete',
                'status_sequence',
            }
            for status in statuses
        )
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


class _RampSeedStub:
    """Minimal stand-in exercising PaintingWrenchReferenceNode._ramp_entry_force_y."""

    def __init__(self, measured, age_s, desired=3.0, sign=-1.0, current=0.0):
        self._measured_force_y = measured
        self._measured_force_y_at = 0.0 if age_s is None else 1000.0 - age_s
        self._measured_force_timeout = 0.20
        self._desired_force = desired
        self._target_sign = sign
        self._current_force_y = current

    _ramp_entry_force_y = (
        reference.PaintingWrenchReferenceNode._ramp_entry_force_y
    )


def _seed(measured, age_s=0.01, **kw):
    import time as _t

    stub = _RampSeedStub(measured, age_s, **kw)
    # _ramp_entry_force_y compares against time.monotonic(); anchor the stub.
    stub._measured_force_y_at = _t.monotonic() - (age_s if age_s is not None else 0.0)
    if age_s is None:
        stub._measured_force_y_at = 0.0
    return stub._ramp_entry_force_y("RAMP_UP")


def test_ramp_seeds_from_existing_contact_force():
    """Handover force cancels the reaction so the tool does not back off."""
    # 1.5 N of wall reaction -> command -1.5 N, driving force exactly zero.
    assert _seed(1.5) == pytest.approx(-1.5)


def test_ramp_seed_is_clamped_to_the_target():
    # A reading above the target must not skip past the commanded force.
    assert _seed(9.0, desired=3.0) == pytest.approx(-3.0)


def test_ramp_seed_uses_reaction_magnitude_for_either_sensor_sign():
    assert _seed(1.5) == pytest.approx(-1.5)
    assert _seed(-1.5) == pytest.approx(-1.5)
    assert _seed(-20.0) == pytest.approx(-3.0)


def test_ramp_seed_falls_back_when_measurement_is_stale():
    stub = _RampSeedStub(1.5, age_s=None, current=-0.7)
    assert stub._ramp_entry_force_y("RAMP_UP") == pytest.approx(-0.7)


def test_ramp_seed_only_applies_to_ramp_up():
    stub = _RampSeedStub(1.5, age_s=0.01, current=-2.2)
    assert stub._ramp_entry_force_y("RAMP_DOWN") == pytest.approx(-2.2)
    assert stub._ramp_entry_force_y("PAINT") == pytest.approx(-2.2)


def _dynamics_stub(monkeypatch, *, simulation=True, initial_ros_time=10.0):
    from types import SimpleNamespace
    from rclpy.time import Time
    from rclpy.clock import ClockType
    node = object.__new__(reference.PaintingWrenchReferenceNode)
    wall, ros = {'now': 100.0}, {'now': initial_ros_time}
    monkeypatch.setattr(reference.time, 'monotonic', lambda: wall['now'])
    node.get_clock = lambda: SimpleNamespace(ros_time_is_active=simulation,
        now=lambda: Time(seconds=ros['now'], clock_type=ClockType.ROS_TIME))
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None,
        warn=lambda *a, **k: None, error=lambda *a, **k: None)
    node._mode, node._aborted = 'IDLE', False
    node._motion_abort_latched = False
    node._desired_force, node._target_sign = 4.0, -1.0
    node._current_force_y = node._ramp_start_force_y = 0.0
    node._ramp_up = node._ramp_down = 2.0
    node._max_force, node._max_force_slew_rate = 20.0, 100.0
    node._enable_force, node._dry_run = True, False
    node._zero_noncontact = node._zero_on_abort = True
    node._dwell_contact_enabled = False
    node._ramp_complete, node._ramp_status_sequence = True, 0
    node._measured_force_y, node._measured_force_y_at, node._measured_force_timeout = 0.0, 0.0, .20
    node._last_time = node._ramp_started_at = (ros['now'] if ros['now'] > 0.0 else None) if simulation else wall['now']
    node._last_log, node._log_period = 0.0, 1.0
    node._tcp_frame, node._ft_frame = 'tcp', 'ft_link'
    node._transform_force_tcp_to_ft = lambda values: values
    for attr in ('_tcp_debug_pub', '_ft_debug_pub', '_mode_pub', '_command_force_pub',
                 '_ramp_complete_pub', '_ramp_status_pub', '_requested_wrench_pub'):
        setattr(node, attr, _CapturePublisher())
    return node, wall, ros


@pytest.mark.parametrize('mode,initial,expected', [('RAMP_UP', 0.0, -1.0), ('RAMP_DOWN', -4.0, -3.0)])
def test_simulated_ramp_progress_follows_ros_time_at_half_realtime(monkeypatch, mode, initial, expected):
    node, wall, ros = _dynamics_stub(monkeypatch)
    node._current_force_y = initial
    node._on_mode(_mode(mode))
    wall['now'], ros['now'] = 101.0, 10.5
    node._on_timer()
    assert node._current_force_y == pytest.approx(expected)
    assert not node._ramp_complete
    status = json.loads(node._ramp_status_pub.messages[-1].data)
    assert status['published_monotonic_s'] == 101.0


def test_simulated_slew_cap_uses_ros_dt_and_freeze_holds_even_without_cap(monkeypatch):
    node, wall, ros = _dynamics_stub(monkeypatch)
    node._ramp_up = node._ramp_down = 0.0
    node._max_force_slew_rate = 2.0
    node._on_mode(_mode('PAINT'))
    wall['now'], ros['now'] = 101.0, 10.5
    node._on_timer()
    assert node._current_force_y == pytest.approx(-1.0)
    node._max_force_slew_rate = 0.0  # Disabled cap must not bypass the clock hold.
    wall['now'] = 102.0
    node._on_timer()
    assert node._current_force_y == pytest.approx(-1.0)
    node._on_abort(_enable(True))
    node._on_timer()
    assert node._current_force_y == 0.0


def test_simulated_ramp_waits_for_first_positive_clock_then_starts_from_zero_time(monkeypatch):
    node, wall, ros = _dynamics_stub(monkeypatch, initial_ros_time=0.0)
    node._on_mode(_mode('RAMP_UP'))
    wall['now'] = 105.0
    node._on_timer()
    assert node._current_force_y == 0.0 and not node._ramp_complete
    wall['now'], ros['now'] = 106.0, 5.0
    node._on_timer()
    assert node._current_force_y == 0.0 and not node._ramp_complete
    wall['now'], ros['now'] = 107.0, 5.5
    node._on_timer()
    assert node._current_force_y == pytest.approx(-1.0)


def test_simulated_ramp_seed_freshness_still_uses_steady_time(monkeypatch):
    node, wall, ros = _dynamics_stub(monkeypatch)
    node._current_force_y = -.7
    node._measured_force_y, node._measured_force_y_at = 1.5, 100.0
    wall['now'] = 100.25  # ROS time did not advance, but sensor freshness expired.
    node._on_mode(_mode('RAMP_UP'))
    assert node._ramp_start_force_y == -.7


def test_physical_ramp_progress_retains_monotonic_time(monkeypatch):
    node, wall, ros = _dynamics_stub(monkeypatch, simulation=False)
    node._on_mode(_mode('RAMP_UP'))
    wall['now'], ros['now'] = 101.0, 10.5
    node._on_timer()
    assert node._current_force_y == pytest.approx(-2.0)


def test_simulated_target_change_restarts_ramp_in_ros_time(monkeypatch):
    from std_msgs.msg import Float64
    node, wall, ros = _dynamics_stub(monkeypatch)
    node._on_mode(_mode('RAMP_UP'))
    wall['now'], ros['now'] = 101.0, 10.5
    node._on_timer()
    assert node._current_force_y == pytest.approx(-1.0)
    node._on_desired_force(Float64(data=8.0))
    wall['now'], ros['now'] = 102.0, 11.0
    node._on_timer()
    assert node._current_force_y == pytest.approx(-2.75)


def test_simulated_zero_and_rewound_clock_do_not_advance_dynamics(monkeypatch):
    node, wall, ros = _dynamics_stub(monkeypatch)
    node._on_mode(_mode('RAMP_UP'))
    wall['now'], ros['now'] = 101.0, 10.5
    node._on_timer()
    force = node._current_force_y
    for step, stamp in enumerate([0.0, 5.0, 5.5, 10.5]):
        wall['now'], ros['now'] = 102.0 + step, stamp
        node._on_timer()
        assert node._current_force_y == force
        assert node._last_time == 10.5
    node._on_enable_force(_enable(False))
    node._on_timer()
    assert node._current_force_y == 0.0


@pytest.mark.parametrize('mode_first',[False,True])
def test_initial_ramp_target_delivery_order_preserves_fresh_contact_seed(monkeypatch,mode_first):
    from std_msgs.msg import Float64
    node,wall,ros=_dynamics_stub(monkeypatch)
    node._mode='CONTACT_SEARCH';node._desired_force=0.
    node._measured_force_y=5.7;node._measured_force_y_at=wall['now']
    if mode_first:
        node._on_mode(_mode('RAMP_UP'))
        node._on_desired_force(Float64(data=20.))
    else:
        node._on_desired_force(Float64(data=20.))
        node._on_mode(_mode('RAMP_UP'))
    assert node._ramp_start_force_y==pytest.approx(-5.7)
    assert node._ramp_started_at==ros['now']


@pytest.mark.parametrize('stale,current,expected',[(True,0.,0.),(False,-.7,-.7)])
def test_delayed_initial_target_does_not_revive_stale_seed_or_jump_active_force(monkeypatch,stale,current,expected):
    from std_msgs.msg import Float64
    node,wall,ros=_dynamics_stub(monkeypatch)
    node._mode='RAMP_UP';node._desired_force=0.;node._current_force_y=current
    node._measured_force_y=5.7;node._measured_force_y_at=wall['now']-(.21 if stale else 0.)
    node._on_desired_force(Float64(data=20.))
    assert node._ramp_start_force_y==pytest.approx(expected)
    assert node._current_force_y==current
