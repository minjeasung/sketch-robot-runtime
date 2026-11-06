import math

import pytest

from rbpodo_painting_control.painting_force_monitor_node import (
    PaintingForceMonitorNode,
    _apply_enabled_upper_bound,
)


def _make_filter(tau_s=0.1):
    monitor = object.__new__(PaintingForceMonitorNode)
    monitor._filter_tau = tau_s
    monitor._filtered = None
    monitor._last_filter_time = None
    return monitor


def _assert_finite(values):
    assert all(math.isfinite(value) for value in values)


def test_filter_recovers_when_first_sample_is_nonfinite():
    monitor = _make_filter()
    invalid = monitor._filter((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0)
    assert math.isnan(invalid[0])
    assert monitor._filtered is None

    first_valid = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    recovered = monitor._filter(first_valid, 1.1)
    _assert_finite(recovered)
    assert recovered == pytest.approx(first_valid)

    next_valid = (3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
    filtered = monitor._filter(next_valid, 1.2)
    _assert_finite(filtered)
    assert filtered == pytest.approx((2.0, 3.0, 4.0, 5.0, 6.0, 7.0))


def test_filter_recovers_after_nonfinite_interrupts_valid_stream():
    monitor = _make_filter()
    assert monitor._filter((0.0,) * 6, 2.0) == pytest.approx((0.0,) * 6)
    assert monitor._filter((2.0,) * 6, 2.1) == pytest.approx((1.0,) * 6)

    invalid = monitor._filter((math.nan,) * 6, 2.2)
    assert all(math.isnan(value) for value in invalid)
    assert monitor._filtered is None

    first_recovered = monitor._filter((4.0,) * 6, 2.3)
    _assert_finite(first_recovered)
    assert first_recovered == pytest.approx((4.0,) * 6)

    filtered = monitor._filter((6.0,) * 6, 2.4)
    _assert_finite(filtered)
    assert filtered == pytest.approx((5.0,) * 6)


def test_global_ceiling_does_not_revive_explicitly_disabled_mode_limit():
    assert _apply_enabled_upper_bound(0.0, 100.0) == 0.0
    assert _apply_enabled_upper_bound(90.0, 100.0) == 90.0
    assert _apply_enabled_upper_bound(150.0, 100.0) == 100.0


def _make_clock_readiness_monitor(monkeypatch, *, simulation=True):
    from types import SimpleNamespace
    from geometry_msgs.msg import WrenchStamped
    from rclpy.clock import ClockType
    from rclpy.time import Time
    from rbpodo_painting_control import painting_force_monitor_node as module
    from rbpodo_painting_control.force_safety import BiasEstimator, ForceSafetyMonitor, SafetyConfig
    clock, wall, published = {'now': 0.0}, {'now': 12.0}, []
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall['now'])
    limits = module._noncontact_limits()
    config = SafetyConfig(mode_limits={'IDLE': limits}, fallback_limits=limits,
        contact_force_sign=1.0, absolute_normal_force=True, contact_detect_n=3.5,
        contact_release_n=2.0, contact_confirm_duration_s=0.04, force_saturation_n=190.0,
        torque_saturation_nm=14.0, filtered_timeout_s=0.20, raw_timeout_s=0.20,
        tf_timeout_s=0.20, max_message_age_s=0.20, reset_max_force_norm_n=0.75,
        reset_max_torque_norm_nm=0.10)
    node = _make_filter(tau_s=0.0)
    node._startup_readiness_grace_s = 5.0
    node._startup_started_at = 10.0
    node._mode = 'IDLE'
    node._force_enabled = False
    node._tcp_frame = 'tcp'
    node._ft_frame = 'ft_link'
    node._force_deadband = node._torque_deadband = 0.0
    node._input_topic, node._raw_topic = 'filtered', 'raw'
    node._safety = ForceSafetyMonitor(config, started_at=10.0)
    node._bias_estimator = BiasEstimator(0.1)
    node._update_bias_estimate = lambda *_args: None
    node._filtered_pub = object()
    node._raw_pub = object()
    node._normal_pub = SimpleNamespace(publish=lambda msg: published.append(msg))
    node._publish_wrench = lambda pub, values: published.append((pub, values))
    node.get_clock = lambda: SimpleNamespace(ros_time_is_active=simulation,
        now=lambda: Time(seconds=clock['now'], clock_type=ClockType.ROS_TIME))
    msg = WrenchStamped()
    msg.header.frame_id = 'tcp'
    msg.header.stamp.sec, msg.header.stamp.nanosec = 4, 284000000
    return node, msg, clock, wall, published


def test_simulation_wrench_before_first_clock_remains_unready_without_latch(monkeypatch):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    node._on_wrench(msg)
    node._on_raw_wrench(msg)
    status = node._safety.status(wall['now'], 'IDLE')
    assert not status['abort_latched']
    assert not status['ft_valid'] and not status['tf_valid']
    assert node._safety.last_filtered_at is None and node._safety.last_raw_at is None
    assert node._filtered is None and not node._bias_estimator.ready
    assert published == []
    clock['now'] = 4.284
    node._on_wrench(msg)
    node._on_raw_wrench(msg)
    status = node._safety.status(wall['now'], 'IDLE')
    assert status['ft_valid'] and status['tf_valid'] and not status['abort_latched']


@pytest.mark.parametrize('stamp_ns', [440000000, 0])
def test_after_first_sim_clock_future_and_stale_wrenches_still_latch(monkeypatch, stamp_ns):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    clock['now'] = 4.284
    node._on_wrench(msg)
    msg.header.stamp.nanosec = stamp_ns
    node._on_wrench(msg)
    assert node._safety.latched_reason == 'FT_STALE'


def test_simulation_clock_reset_cannot_reenter_initial_readiness_grace(monkeypatch):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    clock['now'] = 4.284
    node._on_wrench(msg)
    clock['now'] = 0.0
    node._on_wrench(msg)
    assert node._safety.latched_reason == 'FT_STALE'


@pytest.mark.parametrize('activity', ['expired', 'force', 'motion', 'physical'])
def test_initial_clock_grace_cannot_hide_expiry_activity_or_physical_staleness(monkeypatch, activity):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch,
        simulation=activity != 'physical')
    if activity == 'expired':
        wall['now'] = 15.01
    elif activity == 'force':
        node._force_enabled = True
    elif activity == 'motion':
        node._mode = 'CONTACT_SEARCH'
    node._on_wrench(msg)
    assert node._safety.latched_reason == 'FT_STALE'


def test_startup_tf_and_filtered_before_raw_stays_unready_without_watchdog_latch(monkeypatch):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    clock['now'] = 4.284
    node._on_wrench(msg)
    assert node._safety.last_tf_ok_at is not None
    assert node._safety.last_filtered_at is not None
    assert node._safety.last_raw_at is None
    # A steady timer can run between callbacks for the two independent topics.
    assert node._startup_pair_grace_active(wall['now'])
    status = node._safety.status(wall['now'], 'IDLE')
    assert not status['ft_valid'] and not status['abort_latched']
    wall['now'] += .01
    node._on_raw_wrench(msg)
    assert not node._startup_pair_grace_active(wall['now'])
    assert node._safety.status(wall['now'], 'IDLE')['ft_valid']
    wall['now'] += .21
    assert not node._startup_pair_grace_active(wall['now'])
    node._safety.tick(wall['now'])
    assert node._safety.latched_reason == 'FT_STALE'


def test_startup_missing_raw_grace_expires_without_accepting_missing_data(monkeypatch):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    clock['now'] = 4.284
    node._on_wrench(msg)
    assert node._startup_pair_grace_active(wall['now'])
    wall['now'] = 15.01
    assert not node._startup_pair_grace_active(wall['now'])
    node._safety.tick(wall['now'])
    assert node._safety.latched_reason == 'FT_STALE'


@pytest.mark.parametrize('activity', ['force', 'motion', 'raw_optional'])
def test_startup_missing_pair_grace_cannot_hide_activity_or_optional_raw(monkeypatch, activity):
    node, msg, clock, wall, published = _make_clock_readiness_monitor(monkeypatch)
    clock['now'] = 4.284
    node._on_wrench(msg)
    if activity == 'force':node._force_enabled = True
    elif activity == 'motion':node._mode = 'CONTACT_SEARCH'
    else:node._require_raw = False
    assert not node._startup_pair_grace_active(wall['now'])
