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
