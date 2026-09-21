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
