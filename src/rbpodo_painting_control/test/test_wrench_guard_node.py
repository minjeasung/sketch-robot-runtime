import json
import time

from rbpodo_painting_control.painting_wrench_guard_node import (
    _log_guard_decision,
    PaintingWrenchGuardNode,
)
from rbpodo_painting_control.wrench_guard import GuardDecision, ZERO_WRENCH
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class _CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _decision(*, forwarding: bool) -> GuardDecision:
    if forwarding:
        return GuardDecision(
            output_wrench=(0.0, -1.6, 0.0, 0.0, 0.0, 0.0),
            compliance_enabled=True,
            blockers=(),
            requested_force_norm_n=1.6,
            requested_torque_norm_nm=0.0,
        )
    return GuardDecision(
        output_wrench=ZERO_WRENCH,
        compliance_enabled=False,
        blockers=('FORCE_DISABLED',),
        requested_force_norm_n=0.0,
        requested_torque_norm_nm=0.0,
    )


def test_actual_rclpy_logger_survives_warn_info_warn_guard_transition():
    """Regression for Jazzy's per-callsite logger severity invariant."""
    context = Context()
    context.init(args=[], domain_id=232)
    node = Node(
        'painting_wrench_guard_logger_test',
        context=context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    try:
        logger = node.get_logger()
        _log_guard_decision(logger, _decision(forwarding=False), 'IDLE')
        _log_guard_decision(logger, _decision(forwarding=True), 'RAMP_UP')
        _log_guard_decision(logger, _decision(forwarding=False), 'RAMP_DOWN')
    finally:
        node.destroy_node()
        context.shutdown()


def _set_fresh_guard_state(
    node, *, mode: str, force_enabled: bool, force_y: float,
    compliance_active: bool,
) -> float:
    now = time.monotonic()
    node._requested_wrench = (0.0, force_y, 0.0, 0.0, 0.0, 0.0)
    node._requested_received_at = now
    node._requested_frame_valid = True
    node._mode = mode
    node._mode_received_at = now
    node._force_enable = force_enabled
    node._enable_received_at = now
    node._heartbeat = True
    node._heartbeat_received_at = now
    node._ft_direct_valid = True
    node._ft_received_at = now
    node._safety_ft_valid = True
    node._safety_tf_valid = True
    node._safety_status_valid = True
    node._safety_received_at = now
    node._safety_abort_latched = False
    node._abort_signal_latched = False
    node._compliance_active = compliance_active
    node._compliance_active_received_at = now
    node._normal_limit_reached = False
    node._normal_limit_received_at = now
    return now


def test_timer_status_provenance_is_monotonic_across_guard_transitions():
    """Publish ordered same-host provenance through real guard timer logic."""
    rclpy.init(
        args=[],
        domain_id=231,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    try:
        node = PaintingWrenchGuardNode()
        node._real_painting_enabled = True
        status_pub = _CapturePublisher()
        node._status_pub = status_pub
        node._controller_pub = _CapturePublisher()
        node._compliance_pub = _CapturePublisher()
        node._motion_abort_pub = _CapturePublisher()

        def accept_tf(now):
            node._tf_valid = True
            node._tf_checked_at = now

        node._check_tf = accept_tf
        received_at = []
        received_at.append(
            _set_fresh_guard_state(
                node,
                mode='IDLE',
                force_enabled=False,
                force_y=0.0,
                compliance_active=False,
            )
        )
        node._on_timer()
        received_at.append(
            _set_fresh_guard_state(
                node,
                mode='RAMP_UP',
                force_enabled=True,
                force_y=-1.6,
                compliance_active=True,
            )
        )
        node._on_timer()
        received_at.append(
            _set_fresh_guard_state(
                node,
                mode='RAMP_DOWN',
                force_enabled=False,
                force_y=0.0,
                compliance_active=False,
            )
        )
        node._on_timer()

        statuses = [json.loads(message.data) for message in status_pub.messages]
        assert [status['mode'] for status in statuses] == [
            'IDLE',
            'RAMP_UP',
            'RAMP_DOWN',
        ]
        assert [status['forwarding'] for status in statuses] == [
            False,
            True,
            False,
        ]
        assert [status['status_sequence'] for status in statuses] == [1, 2, 3]
        published_at = [
            status['published_monotonic_s'] for status in statuses
        ]
        assert published_at[0] < published_at[1] < published_at[2]
        assert all(
            published >= received
            for published, received in zip(published_at, received_at)
        )
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
