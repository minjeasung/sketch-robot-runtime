#!/usr/bin/env python3
"""Sole fail-closed publisher of the admittance controller wrench reference."""

from dataclasses import replace
import json
import math
import signal
import time

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros

from .force_safety import SustainedCondition
from .wrench_guard import GuardConfig, GuardInputs, NONZERO_MODES, evaluate_guard


def _log_guard_decision(logger, decision, mode: str) -> None:
    """Log one guard decision without changing severity at one ROS callsite.

    Jazzy's ``RcutilsLogger`` binds a severity to each Python caller location.
    Selecting ``logger.info`` or ``logger.warn`` dynamically and invoking the
    selected method from one line therefore raises when forwarding changes.
    Keep the two severities on distinct source lines.
    """
    message = (
        'wrench_guard forwarding=%s compliance=%s mode=%s blockers=%s'
        % (
            decision.forwarding,
            decision.compliance_enabled,
            mode,
            ','.join(decision.blockers) or 'NONE',
        )
    )
    if decision.forwarding:
        logger.info(message)
    else:
        logger.warn(message)


class PaintingWrenchGuardNode(Node):
    """Validate every force-control dependency and publish wrench or exact zero."""

    def __init__(self) -> None:
        super().__init__("painting_wrench_guard")

        self.declare_parameter(
            "requested_wrench_topic",
            "/painting_admittance/requested_wrench_reference",
        )
        self.declare_parameter(
            "controller_wrench_reference_topic",
            "/admittance_controller/wrench_reference",
        )
        self.declare_parameter(
            "compliance_enable_topic",
            "/admittance_controller/compliance_enable",
        )
        self.declare_parameter(
            "ft_wrench_topic", "/force_torque_sensor_broadcaster/wrench"
        )
        self.declare_parameter("mode_topic", "/painting_admittance/mode")
        self.declare_parameter("force_enable_topic", "/painting_admittance/enable_force")
        self.declare_parameter(
            "executor_heartbeat_topic", "/painting_admittance/executor_heartbeat"
        )
        self.declare_parameter(
            "safety_status_topic", "/painting_admittance/safety_status"
        )
        self.declare_parameter("abort_topic", "/painting_admittance/abort")
        self.declare_parameter(
            "controller_fault_topic", "/painting_admittance/controller_fault"
        )
        self.declare_parameter(
            "compliance_active_topic", "/admittance_controller/compliance_active"
        )
        self.declare_parameter(
            "normal_limit_reached_topic",
            "/admittance_controller/normal_limit_reached",
        )
        self.declare_parameter("guard_status_topic", "/painting_admittance/wrench_guard_status")
        self.declare_parameter("ft_frame", "ft_link")
        self.declare_parameter("tcp_frame", "tcp")
        self.declare_parameter("real_painting_enabled", False)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("requested_wrench_timeout_s", 0.10)
        self.declare_parameter("mode_timeout_s", 0.20)
        self.declare_parameter("enable_timeout_s", 0.20)
        self.declare_parameter("executor_heartbeat_timeout_s", 0.20)
        self.declare_parameter("ft_timeout_s", 0.20)
        self.declare_parameter("tf_timeout_s", 0.20)
        self.declare_parameter("safety_status_timeout_s", 0.20)
        self.declare_parameter("controller_status_timeout_s", 0.20)
        self.declare_parameter("compliance_activation_timeout_s", 0.50)
        self.declare_parameter("normal_limit_abort_duration_s", 0.25)
        self.declare_parameter("max_command_force_n", 15.0)
        self.declare_parameter("max_command_torque_nm", 0.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.01)
        self.declare_parameter("shutdown_zero_publish_count", 5)
        self.declare_parameter("log_period_s", 1.0)

        self._requested_topic = str(
            self.get_parameter("requested_wrench_topic").value
        )
        self._controller_topic = str(
            self.get_parameter("controller_wrench_reference_topic").value
        )
        self._compliance_topic = str(
            self.get_parameter("compliance_enable_topic").value
        )
        self._ft_topic = str(self.get_parameter("ft_wrench_topic").value)
        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._enable_topic = str(self.get_parameter("force_enable_topic").value)
        self._heartbeat_topic = str(
            self.get_parameter("executor_heartbeat_topic").value
        )
        self._safety_topic = str(self.get_parameter("safety_status_topic").value)
        self._abort_topic = str(self.get_parameter("abort_topic").value)
        self._controller_fault_topic = str(
            self.get_parameter("controller_fault_topic").value
        )
        self._compliance_active_topic = str(
            self.get_parameter("compliance_active_topic").value
        )
        self._normal_limit_topic = str(
            self.get_parameter("normal_limit_reached_topic").value
        )
        self._guard_status_topic = str(
            self.get_parameter("guard_status_topic").value
        )
        self._ft_frame = str(self.get_parameter("ft_frame").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._real_painting_enabled = bool(
            self.get_parameter("real_painting_enabled").value
        )
        self._rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._tf_lookup_timeout = max(
            0.0, float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        self._shutdown_zero_count = max(
            1, int(self.get_parameter("shutdown_zero_publish_count").value)
        )
        self._log_period = max(0.1, float(self.get_parameter("log_period_s").value))
        self._compliance_activation_timeout = self._positive_timeout(
            "compliance_activation_timeout_s"
        )
        self._normal_limit_abort_duration = self._positive_timeout(
            "normal_limit_abort_duration_s"
        )

        self._config = GuardConfig(
            requested_wrench_timeout_s=self._positive_timeout(
                "requested_wrench_timeout_s"
            ),
            mode_timeout_s=self._positive_timeout("mode_timeout_s"),
            enable_timeout_s=self._positive_timeout("enable_timeout_s"),
            executor_heartbeat_timeout_s=self._positive_timeout(
                "executor_heartbeat_timeout_s"
            ),
            ft_timeout_s=self._positive_timeout("ft_timeout_s"),
            tf_timeout_s=self._positive_timeout("tf_timeout_s"),
            safety_status_timeout_s=self._positive_timeout(
                "safety_status_timeout_s"
            ),
            controller_status_timeout_s=self._positive_timeout(
                "controller_status_timeout_s"
            ),
            max_command_force_n=max(
                0.0, float(self.get_parameter("max_command_force_n").value)
            ),
            max_command_torque_nm=max(
                0.0, float(self.get_parameter("max_command_torque_nm").value)
            ),
        )
        if self._config.max_command_force_n <= 0.0:
            raise ValueError("max_command_force_n must be positive")

        self._requested_wrench = None
        self._requested_received_at = None
        self._requested_frame_valid = False
        self._mode = "IDLE"
        self._mode_received_at = None
        self._force_enable = False
        self._enable_received_at = None
        self._heartbeat = False
        self._heartbeat_received_at = None
        self._ft_direct_valid = False
        self._ft_received_at = None
        self._safety_ft_valid = False
        self._safety_tf_valid = False
        self._safety_status_valid = False
        self._safety_received_at = None
        self._safety_abort_latched = True
        self._abort_signal_latched = False
        self._compliance_active = False
        self._compliance_active_received_at = None
        self._normal_limit_reached = False
        self._normal_limit_received_at = None
        self._normal_limit_last_true_at = None
        self._normal_limit_qualifier = SustainedCondition(
            self._normal_limit_abort_duration
        )
        self._normal_limit_latched = False
        self._external_fault_signal = False
        self._external_fault_latched = False
        self._compliance_ack_latched = False
        self._watchdog_latched = False
        self._watchdog_reason = ""
        self._active_force_session = False
        self._compliance_requested_since = None
        self._fault_seen_in_safety = False
        self._tf_valid = False
        self._tf_checked_at = None
        self._last_blockers = None
        self._last_log = 0.0
        self._status_sequence = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # This is the only publisher in the package allowed to target the
        # controller wrench reference.
        self._controller_pub = self.create_publisher(
            WrenchStamped, self._controller_topic, 10
        )
        self._compliance_pub = self.create_publisher(Bool, self._compliance_topic, 10)
        self._status_pub = self.create_publisher(String, self._guard_status_topic, 10)
        self._motion_abort_pub = self.create_publisher(Bool, "/motion_abort", 10)

        self.create_subscription(
            WrenchStamped, self._requested_topic, self._on_requested_wrench, 10
        )
        self.create_subscription(String, self._mode_topic, self._on_mode, 10)
        self.create_subscription(Bool, self._enable_topic, self._on_enable, 10)
        self.create_subscription(Bool, self._heartbeat_topic, self._on_heartbeat, 10)
        self.create_subscription(WrenchStamped, self._ft_topic, self._on_ft_wrench, 10)
        self.create_subscription(String, self._safety_topic, self._on_safety_status, 10)
        self.create_subscription(Bool, self._abort_topic, self._on_abort, 10)
        self.create_subscription(
            Bool, self._controller_fault_topic, self._on_controller_fault, 10
        )
        self.create_subscription(
            Bool,
            self._compliance_active_topic,
            self._on_compliance_active,
            10,
        )
        self.create_subscription(
            Bool,
            self._normal_limit_topic,
            self._on_normal_limit_reached,
            10,
        )
        self.create_service(
            Trigger,
            "/painting_admittance/reset_wrench_guard",
            self._on_reset_guard,
        )
        self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().warn(
            "Wrench guard ready: real_painting_enabled=%s request=%s output=%s; "
            "output remains zero until every watchdog is valid"
            % (
                self._real_painting_enabled,
                self._requested_topic,
                self._controller_topic,
            )
        )

    def _positive_timeout(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError("%s must be positive" % name)
        return value

    def _header_fresh(self, msg: WrenchStamped, timeout_s: float) -> bool:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return False
        try:
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        except Exception:
            return False
        return -0.05 <= age <= timeout_s

    @staticmethod
    def _wrench_tuple(msg: WrenchStamped):
        return (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )

    def _on_requested_wrench(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        values = self._wrench_tuple(msg)
        self._requested_wrench = values
        self._requested_received_at = now
        self._requested_frame_valid = (
            msg.header.frame_id == self._ft_frame
            and self._header_fresh(msg, self._config.requested_wrench_timeout_s)
            and all(math.isfinite(value) for value in values)
        )

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data.strip().upper() or "IDLE"
        self._mode_received_at = time.monotonic()
        if not self._normal_limit_force_context():
            self._normal_limit_qualifier.reset()

    def _on_enable(self, msg: Bool) -> None:
        self._force_enable = bool(msg.data)
        self._enable_received_at = time.monotonic()
        if not self._normal_limit_force_context():
            self._normal_limit_qualifier.reset()

    def _on_heartbeat(self, msg: Bool) -> None:
        self._heartbeat = bool(msg.data)
        self._heartbeat_received_at = time.monotonic()

    def _on_ft_wrench(self, msg: WrenchStamped) -> None:
        values = self._wrench_tuple(msg)
        self._ft_received_at = time.monotonic()
        self._ft_direct_valid = (
            all(math.isfinite(value) for value in values)
            and self._header_fresh(msg, self._config.ft_timeout_s)
        )

    def _on_safety_status(self, msg: String) -> None:
        self._safety_received_at = time.monotonic()
        try:
            status = json.loads(msg.data)
            self._safety_ft_valid = status.get("ft_valid") is True
            self._safety_tf_valid = status.get("tf_valid") is True
            self._safety_abort_latched = bool(status.get("abort_latched", True))
            if self._safety_abort_latched and (
                self._normal_limit_latched
                or self._external_fault_latched
                or self._compliance_ack_latched
            ):
                self._fault_seen_in_safety = True
            elif (
                not self._safety_abort_latched
                and self._fault_seen_in_safety
                and not self._normal_limit_force_context()
                and not self._external_fault_signal
            ):
                # A true safety latch followed by the monitor's interlocked
                # reset is the acknowledgement that permits clearing guard
                # controller-fault latches.
                self._normal_limit_latched = False
                self._external_fault_latched = False
                self._compliance_ack_latched = False
                self._fault_seen_in_safety = False
            self._safety_status_valid = (
                isinstance(status.get("reason"), str)
                and "filtered_wrench" in status
                and "raw_wrench" in status
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            self._safety_ft_valid = False
            self._safety_tf_valid = False
            self._safety_abort_latched = True
            self._safety_status_valid = False

    def _on_abort(self, msg: Bool) -> None:
        # This is the monitor's actionable motion-abort output.  The complete
        # reset-only safety latch is received independently in safety_status;
        # pre-contact sensor invalidity can therefore keep compliance blocked
        # without preventing the geometry-only approach to the runtime tare.
        self._abort_signal_latched = bool(msg.data)

    def _on_controller_fault(self, msg: Bool) -> None:
        self._external_fault_signal = bool(msg.data)
        if msg.data:
            self._external_fault_latched = True
            self._publish_motion_abort()

    def _on_compliance_active(self, msg: Bool) -> None:
        self._compliance_active = bool(msg.data)
        self._compliance_active_received_at = time.monotonic()

    def _on_normal_limit_reached(self, msg: Bool) -> None:
        now = time.monotonic()
        self._normal_limit_reached = bool(msg.data)
        self._normal_limit_received_at = now
        if self._normal_limit_reached:
            self._normal_limit_last_true_at = now
        else:
            self._normal_limit_qualifier.reset()

    def _normal_limit_force_context(self) -> bool:
        return self._force_enable and self._mode in NONZERO_MODES

    def _publish_motion_abort(self) -> None:
        msg = Bool()
        msg.data = True
        self._motion_abort_pub.publish(msg)

    def _on_reset_guard(self, request, response):
        del request
        now = time.monotonic()
        blockers = []
        if self._mode != "IDLE":
            blockers.append("MODE_NOT_IDLE")
        if self._force_enable:
            blockers.append("FORCE_ENABLED")
        if self._safety_abort_latched:
            blockers.append("SAFETY_ABORT_LATCHED")
        if self._normal_limit_reached and self._normal_limit_force_context():
            blockers.append("NORMAL_LIMIT_STILL_ACTIVE")
        if self._external_fault_signal:
            blockers.append("EXTERNAL_FAULT_STILL_ACTIVE")
        if not self._safety_status_valid or not self._safety_ft_valid:
            blockers.append("SAFETY_OR_FT_INVALID")
        if not self._safety_tf_valid or not self._tf_valid:
            blockers.append("TF_INVALID")
        if (
            self._safety_received_at is None
            or now - self._safety_received_at > self._config.safety_status_timeout_s
        ):
            blockers.append("SAFETY_STATUS_STALE")
        response.success = not blockers
        if response.success:
            self._normal_limit_latched = False
            self._normal_limit_qualifier.reset()
            self._external_fault_latched = False
            self._compliance_ack_latched = False
            self._watchdog_latched = False
            self._watchdog_reason = ""
            self._active_force_session = False
            self._fault_seen_in_safety = False
            self._compliance_requested_since = None
            response.message = "wrench guard latches reset"
        else:
            response.message = "reset denied: %s" % ",".join(blockers)
        return response

    def _check_tf(self, now: float) -> None:
        try:
            self._tf_buffer.lookup_transform(
                self._tcp_frame,
                self._ft_frame,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout),
            )
            self._tf_valid = True
        except Exception:
            self._tf_valid = False
        self._tf_checked_at = now

    def _make_output(self, values) -> WrenchStamped:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ft_frame
        msg.wrench.force.x = float(values[0])
        msg.wrench.force.y = float(values[1])
        msg.wrench.force.z = float(values[2])
        msg.wrench.torque.x = float(values[3])
        msg.wrench.torque.y = float(values[4])
        msg.wrench.torque.z = float(values[5])
        return msg

    @staticmethod
    def _age(now, timestamp):
        return None if timestamp is None else max(0.0, now - timestamp)

    @staticmethod
    def _json_number(value):
        return value if math.isfinite(value) else None

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._check_tf(now)
        normal_limit_force_context = self._normal_limit_force_context()
        normal_limit_qualified = self._normal_limit_qualifier.update(
            self._normal_limit_reached, normal_limit_force_context, now
        )
        if normal_limit_qualified and not self._normal_limit_latched:
            self._normal_limit_latched = True
            self._publish_motion_abort()
        controller_received_at = None
        if (
            self._compliance_active_received_at is not None
            and self._normal_limit_received_at is not None
        ):
            controller_received_at = min(
                self._compliance_active_received_at,
                self._normal_limit_received_at,
            )
        controller_fault = (
            self._normal_limit_latched
            or self._external_fault_latched
            or self._compliance_ack_latched
            or self._watchdog_latched
        )
        inputs = GuardInputs(
            now=now,
            real_painting_enabled=self._real_painting_enabled,
            requested_wrench=self._requested_wrench,
            requested_wrench_received_at=self._requested_received_at,
            requested_frame_valid=self._requested_frame_valid,
            mode=self._mode,
            mode_received_at=self._mode_received_at,
            force_enable=self._force_enable,
            force_enable_received_at=self._enable_received_at,
            executor_heartbeat=self._heartbeat,
            executor_heartbeat_received_at=self._heartbeat_received_at,
            ft_valid=self._ft_direct_valid and self._safety_ft_valid,
            ft_received_at=self._ft_received_at,
            tf_valid=self._tf_valid and self._safety_tf_valid,
            tf_checked_at=self._tf_checked_at,
            safety_status_valid=self._safety_status_valid,
            safety_status_received_at=self._safety_received_at,
            abort_latched=(
                self._safety_abort_latched or self._abort_signal_latched
            ),
            controller_fault=controller_fault,
            controller_status_received_at=controller_received_at,
        )
        decision = evaluate_guard(self._config, inputs)
        if decision.compliance_enabled:
            if self._compliance_active:
                self._compliance_requested_since = None
            elif self._compliance_requested_since is None:
                self._compliance_requested_since = now
            elif (
                now - self._compliance_requested_since
                > self._compliance_activation_timeout
            ):
                self._compliance_ack_latched = True
                self._publish_motion_abort()
                inputs = replace(inputs, controller_fault=True)
                decision = evaluate_guard(self._config, inputs)
        else:
            self._compliance_requested_since = None

        force_context = self._real_painting_enabled and normal_limit_force_context
        if decision.forwarding:
            self._active_force_session = True
        elif not force_context:
            self._active_force_session = False
        elif (
            self._active_force_session
            and not inputs.controller_fault
            and not inputs.abort_latched
        ):
            self._watchdog_latched = True
            self._watchdog_reason = ",".join(decision.blockers) or "UNKNOWN"
            self._publish_motion_abort()
            inputs = replace(inputs, controller_fault=True)
            decision = evaluate_guard(self._config, inputs)
        self._controller_pub.publish(self._make_output(decision.output_wrench))

        compliance = Bool()
        compliance.data = decision.compliance_enabled
        self._compliance_pub.publish(compliance)

        controller_status_stale = "CONTROLLER_STATUS_STALE" in decision.blockers
        self._status_sequence += 1
        status = {
            "published_monotonic_s": now,
            "status_sequence": self._status_sequence,
            "forwarding": decision.forwarding,
            "compliance_enabled": decision.compliance_enabled,
            "blockers": list(decision.blockers),
            "mode": self._mode,
            "real_painting_enabled": self._real_painting_enabled,
            "force_enable": self._force_enable,
            "executor_heartbeat": self._heartbeat,
            "ft_valid": inputs.ft_valid,
            "tf_valid": inputs.tf_valid,
            "abort_latched": inputs.abort_latched,
            "controller_fault": inputs.controller_fault or controller_status_stale,
            "controller_fault_latches": {
                "normal_limit": self._normal_limit_latched,
                "external": self._external_fault_latched,
                "compliance_ack": self._compliance_ack_latched,
                "watchdog": self._watchdog_latched,
                "watchdog_reason": self._watchdog_reason,
            },
            "compliance_active": self._compliance_active,
            "normal_limit_reached": self._normal_limit_reached,
            "normal_limit_force_context": normal_limit_force_context,
            "normal_limit_qualified": normal_limit_qualified,
            "normal_limit_pending": (
                self._normal_limit_qualifier.pending
                and not self._normal_limit_latched
            ),
            "normal_limit_active_duration_s": (
                self._normal_limit_qualifier.active_duration(now)
            ),
            "normal_limit_abort_duration_s": self._normal_limit_abort_duration,
            "normal_limit_last_true_age_s": self._age(
                now, self._normal_limit_last_true_at
            ),
            "requested_force_norm_n": self._json_number(
                decision.requested_force_norm_n
            ),
            "requested_torque_norm_nm": self._json_number(
                decision.requested_torque_norm_nm
            ),
            "ages_s": {
                "requested_wrench": self._age(now, self._requested_received_at),
                "mode": self._age(now, self._mode_received_at),
                "force_enable": self._age(now, self._enable_received_at),
                "executor_heartbeat": self._age(now, self._heartbeat_received_at),
                "ft": self._age(now, self._ft_received_at),
                "tf": self._age(now, self._tf_checked_at),
                "safety_status": self._age(now, self._safety_received_at),
                "compliance_active": self._age(
                    now, self._compliance_active_received_at
                ),
                "normal_limit_reached": self._age(
                    now, self._normal_limit_received_at
                ),
            },
        }
        status_msg = String()
        status_msg.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self._status_pub.publish(status_msg)

        if decision.blockers != self._last_blockers or now - self._last_log >= self._log_period:
            self._last_blockers = decision.blockers
            self._last_log = now
            _log_guard_decision(self.get_logger(), decision, self._mode)

    def publish_shutdown_zero_burst(self) -> None:
        zero = (0.0,) * 6
        compliance = Bool()
        compliance.data = False
        for _ in range(self._shutdown_zero_count):
            self._controller_pub.publish(self._make_output(zero))
            self._compliance_pub.publish(compliance)
            time.sleep(0.01)


def main(args=None) -> None:
    # Keep the context alive while KeyboardInterrupt is unwound so the final
    # zero burst can actually reach DDS before publishers are destroyed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PaintingWrenchGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Launch may relay more than one SIGINT.  Do not let a second signal
        # interrupt the fail-safe burst or publisher destruction.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.publish_shutdown_zero_burst()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
