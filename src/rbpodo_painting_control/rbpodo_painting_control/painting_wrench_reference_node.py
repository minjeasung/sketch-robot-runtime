#!/usr/bin/env python3
"""Generate a mode-aware *requested* wrench for the guarded output node.

This node intentionally does not own the controller's wrench-reference topic.
Only ``painting_wrench_guard_node`` is allowed to publish there.
"""

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
from std_msgs.msg import Bool, Float64, String
import tf2_ros


CONTACT_MODES = {"PAINT"}
RAMP_MODES = {"RAMP_UP", "RAMP_DOWN"}
ZERO_FORCE_MODES = {
    "IDLE",
    "APPROACH",
    "APPROACH_PRECONTACT",
    "CONTACT_SEARCH",
    "RETRACT",
    "TRAVEL",
    "NONCONTACT",
    "FINISH_RETRACT",
    "FINAL_RETRACT",
    "CONTACT",
    "ABORT",
}
SUPPORTED_MODES = CONTACT_MODES | RAMP_MODES | ZERO_FORCE_MODES | {"DWELL"}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _quat_rotate(q, v):
    """Rotate vector v by quaternion q=(x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # q * v * q^-1, expanded to avoid extra dependencies.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


class PaintingWrenchReferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("painting_wrench_reference")

        self.declare_parameter("controller_name", "admittance_controller")
        self.declare_parameter(
            "requested_wrench_topic",
            "/painting_admittance/requested_wrench_reference",
        )
        # Kept only as a migration alias.  A direct controller topic is always
        # rejected so an old launch file cannot accidentally bypass the guard.
        self.declare_parameter("wrench_reference_topic", "")
        self.declare_parameter("base_frame", "link0")
        self.declare_parameter("tcp_frame", "tcp")
        self.declare_parameter("ft_frame", "ft_link")
        self.declare_parameter("surface_normal_tcp_axis", "+y")
        self.declare_parameter("target_wrench_axis", "y")
        self.declare_parameter("target_wrench_sign", -1.0)
        self.declare_parameter("desired_contact_force_n", 0.0)
        self.declare_parameter("enable_force", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter(
            "ramp_status_topic", "/painting_admittance/ramp_status"
        )
        self.declare_parameter("force_ramp_up_duration_s", 2.0)
        self.declare_parameter("force_ramp_down_duration_s", 1.0)
        self.declare_parameter("max_command_force_n", 20.0)
        self.declare_parameter("max_force_slew_rate_nps", 5.0)
        self.declare_parameter("publish_zero_wrench_when_not_in_contact_mode", True)
        self.declare_parameter("zero_wrench_on_abort", True)
        self.declare_parameter("dwell_contact_enabled", False)
        self.declare_parameter("tf_lookup_timeout_s", 0.02)
        self.declare_parameter("log_period_s", 1.0)
        # TCP-frame filtered wrench from the force monitor, used only to seed
        # the RAMP_UP start value; an empty topic disables the seeding.
        self.declare_parameter(
            "measured_wrench_topic", "/painting_admittance/force_tcp_filtered"
        )
        self.declare_parameter("measured_wrench_timeout_s", 0.20)

        self._requested_wrench_topic = str(
            self.get_parameter("requested_wrench_topic").value
        ).strip()
        legacy_topic = str(self.get_parameter("wrench_reference_topic").value).strip()
        if legacy_topic:
            self.get_logger().warn(
                "wrench_reference_topic is deprecated; use requested_wrench_topic"
            )
            self._requested_wrench_topic = legacy_topic
        canonical_topic = "/" + self._requested_wrench_topic.strip("/")
        if canonical_topic == "/admittance_controller/wrench_reference":
            raise ValueError(
                "painting_wrench_reference_node must not publish directly to "
                "/admittance_controller/wrench_reference; route it through "
                "painting_wrench_guard_node"
            )
        if not self._requested_wrench_topic:
            raise ValueError("requested_wrench_topic must not be empty")
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._ft_frame = str(self.get_parameter("ft_frame").value)
        self._surface_normal_axis = (
            str(self.get_parameter("surface_normal_tcp_axis").value).strip().lower()
        )
        self._target_axis = (
            str(self.get_parameter("target_wrench_axis").value)
            .strip()
            .lower()
            .removeprefix("force_")
        )
        if self._surface_normal_axis != "+y" or self._target_axis != "y":
            raise ValueError(
                "The roller EOAT painting profile only supports TCP +Y as the "
                "surface normal and TCP force.y as the wrench target."
            )
        self._target_sign = float(self.get_parameter("target_wrench_sign").value)
        if not math.isfinite(self._target_sign) or self._target_sign not in (
            -1.0,
            1.0,
        ):
            raise ValueError("target_wrench_sign must be exactly -1.0 or +1.0")

        self._desired_force = abs(
            float(self.get_parameter("desired_contact_force_n").value)
        )
        self._enable_force = bool(self.get_parameter("enable_force").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._ramp_status_topic = str(
            self.get_parameter("ramp_status_topic").value
        ).strip()
        if not self._ramp_status_topic:
            raise ValueError("ramp_status_topic must not be empty")
        self._ramp_up = max(
            0.0, float(self.get_parameter("force_ramp_up_duration_s").value)
        )
        self._ramp_down = max(
            0.0, float(self.get_parameter("force_ramp_down_duration_s").value)
        )
        self._max_force = max(0.0, float(self.get_parameter("max_command_force_n").value))
        self._max_force_slew_rate = max(
            0.0, float(self.get_parameter("max_force_slew_rate_nps").value)
        )
        self._zero_noncontact = bool(
            self.get_parameter("publish_zero_wrench_when_not_in_contact_mode").value
        )
        self._zero_on_abort = bool(self.get_parameter("zero_wrench_on_abort").value)
        self._dwell_contact_enabled = bool(
            self.get_parameter("dwell_contact_enabled").value
        )
        self._tf_timeout = max(0.0, float(self.get_parameter("tf_lookup_timeout_s").value))
        self._log_period = max(0.1, float(self.get_parameter("log_period_s").value))
        self._measured_wrench_topic = str(
            self.get_parameter("measured_wrench_topic").value
        ).strip()
        self._measured_force_timeout = max(
            0.0, float(self.get_parameter("measured_wrench_timeout_s").value)
        )

        if self._desired_force > self._max_force:
            self.get_logger().warn(
                "desired_contact_force_n %.3f exceeds max_command_force_n %.3f; clamping"
                % (self._desired_force, self._max_force)
            )
            self._desired_force = self._max_force

        self._mode = "IDLE"
        self._aborted = False
        self._measured_force_y = 0.0
        self._measured_force_y_at = 0.0
        self._current_force_y = 0.0
        self._ramp_start_force_y = 0.0
        self._ramp_started_at = time.monotonic()
        self._ramp_complete = True
        self._last_time = time.monotonic()
        self._last_tf_warn = 0.0
        self._last_log = 0.0
        self._ramp_status_sequence = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._requested_wrench_pub = self.create_publisher(
            WrenchStamped, self._requested_wrench_topic, 10
        )
        resolved_request_topic = "/" + self._requested_wrench_pub.topic_name.strip("/")
        if resolved_request_topic == "/admittance_controller/wrench_reference":
            raise ValueError(
                "requested wrench publisher resolves to the controller topic "
                "after ROS remapping; direct publication is forbidden"
            )
        self._tcp_debug_pub = self.create_publisher(
            WrenchStamped, "/painting_admittance/target_wrench_tcp", 10
        )
        self._ft_debug_pub = self.create_publisher(
            WrenchStamped, "/painting_admittance/target_wrench_ft", 10
        )
        self._mode_pub = self.create_publisher(
            String, "/painting_admittance/current_mode", 10
        )
        self._command_force_pub = self.create_publisher(
            Float64, "/painting_admittance/command_force_tcp_y_n", 10
        )
        self._ramp_complete_pub = self.create_publisher(
            Bool, "/painting_admittance/ramp_complete", 10
        )
        self._ramp_status_pub = self.create_publisher(
            String, self._ramp_status_topic, 10
        )

        self.create_subscription(
            String, "/painting_admittance/mode", self._on_mode, 10
        )
        self.create_subscription(
            Float64,
            "/painting_admittance/desired_force_n",
            self._on_desired_force,
            10,
        )
        self.create_subscription(
            Bool, "/painting_admittance/enable_force", self._on_enable_force, 10
        )
        self.create_subscription(Bool, "/painting_admittance/abort", self._on_abort, 10)
        if self._measured_wrench_topic:
            self.create_subscription(
                WrenchStamped,
                self._measured_wrench_topic,
                self._on_measured_wrench,
                10,
            )
        self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().warn(
            "Painting wrench reference ready: dry_run=%s, enable_force=%s, "
            "tcp_axis=force.y, tcp_frame=%s, ft_frame=%s, output=%s"
            % (
                self._dry_run,
                self._enable_force,
                self._tcp_frame,
                self._ft_frame,
                self._requested_wrench_topic,
            )
        )

    def _on_measured_wrench(self, msg: WrenchStamped) -> None:
        value = float(msg.wrench.force.y)
        if not math.isfinite(value):
            return
        self._measured_force_y = value
        self._measured_force_y_at = time.monotonic()

    def _ramp_entry_force_y(self, mode: str) -> float:
        """Start RAMP_UP from the force already on the tool, not from zero.

        CONTACT_SEARCH hands over with the roller already pressed against the
        wall (contact_detect_threshold_n).  Ramping the command from 0 makes
        the admittance read that existing reaction as 'pressed too hard' -
        driving force = measured + command > 0 - so it backs off and breaks
        contact for the first part of the ramp.  Measured 2026-08-14: the
        roller was off the wall for 15-66 % of RAMP_UP, versus 0 % once PAINT
        was steady, and two of six force collapses started inside RAMP_UP.

        Starting the ramp at the commanded direction times abs(measured) makes
        the driving force zero at handover even if the upstream AFT normal sign
        has reversed.  The command direction itself remains unchanged.
        """

        if mode != "RAMP_UP":
            return self._current_force_y
        now = time.monotonic()
        if (
            self._measured_force_y_at <= 0.0
            or now - self._measured_force_y_at > self._measured_force_timeout
        ):
            return self._current_force_y
        target = self._target_sign * self._desired_force
        candidate = self._target_sign * abs(self._measured_force_y)
        # Never start beyond the target and never start on the wrong side of
        # zero. An over-reading must not skip past the configured target.
        low, high = min(0.0, target), max(0.0, target)
        return max(low, min(high, candidate))

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        if not mode:
            return
        if mode not in SUPPORTED_MODES:
            self.get_logger().warn("Unknown painting mode %r; treating as IDLE" % mode)
            mode = "IDLE"
        if mode != self._mode:
            self._ramp_start_force_y = self._ramp_entry_force_y(mode)
            self._ramp_started_at = time.monotonic()
            self._ramp_complete = mode not in RAMP_MODES
        self._mode = mode
        if mode == "ABORT":
            self._current_force_y = 0.0
            self._ramp_complete = True

    def _on_desired_force(self, msg: Float64) -> None:
        requested = float(msg.data)
        if not math.isfinite(requested):
            self.get_logger().error(
                "non-finite desired_force_n received; latching local ABORT"
            )
            self._desired_force = 0.0
            self._current_force_y = 0.0
            self._aborted = True
            return
        desired = abs(requested)
        if desired > self._max_force:
            self.get_logger().warn(
                "requested force %.3f N exceeds max_command_force_n %.3f; clamping"
                % (desired, self._max_force)
            )
            desired = self._max_force
        if self._mode == "RAMP_UP" and abs(desired - self._desired_force) > 1e-9:
            self._ramp_start_force_y = self._current_force_y
            self._ramp_started_at = time.monotonic()
            self._ramp_complete = False
        self._desired_force = desired

    def _on_enable_force(self, msg: Bool) -> None:
        self._enable_force = bool(msg.data)
        if not self._enable_force:
            self._current_force_y = 0.0
            self._ramp_complete = True

    def _on_abort(self, msg: Bool) -> None:
        self._aborted = bool(msg.data)
        if self._aborted and self._zero_on_abort:
            self._current_force_y = 0.0
            self._ramp_complete = True

    def _target_force_y_for_mode(self) -> float:
        if self._aborted:
            return 0.0
        if not self._enable_force:
            return 0.0
        if self._mode in CONTACT_MODES or self._mode == "RAMP_UP":
            return self._target_sign * self._desired_force
        if self._mode == "DWELL" and self._dwell_contact_enabled:
            return self._target_sign * self._desired_force
        if self._mode == "RAMP_DOWN":
            return 0.0
        if self._mode in ZERO_FORCE_MODES:
            return 0.0
        return 0.0 if self._zero_noncontact else self._current_force_y

    def _slew_force(self, target: float, dt: float, now: float) -> float:
        if self._aborted or not self._enable_force:
            self._ramp_complete = True
            return 0.0
        if self._mode in ZERO_FORCE_MODES:
            self._ramp_complete = True
            return 0.0
        if self._mode in RAMP_MODES:
            duration = self._ramp_up if self._mode == "RAMP_UP" else self._ramp_down
            if duration <= 0.0:
                self._ramp_complete = True
                return self._apply_slew_cap(target, dt)
            progress = _clamp((now - self._ramp_started_at) / duration, 0.0, 1.0)
            self._ramp_complete = progress >= 1.0
            candidate = self._ramp_start_force_y + (
                target - self._ramp_start_force_y
            ) * progress
            return self._apply_slew_cap(candidate, dt)

        self._ramp_complete = True
        delta = target - self._current_force_y
        if abs(delta) < 1e-9:
            return target
        duration = self._ramp_up if abs(target) > abs(self._current_force_y) else self._ramp_down
        if duration <= 0.0:
            return self._apply_slew_cap(target, dt)
        reference_force = max(abs(target), abs(self._desired_force), 1e-3)
        max_step = reference_force / duration * max(0.0, dt)
        candidate = self._current_force_y + _clamp(delta, -max_step, max_step)
        return self._apply_slew_cap(candidate, dt)

    def _apply_slew_cap(self, candidate: float, dt: float) -> float:
        if self._max_force_slew_rate <= 0.0:
            return candidate
        max_delta = self._max_force_slew_rate * max(0.0, dt)
        return self._current_force_y + _clamp(
            candidate - self._current_force_y,
            -max_delta,
            max_delta,
        )

    def _make_wrench(self, frame_id: str, force_xyz: list[float]) -> WrenchStamped:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.wrench.force.x = float(force_xyz[0])
        msg.wrench.force.y = float(force_xyz[1])
        msg.wrench.force.z = float(force_xyz[2])
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        return msg

    def _transform_force_tcp_to_ft(self, tcp_force: list[float]) -> list[float] | None:
        if self._tcp_frame == self._ft_frame:
            return list(tcp_force)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._ft_frame,
                self._tcp_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_tf_warn > self._log_period:
                self._last_tf_warn = now
                self.get_logger().error(
                    "TF lookup %s <- %s failed; publishing zero wrench: %s"
                    % (self._ft_frame, self._tcp_frame, exc)
                )
            return None
        q = transform.transform.rotation
        return _quat_rotate([q.x, q.y, q.z, q.w], tcp_force)

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = "ABORT" if self._aborted else self._mode
        self._mode_pub.publish(msg)

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        target_force_y = self._target_force_y_for_mode()
        self._current_force_y = self._slew_force(target_force_y, dt, now)
        if not math.isfinite(self._current_force_y):
            self.get_logger().error("non-finite force command generated; latching ABORT")
            self._aborted = True
            self._current_force_y = 0.0
        elif abs(self._current_force_y) > self._max_force:
            self.get_logger().error(
                "command force %.3f exceeds max_command_force_n %.3f; aborting"
                % (self._current_force_y, self._max_force)
            )
            self._aborted = True
            self._current_force_y = 0.0

        tcp_force = [0.0, self._current_force_y, 0.0]
        tcp_msg = self._make_wrench(self._tcp_frame, tcp_force)
        self._tcp_debug_pub.publish(tcp_msg)

        ft_force = self._transform_force_tcp_to_ft(tcp_force)
        if ft_force is None:
            ft_force = [0.0, 0.0, 0.0]
        ft_msg = self._make_wrench(self._ft_frame, ft_force)
        self._ft_debug_pub.publish(ft_msg)
        self._publish_mode()

        command_force = Float64()
        command_force.data = float(self._current_force_y)
        self._command_force_pub.publish(command_force)
        ramp_complete = Bool()
        ramp_complete.data = bool(self._ramp_complete)
        self._ramp_complete_pub.publish(ramp_complete)
        self._ramp_status_sequence += 1
        ramp_status = String()
        ramp_status.data = json.dumps(
            {
                "force_enable": bool(self._enable_force),
                "mode": "ABORT" if self._aborted else self._mode,
                "published_monotonic_s": now,
                "ramp_complete": bool(self._ramp_complete),
                "status_sequence": self._ramp_status_sequence,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        self._ramp_status_pub.publish(ramp_status)

        if self._dry_run:
            requested_msg = self._make_wrench(self._ft_frame, [0.0, 0.0, 0.0])
        else:
            requested_msg = ft_msg
        self._requested_wrench_pub.publish(requested_msg)

        if now - self._last_log > self._log_period:
            self._last_log = now
            self.get_logger().info(
                "mode=%s enable=%s dry_run=%s tcp_force_y=%.3f ft_force=(%.3f, %.3f, %.3f)"
                % (
                    "ABORT" if self._aborted else self._mode,
                    self._enable_force,
                    self._dry_run,
                    self._current_force_y,
                    ft_force[0],
                    ft_force[1],
                    ft_force[2],
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PaintingWrenchReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
