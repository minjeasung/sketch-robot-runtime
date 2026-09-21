#!/usr/bin/env python3
"""Six-axis filtered and raw F/T safety monitor for painting admittance."""

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
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
import tf2_ros

from .force_safety import (
    ADMITTANCE_TRIM_LIMIT,
    BiasEstimator,
    CONTROLLER_FAULT,
    FT_SATURATION,
    ForceSafetyMonitor,
    ModeLimits,
    NONE,
    ResetConditions,
    ROLLER_BALANCE_LIMIT,
    SafetyConfig,
    SustainedCondition,
    consume_free_space_confirmation,
    motion_abort_required,
)


MONITORED_MODES = (
    "PAINT",
    "RAMP_UP",
    "RAMP_DOWN",
    "CONTACT_SEARCH",
    "APPROACH_PRECONTACT",
    "RETRACT",
    "TRAVEL",
    "IDLE",
    "FINAL_RETRACT",
    "ABORT",
)
MODE_ALIASES = {
    "APPROACH": "APPROACH_PRECONTACT",
    "FINISH_RETRACT": "FINAL_RETRACT",
    "NONCONTACT": "TRAVEL",
    "CONTACT": "PAINT",
    "DWELL": "PAINT",
}
NONCONTACT_MODES = {
    "IDLE",
    "APPROACH_PRECONTACT",
    "RETRACT",
    "TRAVEL",
    "FINAL_RETRACT",
    "ABORT",
}


def _apply_enabled_upper_bound(value: float, upper_bound: float) -> float:
    """Cap an enabled per-mode limit without reviving an explicit zero."""

    if value <= 0.0 or upper_bound <= 0.0:
        return value
    return min(value, upper_bound)


def _paint_limits() -> ModeLimits:
    return ModeLimits(
        force_axis_n=(10.0, 15.0, 10.0),
        force_norm_n=18.0,
        torque_axis_nm=(1.5, 1.5, 1.5),
        torque_norm_nm=2.0,
        force_derivative_nps=250.0,
        torque_derivative_nmps=20.0,
        raw_force_axis_n=(25.0, 25.0, 25.0),
        raw_force_norm_n=30.0,
        raw_torque_axis_nm=(3.0, 3.0, 3.0),
        raw_torque_norm_nm=4.0,
        raw_force_derivative_nps=1200.0,
        raw_torque_derivative_nmps=100.0,
        filtered_debounce_s=0.02,
        raw_debounce_s=0.0,
    )


def _search_limits() -> ModeLimits:
    return ModeLimits(
        force_axis_n=(6.0, 8.0, 6.0),
        force_norm_n=10.0,
        torque_axis_nm=(1.0, 1.0, 1.0),
        torque_norm_nm=1.5,
        force_derivative_nps=200.0,
        torque_derivative_nmps=15.0,
        raw_force_axis_n=(15.0, 15.0, 15.0),
        raw_force_norm_n=20.0,
        raw_torque_axis_nm=(2.0, 2.0, 2.0),
        raw_torque_norm_nm=3.0,
        raw_force_derivative_nps=900.0,
        raw_torque_derivative_nmps=75.0,
        contact_opposite_force_n=1.5,
        contact_off_axis_force_n=3.0,
        filtered_debounce_s=0.01,
        raw_debounce_s=0.0,
    )


def _noncontact_limits() -> ModeLimits:
    return ModeLimits(
        force_axis_n=(10.0, 10.0, 10.0),
        force_norm_n=15.0,
        torque_axis_nm=(1.0, 1.0, 1.0),
        torque_norm_nm=1.5,
        force_derivative_nps=200.0,
        torque_derivative_nmps=15.0,
        raw_force_axis_n=(12.0, 12.0, 12.0),
        raw_force_norm_n=15.0,
        raw_torque_axis_nm=(2.0, 2.0, 2.0),
        raw_torque_norm_nm=2.5,
        raw_force_derivative_nps=750.0,
        raw_torque_derivative_nmps=60.0,
        unexpected_contact_n=3.0,
        filtered_debounce_s=0.05,
        raw_debounce_s=0.0,
    )


def _default_limits_for(mode: str) -> ModeLimits:
    if mode == "CONTACT_SEARCH":
        return _search_limits()
    if mode in NONCONTACT_MODES:
        return _noncontact_limits()
    return _paint_limits()


def _quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm3(values) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


class PaintingForceMonitorNode(Node):
    """Transform, filter, diagnose and latch unsafe six-axis wrench data."""

    def __init__(self) -> None:
        super().__init__("painting_force_monitor")
        self._spray_process_mode = False
        self._spray_process_time = 0.0
        self.create_subscription(String, "/painting_system/process_mode", self._on_process_mode, 10)

        self.declare_parameter(
            "input_wrench_topic", "/force_torque_sensor_broadcaster/wrench"
        )
        self.declare_parameter(
            "raw_wrench_topic", "/force_torque_sensor_broadcaster_raw/wrench"
        )
        self.declare_parameter("ft_frame", "ft_link")
        self.declare_parameter("tcp_frame", "tcp")
        self.declare_parameter(
            "controller_fault_topic", "/painting_admittance/controller_fault"
        )
        self.declare_parameter(
            "normal_limit_reached_topic",
            "/admittance_controller/normal_limit_reached",
        )
        self.declare_parameter("force_filter_tau_s", 0.10)
        self.declare_parameter("force_deadband_n", 0.5)
        self.declare_parameter("torque_deadband_nm", 0.02)
        self.declare_parameter("bias_sample_duration_s", 1.0)
        self.declare_parameter("update_bias_in_noncontact", True)
        self.declare_parameter("bias_max_force_norm_n", 1.0)
        self.declare_parameter("bias_max_torque_norm_nm", 0.2)
        self.declare_parameter("bias_contact_holdoff_s", 2.0)
        self.declare_parameter("contact_force_sign", 1.0)
        self.declare_parameter("absolute_normal_force", True)
        self.declare_parameter("contact_detect_threshold_n", 1.5)
        self.declare_parameter("contact_release_threshold_n", 0.8)
        self.declare_parameter("contact_confirm_duration_s", 0.10)
        # Compatibility parameters remain authoritative upper bounds.
        self.declare_parameter("travel_collision_threshold_n", 3.0)
        self.declare_parameter("over_force_warn_n", 10.0)
        self.declare_parameter("over_force_abort_n", 15.0)
        self.declare_parameter("sensor_force_saturation_n", 190.0)
        self.declare_parameter("sensor_torque_saturation_nm", 9.5)
        self.declare_parameter("stale_timeout_s", 0.20)
        self.declare_parameter("raw_stale_timeout_s", 0.20)
        self.declare_parameter("max_message_age_s", 0.20)
        self.declare_parameter("tf_valid_timeout_s", 0.20)
        self.declare_parameter("tf_lookup_timeout_s", 0.02)
        self.declare_parameter("reset_max_force_norm_n", 1.0)
        self.declare_parameter("reset_max_torque_norm_nm", 0.2)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("log_period_s", 1.0)
        self.declare_parameter("normal_limit_abort_duration_s", 0.25)
        self.declare_parameter(
            "roller_balance_limit_reached_topic",
            "/admittance_controller/roller_balance/limit_reached",
        )
        self.declare_parameter("roller_balance_limit_abort_duration_s", 0.25)

        for mode in MONITORED_MODES:
            self._declare_mode_limits(mode, _default_limits_for(mode))

        self._input_topic = str(self.get_parameter("input_wrench_topic").value)
        self._raw_topic = str(self.get_parameter("raw_wrench_topic").value)
        self._require_raw = bool(self._raw_topic)
        self._ft_frame = str(self.get_parameter("ft_frame").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._controller_fault_topic = str(
            self.get_parameter("controller_fault_topic").value
        )
        self._normal_limit_topic = str(
            self.get_parameter("normal_limit_reached_topic").value
        )
        self._roller_balance_limit_topic = str(
            self.get_parameter("roller_balance_limit_reached_topic").value
        )
        self._filter_tau = max(
            0.0, float(self.get_parameter("force_filter_tau_s").value)
        )
        self._force_deadband = max(
            0.0, float(self.get_parameter("force_deadband_n").value)
        )
        self._torque_deadband = max(
            0.0, float(self.get_parameter("torque_deadband_nm").value)
        )
        self._bias_duration = max(
            0.0, float(self.get_parameter("bias_sample_duration_s").value)
        )
        self._update_bias = bool(
            self.get_parameter("update_bias_in_noncontact").value
        )
        self._bias_max_force = max(
            0.0, float(self.get_parameter("bias_max_force_norm_n").value)
        )
        self._bias_max_torque = max(
            0.0, float(self.get_parameter("bias_max_torque_norm_nm").value)
        )
        self._bias_contact_holdoff = max(
            0.0, float(self.get_parameter("bias_contact_holdoff_s").value)
        )
        self._warn_force = max(
            0.0, float(self.get_parameter("over_force_warn_n").value)
        )
        self._tf_lookup_timeout = max(
            0.0, float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        self._rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._log_period = max(0.1, float(self.get_parameter("log_period_s").value))
        self._normal_limit_abort_duration = self._positive(
            "normal_limit_abort_duration_s"
        )
        self._roller_balance_limit_abort_duration = self._positive(
            "roller_balance_limit_abort_duration_s"
        )

        mode_limits = {
            mode: self._read_mode_limits(mode) for mode in MONITORED_MODES
        }
        abort_force = max(
            0.0, float(self.get_parameter("over_force_abort_n").value)
        )
        travel_force = max(
            0.0, float(self.get_parameter("travel_collision_threshold_n").value)
        )
        if abort_force > 0.0:
            for mode, limits in list(mode_limits.items()):
                y_limit = limits.force_axis_n[1]
                y_limit = _apply_enabled_upper_bound(y_limit, abort_force)
                mode_limits[mode] = replace(
                    limits,
                    force_axis_n=(limits.force_axis_n[0], y_limit, limits.force_axis_n[2]),
                )
        if travel_force > 0.0:
            for mode in NONCONTACT_MODES:
                limits = mode_limits[mode]
                threshold = limits.unexpected_contact_n
                threshold = travel_force if threshold <= 0.0 else min(threshold, travel_force)
                mode_limits[mode] = replace(limits, unexpected_contact_n=threshold)

        started_at = time.monotonic()
        contact_force_sign = float(self.get_parameter("contact_force_sign").value)
        if not math.isfinite(contact_force_sign) or contact_force_sign not in (
            -1.0,
            1.0,
        ):
            raise ValueError("contact_force_sign must be exactly -1.0 or +1.0")
        config = SafetyConfig(
            mode_limits=mode_limits,
            fallback_limits=_noncontact_limits(),
            contact_force_sign=contact_force_sign,
            absolute_normal_force=bool(
                self.get_parameter("absolute_normal_force").value
            ),
            contact_detect_n=max(
                0.0, float(self.get_parameter("contact_detect_threshold_n").value)
            ),
            contact_release_n=max(
                0.0, float(self.get_parameter("contact_release_threshold_n").value)
            ),
            contact_confirm_duration_s=max(
                0.0,
                float(self.get_parameter("contact_confirm_duration_s").value),
            ),
            force_saturation_n=max(
                0.0, float(self.get_parameter("sensor_force_saturation_n").value)
            ),
            torque_saturation_nm=max(
                0.0, float(self.get_parameter("sensor_torque_saturation_nm").value)
            ),
            filtered_timeout_s=self._positive("stale_timeout_s"),
            raw_timeout_s=self._positive("raw_stale_timeout_s"),
            tf_timeout_s=self._positive("tf_valid_timeout_s"),
            max_message_age_s=self._positive("max_message_age_s"),
            reset_max_force_norm_n=max(
                0.0, float(self.get_parameter("reset_max_force_norm_n").value)
            ),
            reset_max_torque_norm_nm=max(
                0.0,
                float(self.get_parameter("reset_max_torque_norm_nm").value),
            ),
        )
        self._safety = ForceSafetyMonitor(config, started_at=started_at)

        self._mode = "UNKNOWN"
        self._mode_received = False
        self._force_enabled = False
        self._enable_received = False
        self._trajectory_active = True
        self._robot_stationary = False
        self._free_space = False
        self._bias_estimator = BiasEstimator(self._bias_duration)
        self._filtered = None
        self._last_filter_time = None
        self._last_reason = NONE
        self._last_log = 0.0
        self._normal_limit_reached = False
        self._normal_limit_received_at = None
        self._normal_limit_last_true_at = None
        self._normal_limit_qualifier = SustainedCondition(
            self._normal_limit_abort_duration
        )
        self._roller_balance_limit_reached = False
        self._roller_balance_limit_received_at = None
        self._roller_balance_limit_last_true_at = None
        self._roller_balance_limit_qualifier = SustainedCondition(
            self._roller_balance_limit_abort_duration
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._filtered_pub = self.create_publisher(
            WrenchStamped, "/painting_admittance/force_tcp_filtered", 10
        )
        self._raw_pub = self.create_publisher(
            WrenchStamped, "/painting_admittance/force_tcp_raw", 10
        )
        self._normal_pub = self.create_publisher(
            Float64, "/painting_admittance/normal_force_tcp_y", 10
        )
        self._contact_pub = self.create_publisher(
            String, "/painting_admittance/contact_state", 10
        )
        self._contact_confirmed_pub = self.create_publisher(
            Bool, "/painting_admittance/contact_confirmed", 10
        )
        self._overforce_pub = self.create_publisher(
            String, "/painting_admittance/overforce_state", 10
        )
        self._reason_pub = self.create_publisher(
            String, "/painting_admittance/abort_reason", 10
        )
        self._status_pub = self.create_publisher(
            String, "/painting_admittance/safety_status", 10
        )
        self._abort_pub = self.create_publisher(Bool, "/painting_admittance/abort", 10)
        self._motion_abort_pub = self.create_publisher(Bool, "/motion_abort", 10)

        self.create_subscription(WrenchStamped, self._input_topic, self._on_wrench, 10)
        if self._raw_topic and self._raw_topic != self._input_topic:
            self.create_subscription(
                WrenchStamped, self._raw_topic, self._on_raw_wrench, 10
            )
        self.create_subscription(String, "/painting_admittance/mode", self._on_mode, 10)
        self.create_subscription(
            Bool, "/painting_admittance/enable_force", self._on_enable, 10
        )
        self.create_subscription(
            Bool,
            "/painting_admittance/trajectory_active",
            self._on_trajectory_active,
            10,
        )
        self.create_subscription(
            Bool,
            "/painting_admittance/robot_stationary",
            self._on_robot_stationary,
            10,
        )
        self.create_subscription(
            Bool,
            "/painting_admittance/free_space_confirmed",
            self._on_free_space,
            10,
        )
        self.create_subscription(
            Bool,
            self._controller_fault_topic,
            self._on_controller_fault,
            10,
        )
        self.create_subscription(
            Bool,
            self._normal_limit_topic,
            self._on_normal_limit_reached,
            10,
        )
        self.create_subscription(
            Bool,
            self._roller_balance_limit_topic,
            self._on_roller_balance_limit_reached,
            10,
        )
        self.create_service(
            Trigger, "/painting_admittance/reset_safety", self._on_reset
        )
        self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info(
            "Painting F/T safety ready: filtered=%s raw=%s normal=%s "
            "signed_diagnostic=true contact_sign=%+.0f; "
            "reset requires IDLE/disabled/inactive/stationary/free-space/low-wrench"
            % (
                self._input_topic,
                self._raw_topic or "disabled",
                (
                    "abs(TCP Fy)"
                    if self._safety.config.absolute_normal_force
                    else "signed TCP Fy"
                ),
                self._safety.config.contact_force_sign,
            )
        )

    def _on_process_mode(self, msg):
        try:
            data = json.loads(msg.data)
            self._spray_process_mode = data.get("mode") == "spray"
            self._spray_process_time = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self._spray_process_mode = False

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError("%s must be positive" % name)
        return value

    def _declare_mode_limits(self, mode: str, defaults: ModeLimits) -> None:
        prefix = "limits.%s." % mode.lower()
        self.declare_parameter(prefix + "force_axis_n", list(defaults.force_axis_n))
        self.declare_parameter(prefix + "force_norm_n", defaults.force_norm_n)
        self.declare_parameter(prefix + "torque_axis_nm", list(defaults.torque_axis_nm))
        self.declare_parameter(prefix + "torque_norm_nm", defaults.torque_norm_nm)
        self.declare_parameter(
            prefix + "force_derivative_nps", defaults.force_derivative_nps
        )
        self.declare_parameter(
            prefix + "torque_derivative_nmps", defaults.torque_derivative_nmps
        )
        self.declare_parameter(
            prefix + "raw_force_axis_n", list(defaults.raw_force_axis_n)
        )
        self.declare_parameter(prefix + "raw_force_norm_n", defaults.raw_force_norm_n)
        self.declare_parameter(
            prefix + "raw_torque_axis_nm", list(defaults.raw_torque_axis_nm)
        )
        self.declare_parameter(
            prefix + "raw_torque_norm_nm", defaults.raw_torque_norm_nm
        )
        self.declare_parameter(
            prefix + "raw_force_derivative_nps",
            defaults.raw_force_derivative_nps,
        )
        self.declare_parameter(
            prefix + "raw_torque_derivative_nmps",
            defaults.raw_torque_derivative_nmps,
        )
        self.declare_parameter(
            prefix + "unexpected_contact_n", defaults.unexpected_contact_n
        )
        self.declare_parameter(
            prefix + "contact_opposite_force_n",
            defaults.contact_opposite_force_n,
        )
        self.declare_parameter(
            prefix + "contact_off_axis_force_n",
            defaults.contact_off_axis_force_n,
        )
        self.declare_parameter(
            prefix + "filtered_debounce_s", defaults.filtered_debounce_s
        )
        self.declare_parameter(prefix + "raw_debounce_s", defaults.raw_debounce_s)

    def _vector_parameter(self, name: str):
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != 3 or not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("%s must contain three finite non-negative values" % name)
        return values

    def _read_mode_limits(self, mode: str) -> ModeLimits:
        prefix = "limits.%s." % mode.lower()
        scalar = lambda suffix: max(
            0.0, float(self.get_parameter(prefix + suffix).value)
        )
        return ModeLimits(
            force_axis_n=self._vector_parameter(prefix + "force_axis_n"),
            force_norm_n=scalar("force_norm_n"),
            torque_axis_nm=self._vector_parameter(prefix + "torque_axis_nm"),
            torque_norm_nm=scalar("torque_norm_nm"),
            force_derivative_nps=scalar("force_derivative_nps"),
            torque_derivative_nmps=scalar("torque_derivative_nmps"),
            raw_force_axis_n=self._vector_parameter(prefix + "raw_force_axis_n"),
            raw_force_norm_n=scalar("raw_force_norm_n"),
            raw_torque_axis_nm=self._vector_parameter(prefix + "raw_torque_axis_nm"),
            raw_torque_norm_nm=scalar("raw_torque_norm_nm"),
            raw_force_derivative_nps=scalar("raw_force_derivative_nps"),
            raw_torque_derivative_nmps=scalar("raw_torque_derivative_nmps"),
            unexpected_contact_n=scalar("unexpected_contact_n"),
            contact_opposite_force_n=scalar("contact_opposite_force_n"),
            contact_off_axis_force_n=scalar("contact_off_axis_force_n"),
            filtered_debounce_s=scalar("filtered_debounce_s"),
            raw_debounce_s=scalar("raw_debounce_s"),
        )

    def _normalized_mode(self, value: str) -> str:
        mode = value.strip().upper() or "IDLE"
        mode = MODE_ALIASES.get(mode, mode)
        return mode if mode in MONITORED_MODES else "UNKNOWN"

    def _on_mode(self, msg: String) -> None:
        self._mode = self._normalized_mode(msg.data)
        self._mode_received = True
        self._free_space = consume_free_space_confirmation(
            self._free_space, self._mode, self._trajectory_active
        )
        if self._mode != "IDLE":
            # A free-space confirmation is a one-shot preflight assertion.
            self._reset_bias_window()

    def _on_enable(self, msg: Bool) -> None:
        self._force_enabled = bool(msg.data)
        self._enable_received = True
        if self._force_enabled:
            self._reset_bias_window()

    def _on_trajectory_active(self, msg: Bool) -> None:
        self._trajectory_active = bool(msg.data)
        self._free_space = consume_free_space_confirmation(
            self._free_space, self._mode, self._trajectory_active
        )
        if self._trajectory_active:
            self._reset_bias_window()

    def _on_robot_stationary(self, msg: Bool) -> None:
        self._robot_stationary = bool(msg.data)

    def _on_free_space(self, msg: Bool) -> None:
        self._free_space = consume_free_space_confirmation(
            bool(msg.data), self._mode, self._trajectory_active
        )

    def _on_controller_fault(self, msg: Bool) -> None:
        if msg.data:
            self._safety.latch_external(
                CONTROLLER_FAULT, "controller fault input asserted", time.monotonic()
            )

    def _on_normal_limit_reached(self, msg: Bool) -> None:
        now = time.monotonic()
        self._normal_limit_reached = bool(msg.data)
        self._normal_limit_received_at = now
        if self._normal_limit_reached:
            self._normal_limit_last_true_at = now
        else:
            self._normal_limit_qualifier.reset()

    def _on_roller_balance_limit_reached(self, msg: Bool) -> None:
        now = time.monotonic()
        self._roller_balance_limit_reached = bool(msg.data)
        self._roller_balance_limit_received_at = now
        if self._roller_balance_limit_reached:
            self._roller_balance_limit_last_true_at = now
        else:
            self._roller_balance_limit_qualifier.reset()

    def _message_age(self, msg: WrenchStamped) -> float:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return math.inf
        try:
            return (
                self.get_clock().now() - Time.from_msg(msg.header.stamp)
            ).nanoseconds / 1e9
        except Exception:
            return math.inf

    def _transform_wrench(self, msg: WrenchStamped, now: float):
        source_frame = msg.header.frame_id or self._ft_frame
        force = (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
        )
        torque = (
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )
        if source_frame == self._tcp_frame:
            self._safety.observe_tf(True, now, 0.0)
            return (*force, *torque)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._tcp_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout),
            )
        except Exception as exc:
            self._safety.observe_tf(False, now)
            self._invalidate_bias()
            if self._safety.latched_reason == "TF_INVALID":
                self._safety.latched_detail = "%s <- %s: %s" % (
                    self._tcp_frame,
                    source_frame,
                    exc,
                )
            return None

        q = transform.transform.rotation
        quat = (q.x, q.y, q.z, q.w)
        rotated_force = _quat_rotate(quat, force)
        rotated_torque = _quat_rotate(quat, torque)
        translation = transform.transform.translation
        moment_arm = (translation.x, translation.y, translation.z)
        shifted_torque = tuple(
            torque_value + shift
            for torque_value, shift in zip(
                rotated_torque, _cross(moment_arm, rotated_force)
            )
        )
        source_age = None
        if transform.header.stamp.sec != 0 or transform.header.stamp.nanosec != 0:
            source_age = (
                self.get_clock().now() - Time.from_msg(transform.header.stamp)
            ).nanoseconds / 1e9
        if source_age is not None and (
            source_age < -0.05 or source_age > self._safety.config.tf_timeout_s
        ):
            self._safety.observe_tf(False, now, source_age)
            self._invalidate_bias()
            if self._safety.latched_reason == "TF_INVALID":
                self._safety.latched_detail = "transform source age %.6f s" % source_age
            return None
        self._safety.observe_tf(True, now, source_age)
        return (*rotated_force, *shifted_torque)

    def _reset_bias_window(self) -> None:
        self._bias_estimator.reset_window()

    def _invalidate_bias(self) -> None:
        self._bias_estimator.invalidate(clear_bias=True)

    def _bias_interlocks_ok(self, values, now: float) -> bool:
        recent_contact = self._safety.last_contact_at
        return (
            self._update_bias
            and self._mode_received
            and self._enable_received
            and self._mode == "IDLE"
            and not self._force_enabled
            and not self._trajectory_active
            and self._robot_stationary
            and self._free_space
            and all(math.isfinite(value) for value in values)
            and not self._safety.nonfinite
            and not self._safety.saturated
            and _norm3(values[:3]) <= self._bias_max_force
            and _norm3(values[3:]) <= self._bias_max_torque
            and (
                recent_contact is None
                or now - recent_contact >= self._bias_contact_holdoff
            )
        )

    def _update_bias_estimate(self, values, now: float) -> None:
        self._bias_estimator.observe(
            values, now, eligible=self._bias_interlocks_ok(values, now)
        )

    def _filter(self, values, now: float):
        sample = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in sample):
            # Preserve the invalid sample for fail-closed safety handling, but
            # never retain it as IIR state.  The hardware intentionally emits
            # NaN until tare is ready, and a NaN IIR seed cannot recover when
            # later samples become finite.
            self._filtered = None
            self._last_filter_time = None
            return sample
        if (
            self._filtered is None
            or self._last_filter_time is None
            or not all(math.isfinite(value) for value in self._filtered)
        ):
            self._filtered = list(sample)
            self._last_filter_time = now
            return sample
        dt = max(0.0, now - self._last_filter_time)
        self._last_filter_time = now
        alpha = 1.0 if self._filter_tau <= 0.0 else dt / (self._filter_tau + dt)
        self._filtered = [
            old + alpha * (new - old) for old, new in zip(self._filtered, sample)
        ]
        return tuple(self._filtered)

    def _deadband(self, values):
        result = list(values)
        for index in range(3):
            if abs(result[index]) < self._force_deadband:
                result[index] = 0.0
        for index in range(3, 6):
            if abs(result[index]) < self._torque_deadband:
                result[index] = 0.0
        return tuple(result)

    def _publish_wrench(self, publisher, values) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._tcp_frame
        msg.wrench.force.x = float(values[0])
        msg.wrench.force.y = float(values[1])
        msg.wrench.force.z = float(values[2])
        msg.wrench.torque.x = float(values[3])
        msg.wrench.torque.y = float(values[4])
        msg.wrench.torque.z = float(values[5])
        publisher.publish(msg)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        transformed = self._transform_wrench(msg, now)
        if transformed is None:
            return
        message_age = self._message_age(msg)
        message_fresh = (
            math.isfinite(message_age)
            and message_age >= -0.05
            and message_age <= self._safety.config.max_message_age_s
        )
        if message_fresh and all(math.isfinite(value) for value in transformed):
            self._update_bias_estimate(transformed, now)
        else:
            self._invalidate_bias()
        relative = tuple(
            value - bias for value, bias in zip(transformed, self._bias_estimator.bias)
        )
        filtered = self._deadband(self._filter(relative, now))
        self._safety.process_filtered(
            filtered, now, self._mode, message_age_s=message_age
        )
        self._publish_wrench(self._filtered_pub, filtered)
        normal = Float64()
        normal.data = float(filtered[1])
        self._normal_pub.publish(normal)
        if self._raw_topic == self._input_topic:
            self._process_raw_values(transformed, msg, now)

    def _process_raw_values(self, transformed, msg, now: float) -> None:
        message_age = self._message_age(msg)
        if (
            not all(math.isfinite(value) for value in transformed)
            or not math.isfinite(message_age)
            or message_age < -0.05
            or message_age > self._safety.config.max_message_age_s
        ):
            self._invalidate_bias()
        force_saturation = self._safety.config.force_saturation_n
        torque_saturation = self._safety.config.torque_saturation_nm
        if (
            force_saturation > 0.0
            and any(abs(value) >= force_saturation for value in transformed[:3])
        ) or (
            torque_saturation > 0.0
            and any(abs(value) >= torque_saturation for value in transformed[3:])
        ):
            self._safety.latch_external(
                FT_SATURATION,
                "raw sensor saturation before adaptive bias subtraction",
                now,
            )
        relative = tuple(
            value - bias for value, bias in zip(transformed, self._bias_estimator.bias)
        )
        self._safety.process_raw(
            relative, now, self._mode, message_age_s=message_age
        )
        self._publish_wrench(self._raw_pub, relative)

    def _on_raw_wrench(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        transformed = self._transform_wrench(msg, now)
        if transformed is not None:
            self._process_raw_values(transformed, msg, now)

    def _on_reset(self, request, response):
        del request
        now = time.monotonic()
        result = self._safety.reset(
            ResetConditions(
                mode=self._mode,
                force_enabled=self._force_enabled,
                trajectory_active=self._trajectory_active,
                robot_stationary=self._robot_stationary,
                free_space=self._free_space,
            ),
            now,
            require_raw=self._require_raw,
        )
        response.success = result.success
        if result.success:
            response.message = "safety latch reset"
            self._invalidate_bias()
            self.get_logger().info(response.message)
        else:
            response.message = "reset denied: %s" % ",".join(result.blockers)
            self.get_logger().warn(response.message)
        return response

    def _on_timer(self) -> None:
        now = time.monotonic()
        normal_limit_force_context = self._mode in {
            "RAMP_UP",
            "PAINT",
            "RAMP_DOWN",
        }
        normal_limit_qualified = self._normal_limit_qualifier.update(
            self._normal_limit_reached, normal_limit_force_context, now
        )
        roller_balance_limit_qualified = (
            self._roller_balance_limit_qualifier.update(
                self._roller_balance_limit_reached,
                normal_limit_force_context,
                now,
            )
        )
        if normal_limit_qualified:
            self._safety.latch_external(
                ADMITTANCE_TRIM_LIMIT,
                "admittance normal trim/velocity/acceleration limit sustained",
                now,
            )
        if roller_balance_limit_qualified:
            self._safety.latch_external(
                ROLLER_BALANCE_LIMIT,
                "roller balance rotation trim limit sustained",
                now,
            )
        self._safety.tick(now, require_raw=self._require_raw)
        status = self._safety.status(now, self._mode, require_raw=self._require_raw)
        if not status["ft_valid"] or not status["tf_valid"]:
            self._invalidate_bias()
        status.update(
            {
                "force_enabled": self._force_enabled,
                "trajectory_active": self._trajectory_active,
                "robot_stationary": self._robot_stationary,
                "free_space_confirmed": self._free_space,
                "bias_ready": self._bias_estimator.ready,
                "bias_wrench": list(self._bias_estimator.bias),
                "normal_limit_reached": self._normal_limit_reached,
                "normal_limit_force_context": normal_limit_force_context,
                "normal_limit_qualified": normal_limit_qualified,
                "normal_limit_pending": (
                    self._normal_limit_qualifier.pending
                    and self._safety.latched_reason != ADMITTANCE_TRIM_LIMIT
                ),
                "normal_limit_active_duration_s": (
                    self._normal_limit_qualifier.active_duration(now)
                ),
                "normal_limit_abort_duration_s": (
                    self._normal_limit_abort_duration
                ),
                "normal_limit_last_true_age_s": (
                    None
                    if self._normal_limit_last_true_at is None
                    else max(0.0, now - self._normal_limit_last_true_at)
                ),
                "normal_limit_status_age_s": (
                    None
                    if self._normal_limit_received_at is None
                    else max(0.0, now - self._normal_limit_received_at)
                ),
                "roller_balance_limit_reached": (
                    self._roller_balance_limit_reached
                ),
                "roller_balance_limit_force_context": (
                    normal_limit_force_context
                ),
                "roller_balance_limit_qualified": (
                    roller_balance_limit_qualified
                ),
                "roller_balance_limit_pending": (
                    self._roller_balance_limit_qualifier.pending
                    and self._safety.latched_reason != ROLLER_BALANCE_LIMIT
                ),
                "roller_balance_limit_active_duration_s": (
                    self._roller_balance_limit_qualifier.active_duration(now)
                ),
                "roller_balance_limit_abort_duration_s": (
                    self._roller_balance_limit_abort_duration
                ),
                "roller_balance_limit_last_true_age_s": (
                    None
                    if self._roller_balance_limit_last_true_at is None
                    else max(
                        0.0, now - self._roller_balance_limit_last_true_at
                    )
                ),
                "roller_balance_limit_status_age_s": (
                    None
                    if self._roller_balance_limit_received_at is None
                    else max(
                        0.0, now - self._roller_balance_limit_received_at
                    )
                ),
            }
        )

        status_msg = String()
        status_msg.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self._status_pub.publish(status_msg)

        abort = Bool()
        # Keep the full latch in safety_status for reset/readiness.  The Bool
        # output is the *actionable motion abort*: pre-tare NaN/staleness in a
        # geometry-only mode blocks compliance but does not prevent the arm
        # from reaching the verified pre-contact tare pose.
        abort.data = bool(
            status["abort_latched"]
            and motion_abort_required(
                status.get("reason", NONE), self._mode, self._force_enabled,
                noncontact_spray=(getattr(self, "_spray_process_mode", False) and
                    time.monotonic()-getattr(self, "_spray_process_time", 0.0) < 1.0)
            )
        )
        self._abort_pub.publish(abort)
        if abort.data:
            self._motion_abort_pub.publish(abort)

        reason = String()
        reason.data = str(status["reason"])
        self._reason_pub.publish(reason)
        self._overforce_pub.publish(reason)

        contact_confirmed = Bool()
        contact_confirmed.data = bool(status["contact_confirmed"])
        self._contact_confirmed_pub.publish(contact_confirmed)
        contact = String()
        contact.data = (
            "mode=%s contact=%s Fy=%.3fN contact_force=%.3fN tangential=%.3fN "
            "force_norm=%.3fN torque_norm=%.3fNm"
            % (
                self._mode,
                status["contact_confirmed"],
                self._safety.filtered[1],
                status["contact_force_n"],
                status["contact_tangential_force_n"],
                status["filtered_force_norm_n"],
                status["filtered_torque_norm_nm"],
            )
        )
        self._contact_pub.publish(contact)

        if (
            self._warn_force > 0.0
            and abs(self._safety.filtered[1]) >= self._warn_force
            and now - self._last_log >= self._log_period
        ):
            self.get_logger().warn(
                "TCP Fy %.3f N exceeds warning threshold %.3f N"
                % (self._safety.filtered[1], self._warn_force)
            )

        if status["reason"] != self._last_reason:
            self._last_reason = str(status["reason"])
            if self._last_reason != NONE:
                self.get_logger().error(
                    "painting safety latched: %s: %s"
                    % (self._last_reason, status["detail"])
                )
        if now - self._last_log >= self._log_period:
            self._last_log = now
            self.get_logger().info(contact.data)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PaintingForceMonitorNode()
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
