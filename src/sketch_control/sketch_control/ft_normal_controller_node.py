"""
ft_normal_controller_node - AFT200 force layer for roller contact.

This node does not command the robot directly. It turns raw 6-axis wrench data
into wall-normal force plus a small roller-balance rotation signal. MoveIt can
then use the status for contact/over-force/over-torque interlocks and consume
the published correction vectors while planning short contact-path chunks.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from sketch_control.rotation_utils import quat_apply


WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane"
WORK_AREA_REFINED_PLANE_TOPIC = "/perception/work_area_plane_refined"

STATUS_TOPIC = "/ft/status"
CONTACT_TOPIC = "/ft/contact"
NORMAL_FORCE_TOPIC = "/ft/normal_force"
CORRECTION_TOPIC = "/ft/admittance_correction"
ORIENTATION_CORRECTION_TOPIC = "/ft/orientation_correction"
ZERO_TOPIC = "/ft/zero"
TARGET_CONFIG_TOPIC = "/ft/target_config"
DEFAULT_TARGET_CONFIG_PATH = "~/sketch_robot_ws/aft200_force_threshold.json"
FORCE_BIAS_BASE = "base"
FORCE_BIAS_SENSOR = "sensor"
FORCE_BIAS_SENSOR_PLUS_GRAVITY = "sensor_plus_gravity"

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FTNormalControllerNode(Node):
    def __init__(self):
        super().__init__("ft_normal_controller_node")

        self.declare_parameter("wrench_topic", "/aft200/ft")
        self.declare_parameter("base_frame", "link0")
        self.declare_parameter("sensor_frame", "tcp")
        self.declare_parameter("auto_zero_samples", 50)
        self.declare_parameter("filter_alpha", 0.15)
        self.declare_parameter("force_bias_model", FORCE_BIAS_SENSOR_PLUS_GRAVITY)
        self.declare_parameter("max_zero_observations", 8)
        self.declare_parameter("max_gravity_force_n", 150.0)
        self.declare_parameter("gravity_axis_base", [0.0, 0.0, -1.0])
        self.declare_parameter("force_sign", 1.0)
        self.declare_parameter("contact_threshold_n", 3.0)
        self.declare_parameter("target_force_n", 10.0)
        self.declare_parameter("warn_force_n", 20.0)
        self.declare_parameter("abort_force_n", 5.0)
        self.declare_parameter("admittance_gain_m_per_n", 0.00025)
        self.declare_parameter("max_correction_m", 0.006)
        self.declare_parameter("lock_refined_surface", True)
        # TCP local +X is the roller long axis in the current EOAT model.
        # A one-sided roller contact creates torque around axis x normal.
        self.declare_parameter("roller_axis_sensor", [1.0, 0.0, 0.0])
        self.declare_parameter("torque_warn_nm", 0.12)
        self.declare_parameter("torque_abort_nm", 0.25)
        self.declare_parameter("torque_balance_deadband_nm", 0.02)
        self.declare_parameter("torque_balance_gain_rad_per_nm", 0.08)
        self.declare_parameter("torque_balance_sign", 1.0)
        self.declare_parameter("max_orientation_correction_rad", 0.03)
        self.declare_parameter("refined_surface_fresh_s", 2.0)
        self.declare_parameter("load_target_config", True)
        self.declare_parameter("save_target_config", True)
        self.declare_parameter("target_config_path", DEFAULT_TARGET_CONFIG_PATH)

        self.wrench_topic = str(self.get_parameter("wrench_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.sensor_frame = str(self.get_parameter("sensor_frame").value)
        self.auto_zero_samples = int(self.get_parameter("auto_zero_samples").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.force_bias_model = str(
            self.get_parameter("force_bias_model").value
        ).strip().lower()
        if self.force_bias_model not in (
            FORCE_BIAS_BASE,
            FORCE_BIAS_SENSOR,
            FORCE_BIAS_SENSOR_PLUS_GRAVITY,
        ):
            self.get_logger().warn(
                f"unknown force_bias_model={self.force_bias_model!r}; "
                f"fallback to {FORCE_BIAS_SENSOR_PLUS_GRAVITY!r}")
            self.force_bias_model = FORCE_BIAS_SENSOR_PLUS_GRAVITY
        self.max_zero_observations = max(
            1, int(self.get_parameter("max_zero_observations").value))
        self.max_gravity_force_n = max(
            0.0, float(self.get_parameter("max_gravity_force_n").value))
        self.gravity_axis_base = np.asarray(
            self.get_parameter("gravity_axis_base").value, dtype=float)
        if float(np.linalg.norm(self.gravity_axis_base)) < 1e-9:
            self.gravity_axis_base = np.array([0.0, 0.0, -1.0], dtype=float)
        self.gravity_axis_base = (
            self.gravity_axis_base
            / (np.linalg.norm(self.gravity_axis_base) + 1e-12)
        )
        self.force_sign = float(self.get_parameter("force_sign").value)
        self.contact_threshold_n = float(self.get_parameter("contact_threshold_n").value)
        self.target_force_n = float(self.get_parameter("target_force_n").value)
        self.warn_force_n = float(self.get_parameter("warn_force_n").value)
        self.abort_force_n = float(self.get_parameter("abort_force_n").value)
        self.admittance_gain_m_per_n = float(
            self.get_parameter("admittance_gain_m_per_n").value)
        self.max_correction_m = float(self.get_parameter("max_correction_m").value)
        self.lock_refined_surface = bool(
            self.get_parameter("lock_refined_surface").value)
        self.roller_axis_sensor = np.asarray(
            self.get_parameter("roller_axis_sensor").value, dtype=float)
        if float(np.linalg.norm(self.roller_axis_sensor)) < 1e-9:
            self.roller_axis_sensor = np.array([1.0, 0.0, 0.0], dtype=float)
        self.roller_axis_sensor = (
            self.roller_axis_sensor
            / (np.linalg.norm(self.roller_axis_sensor) + 1e-12)
        )
        self.torque_warn_nm = float(self.get_parameter("torque_warn_nm").value)
        self.torque_abort_nm = float(self.get_parameter("torque_abort_nm").value)
        self.torque_balance_deadband_nm = float(
            self.get_parameter("torque_balance_deadband_nm").value)
        self.torque_balance_gain_rad_per_nm = float(
            self.get_parameter("torque_balance_gain_rad_per_nm").value)
        self.torque_balance_sign = float(
            self.get_parameter("torque_balance_sign").value)
        self.max_orientation_correction_rad = float(
            self.get_parameter("max_orientation_correction_rad").value)
        self.refined_surface_fresh_s = float(
            self.get_parameter("refined_surface_fresh_s").value)
        self.load_target_config = bool(self.get_parameter("load_target_config").value)
        self.save_target_config = bool(self.get_parameter("save_target_config").value)
        self.target_config_path = str(self.get_parameter("target_config_path").value)
        if self.load_target_config:
            self._load_target_config()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.surface_point = None
        self.surface_normal = None
        self.surface_source = "none"
        self.surface_time = 0.0
        self.filtered_force_sensor = None
        self.filtered_torque_sensor = None
        self.filtered_force_base = None
        self.filtered_torque_base = None
        self.bias_force_sensor = np.zeros(3, dtype=float)
        self.bias_torque_sensor = np.zeros(3, dtype=float)
        self.bias_force_base = np.zeros(3, dtype=float)
        self.bias_torque_base = np.zeros(3, dtype=float)
        self.gravity_force_base = np.zeros(3, dtype=float)
        self.gravity_compensation_ready = False
        self.force_zero_observations = []
        self.zero_samples = []
        self.zero_torque_samples = []
        self.zero_rotation_samples = []
        self.bias_ready = self.auto_zero_samples <= 0
        self.last_wrench_time = 0.0

        self.create_subscription(
            WrenchStamped, self.wrench_topic, self._on_wrench, 10)
        self.create_subscription(
            PoseStamped, WORK_AREA_PLANE_TOPIC, self._on_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, WORK_AREA_REFINED_PLANE_TOPIC,
            self._on_refined_surface, LATCHED_QOS)
        self.create_subscription(Bool, ZERO_TOPIC, self._on_zero, 10)
        self.create_subscription(String, TARGET_CONFIG_TOPIC, self._on_target_config, 10)

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.contact_pub = self.create_publisher(Bool, CONTACT_TOPIC, 10)
        self.normal_force_pub = self.create_publisher(
            Float64, NORMAL_FORCE_TOPIC, 10)
        self.correction_pub = self.create_publisher(
            Vector3Stamped, CORRECTION_TOPIC, 10)
        self.orientation_correction_pub = self.create_publisher(
            Vector3Stamped, ORIENTATION_CORRECTION_TOPIC, 10)

        self.get_logger().info(
            "FT normal controller 시작\n"
            f"  wrench : {self.wrench_topic}\n"
            f"  surface: {WORK_AREA_REFINED_PLANE_TOPIC} 우선, "
            f"fallback={WORK_AREA_PLANE_TOPIC}, "
            f"lock_refined={self.lock_refined_surface}\n"
            f"  zero   : {ZERO_TOPIC} 또는 startup {self.auto_zero_samples} samples, "
            f"bias_model={self.force_bias_model}\n"
            f"  config : {TARGET_CONFIG_TOPIC}\n"
            f"  file   : {self.target_config_path if self.load_target_config else 'load disabled'} "
            f"/ {self.target_config_path if self.save_target_config else 'save disabled'}\n"
            f"  roller : axis(sensor)=({self.roller_axis_sensor[0]:+.2f},"
            f"{self.roller_axis_sensor[1]:+.2f},{self.roller_axis_sensor[2]:+.2f}), "
            f"torque warn={self.torque_warn_nm:.2f}Nm, "
            f"abort={self.torque_abort_nm:.2f}Nm, "
            f"gain={self.torque_balance_gain_rad_per_nm:.3f}rad/Nm, "
            f"max_rot={math.degrees(self.max_orientation_correction_rad):.2f}deg\n"
            f"  target : {self.target_force_n:.1f}N "
            f"(warn={self.warn_force_n:.1f}, abort={self.abort_force_n:.1f})"
        )

    def _on_zero(self, msg):
        if not msg.data:
            return
        self.bias_ready = False
        self.zero_samples = []
        self.zero_torque_samples = []
        self.zero_rotation_samples = []
        self.filtered_force_sensor = None
        self.filtered_torque_sensor = None
        self.filtered_force_base = None
        self.filtered_torque_base = None
        self.get_logger().warn(
            "[FT] zero requested. 센서를 접촉 없는 상태로 유지하세요.")

    def _load_target_config(self):
        path = Path(self.target_config_path).expanduser()
        if not path.exists():
            return
        try:
            payload = json.loads(path.read_text())
        except Exception as exc:
            self.get_logger().warn(
                f"[FT] target config load failed ({path}): {exc}")
            return

        numeric_fields = {
            "force_sign": (-1.0, 1.0),
            "max_zero_observations": (1.0, 100.0),
            "max_gravity_force_n": (0.0, 1000.0),
            "target_force_n": (0.0, 200.0),
            "contact_threshold_n": (0.0, 200.0),
            "warn_force_n": (0.0, 300.0),
            "abort_force_n": (0.0, 300.0),
            "admittance_gain_m_per_n": (0.0, 0.05),
            "max_correction_m": (0.0, 0.10),
            "torque_warn_nm": (0.0, 20.0),
            "torque_abort_nm": (0.0, 20.0),
            "torque_balance_deadband_nm": (0.0, 20.0),
            "torque_balance_gain_rad_per_nm": (0.0, 2.0),
            "torque_balance_sign": (-1.0, 1.0),
            "max_orientation_correction_rad": (0.0, 0.20),
        }
        changed = []
        for name, (lo, hi) in numeric_fields.items():
            if name not in payload:
                continue
            try:
                value = float(payload[name])
            except (TypeError, ValueError):
                continue
            if not math.isfinite(value) or value < lo or value > hi:
                continue
            if name == "force_sign":
                value = 1.0 if value >= 0.0 else -1.0
            if name == "max_zero_observations":
                value = int(value)
            setattr(self, name, value)
            changed.append(name)

        if "roller_axis_sensor" in payload:
            try:
                axis = np.asarray(payload["roller_axis_sensor"], dtype=float)
                if axis.shape[0] >= 3 and float(np.linalg.norm(axis[:3])) > 1e-9:
                    self.roller_axis_sensor = axis[:3] / np.linalg.norm(axis[:3])
                    changed.append("roller_axis_sensor")
            except Exception:
                pass
        if "gravity_axis_base" in payload:
            try:
                axis = np.asarray(payload["gravity_axis_base"], dtype=float)
                if axis.shape[0] >= 3 and float(np.linalg.norm(axis[:3])) > 1e-9:
                    self.gravity_axis_base = axis[:3] / np.linalg.norm(axis[:3])
                    changed.append("gravity_axis_base")
            except Exception:
                pass
        if "force_bias_model" in payload:
            model = str(payload["force_bias_model"]).strip().lower()
            if model in (
                FORCE_BIAS_BASE,
                FORCE_BIAS_SENSOR,
                FORCE_BIAS_SENSOR_PLUS_GRAVITY,
            ):
                self.force_bias_model = model
                changed.append("force_bias_model")

        if changed:
            self.get_logger().info(
                f"[FT] target config loaded <- {path} ({', '.join(changed)})")

    def _on_target_config(self, msg):
        try:
            payload = json.loads(msg.data or "{}")
        except json.JSONDecodeError as exc:
            self._publish_status(False, "config_parse_failed", detail=str(exc))
            return

        changed = []
        numeric_fields = {
            "target_force_n": (0.0, 200.0),
            "max_zero_observations": (1.0, 100.0),
            "max_gravity_force_n": (0.0, 1000.0),
            "contact_threshold_n": (0.0, 200.0),
            "warn_force_n": (0.0, 300.0),
            "abort_force_n": (0.0, 300.0),
            "admittance_gain_m_per_n": (0.0, 0.05),
            "max_correction_m": (0.0, 0.10),
            "torque_warn_nm": (0.0, 20.0),
            "torque_abort_nm": (0.0, 20.0),
            "torque_balance_deadband_nm": (0.0, 20.0),
            "torque_balance_gain_rad_per_nm": (0.0, 2.0),
            "torque_balance_sign": (-1.0, 1.0),
            "max_orientation_correction_rad": (0.0, 0.20),
        }
        for name, (lo, hi) in numeric_fields.items():
            if name not in payload:
                continue
            try:
                value = float(payload[name])
            except (TypeError, ValueError):
                self._publish_status(False, "config_invalid", field=name)
                return
            if not math.isfinite(value) or value < lo or value > hi:
                self._publish_status(
                    False, "config_out_of_range", field=name, value=value,
                    min=lo, max=hi)
                return
            if name == "max_zero_observations":
                value = int(value)
            setattr(self, name, value)
            changed.append(name)

        if "force_sign" in payload:
            try:
                value = float(payload["force_sign"])
            except (TypeError, ValueError):
                self._publish_status(False, "config_invalid", field="force_sign")
                return
            if not math.isfinite(value) or abs(value) < 1e-9:
                self._publish_status(
                    False, "config_out_of_range", field="force_sign", value=value)
                return
            self.force_sign = 1.0 if value > 0.0 else -1.0
            changed.append("force_sign")

        if "gravity_axis_base" in payload:
            try:
                axis = np.asarray(payload["gravity_axis_base"], dtype=float)
            except Exception:
                self._publish_status(
                    False, "config_invalid", field="gravity_axis_base")
                return
            if axis.shape[0] < 3 or float(np.linalg.norm(axis[:3])) < 1e-9:
                self._publish_status(
                    False, "config_out_of_range", field="gravity_axis_base")
                return
            self.gravity_axis_base = axis[:3] / np.linalg.norm(axis[:3])
            changed.append("gravity_axis_base")

        if "force_bias_model" in payload:
            model = str(payload["force_bias_model"]).strip().lower()
            if model not in (
                FORCE_BIAS_BASE,
                FORCE_BIAS_SENSOR,
                FORCE_BIAS_SENSOR_PLUS_GRAVITY,
            ):
                self._publish_status(
                    False, "config_out_of_range",
                    field="force_bias_model", value=model)
                return
            self.force_bias_model = model
            changed.append("force_bias_model")

        if not changed:
            self._publish_status(False, "config_empty")
            return

        self.get_logger().info(
            "[FT] target config updated: "
            f"force_sign={self.force_sign:+.0f}, "
            f"bias_model={self.force_bias_model}, "
            f"target={self.target_force_n:.2f}N, "
            f"contact={self.contact_threshold_n:.2f}N, "
            f"warn={self.warn_force_n:.2f}N, "
            f"abort={self.abort_force_n:.2f}N, "
            f"torque_warn={self.torque_warn_nm:.2f}Nm, "
            f"torque_abort={self.torque_abort_nm:.2f}Nm, "
            f"torque_gain={self.torque_balance_gain_rad_per_nm:.3f}rad/Nm, "
            f"max_rot={math.degrees(self.max_orientation_correction_rad):.2f}deg")
        self._save_target_config()
        self._publish_status(
            True,
            "config_updated",
            changed=changed,
            force_sign=self.force_sign,
            force_bias_model=self.force_bias_model,
            max_zero_observations=self.max_zero_observations,
            max_gravity_force_n=self.max_gravity_force_n,
            gravity_axis_base=self.gravity_axis_base.tolist(),
            target_force_n=self.target_force_n,
            contact_threshold_n=self.contact_threshold_n,
            warn_force_n=self.warn_force_n,
            abort_force_n=self.abort_force_n,
            admittance_gain_m_per_n=self.admittance_gain_m_per_n,
            max_correction_m=self.max_correction_m,
            torque_warn_nm=self.torque_warn_nm,
            torque_abort_nm=self.torque_abort_nm,
            torque_balance_deadband_nm=self.torque_balance_deadband_nm,
            torque_balance_gain_rad_per_nm=self.torque_balance_gain_rad_per_nm,
            torque_balance_sign=self.torque_balance_sign,
            max_orientation_correction_rad=self.max_orientation_correction_rad,
            surface_source=self.surface_source,
        )

    def _save_target_config(self):
        if not self.save_target_config:
            return
        payload = {
            "wrench_topic": self.wrench_topic,
            "force_basis": "surface_contact_axis",
            "force_bias_model": self.force_bias_model,
            "force_sign": float(self.force_sign),
            "max_zero_observations": int(self.max_zero_observations),
            "max_gravity_force_n": float(self.max_gravity_force_n),
            "gravity_axis_base": self.gravity_axis_base.tolist(),
            "target_force_n": float(self.target_force_n),
            "contact_threshold_n": float(self.contact_threshold_n),
            "warn_force_n": float(self.warn_force_n),
            "abort_force_n": float(self.abort_force_n),
            "admittance_gain_m_per_n": float(self.admittance_gain_m_per_n),
            "max_correction_m": float(self.max_correction_m),
            "roller_axis_sensor": self.roller_axis_sensor.tolist(),
            "torque_warn_nm": float(self.torque_warn_nm),
            "torque_abort_nm": float(self.torque_abort_nm),
            "torque_balance_deadband_nm": float(self.torque_balance_deadband_nm),
            "torque_balance_gain_rad_per_nm": float(
                self.torque_balance_gain_rad_per_nm),
            "torque_balance_sign": float(self.torque_balance_sign),
            "max_orientation_correction_rad": float(
                self.max_orientation_correction_rad),
            "surface_source": self.surface_source,
            "surface_locked": self._surface_locked(),
            "surface_normal_base": (
                None if self.surface_normal is None else self.surface_normal.tolist()
            ),
            "gravity_compensation_ready": bool(self.gravity_compensation_ready),
            "gravity_force_base_n": self.gravity_force_base.tolist(),
            "bias_force_sensor_n": self.bias_force_sensor.tolist(),
            "zero_observations": len(self.force_zero_observations),
            "updated_at_unix": time.time(),
            "source": self.get_name(),
        }
        path = Path(self.target_config_path).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2))
        except Exception as exc:
            self.get_logger().warn(
                f"[FT] target config save failed ({path}): {exc}")
            return
        self.get_logger().info(f"[FT] target config saved -> {path}")

    def _on_surface(self, msg):
        if self.surface_source == "d405_refined" and self.lock_refined_surface:
            self.get_logger().info(
                "[FT] ignore default work_area_plane: D405 refined surface locked",
                throttle_duration_sec=5.0)
            return
        if (
            self.surface_source == "d405_refined"
            and time.monotonic() - self.surface_time <= self.refined_surface_fresh_s
        ):
            return
        self._set_surface(msg, "zed")

    def _on_refined_surface(self, msg):
        if self.surface_source == "d405_refined" and self.lock_refined_surface:
            self.get_logger().info(
                "[FT] ignore new D405 refined plane: surface already locked",
                throttle_duration_sec=5.0)
            return
        self._set_surface(msg, "d405_refined")

    def _surface_locked(self):
        return bool(self.lock_refined_surface and self.surface_source == "d405_refined")

    def _set_surface(self, msg, source):
        frame = msg.header.frame_id or self.base_frame
        point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        q = msg.pose.orientation
        normal = quat_apply([q.x, q.y, q.z, q.w], [0.0, 0.0, 1.0])
        try:
            point, normal = self._transform_point_vector_to_base(point, normal, frame)
        except TransformException as exc:
            self.get_logger().warn(
                f"[FT] surface TF 실패 ({self.base_frame}<-{frame}): {exc}",
                throttle_duration_sec=2.0)
            return
        normal = np.asarray(normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        self.surface_point = point
        self.surface_normal = normal
        self.surface_source = source
        self.surface_time = time.monotonic()
        lock_text = " locked" if self._surface_locked() else ""
        self.get_logger().info(
            f"[FT] active surface={source}{lock_text}, normal=({normal[0]:+.2f},"
            f"{normal[1]:+.2f},{normal[2]:+.2f})",
            throttle_duration_sec=2.0)

    def _add_force_zero_observation(self, mean_force_sensor, r_base_sensor):
        obs = {
            "force_sensor": np.asarray(mean_force_sensor, dtype=float),
            "r_base_sensor": np.asarray(r_base_sensor, dtype=float),
            "time": time.monotonic(),
        }
        self.force_zero_observations.append(obs)
        keep = max(1, int(self.max_zero_observations))
        if len(self.force_zero_observations) > keep:
            self.force_zero_observations = self.force_zero_observations[-keep:]

    def _recompute_sensor_bias_and_gravity(self):
        """Estimate sensor-fixed force offset and optional base gravity vector.

        Model for each free-space zero observation:
            f_sensor = offset_sensor + R_sensor_base * (g_scalar * gravity_axis_base)

        The gravity direction is known in the robot base frame; the unknown is
        only its signed magnitude. One observation cannot separate the two
        terms, so it falls back to sensor-only offset. Two sufficiently
        different orientations make the least-squares system full-rank.
        """
        if self.force_bias_model == FORCE_BIAS_SENSOR:
            latest = self.force_zero_observations[-1]["force_sensor"]
            self.bias_force_sensor = latest.copy()
            self.gravity_force_base = np.zeros(3, dtype=float)
            self.gravity_compensation_ready = False
            return

        observations = list(self.force_zero_observations)
        if len(observations) < 2:
            latest = observations[-1]["force_sensor"]
            self.bias_force_sensor = latest.copy()
            self.gravity_force_base = np.zeros(3, dtype=float)
            self.gravity_compensation_ready = False
            return

        a_blocks = []
        b_blocks = []
        for obs in observations:
            r_base_sensor = np.asarray(obs["r_base_sensor"], dtype=float)
            r_sensor_base = r_base_sensor.T
            gravity_col = r_sensor_base @ self.gravity_axis_base
            a_blocks.append(np.column_stack([np.eye(3), gravity_col]))
            b_blocks.append(np.asarray(obs["force_sensor"], dtype=float))
        a = np.vstack(a_blocks)
        b = np.concatenate(b_blocks)
        rank = int(np.linalg.matrix_rank(a, tol=1e-6))
        if rank < 4:
            latest = observations[-1]["force_sensor"]
            self.bias_force_sensor = latest.copy()
            self.gravity_force_base = np.zeros(3, dtype=float)
            self.gravity_compensation_ready = False
            self.get_logger().warn(
                f"[FT] gravity calibration rank={rank}/4 -> "
                "sensor-frame offset only (zero 자세 변화 부족)",
                throttle_duration_sec=2.0)
            return

        x, residuals, _rank, _singular = np.linalg.lstsq(a, b, rcond=None)
        offset = x[:3]
        gravity_scalar = float(x[3])
        gravity = self.gravity_axis_base * gravity_scalar
        gravity_norm = abs(gravity_scalar)
        if self.max_gravity_force_n > 0.0 and gravity_norm > self.max_gravity_force_n:
            latest = observations[-1]["force_sensor"]
            self.bias_force_sensor = latest.copy()
            self.gravity_force_base = np.zeros(3, dtype=float)
            self.gravity_compensation_ready = False
            self.get_logger().warn(
                f"[FT] gravity estimate rejected: |G|={gravity_norm:.1f}N "
                f"> {self.max_gravity_force_n:.1f}N")
            return

        self.bias_force_sensor = offset
        self.gravity_force_base = gravity
        self.gravity_compensation_ready = True
        rmse = 0.0
        if a.shape[0] > 0:
            err = a @ x - b
            rmse = float(math.sqrt(float(np.mean(err * err))))
        self.get_logger().info(
            "[FT] gravity calibration OK: "
            f"obs={len(observations)}, |G|={gravity_norm:.1f}N "
            f"(mass≈{gravity_norm / 9.80665:.2f}kg), "
            f"G_base=({gravity[0]:+.1f},{gravity[1]:+.1f},{gravity[2]:+.1f})N, "
            f"rmse={rmse:.2f}N")

    def _log_bias_update(self):
        if self.force_bias_model == FORCE_BIAS_BASE:
            self.get_logger().info(
                f"[FT] base bias set: ({self.bias_force_base[0]:+.2f},"
                f"{self.bias_force_base[1]:+.2f},{self.bias_force_base[2]:+.2f})N, "
                f"torque=({self.bias_torque_base[0]:+.3f},"
                f"{self.bias_torque_base[1]:+.3f},"
                f"{self.bias_torque_base[2]:+.3f})Nm")
            return

        g = self.gravity_force_base
        self.get_logger().info(
            f"[FT] sensor bias set: offset_sensor=("
            f"{self.bias_force_sensor[0]:+.2f},"
            f"{self.bias_force_sensor[1]:+.2f},"
            f"{self.bias_force_sensor[2]:+.2f})N, "
            f"gravity_ready={self.gravity_compensation_ready}, "
            f"G_base=({g[0]:+.2f},{g[1]:+.2f},{g[2]:+.2f})N, "
            f"zero_obs={len(self.force_zero_observations)}, "
            f"torque_offset_sensor=({self.bias_torque_sensor[0]:+.3f},"
            f"{self.bias_torque_sensor[1]:+.3f},"
            f"{self.bias_torque_sensor[2]:+.3f})Nm")

    def _on_wrench(self, msg):
        frame = msg.header.frame_id or self.sensor_frame
        f_sensor = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ], dtype=float)
        t_sensor = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float)
        try:
            f_sensor = self._transform_vector(f_sensor, self.sensor_frame, frame)
            t_sensor = self._transform_vector(t_sensor, self.sensor_frame, frame)
            r_base_sensor = self._rotation_matrix_to_base(self.sensor_frame)
            force_base_raw = r_base_sensor @ f_sensor
            torque_base_raw = r_base_sensor @ t_sensor
            roller_axis_base = self._transform_vector_to_base(
                self.roller_axis_sensor, self.sensor_frame)
        except TransformException as exc:
            self._publish_status(False, "wrench_tf_missing", detail=str(exc))
            return

        if self.filtered_force_sensor is None:
            self.filtered_force_sensor = f_sensor
        else:
            a = float(np.clip(self.filter_alpha, 0.0, 1.0))
            self.filtered_force_sensor = (
                (1.0 - a) * self.filtered_force_sensor + a * f_sensor
            )
        if self.filtered_torque_sensor is None:
            self.filtered_torque_sensor = t_sensor
        else:
            a = float(np.clip(self.filter_alpha, 0.0, 1.0))
            self.filtered_torque_sensor = (
                (1.0 - a) * self.filtered_torque_sensor + a * t_sensor
            )

        if self.filtered_force_base is None:
            self.filtered_force_base = force_base_raw
        else:
            a = float(np.clip(self.filter_alpha, 0.0, 1.0))
            self.filtered_force_base = (
                (1.0 - a) * self.filtered_force_base + a * force_base_raw
            )
        if self.filtered_torque_base is None:
            self.filtered_torque_base = torque_base_raw
        else:
            a = float(np.clip(self.filter_alpha, 0.0, 1.0))
            self.filtered_torque_base = (
                (1.0 - a) * self.filtered_torque_base + a * torque_base_raw
            )

        self.last_wrench_time = time.monotonic()
        if not self.bias_ready:
            if self.force_bias_model == FORCE_BIAS_BASE:
                self.zero_samples.append(self.filtered_force_base.copy())
                self.zero_torque_samples.append(self.filtered_torque_base.copy())
            else:
                self.zero_samples.append(self.filtered_force_sensor.copy())
                self.zero_torque_samples.append(self.filtered_torque_sensor.copy())
                self.zero_rotation_samples.append(r_base_sensor.copy())
            need = max(self.auto_zero_samples, 1)
            if len(self.zero_samples) < need:
                self._publish_status(
                    False, "zeroing", samples=len(self.zero_samples), need=need)
                return
            mean_force = np.mean(np.asarray(self.zero_samples), axis=0)
            mean_torque = np.mean(np.asarray(self.zero_torque_samples), axis=0)
            if self.force_bias_model == FORCE_BIAS_BASE:
                self.bias_force_base = mean_force
                self.bias_torque_base = mean_torque
                self.gravity_force_base = np.zeros(3, dtype=float)
                self.gravity_compensation_ready = False
            else:
                r_zero = (
                    self.zero_rotation_samples[-1]
                    if self.zero_rotation_samples
                    else r_base_sensor
                )
                self._add_force_zero_observation(mean_force, r_zero)
                self._recompute_sensor_bias_and_gravity()
                self.bias_torque_sensor = mean_torque
                self.bias_torque_base = r_base_sensor @ self.bias_torque_sensor
            self.bias_ready = True
            self.zero_samples = []
            self.zero_torque_samples = []
            self.zero_rotation_samples = []
            self._log_bias_update()

        if self.surface_normal is None:
            self._publish_status(False, "waiting_for_surface")
            return

        if self.force_bias_model == FORCE_BIAS_BASE:
            force_base = self.filtered_force_base - self.bias_force_base
            torque_base = self.filtered_torque_base - self.bias_torque_base
            raw_force_sensor = f_sensor
            corrected_force_sensor = r_base_sensor.T @ force_base
            bias_force_base_for_status = self.bias_force_base
        else:
            corrected_force_sensor = self.filtered_force_sensor - self.bias_force_sensor
            corrected_torque_sensor = (
                self.filtered_torque_sensor - self.bias_torque_sensor)
            force_base = r_base_sensor @ corrected_force_sensor
            if self.gravity_compensation_ready:
                force_base = force_base - self.gravity_force_base
            torque_base = r_base_sensor @ corrected_torque_sensor
            raw_force_sensor = self.filtered_force_sensor
            bias_force_base_for_status = r_base_sensor @ self.bias_force_sensor

        force = self.force_sign * force_base
        torque = self.force_sign * torque_base
        contact_axis = -np.asarray(self.surface_normal, dtype=float)
        contact_axis = contact_axis / (np.linalg.norm(contact_axis) + 1e-12)
        normal_force = float(np.dot(force, contact_axis))
        force_error = float(self.target_force_n - normal_force)
        contact = normal_force >= self.contact_threshold_n

        roller_axis_tangent, balance_axis = self._roller_balance_axes(
            roller_axis_base)
        torque_balance_nm = float(np.dot(torque, balance_axis))
        torque_balance_abs_nm = abs(torque_balance_nm)
        torque_about_roller_nm = float(np.dot(torque, roller_axis_tangent))
        torque_about_normal_nm = float(np.dot(torque, self.surface_normal))
        orientation_correction_rad = (
            self._torque_orientation_correction(torque_balance_nm)
            if contact else 0.0
        )
        orientation_correction = balance_axis * orientation_correction_rad
        torque_abort = torque_balance_abs_nm >= self.torque_abort_nm

        correction_along_contact = self.admittance_gain_m_per_n * force_error
        correction_along_contact = float(np.clip(
            correction_along_contact,
            -self.max_correction_m,
            self.max_correction_m,
        ))
        correction = contact_axis * correction_along_contact

        self._publish_numeric(
            normal_force, contact, correction, orientation_correction)
        self._publish_status(
            True,
            "torque_abort" if torque_abort else "ok",
            normal_force_n=normal_force,
            contact=contact,
            target_force_n=self.target_force_n,
            contact_threshold_n=self.contact_threshold_n,
            warn_force_n=self.warn_force_n,
            abort_force_n=self.abort_force_n,
            force_sign=self.force_sign,
            force_bias_model=self.force_bias_model,
            surface_source=self.surface_source,
            surface_locked=self._surface_locked(),
            surface_normal_base=self.surface_normal.tolist(),
            contact_axis_base=contact_axis.tolist(),
            force_base_n=force.tolist(),
            force_base_compensated_n=force_base.tolist(),
            raw_force_sensor_n=raw_force_sensor.tolist(),
            corrected_force_sensor_n=corrected_force_sensor.tolist(),
            bias_force_sensor_n=self.bias_force_sensor.tolist(),
            bias_force_base_n=bias_force_base_for_status.tolist(),
            gravity_force_base_n=self.gravity_force_base.tolist(),
            gravity_axis_base=self.gravity_axis_base.tolist(),
            gravity_compensation_ready=self.gravity_compensation_ready,
            zero_observations=len(self.force_zero_observations),
            force_error_n=force_error,
            torque_base_nm=torque.tolist(),
            bias_torque_sensor_nm=self.bias_torque_sensor.tolist(),
            roller_axis_base=roller_axis_tangent.tolist(),
            torque_balance_axis_base=balance_axis.tolist(),
            torque_balance_nm=torque_balance_nm,
            torque_balance_abs_nm=torque_balance_abs_nm,
            torque_about_roller_nm=torque_about_roller_nm,
            torque_about_normal_nm=torque_about_normal_nm,
            torque_warn_nm=self.torque_warn_nm,
            torque_abort_nm=self.torque_abort_nm,
            torque_abort=torque_abort,
            torque_balance_deadband_nm=self.torque_balance_deadband_nm,
            torque_balance_gain_rad_per_nm=self.torque_balance_gain_rad_per_nm,
            torque_balance_sign=self.torque_balance_sign,
            max_orientation_correction_rad=self.max_orientation_correction_rad,
            orientation_correction_rad=orientation_correction_rad,
            orientation_correction_xyz=orientation_correction.tolist(),
            correction_m=correction_along_contact,
            correction_xyz=correction.tolist(),
            bias_ready=self.bias_ready,
        )

        if normal_force >= self.abort_force_n:
            self.get_logger().error(
                f"[FT] OVER FORCE {normal_force:.1f}N >= {self.abort_force_n:.1f}N",
                throttle_duration_sec=0.5)
        elif normal_force >= self.warn_force_n:
            self.get_logger().warn(
                f"[FT] high force {normal_force:.1f}N >= {self.warn_force_n:.1f}N",
                throttle_duration_sec=0.5)
        if torque_abort:
            self.get_logger().error(
                f"[FT] OVER TORQUE balance {torque_balance_abs_nm:.3f}Nm >= "
                f"{self.torque_abort_nm:.3f}Nm",
                throttle_duration_sec=0.5)
        elif torque_balance_abs_nm >= self.torque_warn_nm:
            self.get_logger().warn(
                f"[FT] high balance torque {torque_balance_abs_nm:.3f}Nm >= "
                f"{self.torque_warn_nm:.3f}Nm",
                throttle_duration_sec=0.5)

    def _roller_balance_axes(self, roller_axis_base):
        roller_axis = np.asarray(roller_axis_base, dtype=float)
        roller_axis /= np.linalg.norm(roller_axis) + 1e-12
        roller_axis -= self.surface_normal * float(
            np.dot(roller_axis, self.surface_normal))
        if float(np.linalg.norm(roller_axis)) < 1e-8:
            if abs(float(np.dot(self.surface_normal, [1.0, 0.0, 0.0]))) < 0.9:
                roller_axis = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                roller_axis = np.array([0.0, 1.0, 0.0], dtype=float)
            roller_axis -= self.surface_normal * float(
                np.dot(roller_axis, self.surface_normal))
        roller_axis /= np.linalg.norm(roller_axis) + 1e-12

        balance_axis = np.cross(roller_axis, self.surface_normal)
        if float(np.linalg.norm(balance_axis)) < 1e-8:
            balance_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        balance_axis /= np.linalg.norm(balance_axis) + 1e-12
        return roller_axis, balance_axis

    def _torque_orientation_correction(self, torque_balance_nm):
        value = float(torque_balance_nm)
        deadband = max(0.0, float(self.torque_balance_deadband_nm))
        if abs(value) <= deadband:
            return 0.0
        value -= math.copysign(deadband, value)
        angle = (
            -float(self.torque_balance_sign)
            * float(self.torque_balance_gain_rad_per_nm)
            * value
        )
        limit = max(0.0, float(self.max_orientation_correction_rad))
        return float(np.clip(angle, -limit, limit))

    def _publish_numeric(
        self, normal_force, contact, correction, orientation_correction
    ):
        force_msg = Float64()
        force_msg.data = float(normal_force)
        self.normal_force_pub.publish(force_msg)

        contact_msg = Bool()
        contact_msg.data = bool(contact)
        self.contact_pub.publish(contact_msg)

        corr_msg = Vector3Stamped()
        corr_msg.header.stamp = self.get_clock().now().to_msg()
        corr_msg.header.frame_id = self.base_frame
        corr_msg.vector.x = float(correction[0])
        corr_msg.vector.y = float(correction[1])
        corr_msg.vector.z = float(correction[2])
        self.correction_pub.publish(corr_msg)

        rot_msg = Vector3Stamped()
        rot_msg.header.stamp = corr_msg.header.stamp
        rot_msg.header.frame_id = self.base_frame
        rot_msg.vector.x = float(orientation_correction[0])
        rot_msg.vector.y = float(orientation_correction[1])
        rot_msg.vector.z = float(orientation_correction[2])
        self.orientation_correction_pub.publish(rot_msg)

    def _publish_status(self, ok, state, **fields):
        payload = {
            "ok": bool(ok),
            "state": state,
            "bias_ready": bool(self.bias_ready),
            "force_sign": float(self.force_sign),
            "force_bias_model": self.force_bias_model,
            "target_force_n": float(self.target_force_n),
            "contact_threshold_n": float(self.contact_threshold_n),
            "warn_force_n": float(self.warn_force_n),
            "abort_force_n": float(self.abort_force_n),
            "torque_warn_nm": float(self.torque_warn_nm),
            "torque_abort_nm": float(self.torque_abort_nm),
            "torque_balance_deadband_nm": float(self.torque_balance_deadband_nm),
            "torque_balance_gain_rad_per_nm": float(
                self.torque_balance_gain_rad_per_nm),
            "torque_balance_sign": float(self.torque_balance_sign),
            "max_orientation_correction_rad": float(
                self.max_orientation_correction_rad),
            "surface_source": self.surface_source,
            "surface_locked": self._surface_locked(),
            "gravity_axis_base": self.gravity_axis_base.tolist(),
        }
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _transform_point_vector_to_base(self, point, vector, frame):
        if frame in (self.base_frame, "world", "World"):
            return point, vector
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, frame, Time(), timeout=Duration(seconds=0.05))
        t = tf.transform.translation
        q = tf.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        p_base = quat_apply(q_tf, point) + np.array([t.x, t.y, t.z], dtype=float)
        v_base = quat_apply(q_tf, vector)
        return p_base, v_base

    def _transform_vector_to_base(self, vector, frame):
        if frame in (self.base_frame, "world", "World"):
            return vector
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, frame, Time(), timeout=Duration(seconds=0.05))
        q = tf.transform.rotation
        return quat_apply([q.x, q.y, q.z, q.w], vector)

    def _transform_vector(self, vector, target_frame, source_frame):
        if target_frame == source_frame:
            return np.asarray(vector, dtype=float)
        if (
            target_frame in (self.base_frame, "world", "World")
            and source_frame in (self.base_frame, "world", "World")
        ):
            return np.asarray(vector, dtype=float)
        tf = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time(), timeout=Duration(seconds=0.05))
        q = tf.transform.rotation
        return np.asarray(
            quat_apply([q.x, q.y, q.z, q.w], vector), dtype=float)

    def _rotation_matrix_to_base(self, frame):
        if frame in (self.base_frame, "world", "World"):
            return np.eye(3, dtype=float)
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, frame, Time(), timeout=Duration(seconds=0.05))
        q = tf.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        return np.column_stack([
            quat_apply(quat, [1.0, 0.0, 0.0]),
            quat_apply(quat, [0.0, 1.0, 0.0]),
            quat_apply(quat, [0.0, 0.0, 1.0]),
        ]).astype(float)


def main(args=None):
    rclpy.init(args=args)
    node = FTNormalControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
