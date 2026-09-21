"""Single-capture D405 refinement for the active painting work area.

ZED supplies the global target and initial work-area geometry.  A capture
trigger arms exactly one subsequently received D405 ``PointCloud2``.  That one
cloud is fitted and accepted or rejected; a rejected attempt needs a new
trigger, while the first accepted result is locked until an explicit lifecycle
invalidation event.
"""

import json
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from sketch_control.plane_lifecycle import (
    PlaneLifecycle,
    SinglePlaneFitConfig,
    calibration_file_sha256,
    canonical_target_id,
    canonical_work_area_id,
    capture_once_and_fit_plane,
    validate_single_plane_result,
)
from sketch_control.rotation_utils import quat_apply, quat_from_matrix


TARGET_SURFACE_TOPIC = "/perception/target_surface"
TARGET_REFINED_TOPIC = "/perception/target_surface_refined"
WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane"
WORK_AREA_CORNERS_TOPIC = "/perception/work_area_corners"
REFINED_PLANE_TOPIC = "/perception/work_area_plane_refined"
STATUS_TOPIC = "/perception/d405_surface_refinement_status"
WORK_AREA_STATE_TOPIC = "/painting_system/work_area_state"
PAINTING_MODE_TOPIC = "/painting_admittance/mode"
REFINE_TARGET_TOPIC = "/refine_target_surface"
REFINE_WORK_AREA_TOPIC = "/refine_work_area"
TARGET_REFINE_STATUS_TOPIC = "/target_refine_status"
CAPTURE_TOPIC = "/d405/refine_capture"
TARGET_CAPTURE_TOPIC = "/d405/refine_target_capture"

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass(frozen=True)
class _PendingCloudCapture:
    """The single cloud already consumed by one capture trigger.

    Keeping the cloud and its trigger-time reference together prevents a later
    D405 cloud or plane update from changing the meaning of this attempt while
    tf2 catches up to the cloud timestamp.
    """

    mode: str
    cloud: object
    reference_plane: object
    target_frame: str
    source_frame: str
    plane_generation_id: str
    target_id: str
    target_stamp_sec: int
    target_stamp_nanosec: int
    deadline_monotonic: float
    queued_monotonic: float
    metadata: dict


def _pose_normal(pose):
    q = pose.orientation
    normal = quat_apply([q.x, q.y, q.z, q.w], [0.0, 0.0, 1.0])
    normal = np.asarray(normal, dtype=float)
    return normal / (np.linalg.norm(normal) + 1e-12)


def _normal_to_quaternion(normal):
    z_axis = np.asarray(normal, dtype=float)
    z_axis /= np.linalg.norm(z_axis) + 1e-12
    seed = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(seed, z_axis))) > 0.95:
        seed = np.array([1.0, 0.0, 0.0], dtype=float)
    x_axis = np.cross(seed, z_axis)
    x_axis /= np.linalg.norm(x_axis) + 1e-12
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis) + 1e-12
    return quat_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


class D405SurfaceRefinerNode(Node):
    def __init__(self):
        super().__init__("d405_surface_refiner_node")

        self.declare_parameter("cloud_topic", "/d405/d405/depth/color/points")
        self.declare_parameter("max_points", 120000)
        self.declare_parameter("voxel_size_m", 0.003)
        self.declare_parameter("ransac_iterations", 180)
        self.declare_parameter("roi_margin_m", 0.08)
        self.declare_parameter("roi_plane_window_m", 0.12)
        self.declare_parameter("target_roi_half_width_m", 0.35)
        self.declare_parameter("target_roi_half_height_m", 0.35)

        # Required real-profile quality controls.
        self.declare_parameter("min_roi_points", 150)
        self.declare_parameter("min_inliers", 100)
        self.declare_parameter("min_inlier_ratio", 0.65)
        self.declare_parameter("ransac_distance_threshold_m", 0.004)
        self.declare_parameter("max_rms_residual_m", 0.004)
        self.declare_parameter("max_residual_m", 0.012)
        self.defer_target_capture_until_arrival = bool(self.declare_parameter("defer_target_capture_until_arrival", True).value)
        self.declare_parameter("max_normal_delta_deg", 12.0)
        self.declare_parameter("max_plane_shift_m", 0.08)
        self.declare_parameter("max_tf_age_s", 0.25)
        self.declare_parameter("support_band_half_width_m", 0.006)
        self.declare_parameter("min_broad_inlier_ratio", 0.15)
        self.declare_parameter("min_support_points", 100)
        self.declare_parameter("support_span_quantile", 0.02)
        self.declare_parameter("min_support_span_m", 0.15)
        self.declare_parameter("max_target_support_offset_m", 0.32)
        self.declare_parameter("min_secondary_plane_inliers", 100)
        self.declare_parameter(
            "max_secondary_plane_relative_inliers", 0.60
        )
        self.declare_parameter("min_secondary_plane_separation_m", 0.020)
        self.declare_parameter(
            "min_secondary_plane_normal_delta_deg", 5.0
        )
        self.declare_parameter("capture_timeout_s", 1.2)
        self.declare_parameter(
            "calibration_file",
            "~/sketch_robot_ws/d405_eyeinhand_charuco_calibration.json",
        )
        self.declare_parameter("calibration_hash_check_period_s", 1.0)

        # Legacy launch arguments remain declared, but the forbidden fusion and
        # continuous-update modes are intentionally never used.
        self.declare_parameter("require_capture_trigger", True)
        self.declare_parameter("lock_after_refinement", True)
        self.declare_parameter("stable_samples", 1)
        self.declare_parameter("stable_shift_std_m", 0.006)
        self.declare_parameter("stable_normal_spread_deg", 2.0)
        self.declare_parameter("spatial_samples", 1)
        self.declare_parameter("max_spatial_samples", 1)
        self.declare_parameter("spatial_sample_separation_m", 0.045)
        self.declare_parameter("max_fit_residual_m", 0.012)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.roi_margin_m = float(self.get_parameter("roi_margin_m").value)
        self.roi_plane_window_m = float(
            self.get_parameter("roi_plane_window_m").value
        )
        self.target_roi_half_width_m = float(
            self.get_parameter("target_roi_half_width_m").value
        )
        self.target_roi_half_height_m = float(
            self.get_parameter("target_roi_half_height_m").value
        )
        self.capture_timeout_s = max(
            0.0, float(self.get_parameter("capture_timeout_s").value)
        )
        self.fit_config = SinglePlaneFitConfig(
            voxel_size_m=float(self.get_parameter("voxel_size_m").value),
            ransac_distance_threshold_m=float(
                self.get_parameter("ransac_distance_threshold_m").value
            ),
            ransac_iterations=int(
                self.get_parameter("ransac_iterations").value
            ),
            min_roi_points=int(self.get_parameter("min_roi_points").value),
            min_inliers=int(self.get_parameter("min_inliers").value),
            min_inlier_ratio=float(
                self.get_parameter("min_inlier_ratio").value
            ),
            max_rms_residual_m=float(
                self.get_parameter("max_rms_residual_m").value
            ),
            max_residual_m=float(
                self.get_parameter("max_residual_m").value
            ),
            max_normal_delta_deg=float(
                self.get_parameter("max_normal_delta_deg").value
            ),
            max_plane_shift_m=float(
                self.get_parameter("max_plane_shift_m").value
            ),
            max_tf_age_s=float(self.get_parameter("max_tf_age_s").value),
            support_band_half_width_m=float(
                self.get_parameter("support_band_half_width_m").value
            ),
            min_broad_inlier_ratio=float(
                self.get_parameter("min_broad_inlier_ratio").value
            ),
            min_support_points=int(
                self.get_parameter("min_support_points").value
            ),
            support_span_quantile=float(
                self.get_parameter("support_span_quantile").value
            ),
            min_support_span_m=float(
                self.get_parameter("min_support_span_m").value
            ),
            max_target_support_offset_m=float(
                self.get_parameter("max_target_support_offset_m").value
            ),
            min_secondary_plane_inliers=int(
                self.get_parameter("min_secondary_plane_inliers").value
            ),
            max_secondary_plane_relative_inliers=float(
                self.get_parameter(
                    "max_secondary_plane_relative_inliers"
                ).value
            ),
            min_secondary_plane_separation_m=float(
                self.get_parameter("min_secondary_plane_separation_m").value
            ),
            min_secondary_plane_normal_delta_deg=float(
                self.get_parameter(
                    "min_secondary_plane_normal_delta_deg"
                ).value
            ),
        )
        self.calibration_file = str(
            self.get_parameter("calibration_file").value
        )
        calibration_period = max(
            0.1,
            float(
                self.get_parameter("calibration_hash_check_period_s").value
            ),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_target_surface = None
        self.latest_work_area_plane = None
        self.latest_corners = None
        self._target_signature = None
        self._target_id = ""
        self._target_stamp = {"sec": 0, "nanosec": 0}
        self._corner_signature = None
        self._expected_refined_corner_signature = None
        self._external_work_area_state_seen = False
        self._external_work_area_state_key = None
        self._external_work_area_id = ""
        self._active_capture_mode = None
        self._active_capture_reference = None
        self._pending_cloud_capture = None
        self._paint_active = False
        self._lifecycles = {
            "work_area": PlaneLifecycle("work_area"),
            "target": PlaneLifecycle("target"),
        }
        try:
            self._calibration_hash = calibration_file_sha256(
                self.calibration_file
            )
        except Exception as exc:
            self._calibration_hash = f"error:{type(exc).__name__}"

        self.create_subscription(
            PoseStamped,
            TARGET_SURFACE_TOPIC,
            self._on_target_surface,
            LATCHED_QOS,
        )
        self.create_subscription(
            PoseStamped,
            WORK_AREA_PLANE_TOPIC,
            self._on_work_area_plane,
            LATCHED_QOS,
        )
        self.create_subscription(
            PoseArray,
            WORK_AREA_CORNERS_TOPIC,
            self._on_work_area_corners,
            LATCHED_QOS,
        )
        self.create_subscription(
            String,
            WORK_AREA_STATE_TOPIC,
            self._on_work_area_state,
            LATCHED_QOS,
        )
        self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, CAPTURE_TOPIC, self._on_capture, 10)
        self.create_subscription(
            Bool, TARGET_CAPTURE_TOPIC, self._on_target_capture, 10
        )
        self.create_subscription(
            Bool, REFINE_TARGET_TOPIC, self._on_refine_target, 10
        )
        self.create_subscription(
            Bool, REFINE_WORK_AREA_TOPIC, self._on_refine_work_area, 10
        )
        self.create_subscription(
            String, PAINTING_MODE_TOPIC, self._on_painting_mode, 10
        )

        self.target_record_pub = self.create_publisher(String, "/perception/refined_target_record", 10)
        self.target_refined_pub = self.create_publisher(
            PoseStamped, TARGET_REFINED_TOPIC, LATCHED_QOS
        )
        self.refined_pub = self.create_publisher(
            PoseStamped, REFINED_PLANE_TOPIC, LATCHED_QOS
        )
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, LATCHED_QOS)
        self.target_status_pub = self.create_publisher(
            String, TARGET_REFINE_STATUS_TOPIC, LATCHED_QOS
        )
        self.create_timer(0.05, self._check_capture_timeout)
        self.create_timer(calibration_period, self._check_calibration_hash)

        if not bool(self.get_parameter("require_capture_trigger").value):
            self.get_logger().warn(
                "require_capture_trigger=false ignored: single-capture policy "
                "always requires an explicit trigger"
            )
        if not bool(self.get_parameter("lock_after_refinement").value):
            self.get_logger().warn(
                "lock_after_refinement=false ignored: first accepted capture "
                "is always locked"
            )
        if int(self.get_parameter("spatial_samples").value) != 1:
            self.get_logger().warn(
                "spatial_samples is deprecated and ignored; candidate poses are "
                "retry locations, never fusion samples"
            )

        self.get_logger().info(
            "D405 single-capture surface refiner started\n"
            f"  cloud   : {self.cloud_topic}\n"
            f"  capture : {CAPTURE_TOPIC} / {TARGET_CAPTURE_TOPIC}\n"
            f"  state   : {WORK_AREA_STATE_TOPIC} (explicit ID preferred)\n"
            f"  refined : {TARGET_REFINED_TOPIC}, {REFINED_PLANE_TOPIC}"
        )

    # ---- lifecycle inputs -------------------------------------------------
    def _on_target_surface(self, msg):
        target_id = canonical_target_id(
            msg.header.frame_id or "zed_left_camera_frame",
            (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        )
        target_event_key = (
            target_id,
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        if not self._stamp_is_valid(msg.header.stamp):
            # A non-zero stamp is the browser request identity.  Accepting an
            # unidentifiable target would let a latched result satisfy a newer
            # Set Target request.
            self._publish_target_status(
                "failed",
                ok=False,
                mode="target",
                accepted=False,
                rejection_reason="target_stamp_invalid",
                target_id=target_id,
                target_stamp={
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
            )
            return
        if target_event_key == self._target_signature:
            return
        self.latest_target_surface = msg
        self._target_signature = target_event_key
        self._target_id = target_id
        self._target_stamp = {
            "sec": int(msg.header.stamp.sec),
            "nanosec": int(msg.header.stamp.nanosec),
        }
        target_invalidated = self._invalidate_lifecycle(
            "target", "target_surface_changed", token=target_event_key
        )
        self._invalidate_lifecycle(
            "work_area",
            "target_surface_changed",
            work_area_id=self._external_work_area_id or None,
            token=target_event_key,
        )
        # Receiving a new, explicit ZED target selection is the trigger for
        # exactly one D405 capture.  Arming here (after storing the exact
        # PoseStamped) avoids cross-topic ordering races between
        # /target_selection_pixels and the legacy /refine_target_surface Bool.
        # A rejected capture is still terminal; either a new target selection
        # or an explicit retry is required to arm another cloud.
        if target_invalidated and not getattr(self, "defer_target_capture_until_arrival", False):
            self._start_capture("target")

    def _on_work_area_plane(self, msg):
        self.latest_work_area_plane = msg

    def _on_work_area_corners(self, msg):
        if len(msg.poses) < 4:
            return
        points = self._corner_points(msg)
        frame_id = msg.header.frame_id or "zed_left_camera_frame"
        if not np.all(np.isfinite(points)):
            self._publish_status(
                False,
                "work_area_corners_rejected",
                mode="work_area",
                rejection_reason="nonfinite_work_area_corners",
            )
            return
        self.latest_corners = msg
        signature = self._corner_signature_from_points(points, frame_id)
        if signature == self._corner_signature:
            return

        # wall_projector projects the same semantic work area onto the accepted
        # D405 plane once.  Do not mistake that feedback for a new selection.
        if (
            self._expected_refined_corner_signature is not None
            and signature == self._expected_refined_corner_signature
        ):
            self._corner_signature = signature
            self._expected_refined_corner_signature = None
            self.get_logger().info(
                "[D405 REFINE] accepted-plane corner correction observed; "
                "work_area_id unchanged"
            )
            return

        self._corner_signature = signature
        if self._external_work_area_state_seen:
            # The explicit state event, not continuously republished geometry,
            # owns identity and invalidation when available.
            return
        fallback_id = canonical_work_area_id(
            frame_id, points
        )
        self._invalidate_lifecycle(
            "work_area",
            "work_area_corners_changed",
            work_area_id=fallback_id,
            token=signature,
        )

    def _on_work_area_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self._publish_status(
                False,
                "work_area_state_rejected",
                mode="work_area",
                rejection_reason="invalid_work_area_state_json",
                detail=str(exc),
            )
            return
        if not isinstance(payload, dict):
            return

        selected = bool(payload.get("selected", True))
        work_area_id = str(payload.get("work_area_id", "")).strip()
        generation_id = str(payload.get("plane_generation_id", "")).strip()
        invalidation_seq = payload.get("invalidation_seq", "")
        state_key = (
            selected,
            work_area_id,
            generation_id,
            str(invalidation_seq),
        )
        if state_key == self._external_work_area_state_key:
            return

        self._external_work_area_state_seen = True
        self._external_work_area_state_key = state_key
        self._external_work_area_id = work_area_id if selected else ""
        # Cross-topic DDS delivery is not ordered.  Never let a newly selected
        # work_area_id arm a capture against latched plane/corners from the
        # previous selection.  wall_projector republishes the current geometry,
        # so this explicitly requires geometry observed after the identity edge.
        self.latest_work_area_plane = None
        self.latest_corners = None
        self._corner_signature = None
        self._expected_refined_corner_signature = None
        if selected and not work_area_id:
            self._publish_status(
                False,
                "work_area_state_rejected",
                mode="work_area",
                rejection_reason="missing_work_area_id",
            )
            return
        self._invalidate_lifecycle(
            "work_area",
            "work_area_selected" if selected else "work_area_cleared",
            work_area_id=self._external_work_area_id,
            generation_id=generation_id or None,
            token=(invalidation_seq, self._target_signature),
        )

    def _on_refine_work_area(self, msg):
        if not msg.data:
            return
        lifecycle = self._lifecycles["work_area"]
        self._invalidate_lifecycle(
            "work_area",
            "explicit_rerefine_requested",
            token=lifecycle.invalidation_seq + 1,
        )

    def _on_painting_mode(self, msg):
        paint_active = str(msg.data).strip().upper() == "PAINT"
        if paint_active == self._paint_active:
            return
        self._paint_active = paint_active
        if paint_active:
            self._active_capture_mode = None
            self._active_capture_reference = None
            self._pending_cloud_capture = None
        for mode, lifecycle in self._lifecycles.items():
            applied_deferred = lifecycle.set_paint_active(paint_active)
            if paint_active:
                self._publish_status(
                    lifecycle.accepted,
                    "paint_locked",
                    mode=mode,
                    rejection_reason="",
                )
            elif applied_deferred:
                self._publish_status(
                    False,
                    "invalidated",
                    mode=mode,
                    rejection_reason=lifecycle.rejection_reason,
                )
            else:
                self._publish_status(
                    lifecycle.accepted,
                    lifecycle.state,
                    mode=mode,
                    rejection_reason=lifecycle.rejection_reason,
                )

    def _invalidate_lifecycle(
        self,
        mode,
        reason,
        *,
        work_area_id=None,
        generation_id=None,
        token="",
    ):
        lifecycle = self._lifecycles[mode]
        applied = lifecycle.invalidate(
            reason,
            work_area_id=work_area_id,
            generation_id=generation_id,
            token=token,
        )
        if applied and self._active_capture_mode == mode:
            self._active_capture_mode = None
            self._active_capture_reference = None
        if applied:
            self._cancel_pending_cloud(mode)
        state = "invalidated" if applied else "paint_locked"
        rejection = reason if applied else f"{reason}_deferred_during_paint"
        self._publish_status(
            False,
            state,
            mode=mode,
            rejection_reason=rejection,
        )
        return applied

    # ---- capture ----------------------------------------------------------
    def _on_capture(self, msg):
        if msg.data:
            self._start_capture("work_area")

    def _on_target_capture(self, msg):
        if msg.data:
            self._start_capture("target")

    def _on_refine_target(self, msg):
        if not msg.data:
            return
        lifecycle = self._lifecycles["target"]
        self._invalidate_lifecycle(
            "target",
            "explicit_target_rerefine_requested",
            token=lifecycle.invalidation_seq + 1,
        )
        self._publish_target_status("requested")
        self._start_capture("target")

    def _start_capture(self, mode):
        if self._active_capture_mode is not None:
            self._publish_status(
                False,
                "capture_rejected",
                mode=mode,
                rejection_reason="another_capture_already_armed",
            )
            return False
        self._supersede_pending_cloud()
        reference_plane = self._reference_plane_for_mode(mode)
        if reference_plane is None:
            reason = (
                "waiting_for_target_surface"
                if mode == "target"
                else "waiting_for_work_area_plane"
            )
            self._lifecycles[mode].reject(reason)
            self._publish_status(
                False,
                "rejected",
                mode=mode,
                rejection_reason=reason,
            )
            return False
        if mode == "work_area" and not self._lifecycles[mode].work_area_id:
            self._lifecycles[mode].reject("missing_work_area_id")
            self._publish_status(
                False,
                "rejected",
                mode=mode,
                rejection_reason="missing_work_area_id",
            )
            return False

        lifecycle = self._lifecycles[mode]
        decision = lifecycle.arm_capture(time.monotonic(), self.capture_timeout_s)
        if not decision.accepted:
            self._publish_status(
                lifecycle.accepted,
                "capture_ignored" if lifecycle.accepted else "capture_rejected",
                mode=mode,
                rejection_reason=decision.rejection_reason,
            )
            return False
        self._active_capture_mode = mode
        self._active_capture_reference = reference_plane
        self._publish_status(
            False,
            "capture_armed",
            mode=mode,
            rejection_reason="",
        )
        self.get_logger().info(
            f"[D405 REFINE] next cloud armed mode={mode} "
            f"generation={lifecycle.plane_generation_id}"
        )
        return True

    def _check_capture_timeout(self):
        if getattr(self, "_pending_cloud_capture", None) is not None:
            self._retry_pending_cloud()
            return
        mode = self._active_capture_mode
        if mode is None:
            return
        lifecycle = self._lifecycles[mode]
        if not lifecycle.expire_capture(time.monotonic()):
            return
        self._active_capture_mode = None
        self._active_capture_reference = None
        self._publish_status(
            False,
            "rejected",
            mode=mode,
            rejection_reason="capture_timeout",
        )

    def _on_cloud(self, msg):
        mode = self._active_capture_mode
        if mode is None:
            return
        lifecycle = self._lifecycles[mode]
        plane_msg = self._active_capture_reference
        capture_deadline = float(lifecycle.capture_deadline)

        # Consume/disarm before parsing, TF, ROI, or RANSAC.  If exact-time TF
        # is not buffered yet, this same cloud is retained for timer retries;
        # no subsequently received cloud can replace it for this trigger.
        consumed = lifecycle.consume_capture(time.monotonic())
        self._active_capture_mode = None
        self._active_capture_reference = None
        if not consumed.accepted:
            self._publish_status(
                False,
                "rejected",
                mode=mode,
                rejection_reason=consumed.rejection_reason,
                **self._capture_metadata(msg),
            )
            return

        metadata = self._capture_metadata(msg)
        if not self._stamp_is_valid(msg.header.stamp):
            self._reject_capture(mode, "capture_stamp_invalid", metadata=metadata)
            return
        if plane_msg is None:
            self._reject_capture(
                mode,
                "waiting_for_target_surface"
                if mode == "target"
                else "waiting_for_work_area_plane",
                metadata=metadata,
            )
            return

        target_frame = plane_msg.header.frame_id or "zed_left_camera_frame"
        source_frame = msg.header.frame_id or ""
        metadata["source_frame"] = source_frame
        if not source_frame:
            self._reject_capture(mode, "cloud_frame_empty", metadata=metadata)
            return

        same_frame = target_frame == source_frame
        transform = None
        transform_age_s = 0.0
        if not same_frame:
            transform, transform_age_s = self._lookup_transform(
                target_frame, source_frame, msg.header.stamp
            )
            if transform is None:
                now = time.monotonic()
                if now > capture_deadline:
                    self._reject_capture(
                        mode,
                        "tf_missing",
                        metadata={
                            **metadata,
                            "target_frame": target_frame,
                            "tf_wait_s": 0.0,
                        },
                    )
                    return
                self._pending_cloud_capture = _PendingCloudCapture(
                    mode=mode,
                    cloud=msg,
                    reference_plane=plane_msg,
                    target_frame=target_frame,
                    source_frame=source_frame,
                    plane_generation_id=lifecycle.plane_generation_id,
                    target_id=(
                        str(getattr(self, "_target_id", ""))
                        if mode == "target"
                        else ""
                    ),
                    target_stamp_sec=(
                        int(
                            getattr(self, "_target_stamp", {}).get("sec", 0)
                        )
                        if mode == "target"
                        else 0
                    ),
                    target_stamp_nanosec=(
                        int(
                            getattr(self, "_target_stamp", {}).get(
                                "nanosec", 0
                            )
                        )
                        if mode == "target"
                        else 0
                    ),
                    deadline_monotonic=capture_deadline,
                    queued_monotonic=now,
                    metadata=dict(metadata),
                )
                self._publish_status(
                    False,
                    "waiting_for_tf",
                    mode=mode,
                    rejection_reason="",
                    target_frame=target_frame,
                    **metadata,
                )
                return
        self._evaluate_captured_cloud(
            mode,
            msg,
            plane_msg,
            target_frame,
            transform,
            transform_age_s,
            metadata,
        )

    def _retry_pending_cloud(self):
        pending = getattr(self, "_pending_cloud_capture", None)
        if pending is None:
            return

        lifecycle = self._lifecycles[pending.mode]
        if lifecycle.plane_generation_id != pending.plane_generation_id:
            self._pending_cloud_capture = None
            return
        if pending.mode == "target" and not self._pending_target_identity_matches(
            pending
        ):
            # Generation equality should already imply identity equality.  Keep
            # this independent guard so future lifecycle changes cannot evaluate
            # a queued cloud against a different target event.
            self._pending_cloud_capture = None
            self._reject_capture("target", "target_identity_changed")
            return

        now = time.monotonic()
        if now > pending.deadline_monotonic:
            self._pending_cloud_capture = None
            self._reject_capture(
                pending.mode,
                "tf_missing",
                metadata={
                    **pending.metadata,
                    "target_frame": pending.target_frame,
                    "tf_wait_s": max(0.0, now - pending.queued_monotonic),
                },
            )
            return

        transform, transform_age_s = self._lookup_transform(
            pending.target_frame,
            pending.source_frame,
            pending.cloud.header.stamp,
        )
        if transform is None:
            return

        # Clear first so an exception or terminal rejection cannot accidentally
        # retry the already-consumed cloud on the next timer tick.
        self._pending_cloud_capture = None
        self._evaluate_captured_cloud(
            pending.mode,
            pending.cloud,
            pending.reference_plane,
            pending.target_frame,
            transform,
            transform_age_s,
            {
                **pending.metadata,
                "tf_wait_s": max(0.0, now - pending.queued_monotonic),
            },
        )

    def _evaluate_captured_cloud(
        self,
        mode,
        msg,
        plane_msg,
        target_frame,
        transform,
        transform_age_s,
        metadata,
    ):
        lifecycle = self._lifecycles[mode]
        metadata = dict(metadata)
        metadata["transform_age_s"] = transform_age_s

        points = self._cloud_to_numpy(msg)
        if points is None:
            self._reject_capture(
                mode, "pointcloud_parse_failed", metadata=metadata
            )
            return
        points = self._transform_points(points, transform)

        p0 = np.array(
            [
                plane_msg.pose.position.x,
                plane_msg.pose.position.y,
                plane_msg.pose.position.z,
            ],
            dtype=float,
        )
        n0 = _pose_normal(plane_msg.pose)
        roi = self._select_surface_roi(
            points,
            p0,
            n0,
            target_frame,
            use_work_area_corners=(mode != "target"),
        )
        result = capture_once_and_fit_plane(roi, p0, n0, self.fit_config)
        decision = validate_single_plane_result(
            result, self.fit_config, transform_age_s
        )
        if not decision.accepted:
            self._reject_capture(
                mode,
                decision.rejection_reason,
                result=result,
                metadata=metadata,
            )
            return

        center = np.asarray(result.center, dtype=float)
        normal = np.asarray(result.normal, dtype=float)
        lifecycle.accept()
        # Publish the accepted generation contract first.  Consumers can then
        # arm themselves for the *next* bare PoseStamped and reject any stale
        # transient-local refined pose retained from an earlier generation.
        self._publish_status(
            True,
            "accepted",
            mode=mode,
            rejection_reason="",
            metrics=result.metrics(),
            target_frame=target_frame,
            **metadata,
        )
        if mode == "work_area":
            self._set_expected_refined_corner_signature(center, normal)
        self._publish_refined_plane(
            target_frame,
            center,
            normal,
            msg.header.stamp,
            mode,
        )

    def _cancel_pending_cloud(self, mode=None):
        pending = getattr(self, "_pending_cloud_capture", None)
        if pending is None or (mode is not None and pending.mode != mode):
            return False
        self._pending_cloud_capture = None
        return True

    def _pending_target_identity_matches(self, pending):
        stamp = getattr(self, "_target_stamp", {})
        return (
            str(getattr(self, "_target_id", "")) == pending.target_id
            and int(stamp.get("sec", 0)) == pending.target_stamp_sec
            and int(stamp.get("nanosec", 0))
            == pending.target_stamp_nanosec
        )

    def _supersede_pending_cloud(self):
        pending = getattr(self, "_pending_cloud_capture", None)
        if pending is None:
            return
        self._pending_cloud_capture = None
        lifecycle = self._lifecycles[pending.mode]
        if lifecycle.plane_generation_id == pending.plane_generation_id:
            lifecycle.reject("capture_superseded")
        self.get_logger().info(
            f"[D405 REFINE] pending cloud superseded mode={pending.mode}"
        )

    def _reject_capture(self, mode, reason, result=None, metadata=None):
        self._lifecycles[mode].reject(reason)
        self._publish_status(
            False,
            "rejected",
            mode=mode,
            rejection_reason=reason,
            metrics=None if result is None else result.metrics(),
            **(metadata or {}),
        )

    # ---- geometry / TF ----------------------------------------------------
    def _reference_plane_for_mode(self, mode):
        if mode == "target":
            return self.latest_target_surface
        return self.latest_work_area_plane

    def _select_surface_roi(
        self,
        points,
        p0,
        n0,
        target_frame,
        use_work_area_corners=True,
    ):
        signed = (points - p0) @ n0
        mask = np.abs(signed) <= self.roi_plane_window_m
        basis = (
            self._work_area_basis(target_frame, p0, n0)
            if use_work_area_corners
            else self._fallback_basis(
                p0,
                n0,
                self.target_roi_half_width_m,
                self.target_roi_half_height_m,
            )
        )
        if basis is not None:
            center, u_axis, v_axis, half_u, half_v = basis
            rel = points - center
            mask &= np.abs(rel @ u_axis) <= half_u
            mask &= np.abs(rel @ v_axis) <= half_v
        return points[mask]

    def _work_area_basis(self, target_frame, p0, n0):
        msg = self.latest_corners
        if msg is None or len(msg.poses) < 4:
            return self._fallback_basis(p0, n0)
        if (msg.header.frame_id or target_frame) != target_frame:
            return self._fallback_basis(p0, n0)
        tl, tr, br, bl = self._corner_points(msg)
        center = (tl + tr + br + bl) / 4.0
        u_vec = ((tr - tl) + (br - bl)) / 2.0
        v_vec = ((bl - tl) + (br - tr)) / 2.0
        width = float(np.linalg.norm(u_vec))
        height = float(np.linalg.norm(v_vec))
        if width < 1e-6 or height < 1e-6:
            return self._fallback_basis(p0, n0)
        return (
            center,
            u_vec / width,
            v_vec / height,
            width / 2.0 + self.roi_margin_m,
            height / 2.0 + self.roi_margin_m,
        )

    def _fallback_basis(self, p0, n0, half_u=None, half_v=None):
        seed = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(seed, n0))) > 0.95:
            seed = np.array([1.0, 0.0, 0.0], dtype=float)
        u_axis = np.cross(n0, seed)
        u_axis /= np.linalg.norm(u_axis) + 1e-12
        v_axis = np.cross(u_axis, n0)
        v_axis /= np.linalg.norm(v_axis) + 1e-12
        return (
            p0,
            u_axis,
            v_axis,
            (0.30 if half_u is None else float(half_u)) + self.roi_margin_m,
            (0.25 if half_v is None else float(half_v)) + self.roi_margin_m,
        )

    def _lookup_transform(self, target_frame, source_frame, stamp):
        if target_frame == source_frame:
            return None, 0.0
        query_time = Time.from_msg(stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                query_time,
                # A positive timeout blocks the single-threaded executor and
                # prevents TransformListener from receiving the very TF being
                # awaited.  The pending-cloud timer performs bounded retries.
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None, None

        transform_stamp = Time.from_msg(transform.header.stamp)
        # Static transforms conventionally carry stamp zero and do not age.
        transform_age_s = (
            0.0
            if transform_stamp.nanoseconds == 0
            else abs(query_time.nanoseconds - transform_stamp.nanoseconds) / 1e9
        )
        return transform, float(transform_age_s)

    @staticmethod
    def _transform_points(points, transform):
        if transform is None:
            return points
        t = transform.transform.translation
        q = transform.transform.rotation
        return quat_apply([q.x, q.y, q.z, q.w], points) + np.array(
            [t.x, t.y, t.z], dtype=float
        )

    def _cloud_to_numpy(self, msg):
        points = []
        try:
            for point in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ):
                try:
                    x, y, z = (
                        float(point[0]),
                        float(point[1]),
                        float(point[2]),
                    )
                except Exception:
                    x, y, z = (
                        float(point["x"]),
                        float(point["y"]),
                        float(point["z"]),
                    )
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
        except Exception:
            return None
        if not points:
            return np.empty((0, 3), dtype=np.float32)
        array = np.asarray(points, dtype=np.float32)
        if array.shape[0] > self.max_points:
            step = max(int(math.ceil(array.shape[0] / self.max_points)), 1)
            array = array[::step]
        return array

    # ---- publication / diagnostics ---------------------------------------
    def _publish_refined_plane(self, frame_id, center, normal, stamp, mode):
        refined = PoseStamped()
        refined.header.stamp = stamp
        refined.header.frame_id = frame_id
        refined.pose.position.x = float(center[0])
        refined.pose.position.y = float(center[1])
        refined.pose.position.z = float(center[2])
        qx, qy, qz, qw = _normal_to_quaternion(normal)
        refined.pose.orientation.x = float(qx)
        refined.pose.orientation.y = float(qy)
        refined.pose.orientation.z = float(qz)
        refined.pose.orientation.w = float(qw)
        if mode == "target":
            self.target_refined_pub.publish(refined)
            self.target_record_pub.publish(String(data=json.dumps(dict(
                target_stamp=self._target_stamp, frame_id=frame_id,
                position=[float(x) for x in center],
                orientation=[float(qx), float(qy), float(qz), float(qw)]))))
        else:
            self.refined_pub.publish(refined)

    def _publish_status(
        self,
        ok,
        state,
        *,
        mode="work_area",
        rejection_reason="",
        metrics=None,
        **fields,
    ):
        lifecycle = self._lifecycles[mode]
        if mode == "target":
            # Every target lifecycle event carries the identity of the target
            # event it describes.  This lets transient-local subscribers reject
            # stale terminal states from a previous Set Target request.
            fields = {**fields, **self._target_identity_fields()}
        payload = lifecycle.snapshot()
        payload.update(self._empty_metrics())
        if metrics:
            payload.update(metrics)
        payload.update(
            {
                "ok": bool(ok),
                "state": str(state),
                "mode": mode,
                "accepted": bool(lifecycle.accepted),
                "rejection_reason": str(rejection_reason or ""),
            }
        )
        payload.update(fields)
        payload = self._json_safe(payload)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)

        if mode == "target":
            if state == "accepted":
                target_state = "done"
            elif rejection_reason == "capture_timeout":
                target_state = "timeout"
            elif state in {"capture_armed", "invalidated"}:
                target_state = state
            elif state in {"rejected", "capture_rejected"}:
                target_state = "failed"
            else:
                target_state = state
            target_payload = dict(payload)
            target_payload.pop("state", None)
            self._publish_target_status(target_state, **target_payload)
        else:
            self.status_pub.publish(message)

        if state == "accepted":
            self.get_logger().info(
                "[D405 REFINE] accepted "
                f"generation={lifecycle.plane_generation_id}, "
                f"shift={payload.get('plane_shift_m_from_zed', 0.0)*1000:+.1f}mm, "
                f"inliers={payload.get('inlier_count', 0)}, "
                f"ratio={payload.get('inlier_ratio', 0.0):.3f}, "
                f"broad_ratio={payload.get('broad_inlier_ratio', 0.0):.3f}, "
                f"span={payload.get('support_span_major_m', 0.0):.3f}x"
                f"{payload.get('support_span_minor_m', 0.0):.3f}m"
            )
        elif state == "rejected":
            metric_text = ""
            if int(payload.get("voxel_point_count", 0) or 0) > 0:
                metric_text = (
                    f", roi={int(payload.get('roi_point_count', 0) or 0)}, "
                    f"voxels={int(payload.get('voxel_point_count', 0) or 0)}, "
                    f"inliers={int(payload.get('inlier_count', 0) or 0)}, "
                    f"ratio={float(payload.get('inlier_ratio', 0.0) or 0.0):.3f}, "
                    f"broad_ratio="
                    f"{float(payload.get('broad_inlier_ratio', 0.0) or 0.0):.3f}"
                )
            self.get_logger().warn(
                f"[D405 REFINE] rejected: {rejection_reason}{metric_text}"
            )

    def _publish_target_status(self, state, **fields):
        payload = dict(fields)
        identity = self._target_identity_fields()
        payload.setdefault("target_id", identity["target_id"])
        payload.setdefault("target_stamp", identity["target_stamp"])
        payload["state"] = state
        message = String()
        message.data = json.dumps(
            self._json_safe(payload), ensure_ascii=False, sort_keys=True
        )
        self.target_status_pub.publish(message)

    def _target_identity_fields(self):
        stamp = getattr(self, "_target_stamp", {})
        return {
            "target_id": str(getattr(self, "_target_id", "")),
            "target_stamp": {
                "sec": int(stamp.get("sec", 0)),
                "nanosec": int(stamp.get("nanosec", 0)),
            },
        }

    @staticmethod
    def _empty_metrics():
        return {
            "roi_point_count": 0,
            "voxel_point_count": 0,
            "inlier_count": 0,
            "inlier_ratio": 0.0,
            "rms_residual_m": None,
            "max_residual_m": None,
            "normal_delta_deg_from_zed": None,
            "plane_shift_m_from_zed": None,
            "broad_inlier_count": 0,
            "broad_inlier_ratio": 0.0,
            "support_point_count": 0,
            "support_span_major_m": 0.0,
            "support_span_minor_m": 0.0,
            "target_support_offset_m": None,
            "secondary_plane_inlier_count": 0,
            "secondary_plane_relative_inliers": 0.0,
            "secondary_plane_separation_m": 0.0,
            "secondary_plane_normal_delta_deg": 0.0,
            "capture_stamp": {"sec": 0, "nanosec": 0},
            "source_frame": "",
            "transform_age_s": None,
        }

    @staticmethod
    def _capture_metadata(msg):
        return {
            "capture_stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "source_frame": msg.header.frame_id or "",
            "transform_age_s": None,
        }

    @staticmethod
    def _stamp_is_valid(stamp):
        return (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        ) > 0

    @classmethod
    def _json_safe(cls, value):
        if isinstance(value, dict):
            return {str(k): cls._json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_safe(v) for v in value]
        if isinstance(value, (np.integer,)):
            return int(value)
        if isinstance(value, (np.floating, float)):
            number = float(value)
            return number if math.isfinite(number) else None
        return value

    @staticmethod
    def _corner_points(msg):
        return np.asarray(
            [
                [p.position.x, p.position.y, p.position.z]
                for p in msg.poses[:4]
            ],
            dtype=float,
        )

    @staticmethod
    def _corner_signature_from_points(points, frame_id=""):
        return (
            str(frame_id),
            tuple(
                tuple(round(float(value), 5) for value in point[:3])
                for point in np.asarray(points, dtype=float)[:4]
            ),
        )

    def _set_expected_refined_corner_signature(self, center, normal):
        if self.latest_corners is None or len(self.latest_corners.poses) < 4:
            self._expected_refined_corner_signature = None
            return
        corners = self._corner_points(self.latest_corners)
        signed = (corners - center) @ normal
        projected = corners - signed[:, None] * normal
        self._expected_refined_corner_signature = (
            self._corner_signature_from_points(
                projected,
                self.latest_corners.header.frame_id
                or "zed_left_camera_frame",
            )
        )

    def _check_calibration_hash(self):
        try:
            current_hash = calibration_file_sha256(self.calibration_file)
        except Exception as exc:
            current_hash = f"error:{type(exc).__name__}"
        if current_hash == self._calibration_hash:
            return
        previous_hash = self._calibration_hash
        self._calibration_hash = current_hash
        token = (previous_hash, current_hash)
        self._invalidate_lifecycle(
            "target", "calibration_file_changed", token=token
        )
        self._invalidate_lifecycle(
            "work_area", "calibration_file_changed", token=token
        )


def main(args=None):
    rclpy.init(args=args)
    node = D405SurfaceRefinerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
