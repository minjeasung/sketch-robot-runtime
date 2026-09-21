"""
sketch_to_waypoints_node — 브라우저 (u, v) → world 3D waypoints.

입력:
  /sketch_pixels          (geometry_msgs/PoseArray)
                          header.frame_id = "wall_front" | "zed_raw"
                          poses[i].position.{x=u, y=v, z=0}
  /perception/work_area_plane
                          (geometry_msgs/PoseStamped, in zed_left_camera_frame)
                          yellow border 로 검출한 작업영역 중심 + normal — 캐시
  /perception/work_area_corners
                          (geometry_msgs/PoseArray, TL/TR/BR/BL, same camera frame)
  TF                      world ← zed_left_camera_frame

출력:
  /sketch_eoat_segments   (version 3 canonical JSON, final planning frame)
  /sketch_waypoints       (legacy visualization PoseArray, segment와 동일 geometry)
  /sketch_markers         (final segment row에서 직접 생성)
  /painting_system/plan_status (JSON diagnostic/status)

알고리즘 (wall_front 모드만 — zed_raw 는 TODO):
  1) 선택 pixel rect 내부 여부를 2D에서 검사한다(clip 금지).
  2) wall-front extent로 surface point를 만들고 선택 3D quad 내부인지 재검사한다.
  3) 최종 planning frame에서 v3 surface-point segment와 SHA-256을 만든다.
  4) roller-center pose = surface + normal*(26 mm geometry + row offset).
  5) TCP local +Y = normal, +X = roller long axis로 연속 자세를 만든다.
"""
import json
import math
import numpy as np
import time
from dataclasses import replace
from itertools import groupby
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)

from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from rbpodo_painting_control.segment_path import (
    MOTION_MODES,
    SEGMENT_SCHEMA_VERSION,
    SegmentPathError,
    attach_plan_hash,
    canonical_segment_json,
    parse_segment_path,
    rotation_from_surface_path,
    segment_waypoint_position,
    validate_segment_path_for_real_execution,
)
from rbpodo_painting_control.spray_path import make_spray_rows
from sketch_control.rotation_utils import quat_from_matrix, quat_to_matrix
from sketch_control.work_area_geometry import (
    WorkAreaGeometryError,
    bilinear_quad_point,
    generate_fill_strokes,
    outside_pixel_rect_indices,
    outside_quad_3d_indices,
    pixel_rect_from_points,
    quad_size_m,
)


# ---- 파라미터 ----------------------------------------------------------------
SKETCH_PIXELS_TOPIC = "/sketch_pixels"
WORK_AREA_TOPIC = "/perception/work_area_plane"
WORK_AREA_REFINED_TOPIC = "/perception/work_area_plane_refined"
WORK_AREA_CORNERS_TOPIC = "/perception/work_area_corners"
WORK_AREA_PIXELS_TOPIC = "/work_area_pixels"
WALL_FRONT_EXTENT_TOPIC = "/perception/wall_front_extent"
WALL_FRONT_IMAGE_TOPIC = "/perception/wall_front_view"
D405_REFINEMENT_STATUS_TOPIC = "/perception/d405_surface_refinement_status"
WORK_AREA_STATE_TOPIC = "/painting_system/work_area_state"
WAYPOINTS_TOPIC = "/sketch_waypoints"
MARKERS_TOPIC = "/sketch_markers"
FILL_WORK_AREA_TOPIC = "/fill_work_area"
EOAT_SEGMENTS_TOPIC = "/sketch_eoat_segments"
PLAN_STATUS_TOPIC = "/painting_system/plan_status"
FILL_PREVIEW_TOPIC = "/painting_system/fill_preview_pixels"

WORLD_FRAME = "World"
CAM_FRAME = "zed_left_camera_frame"

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# wall_projector_node 와 일치해야 함. view 해상도는 wall_front Image 를
# 구독해서 동적으로 맞춘다. 아래 값은 첫 이미지 수신 전 fallback.
WALL_W = 0.5     # m
WALL_H = 0.4     # m
DEFAULT_VIEW_W = 800     # px
DEFAULT_VIEW_H = 800     # px

# Segment points are physical surface points. Roller-center poses are derived by
# adding the measured 26 mm radius; precontact clearance is a separate offset.
ROLLER_RADIUS = 0.026
ROLLER_LENGTH = 0.175
CONTACT_CLEARANCE = 0.005
PRECONTACT_TARE_CLEARANCE = 0.010
# Backward import compatibility only. It now means physical contact geometry,
# never geometry plus planning clearance.
EOAT_SURFACE_OFFSET = ROLLER_RADIUS
FILL_OVERLAP = 0.30


def _quat_to_rot(qx, qy, qz, qw):
    return quat_to_matrix([qx, qy, qz, qw])


class SketchToWaypointsNode(Node):
    def __init__(self):
        super().__init__("sketch_to_waypoints_node")

        self.process_mode = "paint"
        self.create_subscription(String, "/painting_system/process_mode", self._on_process_mode, LATCHED_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_work_area = None  # PoseStamped from ZED/global perception
        self.latest_refined_work_area = None  # PoseStamped refined by wrist D405
        self.latest_refined_work_area_time = 0.0
        self.latest_work_area_corners = None
        self.latest_work_area_pixels = None
        self.latest_work_area_rect_px = None
        self.latest_work_area_selection_time = 0.0
        self.latest_work_area_selection_id = ""
        self.current_work_area_id = ""
        self.work_area_invalidation_seq = ""
        self.d405_status = {
            "accepted": False,
            "work_area_id": "",
            "plane_generation_id": "",
            "reason": "not_received",
        }
        self.d405_status_time = 0.0
        # The refiner publishes accepted status before the matching PoseStamped.
        # Only that next pose may populate the cache; stale transient-local pose
        # samples from an older generation are otherwise indistinguishable.
        self.d405_refined_pose_armed = False
        self.d405_refined_pose_generation_id = ""
        # wall_front 전체 뷰가 대응하는 평면 사각형. 픽셀->3D 매핑 기준(작업영역이
        # 아니라 전체 뷰). 크롭 없는 흐름에서 wall_front pixel 이 전체 뷰이기 때문.
        self.latest_front_extent = None
        self.view_w = DEFAULT_VIEW_W
        self.view_h = DEFAULT_VIEW_H
        self.eoat_segment_frame = str(
            self.declare_parameter("eoat_segment_frame", "link0").value
        ).strip() or WORLD_FRAME
        self.publish_eoat_segments = bool(
            self.declare_parameter("publish_eoat_segments", True).value
        )
        self.real_painting_enabled = bool(
            self.declare_parameter("real_painting_enabled", False).value
        )
        self.dry_run = bool(self.declare_parameter("dry_run", True).value)
        self.default_paint_force_n = max(
            0.0, float(self.declare_parameter("default_paint_force_n", 0.0).value)
        )
        # Operator-adjustable per run (2026-08-14): the process force is a
        # 3-15 N commissioning range, bounded downstream by the executor's
        # max_paint_force_n and the wrench-reference/guard 15 N command caps.
        # A dynamic update takes effect at the next waypoint generation:
        #   ros2 param set /sketch_to_waypoints default_paint_force_n 8.0
        self.add_on_set_parameters_callback(self._on_parameter_update)
        self.paint_speed_mps = max(
            0.0, float(self.declare_parameter("paint_speed_mps", 0.020).value)
        )
        self.contact_geometry_offset_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "contact_geometry_offset_m", ROLLER_RADIUS
                ).value
            ),
        )
        self.precontact_clearance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "precontact_clearance_m", PRECONTACT_TARE_CLEARANCE
                ).value
            ),
        )
        self.travel_clearance_m = max(
            0.0, float(self.declare_parameter("travel_clearance_m", 0.010).value)
        )
        self.safety_approach_offset_m = max(
            self.precontact_clearance_m,
            float(
                self.declare_parameter(
                    "safety_approach_offset_m", 0.080
                ).value
            ),
        )
        self.final_retreat_offset_m = max(
            self.travel_clearance_m,
            float(
                self.declare_parameter(
                    "final_retreat_offset_m", 0.080
                ).value
            ),
        )
        self.contact_search_speed_mps = max(
            0.0,
            float(
                self.declare_parameter(
                    "contact_search_speed_mps", 0.002
                ).value
            ),
        )
        self.contact_search_max_distance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "contact_search_max_distance_m", 0.015
                ).value
            ),
        )
        self.contact_search_timeout_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "contact_search_timeout_s", 10.0
                ).value
            ),
        )
        self.retract_speed_mps = max(
            0.0, float(self.declare_parameter("retract_speed_mps", 0.010).value)
        )
        self.approach_speed_mps = max(
            0.0, float(self.declare_parameter("approach_speed_mps", 0.005).value)
        )
        self.travel_speed_mps = max(
            0.0, float(self.declare_parameter("travel_speed_mps", 0.030).value)
        )
        self.fill_overlap = float(
            self.declare_parameter("fill_overlap", FILL_OVERLAP).value
        )
        self.roller_length_m = max(
            0.0,
            float(self.declare_parameter("roller_length_m", ROLLER_LENGTH).value),
        )
        self.work_area_pixel_tolerance_px = max(
            0.0,
            float(
                self.declare_parameter(
                    "work_area_pixel_tolerance_px", 1.0
                ).value
            ),
        )
        self.work_area_containment_tolerance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "work_area_containment_tolerance_m", 0.001
                ).value
            ),
        )
        self.work_area_plane_tolerance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "work_area_plane_tolerance_m", 0.002
                ).value
            ),
        )

        self.create_subscription(
            PoseStamped, WORK_AREA_TOPIC, self._on_work_area, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, WORK_AREA_REFINED_TOPIC,
            self._on_refined_work_area, LATCHED_QOS)
        self.create_subscription(
            PoseArray, WORK_AREA_CORNERS_TOPIC, self._on_work_area_corners, LATCHED_QOS)
        self.create_subscription(
            PoseArray, WORK_AREA_PIXELS_TOPIC, self._on_work_area_pixels, 10)
        self.create_subscription(
            PoseArray, WALL_FRONT_EXTENT_TOPIC, self._on_front_extent, LATCHED_QOS)
        self.create_subscription(
            Image, WALL_FRONT_IMAGE_TOPIC, self._on_wall_front_image,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseArray, SKETCH_PIXELS_TOPIC, self._on_sketch, 10)
        self.create_subscription(
            Empty, FILL_WORK_AREA_TOPIC, self._on_fill_work_area, 10)
        self.create_subscription(
            String,
            D405_REFINEMENT_STATUS_TOPIC,
            self._on_d405_refinement_status,
            LATCHED_QOS,
        )
        self.create_subscription(
            String,
            WORK_AREA_STATE_TOPIC,
            self._on_work_area_state,
            LATCHED_QOS,
        )
        self.pub = self.create_publisher(PoseArray, WAYPOINTS_TOPIC, 10)
        self.segment_pub = self.create_publisher(String, EOAT_SEGMENTS_TOPIC, 10)
        self.plan_status_pub = self.create_publisher(
            String, PLAN_STATUS_TOPIC, LATCHED_QOS
        )
        self.fill_preview_pub = self.create_publisher(
            PoseArray, FILL_PREVIEW_TOPIC, 10
        )
        # latch-like: RViz 가 늦게 켜져도 마지막 marker 보이게 transient_local 도 좋지만
        # 매 sketch 마다 갱신하므로 일반 10 depth 로 충분.
        self.marker_pub = self.create_publisher(MarkerArray, MARKERS_TOPIC, 10)

        self.get_logger().info(
            f"sketch_to_waypoints_node 시작\n"
            f"  work  : {WORK_AREA_TOPIC}\n"
            f"          {WORK_AREA_REFINED_TOPIC} (D405 우선, fresh only)\n"
            f"          {WORK_AREA_CORNERS_TOPIC}\n"
            f"          {WORK_AREA_PIXELS_TOPIC}\n"
            f"          {D405_REFINEMENT_STATUS_TOPIC}\n"
            f"  sketch: {SKETCH_PIXELS_TOPIC}\n"
            f"  out   : {WAYPOINTS_TOPIC} (frame={WORLD_FRAME})\n"
            f"          {MARKERS_TOPIC} (MarkerArray, 같은 frame)\n"
            f"          {EOAT_SEGMENTS_TOPIC} (v3 segment rows, frame={self.eoat_segment_frame})\n"
            f"          {PLAN_STATUS_TOPIC} (JSON diagnostics)\n"
            f"  wall_front image size is tracked dynamically")

    # ---- callbacks ----
    def _on_parameter_update(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name != "default_paint_force_n":
                continue
            try:
                requested = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason="default_paint_force_n must be a number"
                )
            if not (0.0 <= requested <= 15.0):
                # 15 N is the hard command cap shared by the executor
                # (max_paint_force_n), the wrench reference and the guard;
                # accepting more here would only be clamped downstream.
                return SetParametersResult(
                    successful=False,
                    reason="default_paint_force_n must be within [0, 15] N",
                )
            self.default_paint_force_n = requested
            self.get_logger().info(
                f"default_paint_force_n -> {requested:.2f} N "
                "(applies to the next generated path)"
            )
        return SetParametersResult(successful=True)

    def _on_work_area(self, msg: PoseStamped):
        self.latest_work_area = msg

    def _on_refined_work_area(self, msg: PoseStamped):
        if not getattr(self, "d405_refined_pose_armed", False):
            self.get_logger().warn(
                "arm되지 않은 D405 refined Pose 무시 (stale/extra sample)"
            )
            return
        self.latest_refined_work_area = msg
        self.latest_refined_work_area_time = time.monotonic()
        self.d405_refined_pose_armed = False

    def _real_execution_enabled(self):
        # ``dry_run`` disables force output; it does not relax the real-robot
        # plane/TF/schema contract.  Any run with real_painting_enabled uses the
        # accepted D405 generation and final link0 v3 path fail-closed.
        return bool(getattr(self, "real_painting_enabled", False))

    def _current_work_area(self):
        if self.real_painting_enabled:
            if self.d405_status.get("accepted") and self.latest_refined_work_area is not None:
                return self.latest_refined_work_area, "d405_refined"
            return None, "d405_required"
        if (
            self.d405_status.get("accepted")
            and self.latest_refined_work_area is not None
        ):
            return self.latest_refined_work_area, "d405_refined"
        return self.latest_work_area, "zed"

    def _on_work_area_corners(self, msg: PoseArray):
        if len(msg.poses) >= 4:
            self.latest_work_area_corners = msg

    def _on_work_area_pixels(self, msg: PoseArray):
        if (msg.header.frame_id or "") != "wall_front":
            self._publish_plan_status(
                "rejected",
                reason="WORK_AREA_PIXELS_FRAME_INVALID",
                frame_id=msg.header.frame_id or "",
            )
            return
        points = [(pose.position.x, pose.position.y) for pose in msg.poses]
        try:
            rect = pixel_rect_from_points(
                points,
                self.view_w,
                self.view_h,
                boundary_tolerance_px=self.work_area_pixel_tolerance_px,
            )
        except WorkAreaGeometryError as exc:
            self.latest_work_area_pixels = None
            self.latest_work_area_rect_px = None
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_PIXELS_INVALID", detail=str(exc)
            )
            return
        self.latest_work_area_pixels = msg
        self.latest_work_area_rect_px = rect
        self.latest_work_area_selection_time = time.monotonic()
        selection_id = self._stamp_path_id(msg.header.stamp)
        self.latest_work_area_selection_id = selection_id or (
            f"work-area-{time.monotonic_ns()}"
        )
        # A new selection invalidates any previously accepted D405 result until
        # the refiner publishes a status tied to the new work area.
        self.latest_refined_work_area = None
        self.latest_refined_work_area_time = 0.0
        self.d405_refined_pose_armed = False
        self.d405_refined_pose_generation_id = ""
        self.d405_status = {
            "accepted": False,
            "work_area_id": "",
            "plane_generation_id": "",
            "reason": "work_area_changed",
        }
        self.d405_status_time = 0.0
        self._publish_plan_status(
            "invalidated",
            reason="WORK_AREA_CHANGED",
            work_area_selection_id=self.latest_work_area_selection_id,
        )
        self._publish_marker_clear(WORLD_FRAME)

    def _on_d405_refinement_status(self, msg: String):
        try:
            payload = json.loads(msg.data or "{}")
        except json.JSONDecodeError as exc:
            self.latest_refined_work_area = None
            self.latest_refined_work_area_time = 0.0
            self.d405_refined_pose_armed = False
            self.d405_refined_pose_generation_id = ""
            self.d405_status = {
                "accepted": False,
                "work_area_id": "",
                "plane_generation_id": "",
                "reason": f"invalid_json:{exc}",
            }
            self.d405_status_time = time.monotonic()
            self._publish_plan_status(
                "rejected", reason="D405_STATUS_INVALID_JSON", detail=str(exc)
            )
            return
        if not isinstance(payload, dict):
            payload = {}
        if str(payload.get("mode", "")).strip().lower() != "work_area":
            return
        accepted_value = payload.get("accepted")
        if accepted_value is None:
            accepted_value = payload.get("ok", False)
        # Fail closed on malformed values such as the string "false", which is
        # truthy in Python but is not a JSON boolean acceptance decision.
        accepted = accepted_value is True
        work_area_id = str(payload.get("work_area_id", "")).strip()
        plane_generation_id = str(
            payload.get("plane_generation_id", "")
        ).strip()
        current_work_area_id = str(
            getattr(self, "current_work_area_id", "")
        ).strip()
        if accepted and (not work_area_id or not plane_generation_id):
            accepted = False
            reason = "accepted_status_missing_ids"
        elif (
            accepted
            and current_work_area_id
            and work_area_id != current_work_area_id
        ):
            accepted = False
            reason = "accepted_status_work_area_mismatch"
        elif accepted and self.real_painting_enabled and not current_work_area_id:
            accepted = False
            reason = "current_work_area_id_missing"
        else:
            reason = str(
                payload.get("rejection_reason", payload.get("reason", ""))
            ).strip()
        self.d405_status = {
            "accepted": accepted,
            "work_area_id": work_area_id,
            "plane_generation_id": plane_generation_id,
            "reason": reason,
        }
        self.d405_status_time = time.monotonic()
        # Status-before-pose is the lifecycle barrier. Clear any transient pose
        # cached from an older generation and consume exactly one subsequent
        # PoseStamped for this accepted generation.
        self.latest_refined_work_area = None
        self.latest_refined_work_area_time = 0.0
        self.d405_refined_pose_armed = accepted
        self.d405_refined_pose_generation_id = (
            plane_generation_id if accepted else ""
        )
        self._publish_plan_status(
            "plane_accepted" if accepted else "plane_rejected",
            reason=reason,
            work_area_id=work_area_id,
            plane_generation_id=plane_generation_id,
        )

    def _on_work_area_state(self, msg: String):
        try:
            payload = json.loads(msg.data or "{}")
        except json.JSONDecodeError as exc:
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_STATE_INVALID_JSON", detail=str(exc)
            )
            return
        if not isinstance(payload, dict):
            return
        selected = payload.get("selected") is True
        work_area_id = str(payload.get("work_area_id", "")).strip()
        next_id = work_area_id if selected else ""
        next_seq = str(payload.get("invalidation_seq", ""))
        if (
            next_id == getattr(self, "current_work_area_id", "")
            and next_seq == getattr(self, "work_area_invalidation_seq", "")
        ):
            return
        self.current_work_area_id = next_id
        self.work_area_invalidation_seq = next_seq
        self.latest_refined_work_area = None
        self.latest_refined_work_area_time = 0.0
        self.d405_refined_pose_armed = False
        self.d405_refined_pose_generation_id = ""
        self.d405_status = {
            "accepted": False,
            "work_area_id": "",
            "plane_generation_id": "",
            "reason": "work_area_state_changed",
        }
        self._publish_plan_status(
            "invalidated",
            reason="WORK_AREA_STATE_CHANGED",
            work_area_id=next_id,
            work_area_invalidation_seq=next_seq,
        )
        self._publish_marker_clear(WORLD_FRAME)

    @staticmethod
    def _stamp_path_id(stamp) -> str:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return str(stamp_ns) if stamp_ns > 0 else ""

    def _publish_plan_status(self, state, reason="", **fields):
        payload = {
            "state": str(state),
            "reason": str(reason),
            "real_painting_enabled": bool(
                getattr(self, "real_painting_enabled", False)
            ),
            "dry_run": bool(getattr(self, "dry_run", True)),
            "real_execution_enabled": self._real_execution_enabled(),
        }
        payload.update(fields)
        publisher = getattr(self, "plan_status_pub", None)
        if publisher is not None:
            status = String()
            status.data = json.dumps(
                payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
            )
            publisher.publish(status)

    def _segment_context(self):
        status = getattr(self, "d405_status", {}) or {}
        accepted = bool(status.get("accepted", False))
        work_area_id = str(status.get("work_area_id", "")).strip()
        plane_generation_id = str(status.get("plane_generation_id", "")).strip()
        current_work_area_id = str(
            getattr(self, "current_work_area_id", "")
        ).strip()
        if (
            getattr(self, "real_painting_enabled", False)
            and (
                not accepted
                or not work_area_id
                or not plane_generation_id
                or not current_work_area_id
                or work_area_id != current_work_area_id
            )
        ):
            reason = str(status.get("reason", "D405 refined plane unavailable"))
            self._publish_plan_status(
                "rejected",
                reason="D405_REFINED_PLANE_REQUIRED",
                detail=reason,
            )
            return None
        if not work_area_id:
            work_area_id = current_work_area_id
        if not work_area_id:
            work_area_id = str(
                getattr(self, "latest_work_area_selection_id", "")
            ).strip() or "dry-run-work-area"
        if not plane_generation_id:
            plane_generation_id = "dry-run-unrefined"
        return work_area_id, plane_generation_id

    def _on_front_extent(self, msg: PoseArray):
        if len(msg.poses) >= 4:
            self.latest_front_extent = msg

    def _on_wall_front_image(self, msg: Image):
        if msg.width <= 0 or msg.height <= 0:
            return
        if msg.width != self.view_w or msg.height != self.view_h:
            self.view_w = int(msg.width)
            self.view_h = int(msg.height)
            self.get_logger().info(
                f"wall_front view size 갱신: {self.view_w}x{self.view_h}")

    def _on_sketch(self, msg: PoseArray):
        # A fill preview represents an executable backend plan, not merely a
        # raster candidate. Any new path attempt invalidates the old preview;
        # the fill callback republishes it only after this method succeeds.
        self._publish_fill_preview((), stamp=msg.header.stamp)
        view = msg.header.frame_id or ""
        if view != "wall_front":
            self.get_logger().warn(
                f"sketch frame_id='{view}' — wall_front 만 지원. skip")
            self._publish_plan_status(
                "rejected", reason="PATH_FRAME_INVALID", frame_id=view
            )
            return
        if not msg.poses:
            self.get_logger().warn("빈 sketch — skip")
            self._publish_plan_status("rejected", reason="EMPTY_PATH")
            return
        rect = self.latest_work_area_rect_px
        if rect is None:
            self.get_logger().warn("선택된 /work_area_pixels 없음 — path 거부")
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_PIXELS_REQUIRED"
            )
            return
        pixels = [(pose.position.x, pose.position.y) for pose in msg.poses]
        try:
            outside_2d = outside_pixel_rect_indices(
                pixels,
                rect,
                tolerance_px=self.work_area_pixel_tolerance_px,
            )
        except WorkAreaGeometryError as exc:
            self._publish_plan_status(
                "rejected", reason="PATH_PIXELS_INVALID", detail=str(exc)
            )
            return
        if outside_2d:
            self.get_logger().error(
                f"작업영역 밖 자유 스케치 {len(outside_2d)}점 — path 거부")
            self._publish_plan_status(
                "rejected",
                reason="PATH_OUTSIDE_WORK_AREA_2D",
                outside_point_count=len(outside_2d),
                outside_point_indices=outside_2d[:20],
            )
            self._publish_marker_clear(WORLD_FRAME)
            return
        work_area, work_area_source = self._current_work_area()
        if work_area is None:
            self.get_logger().warn(
                f"유효한 work area plane 없음(source={work_area_source}) — path 거부")
            self._publish_plan_status(
                "rejected", reason="D405_REFINED_PLANE_REQUIRED"
            )
            return

        source_frame = (
            work_area.header.frame_id
            or CAM_FRAME
        )

        # TF: world ← camera/surface frame
        T_wc = np.eye(4)
        if source_frame != WORLD_FRAME:
            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME, source_frame, Time(),
                    timeout=Duration(seconds=0.5))
            except TransformException as e:
                self.get_logger().warn(
                    f"TF lookup 실패 ({WORLD_FRAME}←{source_frame}): {e}")
                self._publish_plan_status(
                    "rejected", reason="PATH_TF_UNAVAILABLE", detail=str(e)
                )
                return

            t = tf.transform.translation
            q = tf.transform.rotation
            T_wc[:3, :3] = _quat_to_rot(q.x, q.y, q.z, q.w)
            T_wc[:3, 3] = [t.x, t.y, t.z]

        # work_area centroid (camera frame → world)
        wp = work_area.pose
        cent_cam_h = np.array(
            [wp.position.x, wp.position.y, wp.position.z, 1.0])
        cent_world = (T_wc @ cent_cam_h)[:3]

        # work_area normal — pose.orientation 의 local +Z
        R_plane_cam = _quat_to_rot(
            wp.orientation.x, wp.orientation.y,
            wp.orientation.z, wp.orientation.w)
        normal_cam = R_plane_cam @ np.array([0.0, 0.0, 1.0])
        normal_world = T_wc[:3, :3] @ normal_cam
        normal_world /= np.linalg.norm(normal_world) + 1e-12

        front_corners_world = self._pose_array_corners_world(
            self.latest_front_extent, T_wc, source_frame
        )
        selected_corners_world = self._pose_array_corners_world(
            self.latest_work_area_corners, T_wc, source_frame
        )
        if front_corners_world is None or selected_corners_world is None:
            self.get_logger().error(
                "wall-front extent/work-area 3D corners 없음 또는 frame 불일치")
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_3D_CORNERS_REQUIRED"
            )
            return
        if work_area_source == "d405_refined":
            front_corners_world = self._project_points_to_plane(
                front_corners_world, cent_world, normal_world)
            selected_corners_world = self._project_points_to_plane(
                selected_corners_world, cent_world, normal_world)

        # Wall-plane right axis, also used as the fallback tangent for a
        # degenerate one-point stroke.
        world_up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(world_up, normal_world))) > 0.95:
            world_up = np.array([1.0, 0.0, 0.0])
        right = np.cross(normal_world, world_up)
        right /= np.linalg.norm(right) + 1e-12

        surface_points = []
        for p in msg.poses:
            u = float(p.position.x)
            v = float(p.position.y)
            p_surface = self._bilinear_point(
                front_corners_world, u, v
            )
            surface_points.append(np.asarray(p_surface, dtype=float))

        try:
            outside_3d = outside_quad_3d_indices(
                surface_points,
                selected_corners_world,
                boundary_tolerance_m=self.work_area_containment_tolerance_m,
                plane_tolerance_m=self.work_area_plane_tolerance_m,
            )
        except WorkAreaGeometryError as exc:
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_3D_INVALID", detail=str(exc)
            )
            return
        if outside_3d:
            self.get_logger().error(
                f"3D 작업영역 밖 surface point {len(outside_3d)}점 — path 거부")
            self._publish_plan_status(
                "rejected",
                reason="PATH_OUTSIDE_WORK_AREA_3D",
                outside_point_count=len(outside_3d),
                outside_point_indices=outside_3d[:20],
            )
            self._publish_marker_clear(WORLD_FRAME)
            return

        stroke_ids = self._stroke_ids_from_msg(msg)
        strokes = self._group_surface_points(surface_points, stroke_ids)
        if not strokes:
            self.get_logger().warn("surface stroke 생성 실패 — skip")
            return
        stamp = self.get_clock().now().to_msg()
        path_id = self._stamp_path_id(stamp) or str(time.monotonic_ns())
        path = self._publish_eoat_segments(
            strokes,
            normal_world,
            right,
            path_id=path_id,
            source={
                "view": view,
                "plane": work_area_source,
                "surface_point_count": len(surface_points),
            },
        )
        if path is None:
            return
        out = self._compat_pose_array_from_segment(path, stamp)
        self.pub.publish(out)
        self._publish_segment_markers(path, stamp)

        self.get_logger().info(
            f"{len(out.poses)} waypoints published "
            f"({len(strokes)} stroke groups, {len(surface_points)} surface points) "
            f"plan_hash={path.plan_hash[:8]} "
            f"(view={view}, plane={work_area_source}, centroid_world="
            f"({cent_world[0]:+.3f},{cent_world[1]:+.3f},{cent_world[2]:+.3f}), "
            f"normal_world=({normal_world[0]:+.2f},{normal_world[1]:+.2f},{normal_world[2]:+.2f}))")
        return path

    def _on_fill_work_area(self, _msg: Empty):
        rect = self.latest_work_area_rect_px
        if rect is None:
            self._publish_fill_preview(())
            self.get_logger().error("[FILL] 선택된 /work_area_pixels 없음")
            self._publish_plan_status(
                "rejected", reason="WORK_AREA_PIXELS_REQUIRED"
            )
            return
        try:
            width_m, height_m = self._current_work_area_size_m()
            strokes = generate_fill_strokes(
                rect,
                work_area_width_m=width_m,
                work_area_height_m=height_m,
                roller_length_m=self.roller_length_m,
                overlap=self.fill_overlap,
            )
        except WorkAreaGeometryError as exc:
            self._publish_fill_preview(())
            self.get_logger().error(f"[FILL] 거부: {exc}")
            self._publish_plan_status(
                "rejected", reason="FILL_GEOMETRY_INVALID", detail=str(exc)
            )
            self._publish_marker_clear(WORLD_FRAME)
            return

        fill_msg = PoseArray()
        fill_msg.header.stamp = self.get_clock().now().to_msg()
        fill_msg.header.frame_id = "wall_front"
        for stroke_idx, stroke in enumerate(strokes):
            for u, v in stroke:
                pose = Pose()
                pose.position.x = float(u)
                pose.position.y = float(v)
                pose.position.z = float(stroke_idx)
                pose.orientation.w = 1.0
                fill_msg.poses.append(pose)

        self.get_logger().info(
            "[FILL] generated %d strokes inside selected work area "
            "(roller=%.0fmm, overlap=%.0f%%, area=%.3fx%.3fm)"
            % (
                len(strokes),
                self.roller_length_m * 1000.0,
                self.fill_overlap * 100.0,
                width_m,
                height_m,
            )
        )
        path = self._on_sketch(fill_msg)
        if path is None:
            self._publish_fill_preview((), stamp=fill_msg.header.stamp)
            return
        self._publish_fill_preview(strokes, stamp=fill_msg.header.stamp)

    def _publish_fill_preview(self, strokes, stamp=None):
        """Publish the exact backend fill pixels for the wall-front overlay."""

        preview = PoseArray()
        preview.header.stamp = stamp or self.get_clock().now().to_msg()
        preview.header.frame_id = "wall_front"
        for stroke_idx, stroke in enumerate(strokes):
            for u, v in stroke:
                pose = Pose()
                pose.position.x = float(u)
                pose.position.y = float(v)
                pose.position.z = float(stroke_idx)
                pose.orientation.w = 1.0
                preview.poses.append(pose)
        publisher = getattr(self, "fill_preview_pub", None)
        if publisher is not None:
            publisher.publish(preview)

    def _current_work_area_size_m(self):
        msg = self.latest_work_area_corners
        if msg is None or len(msg.poses) < 4:
            raise WorkAreaGeometryError("selected work-area 3D corners are unavailable")
        points = [
            [pose.position.x, pose.position.y, pose.position.z]
            for pose in msg.poses[:4]
        ]
        return quad_size_m(points)

    @staticmethod
    def _stroke_ids_from_msg(msg: PoseArray):
        ids = []
        for pose in msg.poses:
            z = float(pose.position.z)
            if not math.isfinite(z):
                z = 0.0
            ids.append(int(round(z)))
        return ids

    @staticmethod
    def _group_surface_points(points, stroke_ids):
        strokes = []
        current = []
        current_id = None
        for point, stroke_id in zip(points, stroke_ids):
            if current and stroke_id != current_id:
                strokes.append(current)
                current = []
            current.append(np.asarray(point, dtype=float))
            current_id = stroke_id
        if current:
            strokes.append(current)
        return strokes

    @staticmethod
    def _stroke_tangent(stroke, fallback):
        if len(stroke) >= 2:
            tangent = np.asarray(stroke[-1], dtype=float) - np.asarray(
                stroke[0], dtype=float)
            if float(np.linalg.norm(tangent)) > 1e-8:
                return tangent
        return np.asarray(fallback, dtype=float)

    def _segment_frame_transform(self):
        target_frame = self.eoat_segment_frame or WORLD_FRAME
        if target_frame == WORLD_FRAME:
            return WORLD_FRAME, np.eye(3), np.zeros(3)
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                WORLD_FRAME,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            if getattr(self, "real_painting_enabled", False):
                self.get_logger().error(
                    "[EOAT SEGMENTS] real mode TF 실패 (%s<-%s): %s"
                    % (target_frame, WORLD_FRAME, exc)
                )
                return None
            self.get_logger().warn(
                "[EOAT SEGMENTS] dry-run TF 실패 (%s<-%s): %s. frame=%s 로 발행"
                % (target_frame, WORLD_FRAME, exc, WORLD_FRAME),
                throttle_duration_sec=2.0,
            )
            return WORLD_FRAME, np.eye(3), np.zeros(3)
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = _quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        return target_frame, rot, trans

    def _on_process_mode(self, msg):
        try:
            mode = json.loads(msg.data)["mode"]
        except (ValueError, KeyError, TypeError):
            return
        if mode not in {"paint", "spray"} or mode == self.process_mode:
            return
        self.process_mode = mode
        self._publish_plan_status("invalidated", reason="PROCESS_MODE_CHANGED")
        self._publish_fill_preview(())
        self._publish_marker_clear(self.eoat_segment_frame or WORLD_FRAME)

    def _publish_eoat_segments(
        self, strokes, normal_world, fallback_tangent, path_id, source
    ):
        if not getattr(self, "publish_eoat_segments", True):
            self._publish_plan_status(
                "rejected", reason="SEGMENT_PUBLISHING_DISABLED"
            )
            return None
        context = self._segment_context()
        if context is None:
            return None
        work_area_id, plane_generation_id = context
        transform = self._segment_frame_transform()
        if transform is None:
            self._publish_plan_status(
                "rejected", reason="SEGMENT_FINAL_FRAME_TF_UNAVAILABLE"
            )
            return None
        frame_id, rot, trans = transform
        normal = rot @ np.asarray(normal_world, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12

        def _tf_point(point):
            return rot @ np.asarray(point, dtype=float) + trans

        def _tf_tangent(tangent):
            out = rot @ np.asarray(tangent, dtype=float)
            out = out - normal * float(np.dot(out, normal))
            if float(np.linalg.norm(out)) < 1e-8:
                out = rot @ np.asarray(fallback_tangent, dtype=float)
                out = out - normal * float(np.dot(out, normal))
            out /= np.linalg.norm(out) + 1e-12
            return out

        valid_strokes = [s for s in strokes if s]
        if not valid_strokes:
            self._publish_plan_status("rejected", reason="EMPTY_PATH")
            return None
        rows = []
        for idx, stroke in enumerate(valid_strokes):
            tangent_world = self._stroke_tangent(stroke, fallback_tangent)
            tangent = _tf_tangent(tangent_world)
            start = _tf_point(stroke[0])
            end = _tf_point(stroke[-1])
            if idx == 0:
                rows.append(
                    self._segment_row(
                        "APPROACH_PRECONTACT",
                        start,
                        normal,
                        tangent,
                        0.0,
                        self.precontact_clearance_m,
                        self.approach_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row(
                        "CONTACT_SEARCH",
                        start,
                        normal,
                        tangent,
                        0.0,
                        self.precontact_clearance_m,
                        self.contact_search_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row("RAMP_UP", start, normal, tangent, self.default_paint_force_n, 0.0, 0.0)
                )
            for point in stroke:
                rows.append(
                    self._segment_row(
                        "PAINT",
                        _tf_point(point),
                        normal,
                        tangent,
                        self.default_paint_force_n,
                        0.0,
                        self.paint_speed_mps,
                    )
                )
            if idx < len(valid_strokes) - 1:
                next_stroke = valid_strokes[idx + 1]
                next_tangent = _tf_tangent(
                    self._stroke_tangent(next_stroke, fallback_tangent))
                next_start = _tf_point(next_stroke[0])
                rows.append(self._segment_row("RAMP_DOWN", end, normal, tangent, 0.0, 0.0, 0.0))
                rows.append(
                    self._segment_row(
                        "RETRACT",
                        end,
                        normal,
                        tangent,
                        0.0,
                        self.travel_clearance_m,
                        self.retract_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row(
                        "TRAVEL",
                        next_start,
                        normal,
                        next_tangent,
                        0.0,
                        self.travel_clearance_m,
                        self.travel_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row(
                        "APPROACH_PRECONTACT",
                        next_start,
                        normal,
                        next_tangent,
                        0.0,
                        self.precontact_clearance_m,
                        self.approach_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row(
                        "CONTACT_SEARCH",
                        next_start,
                        normal,
                        next_tangent,
                        0.0,
                        self.precontact_clearance_m,
                        self.contact_search_speed_mps,
                    )
                )
                rows.append(
                    self._segment_row(
                        "RAMP_UP",
                        next_start,
                        normal,
                        next_tangent,
                        self.default_paint_force_n,
                        0.0,
                        0.0,
                    )
                )
            else:
                rows.append(self._segment_row("RAMP_DOWN", end, normal, tangent, 0.0, 0.0, 0.0))
                rows.append(
                    self._segment_row(
                        "FINAL_RETRACT",
                        end,
                        normal,
                        tangent,
                        0.0,
                        self.final_retreat_offset_m,
                        self.retract_speed_mps,
                    )
                )

        spray = getattr(self, "process_mode", "paint") == "spray"
        if spray:
            try:
                rows = make_spray_rows(valid_strokes, normal, fallback_tangent,
                    _tf_point, _tf_tangent, self._segment_row,
                    self.paint_speed_mps, self.travel_speed_mps)
            except ValueError as exc:
                self._publish_plan_status("rejected", reason=str(exc))
                return None
        payload = attach_plan_hash(
            {
                "version": SEGMENT_SCHEMA_VERSION,
                "process_mode": "spray" if spray else "paint",
                "frame_id": frame_id,
                "path_id": str(path_id),
                "plan_hash": "",
                "work_area_id": work_area_id,
                "plane_generation_id": plane_generation_id,
                "point_semantics": "surface_point",
                "contact_geometry_offset_m": float(
                    self.contact_geometry_offset_m
                ),
                "precontact_clearance_m": float(0.5 if spray else self.precontact_clearance_m),
                "travel_clearance_m": float(0.5 if spray else self.travel_clearance_m),
                "safety_approach_offset_m": float(0.5 if spray else self.safety_approach_offset_m),
                "final_retreat_offset_m": float(0.5 if spray else self.final_retreat_offset_m),
                "contact_search_max_distance_m": float(
                    self.contact_search_max_distance_m
                ),
                "contact_search_timeout_s": float(
                    self.contact_search_timeout_s
                ),
                "normal_axis": "surface_z",
                "tcp_normal_axis": "+y",
                "tangent_semantics": "paint_motion_direction",
                "preserve_orientation_continuity": True,
                "rows": rows,
                "source": {
                    **dict(source),
                    "roller_usable_length_m": float(self.roller_length_m),
                    "real_painting_enabled": bool(
                        getattr(self, "real_painting_enabled", False)
                    ),
                    "dry_run": bool(getattr(self, "dry_run", True)),
                },
            }
        )
        canonical_payload = canonical_segment_json(
            payload, exclude_plan_hash=False
        )
        try:
            path = parse_segment_path(
                canonical_payload,
                default_contact_offset_m=self.contact_geometry_offset_m,
                max_force_n=max(20.0, self.default_paint_force_n),
                minimum_clearance_m=self.travel_clearance_m,
                allow_legacy=False,
            )
            if getattr(self, "real_painting_enabled", False):
                validate_segment_path_for_real_execution(
                    path,
                    expected_path_id=str(path_id),
                    expected_work_area_id=work_area_id,
                    expected_plane_generation_id=plane_generation_id,
                )
        except SegmentPathError as exc:
            self.get_logger().error(f"[EOAT SEGMENTS] generated path invalid: {exc}")
            self._publish_plan_status(
                "rejected", reason="GENERATED_SEGMENT_INVALID", detail=str(exc)
            )
            return None

        msg = String()
        msg.data = canonical_payload
        self.segment_pub.publish(msg)
        self._publish_plan_status(
            "generated",
            path_id=path.path_id,
            plan_hash=path.plan_hash,
            work_area_id=path.work_area_id,
            plane_generation_id=path.plane_generation_id,
            target_force_n=0.0 if spray else float(self.default_paint_force_n),
            process_mode="spray" if spray else "paint",
            row_count=len(path.rows),
        )
        self.get_logger().info(
            "[EOAT SEGMENTS] published %d rows from %d paint strokes "
            "(frame=%s, force=%.2fN, hash=%s)"
            % (
                len(rows),
                len(valid_strokes),
                frame_id,
                self.default_paint_force_n,
                path.plan_hash[:8],
            )
        )
        return path

    @staticmethod
    def _segment_row(mode, point, normal, tangent, force_n, offset_m, speed_mps):
        return {
            "mode": str(mode),
            "x": float(point[0]),
            "y": float(point[1]),
            "z": float(point[2]),
            "nx": float(normal[0]),
            "ny": float(normal[1]),
            "nz": float(normal[2]),
            "tx": float(tangent[0]),
            "ty": float(tangent[1]),
            "tz": float(tangent[2]),
            "force_n": float(force_n),
            "offset_m": float(offset_m),
            "speed_mps": float(speed_mps),
        }

    @staticmethod
    def _marker_color(mode):
        colors = {
            "PAINT": (0.10, 0.90, 0.25, 1.0),
            "SPRAY": (0.10, 0.80, 1.0, 1.0),
            "SPRAY_TRAVEL": (0.65, 0.70, 0.75, 1.0),
            "SPRAY_APPROACH": (1.0, 0.82, 0.10, 1.0),
            "SPRAY_FINISH": (0.65, 0.70, 0.75, 1.0),
            "APPROACH_PRECONTACT": (1.00, 0.82, 0.10, 1.0),
            "CONTACT_SEARCH": (1.00, 0.35, 0.05, 1.0),
            "RAMP_UP": (0.90, 0.20, 0.90, 1.0),
            "RAMP_DOWN": (0.55, 0.20, 0.85, 1.0),
            "RETRACT": (0.15, 0.55, 1.00, 1.0),
            "TRAVEL": (0.65, 0.70, 0.75, 1.0),
            "FINAL_RETRACT": (0.10, 0.35, 1.00, 1.0),
        }
        r, g, b, a = colors.get(mode, (0.80, 0.80, 0.80, 1.0))
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def _compat_pose_array_from_segment(self, path, stamp):
        """Build the legacy PoseArray from the exact final segment geometry."""

        out = PoseArray()
        out.header.stamp = stamp
        out.header.frame_id = path.frame_id
        previous_tcp_x = None
        for row in path.rows:
            if row.mode not in MOTION_MODES:
                continue
            position = segment_waypoint_position(path, row)
            rotation = rotation_from_surface_path(
                row.normal,
                row.tangent,
                previous_tcp_x=previous_tcp_x,
            )
            previous_tcp_x = rotation[:, 0].copy()
            qx, qy, qz, qw = quat_from_matrix(rotation)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            out.poses.append(pose)
        return out

    def _publish_marker_clear(self, frame_id, stamp=None):
        publisher = getattr(self, "marker_pub", None)
        if publisher is None:
            return
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame_id or WORLD_FRAME
        clear.header.stamp = stamp
        clear.ns = "painting_segment"
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        publisher.publish(marker_array)

    def _publish_segment_markers(self, path, stamp):
        """Publish mode-colored markers directly from the validated v3 rows."""

        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = path.frame_id
        clear.header.stamp = stamp
        clear.ns = "painting_segment"
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        base_ns = (
            f"painting_segment/{path.plan_hash}/"
            f"{path.plane_generation_id}"
        )
        marker_id = 0

        # Stage 1 is a real, hashed part of execution even though it is not a
        # process row.  Display it explicitly so the first commanded pose and
        # RViz preview share the same v3 root geometry.
        first_motion = next(
            (row for row in path.rows if row.mode in MOTION_MODES), None
        )
        if first_motion is not None:
            safety_row = replace(
                first_motion,
                offset_m=path.safety_approach_offset_m,
            )
            safety_position = segment_waypoint_position(path, safety_row)
            precontact_position = segment_waypoint_position(path, first_motion)
            safety_line = Marker()
            safety_line.header.frame_id = path.frame_id
            safety_line.header.stamp = stamp
            safety_line.ns = f"{base_ns}_safety_approach"
            safety_line.id = marker_id
            marker_id += 1
            safety_line.type = Marker.LINE_STRIP
            safety_line.action = Marker.ADD
            safety_line.pose.orientation.w = 1.0
            safety_line.scale.x = 0.006
            safety_line.color = ColorRGBA(
                r=1.0, g=0.85, b=0.10, a=1.0
            )
            safety_line.points = [
                Point(
                    x=float(safety_position[0]),
                    y=float(safety_position[1]),
                    z=float(safety_position[2]),
                ),
                Point(
                    x=float(precontact_position[0]),
                    y=float(precontact_position[1]),
                    z=float(precontact_position[2]),
                ),
            ]
            marker_array.markers.append(safety_line)

        for mode, grouped in groupby(path.rows, key=lambda row: row.mode):
            rows = list(grouped)
            points = [
                Point(x=position[0], y=position[1], z=position[2])
                for position in (
                    segment_waypoint_position(path, row) for row in rows
                )
            ]

            spheres = Marker()
            spheres.header.frame_id = path.frame_id
            spheres.header.stamp = stamp
            spheres.ns = f"{base_ns}_{mode.lower()}"
            spheres.id = marker_id
            marker_id += 1
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            spheres.pose.orientation.w = 1.0
            spheres.scale.x = 0.012
            spheres.scale.y = 0.012
            spheres.scale.z = 0.012
            spheres.color = self._marker_color(mode)
            spheres.points = points
            marker_array.markers.append(spheres)

            if mode in MOTION_MODES and len(points) >= 2:
                line = Marker()
                line.header.frame_id = path.frame_id
                line.header.stamp = stamp
                line.ns = f"{base_ns}_{mode.lower()}"
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.004
                line.color = self._marker_color(mode)
                line.points = points
                marker_array.markers.append(line)

            if mode == "CONTACT_SEARCH":
                try:
                    max_distance_m = float(
                        (path.raw_payload or {}).get(
                            "contact_search_max_distance_m", 0.0
                        )
                    )
                except (TypeError, ValueError):
                    max_distance_m = 0.0
                if not math.isfinite(max_distance_m) or max_distance_m <= 0.0:
                    max_distance_m = max(row.offset_m for row in rows)
                for row, start in zip(rows, points):
                    normal = np.asarray(row.normal, dtype=float)
                    start_position = np.array(
                        [start.x, start.y, start.z], dtype=float
                    )
                    search_limit = start_position - normal * max_distance_m
                    arrow = Marker()
                    arrow.header.frame_id = path.frame_id
                    arrow.header.stamp = stamp
                    arrow.ns = f"{base_ns}_contact_search"
                    arrow.id = marker_id
                    marker_id += 1
                    arrow.type = Marker.ARROW
                    arrow.action = Marker.ADD
                    arrow.pose.orientation.w = 1.0
                    arrow.scale.x = 0.004
                    arrow.scale.y = 0.008
                    arrow.scale.z = 0.010
                    arrow.color = self._marker_color(mode)
                    arrow.points = [
                        start,
                        Point(
                            x=float(search_limit[0]),
                            y=float(search_limit[1]),
                            z=float(search_limit[2]),
                        ),
                    ]
                    marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    @staticmethod
    def _pose_array_corners_world(msg, T_wc, source_frame):
        if msg is None or len(msg.poses) < 4:
            return None
        if (msg.header.frame_id or source_frame) != source_frame:
            return None
        pts = []
        for pose in msg.poses[:4]:
            p = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                1.0,
            ])
            pts.append((T_wc @ p)[:3])
        return np.asarray(pts, dtype=float)

    def _bilinear_point(self, corners, u, v):
        # corners: TL, TR, BR, BL. wall_front pixel: u right, v down.
        su = float(u) / max(self.view_w - 1, 1)
        sv = float(v) / max(self.view_h - 1, 1)
        return bilinear_quad_point(corners, su, sv)

    @staticmethod
    def _surface_tangent_at(points, idx, fallback):
        pts = [np.asarray(p, dtype=float) for p in points]
        if len(pts) >= 2:
            if idx <= 0:
                tangent = pts[1] - pts[0]
            elif idx >= len(pts) - 1:
                tangent = pts[-1] - pts[-2]
            else:
                tangent = pts[idx + 1] - pts[idx - 1]
            if float(np.linalg.norm(tangent)) > 1e-8:
                return tangent
        return np.asarray(fallback, dtype=float)

    @staticmethod
    def _project_points_to_plane(points, plane_point, normal):
        pts = np.asarray(points, dtype=float)
        n = np.asarray(normal, dtype=float)
        n /= np.linalg.norm(n) + 1e-12
        signed = (pts - np.asarray(plane_point, dtype=float)) @ n
        return pts - signed[:, None] * n


def main(args=None):
    rclpy.init(args=args)
    node = SketchToWaypointsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
