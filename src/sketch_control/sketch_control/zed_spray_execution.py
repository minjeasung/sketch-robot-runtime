"""Atomic ZED work-area acceptance, independent of D405 callbacks."""
import json
import time

import numpy as np

from sketch_control.rotation_utils import quat_apply
from sketch_control.work_area_geometry import outside_quad_3d_indices


def selection_stamp_id(stamp):
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return ""
    value = sec * 1_000_000_000 + nanosec
    return str(value) if value > 0 else ""


def validate_zed_work_area(payload, generation, selection_id):
    """Validate geometry and the two independent operator event identities."""
    if not isinstance(payload, dict) or not generation or not selection_id:
        raise ValueError("current ZED target and work-area selection required")
    if (payload.get("source") != "zed" or payload.get("mode") != "work_area"
            or payload.get("accepted") is not True or payload.get("state") != "locked"):
        raise ValueError("locked ZED work area required")
    if (payload.get("plane_generation_id") != generation
            or payload.get("selection_id") != selection_id):
        raise ValueError("stale ZED work-area identity")
    if any(not isinstance(payload.get(key), str) or not payload[key].strip()
           for key in ("frame_id", "work_area_id")):
        raise ValueError("ZED frame and work-area ID required")
    try:
        point = np.asarray(payload["position"], dtype=float)
        quaternion = np.asarray(payload["orientation"], dtype=float)
        corners = np.asarray(payload["corners"], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid ZED work-area geometry") from exc
    if (point.shape != (3,) or quaternion.shape != (4,) or corners.shape != (4, 3)
            or not all(np.isfinite(v).all() for v in (point, quaternion, corners))
            or abs(np.linalg.norm(quaternion) - 1.) > .001):
        raise ValueError("non-finite or malformed ZED work area")
    normal = quat_apply(quaternion, [0., 0., 1.])
    edges = np.roll(corners, -1, axis=0) - corners
    turns = np.cross(edges, np.roll(edges, -1, axis=0)) @ normal
    if (np.min(np.linalg.norm(edges, axis=1)) <= .001
            or not (np.all(turns > 1e-8) or np.all(turns < -1e-8))
            or np.max(np.abs((corners - point) @ normal)) > .002
            or np.linalg.norm(corners.mean(axis=0) - point) > .002):
        raise ValueError("degenerate, non-convex or non-planar ZED work area")
    return point, normal, corners


class ZedSprayExecutionMixin:
    def _init_zed_spray(self):
        from geometry_msgs.msg import PoseArray
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy

        self._zed_plane_accepted = False
        self._zed_target_lock = None
        self._zed_work_area_selection_id = ""
        self._zed_pending_area = None
        self._zed_accepted_area = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._zed_target_lock_pub = self.create_publisher(
            String, "/perception/zed_target_lock", latched)
        self.create_subscription(
            String, "/perception/zed_surface_status", self.on_zed_surface_status, latched)
        self.create_subscription(
            PoseArray, "/work_area_pixels", self.on_zed_area_pixels, 10)
        self.create_subscription(
            PoseArray, "/target_selection_pixels", self.on_zed_target_selection, 10)
        self._zed_selection_generation = ""

    def _invalidate_zed_work_area(self, reason):
        self._zed_plane_accepted = False
        self._zed_pending_area = None
        self._zed_accepted_area = None
        self._current_work_area_id = ""
        self._current_plane_generation_id = ""
        self._accepted_plan_hash = ""
        self._accepted_plan_path_id = ""
        self._segment_path = None
        self.current_waypoints = []
        self._pending_surface_msg = None
        self._pending_corners_msg = None
        self.dynamic_work_area_corners = None
        self._work_area_corners_signature = None
        self._publish_execution_status("PLAN_INVALIDATED", reason)
        self._mark_scene_dirty(self)

    def _invalidate_zed_target(self, reason):
        from std_msgs.msg import String

        self._invalidate_zed_work_area(reason)
        self._zed_target_lock = None
        self._zed_work_area_selection_id = ""
        self.dynamic_surface_point = None
        self.dynamic_surface_normal = None
        self.dynamic_surface_source = "invalidated"
        self._zed_target_lock_pub.publish(String(data=json.dumps(
            dict(source="zed", accepted=False, state="invalidated", reason=reason))))

    def on_zed_target_selection(self, msg):
        if getattr(self, "process_mode", "paint") != "spray":
            return
        if self._execution_snapshot_updates_locked(self, "ZED target selection"):
            self._defer_candidate_invalidation(self, "ZED_TARGET_CHANGED_DURING_EXECUTION")
            return
        if self.executing:
            return
        generation = selection_stamp_id(msg.header.stamp)
        if generation == getattr(self, "_zed_selection_generation", ""):
            return
        self._zed_selection_generation = generation
        self._invalidate_zed_target("ZED_TARGET_SELECTION_CHANGED")
        self._multi_selected = []
        self._multi_active_id = ""
        self._multi_refined = {}

    def on_zed_area_pixels(self, msg):
        if getattr(self, "process_mode", "paint") != "spray":
            return
        if self._execution_snapshot_updates_locked(self, "ZED work-area selection"):
            self._defer_candidate_invalidation(self, "ZED_WORK_AREA_CHANGED_DURING_EXECUTION")
            return
        if self.executing:
            return
        selection_id = selection_stamp_id(msg.header.stamp)
        pending = getattr(self, "_zed_pending_area", None)
        self._invalidate_zed_work_area("ZED_WORK_AREA_SELECTION_CHANGED")
        valid = msg.header.frame_id == "wall_front" and len(msg.poses) in (2, 4, 5)
        self._zed_work_area_selection_id = selection_id if valid else ""
        if pending and pending.get("selection_id") == self._zed_work_area_selection_id:
            self._zed_pending_area = pending
            self._accept_pending_zed_area()

    def on_zed_surface_status(self, msg):
        if getattr(self, "process_mode", "paint") != "spray":
            return
        if self._execution_snapshot_updates_locked(self, "ZED surface status"):
            return
        if self.executing:
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict) or payload.get("source") != "zed":
                raise ValueError("invalid ZED source")
        except (ValueError, TypeError):
            self._invalidate_zed_work_area("ZED_STATUS_INVALID")
            self._zed_work_area_selection_id = ""
            return
        lock = getattr(self, "_zed_target_lock", None) or {}
        if payload.get("plane_generation_id") != lock.get("plane_generation_id"):
            return
        if payload.get("mode") != "work_area":
            return
        if payload.get("accepted") is not True:
            if payload.get("selection_id", "") in ("", self._zed_work_area_selection_id):
                self._invalidate_zed_work_area("ZED_WORK_AREA_INVALIDATED")
                self._zed_work_area_selection_id = ""
            return
        accepted = getattr(self, "_zed_accepted_area", None)
        if accepted and payload.get("selection_id") == accepted.get("selection_id"):
            keys = ("work_area_id", "plane_generation_id", "frame_id", "position", "orientation", "corners")
            if any(payload.get(key) != accepted.get(key) for key in keys):
                self._invalidate_zed_work_area("ZED_LOCK_GEOMETRY_CHANGED")
                self._zed_work_area_selection_id = ""
                return
        self._zed_pending_area = payload
        self._accept_pending_zed_area()

    def _accept_pending_zed_area(self):
        payload = getattr(self, "_zed_pending_area", None)
        lock = getattr(self, "_zed_target_lock", None) or {}
        if not payload:
            return
        try:
            point, normal, corners = validate_zed_work_area(
                payload, lock.get("plane_generation_id", ""),
                self._zed_work_area_selection_id)
        except ValueError:
            return
        if (payload["frame_id"] != lock.get("frame_id")
                or payload.get("target_stamp") != lock.get("stamp")):
            return
        if (np.dot(normal, np.asarray(lock["normal"])) < .999
                or outside_quad_3d_indices(corners, lock["corners"],
                                          boundary_tolerance_m=.001,
                                          plane_tolerance_m=.002)):
            return
        frame = self._canonical_world_frame(payload["frame_id"])
        if frame != "link0":
            transform = self._lookup_transform_to_base(frame, timeout_s=.2)
            if transform is None:
                return
            t, q = transform.transform.translation, transform.transform.rotation
            quat, translation = [q.x, q.y, q.z, q.w], np.array([t.x, t.y, t.z])
            if (not np.isfinite(quat + list(translation)).all()
                    or abs(np.linalg.norm(quat) - 1.) > .001):
                return
            point = quat_apply(quat, point) + translation
            normal = quat_apply(quat, normal)
            corners = quat_apply(quat, corners) + translation
        if (self._zed_plane_accepted and self._current_work_area_id == payload["work_area_id"]
                and self._current_plane_generation_id == payload["plane_generation_id"]):
            return
        self.dynamic_surface_point = point
        self.dynamic_surface_normal = normal
        self.dynamic_surface_source = "zed_locked"
        self.dynamic_surface_source_time = time.monotonic()
        self._set_dynamic_work_area_corners(corners)
        self._current_work_area_id = payload["work_area_id"]
        self._current_plane_generation_id = payload["plane_generation_id"]
        self._zed_plane_accepted = True
        self._zed_accepted_area = payload
        self._d405_plane_accepted = False
        self._zed_pending_area = None
        self._mark_scene_dirty(self)
