"""
wall_projector_node — ZED RGB + 선택된 target surface → 작업영역 정면 view 생성.

입력:
  /zed/zed_node/rgb/color/rect/image  (sensor_msgs/Image, rgb8 | bgr8)
  /zed/zed_node/rgb/color/rect/camera_info       (sensor_msgs/CameraInfo)  — K 매트릭스
  /perception/target_surface            (geometry_msgs/PoseStamped)
                                         — sketch 기반 target surface
  /perception/target_surface_refined    (geometry_msgs/PoseStamped)
                                         — D405 보정 target surface
  /work_area_pixels                     optional sketch 기반 작업영역

출력:
  /perception/wall_front_view          (sensor_msgs/Image, rgb8)
  /perception/work_area_plane          (geometry_msgs/PoseStamped)
  /perception/work_area_corners        (geometry_msgs/PoseArray, TL/TR/BR/BL)

알고리즘:
  1. target/work area plane parameters (centroid, normal — zed_left_camera_frame)
     normal = quaternion 이 +Z 를 회전시킨 vector
  2. wall plane 위 right/up axes 정의 (camera +Y down 기준 horizontal/vertical)
  3. 4 꼭짓점 (centroid ± W/2 right ± H/2 up) — 작업 영역
  4. K 로 카메라 픽셀 projection (u = fx·X/Z + cx, v = fy·Y/Z + cy)
  5. cv2.getPerspectiveTransform + warpPerspective → 정면 view
  6. /perception/wall_front_view 발행
"""
import json
import uuid

import numpy as np
import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener, TransformException


# ---- 파라미터 ----------------------------------------------------------------
INPUT_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
INPUT_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"
INPUT_WALL_TOPIC = "/perception/wall_plane"
INPUT_TARGET_TOPIC = "/perception/target_surface"
INPUT_REFINED_TARGET_TOPIC = "/perception/target_surface_refined"
INPUT_REFINED_WORK_AREA_TOPIC = "/perception/work_area_plane_refined"
WORK_AREA_PIXELS_TOPIC = "/work_area_pixels"
OUTPUT_TOPIC = "/perception/wall_front_view"
WORK_AREA_TOPIC = "/perception/work_area_plane"
WORK_AREA_CORNERS_TOPIC = "/perception/work_area_corners"
# 전체 wall_front 뷰가 대응하는 평면 사각형(TL/TR/BR/BL). sketch_to_waypoints 가
# 픽셀->3D 매핑에 쓴다 (work_area 는 그 안의 부분영역).
WALL_FRONT_EXTENT_TOPIC = "/perception/wall_front_extent"
WORK_AREA_STATE_TOPIC = "/painting_system/work_area_state"
FILL_PREVIEW_TOPIC = "/painting_system/fill_preview_pixels"

# D405 (eye-in-hand) color stream — used to build an undistorted frontal view of
# the target plane that the operator draws the work area on.
D405_IMAGE_TOPIC = "/d405/d405/color/image_raw"
D405_INFO_TOPIC = "/d405/d405/color/camera_info"
D405_OPTICAL_FRAME = "d405_color_optical_frame"

OUTPUT_LONG_EDGE = 900   # 가상 정면 view 의 긴 변 픽셀
OUTPUT_MIN_EDGE = 360
OUTPUT_MAX_EDGE = 1200

WALL_RECT_W = 0.5  # 벽 평면 위 작업 영역 (m)
WALL_RECT_H = 0.4
DEFAULT_TARGET_VIEW_W = 0.5
DEFAULT_TARGET_VIEW_H = 0.4

# Fill 미리보기(작업영역 안 세로 왕복). moveit_executor 의 값과 일치시킬 것.
FILL_TRIGGER_TOPIC = "/fill_work_area"

YELLOW_HSV_LOWER = np.array([18, 80, 80], dtype=np.uint8)
YELLOW_HSV_UPPER = np.array([45, 255, 255], dtype=np.uint8)
MIN_YELLOW_AREA_PX = 800
MIN_YELLOW_SIDE_PX = 20

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _quat_z_axis(q):
    """quaternion (x,y,z,w) 가 local +Z 를 회전시킨 vector. wall_detector 의 convention."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    ], dtype=float)


def _normal_to_quaternion(normal: np.ndarray):
    n = np.asarray(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-12)
    z = np.array([0.0, 0.0, 1.0])
    dot = float(np.dot(z, n))
    if dot > 0.9999:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -0.9999:
        return (1.0, 0.0, 0.0, 0.0)
    axis = np.cross(z, n)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    angle = np.arccos(dot)
    s = np.sin(angle / 2.0)
    return (float(axis[0] * s), float(axis[1] * s),
            float(axis[2] * s), float(np.cos(angle / 2.0)))


def _order_quad_points(pts):
    """Return points as TL, TR, BR, BL in image coordinates."""
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(d)]
    ordered[3] = pts[np.argmax(d)]
    return ordered


def _detect_yellow_quad(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < MIN_YELLOW_AREA_PX:
        return None, area

    rect = cv2.minAreaRect(contour)
    w, h = rect[1]
    if min(w, h) < MIN_YELLOW_SIDE_PX:
        return None, area
    return _order_quad_points(cv2.boxPoints(rect)), area


def _intersect_pixel_with_plane(pixel, K, plane_point, normal):
    u, v = float(pixel[0]), float(pixel[1])
    ray = np.linalg.inv(K) @ np.array([u, v, 1.0], dtype=float)
    ray = ray / (np.linalg.norm(ray) + 1e-12)
    denom = float(np.dot(ray, normal))
    if abs(denom) < 1e-9:
        return None
    t = float(np.dot(plane_point, normal)) / denom
    if t <= 0.0:
        return None
    return ray * t


def _quat_to_R(q):
    """quaternion (x, y, z, w) → 3x3 rotation matrix."""
    x, y, z, w = [float(v) for v in q]
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _ray_plane_intersection(origin, direction, plane_point, normal):
    """General ray ∩ plane (ray may start away from the frame origin)."""
    direction = np.asarray(direction, dtype=float)
    denom = float(np.dot(direction, normal))
    if abs(denom) < 1e-9:
        return None
    t = float(np.dot(np.asarray(plane_point, dtype=float) - origin, normal)) / denom
    if t <= 0.0:
        return None
    return np.asarray(origin, dtype=float) + t * direction


def _bilinear_quad(corners, su, sv):
    """Bilinear interpolation over a quad ordered TL, TR, BR, BL.

    su, sv ∈ [0, 1] with su to the right and sv downward — matches
    sketch_to_waypoints._bilinear_point so work-area and path snapping agree.
    """
    tl, tr, br, bl = [np.asarray(c, dtype=float) for c in corners[:4]]
    top = tl + (tr - tl) * su
    bottom = bl + (br - bl) * su
    return top + (bottom - top) * sv


def _plane_axes(normal):
    camera_up_ref = np.array([0.0, -1.0, 0.0])
    if abs(float(np.dot(camera_up_ref, normal))) > 0.99:
        camera_up_ref = np.array([1.0, 0.0, 0.0])
    right = np.cross(camera_up_ref, normal)
    right /= np.linalg.norm(right) + 1e-12
    up = np.cross(normal, right)
    up /= np.linalg.norm(up) + 1e-12
    return right, up


def _project_points(points_3d, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    pts = np.asarray(points_3d, dtype=float)
    if np.any(pts[:, 2] <= 1e-6):
        return None
    out = np.zeros((pts.shape[0], 2), dtype=np.float32)
    out[:, 0] = fx * pts[:, 0] / pts[:, 2] + cx
    out[:, 1] = fy * pts[:, 1] / pts[:, 2] + cy
    return out


def _decode_image(msg):
    """sensor_msgs/Image (rgb8 | bgr8) → numpy HxWx3 (RGB)."""
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "rgb8":
        return arr.reshape(h, w, 3).copy()
    if msg.encoding == "bgra8":
        return cv2.cvtColor(arr.reshape(h, w, 4), cv2.COLOR_BGRA2RGB)
    if msg.encoding == "rgba8":
        return arr.reshape(h, w, 4)[:, :, :3].copy()
    if msg.encoding == "bgr8":
        return cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_BGR2RGB)
    raise ValueError(f"unsupported encoding: {msg.encoding}")


def _encode_rgb(rgb, frame_id, stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


class WallProjectorNode(Node):
    def __init__(self):
        super().__init__("wall_projector_node")

        self.create_subscription(
            Image, INPUT_IMAGE_TOPIC, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, INPUT_INFO_TOPIC, self._on_info, qos_profile_sensor_data)
        self.declare_parameter("allow_wall_fallback", False)
        self.allow_wall_fallback = bool(
            self.get_parameter("allow_wall_fallback").value)
        self.create_subscription(
            PoseStamped, INPUT_WALL_TOPIC, self._on_wall, 10)
        self.create_subscription(
            PoseStamped, INPUT_TARGET_TOPIC, self._on_target_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, INPUT_REFINED_TARGET_TOPIC,
            self._on_refined_target_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, INPUT_REFINED_WORK_AREA_TOPIC,
            self._on_refined_work_area, LATCHED_QOS)
        self.create_subscription(
            PoseArray, WORK_AREA_PIXELS_TOPIC, self._on_work_area_pixels, 10)
        self.create_subscription(
            Empty, FILL_TRIGGER_TOPIC, self._on_fill_trigger, 10)
        self.create_subscription(
            PoseArray, FILL_PREVIEW_TOPIC, self._on_fill_preview, 10)

        # D405 frontal-view source -------------------------------------------
        self.front_view_source = str(
            self.declare_parameter("front_view_source", "d405").value).lower()
        self.d405_image_topic = str(
            self.declare_parameter("d405_image_topic", D405_IMAGE_TOPIC).value)
        self.d405_info_topic = str(
            self.declare_parameter("d405_camera_info_topic", D405_INFO_TOPIC).value)
        self.d405_optical_frame = str(
            self.declare_parameter("d405_optical_frame", D405_OPTICAL_FRAME).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.d405_info_topic, self._on_d405_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.d405_image_topic, self._on_d405_image,
            qos_profile_sensor_data)

        self.front_pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.work_area_pub = self.create_publisher(
            PoseStamped, WORK_AREA_TOPIC, LATCHED_QOS)
        self.work_area_corners_pub = self.create_publisher(
            PoseArray, WORK_AREA_CORNERS_TOPIC, LATCHED_QOS)
        self.front_extent_pub = self.create_publisher(
            PoseArray, WALL_FRONT_EXTENT_TOPIC, LATCHED_QOS)
        self.work_area_state_pub = self.create_publisher(
            String, WORK_AREA_STATE_TOPIC, LATCHED_QOS)

        self.K = None
        self.d405_K = None
        self.latest_surface = None  # (centroid, normal, frame_id, source)
        self.latest_work_area_pixels = None
        self.locked_work_area = None
        self._work_area_id = ""
        self._work_area_invalidation_seq = 0
        # Phase A frontal view ↔ target-plane mapping (D405 source).
        self.front_view_extent = None   # (4,3) surface-frame TL/TR/BR/BL
        self.front_view_size = None     # (out_w, out_h) of last wall_front image
        # work area lock 시 그릴 때의 extent 를 고정한다. refine 이 평면 깊이를 바꿔도
        # 표시 크기가 변하지 않아 그린 작업영역과 일치한다.
        self._locked_extent = None
        self._locked_extent_size = None
        # Fill 미리보기는 Fill 버튼(/fill_work_area) 을 눌렀을 때만 표시.
        self._show_fill_preview = False
        self._fill_preview_strokes = []
        self._warned_behind_camera = False
        self._warned_K_missing = False
        self._yellow_warn_count = 0
        self._d405_warn_count = 0

        self.get_logger().info(
            f"wall_projector_node 시작 (front_view_source={self.front_view_source})\n"
            f"  in : {INPUT_IMAGE_TOPIC}\n"
            f"       {INPUT_INFO_TOPIC}\n"
            f"       {self.d405_image_topic} (D405 color)\n"
            f"       {INPUT_TARGET_TOPIC}"
            f" (wall fallback={'on' if self.allow_wall_fallback else 'off'})\n"
            f"       {INPUT_REFINED_TARGET_TOPIC} (D405 target refined)\n"
            f"       {INPUT_REFINED_WORK_AREA_TOPIC} (D405 refined)\n"
            f"       {WORK_AREA_PIXELS_TOPIC}\n"
            f"  out: {OUTPUT_TOPIC}  (physical aspect-ratio preserving, "
            f"D405 frontal / yellow / sketch work area)\n"
            f"       {WORK_AREA_TOPIC}, {WORK_AREA_CORNERS_TOPIC}")

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _on_d405_info(self, msg: CameraInfo):
        self.d405_K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _on_wall(self, msg: PoseStamped):
        if not self.allow_wall_fallback:
            return
        if (
            self.latest_surface is not None
            and self.latest_surface[3] in ("target", "target_refined")
        ):
            return
        self._cache_surface(msg, "wall")

    def _on_target_surface(self, msg: PoseStamped):
        self._clear_locked_work_area("target surface updated")
        self.latest_work_area_pixels = None
        self._cache_surface(msg, "target")

    def _on_refined_target_surface(self, msg: PoseStamped):
        self._clear_locked_work_area("target surface refined")
        self.latest_work_area_pixels = None
        self._cache_surface(msg, "target_refined")
        self.get_logger().info(
            "D405 refined target surface 적용 — 이후 work_area/front_view 는 "
            "보정 target 기준")

    def _on_refined_work_area(self, msg: PoseStamped):
        if (
            self.locked_work_area is not None
            and self.locked_work_area.get("d405_refined_locked", False)
        ):
            self.get_logger().info(
                "D405 refined plane 추가 갱신 무시 — work_area 는 이미 lock 됨",
                throttle_duration_sec=3.0)
            return

        frame_id = msg.header.frame_id or "zed_left_camera_frame"
        refined_point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        refined_normal = _quat_z_axis(msg.pose.orientation)
        n_norm = float(np.linalg.norm(refined_normal))
        if n_norm < 1e-6:
            return
        refined_normal /= n_norm
        self.latest_surface = (
            refined_point, refined_normal, frame_id, "d405_refined")

        if self.locked_work_area is None:
            return
        if self.locked_work_area.get("frame_id") != frame_id:
            self.get_logger().warn(
                "D405 refined frame 이 locked work area 와 달라서 skip: "
                f"{frame_id} != {self.locked_work_area.get('frame_id')}",
                throttle_duration_sec=2.0)
            return

        corners = np.asarray(
            self.locked_work_area["corners_3d"], dtype=float)
        signed = (corners - refined_point) @ refined_normal
        corners_refined = corners - signed[:, None] * refined_normal
        self.locked_work_area["corners_3d"] = corners_refined
        self.locked_work_area["normal"] = refined_normal.copy()
        # D405 source recomputes src_pts live each frame (camera moves), so only
        # the static ZED path caches a reprojected src_pts here.
        src_updated = False
        if self.front_view_source == "zed" and self.K is not None:
            src_refined = _project_points(corners_refined, self.K)
            if (
                src_refined is not None
                and np.all(np.isfinite(src_refined))
            ):
                self.locked_work_area["src_pts"] = src_refined.astype(
                    np.float32)
                src_updated = True
        base_mode = self.locked_work_area.get("base_mode", "work_area")
        self.locked_work_area["mode"] = f"locked:{base_mode}+d405_refined"
        self.locked_work_area["d405_refined_locked"] = True
        self.get_logger().info(
            "work_area lock 을 D405 refined plane 으로 보정 "
            f"(shift mean={float(np.mean(signed))*1000:+.1f}mm, "
            f"image_src={'updated' if src_updated else 'kept'}, locked=true)")

    def _cache_surface(self, msg: PoseStamped, source: str):
        centroid = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        normal = _quat_z_axis(msg.pose.orientation)
        n_norm = np.linalg.norm(normal)
        if n_norm < 1e-6:
            return
        normal = normal / n_norm
        self.latest_surface = (
            centroid, normal,
            msg.header.frame_id or "zed_left_camera_frame",
            source,
        )

    def _on_work_area_pixels(self, msg: PoseArray):
        frame = msg.header.frame_id or ""
        if not msg.poses:
            self._clear_locked_work_area("empty work area selection")
            return
        if frame == "wall_front":
            self._lock_work_area_from_wall_front(msg)
            return
        if frame != "zed_raw":
            self.get_logger().warn(
                f"work_area frame_id='{frame}' — zed_raw / wall_front 만 지원")
            self._clear_locked_work_area("invalid work area frame")
            return
        pts = np.asarray(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=float
        )
        if (
            not np.all(np.isfinite(pts))
            or float(np.ptp(pts[:, 0])) < MIN_YELLOW_SIDE_PX
            or float(np.ptp(pts[:, 1])) < MIN_YELLOW_SIDE_PX
        ):
            self.get_logger().warn("work area sketch 너무 작거나 유효하지 않음")
            self._clear_locked_work_area("invalid work area selection")
            return
        self.latest_work_area_pixels = msg
        self._clear_locked_work_area("new work area sketch")
        self._begin_work_area_selection(frame, msg.header.stamp)
        self.get_logger().info(
            f"work_area sketch 수신: {len(msg.poses)} px")

    def _clear_locked_work_area(self, reason: str):
        if self.locked_work_area is not None:
            self.get_logger().info(f"work_area lock 해제: {reason}")
        self.locked_work_area = None
        self._locked_extent = None
        self._locked_extent_size = None
        self._show_fill_preview = False
        self._fill_preview_strokes = []
        if self._work_area_id:
            self._work_area_id = ""
            self._work_area_invalidation_seq += 1
            self._publish_work_area_state(False, reason)

    def _begin_work_area_selection(self, source_frame, stamp):
        """Assign a new identity even when the selected pixels are identical."""
        self._work_area_invalidation_seq += 1
        self._work_area_id = f"wa-{uuid.uuid4().hex}"
        self._publish_work_area_state(
            True,
            "selected_pending_d405",
            source_frame=source_frame,
            stamp=stamp,
        )

    def _publish_work_area_state(
        self, selected, state, *, source_frame="", stamp=None
    ):
        stamp_ns = 0
        if stamp is not None:
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        payload = {
            "selected": bool(selected),
            "state": str(state),
            "work_area_id": self._work_area_id if selected else "",
            "plane_generation_id": "",
            "invalidation_seq": int(self._work_area_invalidation_seq),
            "source_frame": str(source_frame),
            "selection_stamp_ns": int(stamp_ns),
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
        self.work_area_state_pub.publish(message)

    def _on_fill_trigger(self, msg: Empty):
        del msg
        # Wait for sketch_to_waypoints to publish the exact backend result.
        # Keeping an old/local preview across a new request is misleading.
        self._show_fill_preview = False
        self._fill_preview_strokes = []

    def _on_fill_preview(self, msg: PoseArray):
        if (msg.header.frame_id or "") != "wall_front" or not msg.poses:
            self._show_fill_preview = False
            self._fill_preview_strokes = []
            return
        grouped = {}
        order = []
        for pose in msg.poses:
            u = float(pose.position.x)
            v = float(pose.position.y)
            if not np.isfinite(u) or not np.isfinite(v):
                self._show_fill_preview = False
                self._fill_preview_strokes = []
                return
            stroke_id = int(round(float(pose.position.z)))
            if stroke_id not in grouped:
                grouped[stroke_id] = []
                order.append(stroke_id)
            grouped[stroke_id].append((u, v))
        strokes = [grouped[key] for key in order if len(grouped[key]) >= 2]
        self._fill_preview_strokes = strokes
        self._show_fill_preview = bool(strokes)
        if strokes:
            self.get_logger().info(
                "[FILL] backend preview received: %d strokes" % len(strokes)
            )

    # ------------------------------------------------------------------
    # D405 frontal view (front_view_source == "d405")
    # ------------------------------------------------------------------
    def _d405_warn(self, text: str):
        self._d405_warn_count += 1
        if self._d405_warn_count % 30 == 1:
            self.get_logger().warn(f"[D405 front view] {text}")

    def _lookup_surface_from_d405(self, surface_frame: str):
        """TF (R, t) that maps a point in d405_optical_frame to surface_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                surface_frame, self.d405_optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_R([q.x, q.y, q.z, q.w])
        return R, np.array([tr.x, tr.y, tr.z], dtype=float)

    def _on_d405_image(self, msg: Image):
        if self.front_view_source != "d405":
            return
        if self.d405_K is None:
            self._d405_warn(f"{self.d405_info_topic} 미수신")
            return
        if self.latest_surface is None:
            return
        centroid, normal, frame, _source = self.latest_surface
        pose = self._lookup_surface_from_d405(frame)
        if pose is None:
            self._d405_warn(
                f"TF {frame} <- {self.d405_optical_frame} 미수신")
            return
        R, t = pose
        try:
            rgb = _decode_image(msg)
        except Exception as e:
            self.get_logger().warn(f"D405 image decode 실패: {e}")
            return

        # 항상 전체 D405 정면 뷰를 유지한다(크롭하지 않음). work area 가 lock 되면
        # 그 사각형만 오버레이로 표시하고, contact/fill 은 그 영역에서만 일어난다.
        self._d405_phase_a(rgb, R, t, centroid, normal, msg.header.stamp)

    def _d405_phase_a(self, rgb, R, t, centroid, normal, stamp):
        """Rectify the live D405 view of the target plane → wall_front canvas.

        1) ray-cast the 4 D405 image corners onto the target plane (the visible
           region, a general quad), 2) take its axis-aligned bounding box in the
           plane's right/up axes to define a TRUE rectangle on the plane
           (front_view_extent), 3) project that rectangle back into the D405 image
           and warp → frontal view. The rectangle makes the bilinear back-
           projection in _lock_work_area_from_wall_front exact.
        """
        # work area lock 후에는 그릴 때의 extent 를 고정해서 사용한다 (refine 이 평면
        # 깊이를 바꿔도 표시 크기가 변하지 않음 -> 그린 작업영역과 일치).
        if self.locked_work_area is not None and self._locked_extent is not None:
            lw, lh = self._locked_extent_size
            self._warp_overlay_publish(
                rgb, self._locked_extent, lw, lh, R, t, stamp)
            return

        h, w = rgb.shape[:2]
        img_corners = [(0.0, 0.0), (w - 1.0, 0.0), (w - 1.0, h - 1.0), (0.0, h - 1.0)]
        Kinv = np.linalg.inv(self.d405_K)
        raw_quad = []
        for (u, v) in img_corners:
            ray_cam = Kinv @ np.array([u, v, 1.0], dtype=float)
            ray_cam /= np.linalg.norm(ray_cam) + 1e-12
            ray_surf = R @ ray_cam
            p = _ray_plane_intersection(t, ray_surf, centroid, normal)
            if p is None:
                self._d405_warn(
                    "D405 시야 코너가 target 평면과 교차 안 됨 (시야 밖/기울기)")
                return
            raw_quad.append(p)
        raw_quad = np.asarray(raw_quad, dtype=float)

        right, up = _plane_axes(np.asarray(normal, dtype=float))
        origin = np.asarray(centroid, dtype=float)
        rel = raw_quad - origin
        us, vs = rel @ right, rel @ up
        # 가시영역(raw_quad)이 비스듬한 사각형이면 bounding box 는 코너에 검은 여백을
        # 남긴다. 대신 내접 사각형(2nd/3rd order statistic)을 써서 실제로 꽉 보이는
        # 영역만 wall_front 로 쓴다 -> 가장자리(특히 아래) 짤림 방지.
        us_s, vs_s = np.sort(us), np.sort(vs)
        umin, umax = float(us_s[1]), float(us_s[2])
        vmin, vmax = float(vs_s[1]), float(vs_s[2])
        # Orient the rectangle to match the D405 camera image so the rectified
        # wall_front is right-side up (not mirrored/flipped). raw_quad is in image
        # corner order TL,TR,BR,BL: its right edge is [1]-[0], its down edge [3]-[0].
        cam_right = raw_quad[1] - raw_quad[0]
        cam_down = raw_quad[3] - raw_quad[0]
        u_lo, u_hi = (umin, umax) if float(np.dot(cam_right, right)) >= 0 else (umax, umin)
        v_lo, v_hi = (vmin, vmax) if float(np.dot(cam_down, up)) >= 0 else (vmax, vmin)
        extent = np.array([
            origin + u_lo * right + v_lo * up,   # TL
            origin + u_hi * right + v_lo * up,   # TR
            origin + u_hi * right + v_hi * up,   # BR
            origin + u_lo * right + v_hi * up,   # BL
        ], dtype=float)

        out_w, out_h, _pw, _ph = self._front_view_size(extent)
        self._warp_overlay_publish(rgb, extent, out_w, out_h, R, t, stamp)

    def _warp_overlay_publish(self, rgb, extent, out_w, out_h, R, t, stamp):
        src_pts = self._project_corners_to_d405(extent, R, t)
        if src_pts is None:
            self._d405_warn("frontal rectangle 가 D405 시야 밖/뒤 — 보류")
            return
        dst = np.array([
            [0.0, 0.0], [out_w - 1.0, 0.0],
            [out_w - 1.0, out_h - 1.0], [0.0, out_h - 1.0],
        ], dtype=np.float32)
        try:
            H = cv2.getPerspectiveTransform(src_pts, dst)
            front = cv2.warpPerspective(
                rgb, H, (out_w, out_h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        except Exception as e:
            self.get_logger().warn(f"D405 frontal warp 실패: {e}")
            return

        self.front_view_extent = np.asarray(extent, dtype=float)
        self.front_view_size = (out_w, out_h)

        # work area 가 lock 되어 있으면 전체 뷰는 유지한 채 사각형만 오버레이로 표시하고
        # work_area pose/corners 를 발행한다 (크롭 없음).
        if self.locked_work_area is not None:
            self._overlay_and_publish_work_area(
                front, np.asarray(extent, dtype=float), out_w, out_h, stamp)

        self.front_pub.publish(_encode_rgb(front, "wall_front_view", stamp))
        self._publish_front_view_extent(np.asarray(extent, dtype=float), stamp)
        self._d405_warn_count = 0
        self.get_logger().info(
            f"D405 frontal view publish: {out_w}x{out_h} "
            + ("(work area 사각형 표시 중)" if self.locked_work_area is not None
               else "(여기에 작업영역을 그리세요)"),
            throttle_duration_sec=2.0)

    def _front_view_uv(self, extent):
        """extent(TL,TR,BR,BL) 기준 (origin, u_hat, width, v_hat, height)."""
        tl = np.asarray(extent[0], dtype=float)
        u_dir = np.asarray(extent[1], dtype=float) - tl
        v_dir = np.asarray(extent[3], dtype=float) - tl
        width = float(np.linalg.norm(u_dir))
        height = float(np.linalg.norm(v_dir))
        if width < 1e-6 or height < 1e-6:
            return None
        return tl, u_dir / width, width, v_dir / height, height

    def _overlay_and_publish_work_area(self, front, extent, out_w, out_h, stamp):
        lock = self.locked_work_area
        corners_3d = np.asarray(lock["corners_3d"], dtype=float)
        normal = np.asarray(lock["normal"], dtype=float)
        frame = lock["frame_id"]
        uv = self._front_view_uv(extent)
        if uv is not None:
            tl, u_hat, width, v_hat, height = uv

            def _to_px(P):
                su = float(np.dot(P - tl, u_hat)) / width
                sv = float(np.dot(P - tl, v_hat)) / height
                return [su * (out_w - 1), sv * (out_h - 1)]

            pts = np.asarray([_to_px(P) for P in corners_3d],
                             dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(front, [pts], isClosed=True,
                          color=(0, 255, 0), thickness=3)
            if self._show_fill_preview:
                self._draw_fill_preview(front)

        work_center = corners_3d.mean(axis=0)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame
        pose.pose.position.x = float(work_center[0])
        pose.pose.position.y = float(work_center[1])
        pose.pose.position.z = float(work_center[2])
        qx, qy, qz, qw = _normal_to_quaternion(normal)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.work_area_pub.publish(pose)
        self._publish_corners(corners_3d, frame, stamp)

    def _draw_fill_preview(self, front):
        """Draw the exact pixel strokes emitted by the backend generator."""

        previous_end = None
        for stroke in self._fill_preview_strokes:
            points = [
                (int(round(point[0])), int(round(point[1])))
                for point in stroke
            ]
            if previous_end is not None:
                cv2.line(front, previous_end, points[0], (160, 160, 160), 1)
            for first, second in zip(points, points[1:]):
                cv2.line(front, first, second, (245, 158, 11), 2)
            previous_end = points[-1]

    def _publish_front_view_extent(self, extent, stamp):
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = (
            self.latest_surface[2] if self.latest_surface else "")
        for p in extent:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.front_extent_pub.publish(pa)

    def _d405_phase_b(self, rgb, R, t, stamp):
        """After work area is locked, crop the D405 view to the work area."""
        lock = self.locked_work_area
        corners_3d = np.asarray(lock["corners_3d"], dtype=float)
        normal = np.asarray(lock["normal"], dtype=float)
        frame = lock["frame_id"]
        src_pts = self._project_corners_to_d405(corners_3d, R, t)
        if src_pts is None:
            self._d405_warn("work area corner 가 D405 시야 밖/뒤 — crop 보류")
            return
        out_w, out_h, physical_w, physical_h = self._front_view_size(corners_3d)
        dst = np.array([
            [0.0, 0.0], [out_w - 1.0, 0.0],
            [out_w - 1.0, out_h - 1.0], [0.0, out_h - 1.0],
        ], dtype=np.float32)
        try:
            H = cv2.getPerspectiveTransform(src_pts, dst)
            front = cv2.warpPerspective(
                rgb, H, (out_w, out_h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        except Exception as e:
            self.get_logger().warn(f"D405 work-area warp 실패: {e}")
            return

        self.front_pub.publish(_encode_rgb(front, "wall_front_view", stamp))
        work_center = corners_3d.mean(axis=0)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame
        pose.pose.position.x = float(work_center[0])
        pose.pose.position.y = float(work_center[1])
        pose.pose.position.z = float(work_center[2])
        qx, qy, qz, qw = _normal_to_quaternion(normal)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.work_area_pub.publish(pose)
        self._publish_corners(corners_3d, frame, stamp)
        self.get_logger().info(
            f"D405 work-area crop publish ({lock.get('mode')}) "
            f"{out_w}x{out_h}, physical={physical_w:.3f}x{physical_h:.3f}m",
            throttle_duration_sec=2.0)

    def _project_corners_to_d405(self, corners_3d, R, t):
        """surface-frame corners → D405 image pixels (or None if behind/invalid)."""
        rel = np.asarray(corners_3d, dtype=float) - np.asarray(t, dtype=float)
        pts_cam = rel @ R  # == (R^T @ rel^T)^T : surface → d405 optical
        if np.any(pts_cam[:, 2] <= 1e-6):
            return None
        src = _project_points(pts_cam, self.d405_K)
        if src is None or not np.all(np.isfinite(src)):
            return None
        return src.astype(np.float32)

    def _lock_work_area_from_wall_front(self, msg: PoseArray):
        """Map a work-area sketch drawn on the D405 wall_front back to 3D."""
        self._show_fill_preview = False   # 새 작업영역 -> 이전 fill 미리보기 숨김
        self._fill_preview_strokes = []
        if self.front_view_extent is None or self.front_view_size is None:
            self.get_logger().warn(
                "work_area(wall_front) 수신했지만 D405 frontal view 미준비 — "
                "EOAT 를 target 이 보이게 두고 다시 시도")
            return
        if self.latest_surface is None:
            return
        out_w, out_h = self.front_view_size
        pts = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        u0, v0 = pts.min(axis=0)
        u1, v1 = pts.max(axis=0)
        if (u1 - u0) < MIN_YELLOW_SIDE_PX or (v1 - v0) < MIN_YELLOW_SIDE_PX:
            self.get_logger().warn("work area sketch 너무 작음 — 무시")
            self._clear_locked_work_area("work area selection too small")
            return
        px_corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]  # TL,TR,BR,BL
        corners_3d = []
        for (u, v) in px_corners:
            su = float(np.clip(u / max(out_w - 1.0, 1.0), 0.0, 1.0))
            sv = float(np.clip(v / max(out_h - 1.0, 1.0), 0.0, 1.0))
            corners_3d.append(_bilinear_quad(self.front_view_extent, su, sv))
        corners_3d = np.asarray(corners_3d, dtype=float)
        _centroid, normal, frame_id, _source = self.latest_surface
        self._clear_work_area_geometry_only()
        lock = {
            "corners_3d": corners_3d,
            "normal": np.asarray(normal, dtype=float).copy(),
            "frame_id": frame_id,
            "base_mode": "wall_front_sketch",
            "mode": "locked:wall_front_sketch",
            "d405_refined_locked": False,
            "src_pts": None,
        }
        self._begin_work_area_selection("wall_front", msg.header.stamp)
        lock["work_area_id"] = self._work_area_id
        self.locked_work_area = lock
        # 그릴 때의 extent/size 고정 -> refine 으로 평면이 바뀌어도 표시 크기 일관.
        self._locked_extent = np.asarray(self.front_view_extent, dtype=float).copy()
        self._locked_extent_size = (int(out_w), int(out_h))
        center = corners_3d.mean(axis=0)
        self._publish_corners(corners_3d, frame_id, self.get_clock().now().to_msg())
        self.get_logger().info(
            "work_area lock (D405 wall_front sketch) — "
            f"center=({center[0]:+.3f},{center[1]:+.3f},{center[2]:+.3f})")

    def _clear_work_area_geometry_only(self):
        self.locked_work_area = None
        self._locked_extent = None
        self._locked_extent_size = None
        self._show_fill_preview = False
        self._fill_preview_strokes = []

    def _on_image(self, msg: Image):
        if self.front_view_source != "zed":
            return
        if self.K is None:
            if not self._warned_K_missing:
                self._warned_K_missing = True
                self.get_logger().warn(
                    f"{INPUT_INFO_TOPIC} 미수신 — projection 보류")
            return
        if self.latest_surface is None:
            return

        try:
            rgb = _decode_image(msg)
        except Exception as e:
            self.get_logger().warn(f"image decode 실패: {e}")
            return

        centroid, normal, _frame, source = self.latest_surface

        if self.locked_work_area is not None:
            lock = self.locked_work_area
            src_pts = lock["src_pts"].copy()
            corners_3d = lock["corners_3d"].copy()
            normal = lock["normal"].copy()
            _frame = lock["frame_id"]
            mode = lock["mode"]
        else:
            src_pts, corners_3d, mode = self._choose_work_area(
                rgb, centroid, normal)
            if (
                src_pts is not None
                and corners_3d is not None
                and self.latest_work_area_pixels is not None
            ):
                self.locked_work_area = {
                    "src_pts": np.asarray(src_pts, dtype=np.float32).copy(),
                    "corners_3d": np.asarray(corners_3d, dtype=float).copy(),
                    "normal": np.asarray(normal, dtype=float).copy(),
                    "frame_id": _frame,
                    "base_mode": mode,
                    "mode": f"locked:{mode}",
                    "d405_refined_locked": False,
                }
                mode = self.locked_work_area["mode"]
                self.get_logger().info(
                    f"work_area lock 설정 ({mode}) — 이후 robot occlusion 에도 "
                    "plane/corners 재검출 안 함")

        if src_pts is None or corners_3d is None:
            if source in ("target", "target_refined"):
                # target 은 선택됐지만 작업영역 sketch/yellow 가 아직 없는 상태.
                self._yellow_warn_count += 1
                if self._yellow_warn_count % 30 == 1:
                    self.get_logger().warn(
                        "work area 미지정 — ZED Raw 에서 Work Area 를 그리거나 "
                        "Work Area sketch 주변의 노란 사각형을 보조로 사용하세요")
            else:
                self._yellow_warn_count += 1
                if self._yellow_warn_count % 30 == 1:
                    self.get_logger().warn(
                        "yellow/work area 미검출 — front_view publish 보류")
            return
        self._yellow_warn_count = 0
        work_center = corners_3d.mean(axis=0)
        out_w, out_h, physical_w, physical_h = self._front_view_size(corners_3d)

        dst_pts = np.array([
            [0.0,            0.0],
            [out_w - 1.0,    0.0],
            [out_w - 1.0,    out_h - 1.0],
            [0.0,            out_h - 1.0],
        ], dtype=np.float32)

        try:
            H = cv2.getPerspectiveTransform(src_pts, dst_pts)
            front = cv2.warpPerspective(
                rgb, H, (out_w, out_h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        except Exception as e:
            self.get_logger().warn(f"warpPerspective 실패: {e}")
            return

        out = _encode_rgb(front, "wall_front_view", msg.header.stamp)
        self.front_pub.publish(out)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = _frame
        pose.pose.position.x = float(work_center[0])
        pose.pose.position.y = float(work_center[1])
        pose.pose.position.z = float(work_center[2])
        qx, qy, qz, qw = _normal_to_quaternion(normal)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.work_area_pub.publish(pose)
        self._publish_corners(corners_3d, _frame, msg.header.stamp)

        self.get_logger().info(
            f"front_view/work_area publish ({mode}, surface={source}) "
            f"view={out_w}x{out_h}, physical={physical_w:.3f}x{physical_h:.3f}m, "
            f"center=({work_center[0]:+.3f},{work_center[1]:+.3f},"
            f"{work_center[2]:+.3f})")

    def _front_view_size(self, corners_3d):
        pts = np.asarray(corners_3d, dtype=float)
        tl, tr, br, bl = pts[:4]
        physical_w = 0.5 * (
            float(np.linalg.norm(tr - tl)) +
            float(np.linalg.norm(br - bl))
        )
        physical_h = 0.5 * (
            float(np.linalg.norm(bl - tl)) +
            float(np.linalg.norm(br - tr))
        )
        if physical_w < 1e-4 or physical_h < 1e-4:
            physical_w = WALL_RECT_W
            physical_h = WALL_RECT_H

        aspect = float(np.clip(
            physical_w / max(physical_h, 1e-6),
            0.20,
            5.00,
        ))
        if aspect >= 1.0:
            out_w = OUTPUT_LONG_EDGE
            out_h = int(round(OUTPUT_LONG_EDGE / aspect))
        else:
            out_h = OUTPUT_LONG_EDGE
            out_w = int(round(OUTPUT_LONG_EDGE * aspect))

        out_w = int(np.clip(out_w, OUTPUT_MIN_EDGE, OUTPUT_MAX_EDGE))
        out_h = int(np.clip(out_h, OUTPUT_MIN_EDGE, OUTPUT_MAX_EDGE))
        # Even dimensions are friendlier for browser/video tooling and avoid
        # tiny flicker from one-pixel aspect rounding.
        out_w = max(2, int(round(out_w / 2.0) * 2))
        out_h = max(2, int(round(out_h / 2.0) * 2))
        return out_w, out_h, physical_w, physical_h

    def _choose_work_area(self, rgb, centroid, normal):
        if self.latest_work_area_pixels is not None:
            src_pts, area = _detect_yellow_quad(rgb)
            if src_pts is not None and self._yellow_matches_work_area_sketch(src_pts):
                corners = self._corners_from_pixels(src_pts, centroid, normal)
                if corners is not None:
                    return src_pts, corners, f"yellow+sketch(area={area:.0f}px)"

            result = self._work_area_from_sketch(centroid, normal)
            if result[0] is not None:
                return result[0], result[1], "sketch-fallback"

        return None, None, "none"

    def _yellow_matches_work_area_sketch(self, yellow_pts):
        msg = self.latest_work_area_pixels
        if msg is None or not msg.poses:
            return True
        pts = np.array([[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        u0, v0 = pts.min(axis=0)
        u1, v1 = pts.max(axis=0)
        margin = 30.0
        center = np.asarray(yellow_pts, dtype=float).mean(axis=0)
        return (
            u0 - margin <= center[0] <= u1 + margin
            and v0 - margin <= center[1] <= v1 + margin
        )

    def _work_area_from_sketch(self, centroid, normal):
        msg = self.latest_work_area_pixels
        pts_px = np.array([
            [p.position.x, p.position.y] for p in msg.poses
        ], dtype=float)
        u0 = float(np.clip(pts_px[:, 0].min(), 0, None))
        u1 = float(np.clip(pts_px[:, 0].max(), 0, None))
        v0 = float(np.clip(pts_px[:, 1].min(), 0, None))
        v1 = float(np.clip(pts_px[:, 1].max(), 0, None))
        if (u1 - u0) < MIN_YELLOW_SIDE_PX or (v1 - v0) < MIN_YELLOW_SIDE_PX:
            return None, None, "sketch-too-small"

        src_pts = np.array([
            [u0, v0],
            [u1, v0],
            [u1, v1],
            [u0, v1],
        ], dtype=np.float32)
        corners = self._corners_from_pixels(src_pts, centroid, normal)
        if corners is None:
            return None, None, "sketch-intersection-failed"
        return src_pts, corners, "sketch"

    def _corners_from_pixels(self, src_pts, centroid, normal):
        corners = []
        for px in src_pts:
            p = _intersect_pixel_with_plane(px, self.K, centroid, normal)
            if p is None:
                return None
            if p[2] <= 1e-6:
                if not self._warned_behind_camera:
                    self._warned_behind_camera = True
                    self.get_logger().warn(
                        "work area corner 중 Z<=0 — surface 가 카메라 뒤. skip")
                return None
            corners.append(p)
        return np.asarray(corners, dtype=float)

    def _publish_corners(self, corners_3d, frame_id, stamp):
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = frame_id
        for p in corners_3d:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.work_area_corners_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = WallProjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
