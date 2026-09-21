"""
environment_scanner_node — ZED point cloud 정적 스캔 → 다중 평면 후보.

입력
  /zed/zed_node/point_cloud/cloud_registered  (sensor_msgs/PointCloud2)
  /perception/scan_trigger                    (std_msgs/Empty)   — 재스캔 요청

출력 (모두 입력 cloud 의 frame_id 그대로, 보통 "zed_left_camera_frame")
  /perception/planes         (geometry_msgs/PoseArray)
      pose.position    = 평면 inlier centroid
      pose.orientation = +Z 축이 plane normal 을 향하도록 회전한 quaternion
  /perception/plane_labels   (std_msgs/String, JSON)
      {"planes":[{"id":0,"type":"wall","size":[w,h],"n_inliers":N}, ...]}
  /perception/obstacles      (visualization_msgs/MarkerArray)
      기본은 비활성: 기존 ZED voxel obstacle 을 DELETEALL 로 비운다.
      publish_scene_obstacles:=true 로 명시한 경우에만 잔여 scene points 를
      voxel grid 로 발행한다.

알고리즘:
  1. 시작 시 1회 또는 /perception/scan_trigger 수신 시 ZED cloud 를 일정 시간 누적
  2. voxel_down_sample(0.02), crop (5m 이내)
  3. 반복 RANSAC (최대 5회): segment_plane(d=0.01, n=3, iter=2000)
     - inliers > 1000 점 인 평면만 인정, 그 외엔 중단
  4. 평면 normal.z 로 분류:
       |nz| < 0.3   → vertical (벽 후보)
       |nz| > 0.7   → horizontal (바닥/테이블 후보)
       그 외        → other
     수직 중 최대 → "wall", 수평 중 가장 낮은 Z(centroid.z 작음) → "floor", 나머지 → "other"
  5. plane 후보는 publish 한다.
  6. 기본 collision 은 objects.yaml 정적 물체와 로봇/EOAT 모델만 사용한다.
     ZED 잔여 point obstacle 은 보수적인 planning 을 만들 수 있어 기본 제거.

규칙:
  ~/sketch_robot_ws/perception/ransac_multiplane.py 의 segment_plane wrapping 패턴을 참고.
"""

import json
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Empty, String
from sketch_control.pointcloud_utils import ransac_plane, voxel_downsample
from sketch_control.rotation_utils import quat_apply, quat_to_matrix
from visualization_msgs.msg import Marker, MarkerArray
from sketch_control.targets import load_objects_config


# ── 입출력 토픽 ─────────────────────────────────────────────
INPUT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
TRIGGER_TOPIC = "/perception/scan_trigger"
PLANES_TOPIC = "/perception/planes"
LABELS_TOPIC = "/perception/plane_labels"
OBSTACLES_TOPIC = "/perception/obstacles"
TARGET_SURFACE_TOPIC = "/perception/target_surface"
WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane"

# ── 처리 파라미터 ────────────────────────────────────────────
VOXEL_DOWN = 0.02         # m, 입력 down-sample
CROP_MAX_DIST = 5.0       # m, 이 거리 초과 점은 폐기
RANSAC_DIST = 0.01        # m
RANSAC_N = 3
RANSAC_ITERS = 2000
MAX_PLANES = 5
MIN_PLANE_INLIERS = 1000  # 점 수 < 이 값이면 평면으로 인정 안 함
OBSTACLE_VOXEL = 0.05     # m, 잔여 점들의 voxel grid 크기
PUBLISH_SCENE_OBSTACLES = False
AUTO_RESCAN_PERIOD_S = 1.0
SCAN_ACCUMULATION_SEC = 5.0
STATIC_OBJECT_PADDING = 0.04
BASE_FRAME = "link0"
ROBOT_SELF_PADDING = 0.08
TARGET_SURFACE_EXCLUSION_DIST = 0.035
TARGET_PLANE_MATCH_MAX_DIST = 0.08
TARGET_PLANE_MATCH_MIN_ALIGN = 0.90
ROBOT_LINK_FRAMES = ["link0", "link1", "link2", "link3", "link4", "link5", "link6", "tcp"]
ROBOT_LINK_CAPSULE_RADIUS = {
    ("link0", "link1"): 0.18,
    ("link1", "link2"): 0.18,
    ("link2", "link3"): 0.16,
    ("link3", "link4"): 0.14,
    ("link4", "link5"): 0.12,
    ("link5", "link6"): 0.12,
    ("link6", "tcp"): 0.16,
}

# Attached EOAT: tcp -> AFT200 -> EOAT no-camera -> D405. CAD +Z is installed as TCP -Y.
EOAT_AXIS_LOCAL = np.array([0.0, -1.0, 0.0], dtype=float)
EOAT_ROLLER_AXIS_LOCAL = np.array([1.0, 0.0, 0.0], dtype=float)
EOAT_TIP_OFFSET = 0.2442
EOAT_SELF_RADIUS = 0.055
EOAT_ROLLER_LENGTH = 0.175
EOAT_ROLLER_RADIUS = 0.026
D405_CENTER_LOCAL = np.array([0.0, -0.06870, 0.04375], dtype=float)
D405_HALF_SIZE_LOCAL = np.array([0.021, 0.0115, 0.021], dtype=float)
STATIC_FILTER_NAMES = {
    "wall", "table", "plate", "mount_seg1", "mount_seg2", "zed_camera",
    "camera_mount_ballhead", "camera_mount_bolt",
}

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# 분류 기준
VERT_NZ_MAX = 0.3
HORIZ_NZ_MIN = 0.7


def _normal_to_quaternion(normal: np.ndarray):
    """+Z = (0,0,1) 을 normal 단위벡터로 회전시키는 quaternion (x,y,z,w)."""
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
    return (float(axis[0] * s), float(axis[1] * s), float(axis[2] * s),
            float(np.cos(angle / 2.0)))


def _plane_size_on_axes(inlier_pts: np.ndarray, normal: np.ndarray):
    """평면 inlier 점들을 평면 내 2개 직교축에 투영해 (w, h) 측정."""
    n = normal / (np.linalg.norm(normal) + 1e-12)
    # n 과 가장 평행하지 않은 기준축으로 직교벡터 잡기
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    v /= (np.linalg.norm(v) + 1e-12)
    centered = inlier_pts - inlier_pts.mean(axis=0)
    pu = centered @ u
    pv = centered @ v
    return float(pu.max() - pu.min()), float(pv.max() - pv.min())


class EnvironmentScannerNode(Node):
    def __init__(self):
        super().__init__("environment_scanner_node")

        self.scan_accumulation_sec = float(
            self.declare_parameter(
                "scan_accumulation_sec", SCAN_ACCUMULATION_SEC).value)
        self.obstacle_voxel_m = float(
            self.declare_parameter(
                "obstacle_voxel_m", OBSTACLE_VOXEL).value)
        self.auto_scan_on_start = bool(
            self.declare_parameter("auto_scan_on_start", True).value)
        self.auto_rescan = bool(
            self.declare_parameter("auto_rescan", False).value)
        self.publish_scene_obstacles = bool(
            self.declare_parameter(
                "publish_scene_obstacles", PUBLISH_SCENE_OBSTACLES).value)

        self._pending_scan = self.auto_scan_on_start
        self._collecting_scan = False
        self._scan_deadline = 0.0
        self._scan_chunks = []
        self._scan_stamp = None
        self._scene_scan = None
        self._obstacles_published_for_scene = False
        self._latest_frame_id = "zed_left_camera_frame"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_objects = self._load_static_objects()
        self.locked_target_point = None
        self.locked_target_normal = None
        self.locked_target_source = None

        self.create_subscription(
            PointCloud2, INPUT_CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            Empty, TRIGGER_TOPIC, self._on_trigger, 10)
        self.create_subscription(
            PoseStamped, TARGET_SURFACE_TOPIC, self._on_target_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, WORK_AREA_PLANE_TOPIC, self._on_work_area_plane, LATCHED_QOS)
        if self.auto_rescan:
            self.create_timer(AUTO_RESCAN_PERIOD_S, self._on_periodic_scan)

        self.planes_pub = self.create_publisher(PoseArray, PLANES_TOPIC, 10)
        self.labels_pub = self.create_publisher(String, LABELS_TOPIC, 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, OBSTACLES_TOPIC, 10)

        self.get_logger().info(
            f"environment_scanner 시작 — 정적 scene scan "
            f"(trigger: {TRIGGER_TOPIC}, accumulate={self.scan_accumulation_sec:.1f}s), "
            f"obstacle_voxel={self.obstacle_voxel_m:.3f}m, "
            f"publish_scene_obstacles={self.publish_scene_obstacles}, "
            f"auto_scan_on_start={self.auto_scan_on_start}, "
            f"auto_rescan={self.auto_rescan}, "
            f"known static filter={len(self.static_objects)}개")

    def _load_static_objects(self):
        try:
            cfg = load_objects_config()
        except Exception as e:
            self.get_logger().warn(f"objects.yaml 로드 실패 — static filter 비활성: {e}")
            return []
        objects = []
        for obj in cfg.get("objects", []):
            if not obj.get("enabled", True):
                continue
            if obj.get("name") not in STATIC_FILTER_NAMES:
                continue
            objects.append({
                "name": obj["name"],
                "position": np.asarray(obj["position"], dtype=float),
                "half": np.asarray(obj["size"], dtype=float) / 2.0
                + STATIC_OBJECT_PADDING,
            })
        return objects

    # ── trigger 처리 ────────────────────────────────────────
    def _on_trigger(self, _msg: Empty):
        self.get_logger().info("scan_trigger 수신 — 새 정적 scene scan 시작")
        self._pending_scan = True
        self._collecting_scan = False
        self._scan_chunks = []
        self._scene_scan = None
        self._obstacles_published_for_scene = False

    def _on_target_surface(self, msg: PoseStamped):
        self._lock_target_surface(msg, "target_surface", force_log=True)

    def _on_work_area_plane(self, msg: PoseStamped):
        self._lock_target_surface(msg, "work_area_plane", force_log=False)

    def _lock_target_surface(self, msg: PoseStamped, source: str, force_log: bool = False):
        if source == "work_area_plane" and self.locked_target_point is not None:
            self.get_logger().info(
                "work_area_plane 갱신 무시 — target surface 이미 lock 됨",
                throttle_duration_sec=5.0)
            return

        frame_id = msg.header.frame_id or BASE_FRAME
        point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        q_msg = msg.pose.orientation
        normal = quat_apply(
            [q_msg.x, q_msg.y, q_msg.z, q_msg.w],
            [0.0, 0.0, 1.0],
        )

        try:
            if frame_id != BASE_FRAME:
                tf = self.tf_buffer.lookup_transform(
                    BASE_FRAME, frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                q = tf.transform.rotation
                q_tf = [q.x, q.y, q.z, q.w]
                point = quat_apply(q_tf, point) + np.array([t.x, t.y, t.z])
                normal = quat_apply(q_tf, normal)
        except Exception as e:
            self.get_logger().warn(
                f"{source} TF 실패 ({BASE_FRAME}<-{frame_id}): {e}")
            return

        normal = np.asarray(normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        if (
            self.locked_target_normal is not None
            and float(np.dot(normal, self.locked_target_normal)) < 0.0
        ):
            normal = -normal

        changed = self.locked_target_point is None
        if self.locked_target_point is not None and self.locked_target_normal is not None:
            offset_new = float(np.dot(point, normal))
            offset_old = float(np.dot(self.locked_target_point, normal))
            normal_delta = 1.0 - abs(float(np.dot(normal, self.locked_target_normal)))
            changed = abs(offset_new - offset_old) > 0.01 or normal_delta > 0.01

        self.locked_target_point = point
        self.locked_target_normal = normal
        self.locked_target_source = source
        if force_log or changed:
            obstacle_note = (
                "이 평면 근처 점은 scene obstacle 에서 제외"
                if self.publish_scene_obstacles
                else "ZED scene obstacle 발행은 비활성"
            )
            self.get_logger().info(
                f"target plane lock 설정({source}): point=({point[0]:+.3f},"
                f"{point[1]:+.3f},{point[2]:+.3f}) normal=({normal[0]:+.2f},"
                f"{normal[1]:+.2f},{normal[2]:+.2f}); {obstacle_note}")
        if self.publish_scene_obstacles and self._scene_scan is not None and (
            changed or not self._obstacles_published_for_scene
        ):
            self._publish_obstacles_from_cached_scan()

    def _on_periodic_scan(self):
        """옵션. 기본은 꺼져 있으며, 명시적으로 켠 경우에만 재스캔한다."""
        if self.auto_rescan and not self._pending_scan and not self._collecting_scan:
            self._pending_scan = True

    # ── cloud 콜백 (pending 시에만 처리) ───────────────────
    def _on_cloud(self, msg: PointCloud2):
        if not self._pending_scan and not self._collecting_scan:
            return

        pts = self._cloud_to_numpy(msg)
        if pts is None or pts.shape[0] < 100:
            self.get_logger().warn("scan: 점 수 부족 — 재시도 대기")
            self._pending_scan = True
            return

        self._latest_frame_id = msg.header.frame_id or "zed_left_camera_frame"
        if self._pending_scan and not self._collecting_scan:
            self._pending_scan = False
            self._collecting_scan = True
            self._scan_chunks = []
            self._scan_deadline = time.monotonic() + max(
                0.1, self.scan_accumulation_sec)
            self._scan_stamp = msg.header.stamp
            self.get_logger().info(
                f"scan 누적 시작: frame={self._latest_frame_id}, "
                f"duration={self.scan_accumulation_sec:.1f}s")

        dist = np.linalg.norm(pts, axis=1)
        cropped = pts[dist < CROP_MAX_DIST]
        if cropped.shape[0] >= 100:
            # 프레임별로 먼저 downsample 해서 메모리와 중복점을 줄인다.
            self._scan_chunks.append(voxel_downsample(cropped, VOXEL_DOWN))

        if time.monotonic() < self._scan_deadline:
            self.get_logger().info(
                f"scan 누적 중: chunks={len(self._scan_chunks)}",
                throttle_duration_sec=1.0)
            return

        self._collecting_scan = False
        if not self._scan_chunks:
            self.get_logger().warn("scan 누적 실패 — 유효 cloud 없음, 재시도 대기")
            self._pending_scan = True
            return

        pts_accum = np.concatenate(self._scan_chunks, axis=0)
        self._scan_chunks = []
        self.get_logger().info(
            f"scan 누적 완료: {pts_accum.shape[0]} pts "
            f"from accumulated frames")
        self._process_scan_points(pts_accum, self._scan_stamp or msg.header.stamp)

    def _process_scan_points(self, pts: np.ndarray, stamp):
        self.get_logger().info(
            f"scan 시작: {pts.shape[0]} pts (frame={self._latest_frame_id})")

        # ── voxel down-sample + crop ────────────────────────
        arr = voxel_downsample(pts, VOXEL_DOWN)
        dist = np.linalg.norm(arr, axis=1)
        remaining = arr[dist < CROP_MAX_DIST]
        if remaining.shape[0] < 100:
            self.get_logger().warn("crop 후 점 수 부족 — abort")
            return
        self.get_logger().info(
            f"  preprocess: {remaining.shape[0]} pts "
            f"(voxel={VOXEL_DOWN}, crop<{CROP_MAX_DIST}m)")

        # ── 반복 RANSAC: 최대 MAX_PLANES 개 추출 ───────────
        planes = []  # list of dict {normal, centroid, inlier_pts, n_inliers}
        for i in range(MAX_PLANES):
            if remaining.shape[0] < MIN_PLANE_INLIERS:
                break
            try:
                model, inliers = ransac_plane(
                    remaining, RANSAC_DIST, RANSAC_ITERS)
            except Exception as e:
                self.get_logger().warn(f"segment_plane 실패: {e}")
                break
            if len(inliers) < MIN_PLANE_INLIERS:
                self.get_logger().info(
                    f"  plane#{i}: inliers {len(inliers)} < {MIN_PLANE_INLIERS} — 종료")
                break

            a, b, c, _d = model
            n = np.array([a, b, c], dtype=float)
            n_norm = np.linalg.norm(n)
            if n_norm < 1e-6:
                break
            n /= n_norm

            inlier_idx = np.asarray(inliers, dtype=int)
            inlier_pts = remaining[inlier_idx]
            centroid = inlier_pts.mean(axis=0)

            planes.append({
                "normal": n,
                "centroid": centroid,
                "inlier_pts": inlier_pts,
                "n_inliers": int(len(inliers)),
            })
            self.get_logger().info(
                f"  plane#{i}: n=({n[0]:+.2f},{n[1]:+.2f},{n[2]:+.2f}) "
                f"inliers={len(inliers)} centroid=({centroid[0]:+.2f},"
                f"{centroid[1]:+.2f},{centroid[2]:+.2f})")

            mask = np.ones(remaining.shape[0], dtype=bool)
            mask[inlier_idx] = False
            remaining = remaining[mask]

        # ── 평면 분류 ───────────────────────────────────────
        labels = self._classify(planes)

        # ── publish ──────────────────────────────────────────
        self._scene_scan = {
            "planes": planes,
            "labels": labels,
            "residual": remaining,
            "stamp": stamp,
            "frame_id": self._latest_frame_id,
        }
        self._obstacles_published_for_scene = False
        self._publish_planes(planes, labels, stamp)

        if not self.publish_scene_obstacles:
            self._publish_obstacles(np.empty((0, 3), dtype=float), stamp)
            self._obstacles_published_for_scene = True
            obstacle_msg = "ZED scene voxel obstacles disabled"
        elif self.locked_target_point is not None and self.locked_target_normal is not None:
            self._publish_obstacles_from_cached_scan()
            obstacle_msg = "obstacles published"
        else:
            self._publish_obstacles(np.empty((0, 3), dtype=float), stamp)
            obstacle_msg = "obstacles pending target selection"

        self.get_logger().info(
            f"scan 완료: planes={len(planes)} (labels={[l['type'] for l in labels]}) "
            f"{obstacle_msg}")

    # ── PointCloud2 → Nx3 numpy ─────────────────────────────
    def _cloud_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        try:
            raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            try:
                points = np.column_stack(
                    [raw["x"], raw["y"], raw["z"]]).astype(np.float32)
            except (TypeError, IndexError, ValueError):
                points = np.array(
                    [(p[0], p[1], p[2]) for p in raw], dtype=np.float32)
            points = points[np.isfinite(points).all(axis=1)]
            return points
        except Exception as e:
            self.get_logger().warn(f"PointCloud2 parse 실패: {e}")
            return None

    def _publish_obstacles_from_cached_scan(self):
        if not self.publish_scene_obstacles:
            if self._scene_scan is not None:
                self._publish_obstacles(
                    np.empty((0, 3), dtype=float),
                    self._scene_scan["stamp"],
                )
            self._obstacles_published_for_scene = True
            return
        if self._scene_scan is None:
            return
        planes = self._scene_scan["planes"]
        residual = self._scene_scan["residual"]
        stamp = self._scene_scan["stamp"]
        frame_id = self._scene_scan["frame_id"]

        selected_idx = self._match_selected_target_plane(planes, frame_id)
        obstacle_parts = [residual]
        for idx, plane in enumerate(planes):
            if idx == selected_idx:
                continue
            obstacle_parts.append(plane["inlier_pts"])
        obstacle_points = (
            np.concatenate(obstacle_parts, axis=0)
            if obstacle_parts else np.empty((0, 3), dtype=float)
        )
        obstacle_points, removed_static, removed_target, removed_robot = (
            self._filter_known_static_points(obstacle_points, frame_id)
        )
        voxel_centers = self._voxelize(obstacle_points, self.obstacle_voxel_m)
        self._publish_obstacles(voxel_centers, stamp)
        self._obstacles_published_for_scene = True

        selected_msg = (
            f"selected_plane=#{selected_idx}"
            if selected_idx is not None else "selected_plane=target-distance-filter"
        )
        self.get_logger().info(
            f"scene obstacle snapshot publish: {selected_msg}, "
            f"candidate_pts={obstacle_points.shape[0]} "
            f"(known static 제거={removed_static}, "
            f"target 제거={removed_target}, robot/self 제거={removed_robot}) "
            f"→ obstacle voxels={voxel_centers.shape[0]}")

    def _match_selected_target_plane(self, planes, frame_id: str):
        if (
            self.locked_target_point is None
            or self.locked_target_normal is None
            or not planes
        ):
            return None
        best_idx = None
        best_score = float("inf")
        for idx, plane in enumerate(planes):
            point = np.asarray(plane["centroid"], dtype=float)
            normal = np.asarray(plane["normal"], dtype=float)
            try:
                point_base, normal_base = self._transform_point_normal_to_base(
                    point, normal, frame_id)
            except Exception as e:
                self.get_logger().warn(
                    f"target plane match TF 실패 ({BASE_FRAME}<-{frame_id}): {e}",
                    throttle_duration_sec=2.0)
                return None

            align = abs(float(np.dot(normal_base, self.locked_target_normal)))
            plane_dist = abs(float(
                np.dot(
                    point_base - self.locked_target_point,
                    self.locked_target_normal,
                )
            ))
            if (
                align < TARGET_PLANE_MATCH_MIN_ALIGN
                or plane_dist > TARGET_PLANE_MATCH_MAX_DIST
            ):
                continue
            score = plane_dist + (1.0 - align) * 0.25
            if score < best_score:
                best_idx = idx
                best_score = score
        return best_idx

    def _transform_point_normal_to_base(self, point, normal, frame_id: str):
        point = np.asarray(point, dtype=float)
        normal = np.asarray(normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        if frame_id == BASE_FRAME:
            return point, normal
        tf = self.tf_buffer.lookup_transform(
            BASE_FRAME, frame_id, rclpy.time.Time(),
            timeout=Duration(seconds=0.5))
        t = tf.transform.translation
        q = tf.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        point_base = quat_apply(q_tf, point) + np.array(
            [t.x, t.y, t.z], dtype=float)
        normal_base = quat_apply(q_tf, normal)
        normal_base = np.asarray(normal_base, dtype=float)
        normal_base /= np.linalg.norm(normal_base) + 1e-12
        return point_base, normal_base

    def _filter_known_static_points(self, pts: np.ndarray, frame_id: str):
        """Known fixed geometry 는 dynamic obstacle 로 중복 등록하지 않는다."""
        if pts.shape[0] == 0:
            return pts, 0, 0, 0
        try:
            pts_base = self._transform_points_to_base(pts, frame_id)
        except Exception as e:
            self.get_logger().warn(
                f"known static filter TF 실패 ({BASE_FRAME}<-{frame_id}): {e}")
            return pts, 0, 0, 0

        keep = np.ones(pts.shape[0], dtype=bool)
        for obj in self._static_objects_in_base():
            rel_pts = pts_base - obj["position"]
            rel = np.abs(quat_apply(obj["q_inv"], rel_pts))
            inside = np.all(rel <= obj["half"], axis=1)
            keep &= ~inside
        removed_static = int(np.count_nonzero(~keep))

        target_keep = self._target_surface_keep_mask(pts_base)
        keep &= target_keep
        removed_after_target = int(np.count_nonzero(~keep))
        removed_target = max(0, removed_after_target - removed_static)

        robot_keep = self._robot_self_keep_mask(pts_base)
        keep &= robot_keep
        removed_total = int(np.count_nonzero(~keep))
        removed_robot = max(0, removed_total - removed_after_target)
        return pts[keep], removed_static, removed_target, removed_robot

    def _static_objects_in_base(self):
        """Return objects.yaml static boxes in the MoveIt planning frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, "World", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
            t = tf.transform.translation
            q = tf.transform.rotation
            q_world_to_base = [q.x, q.y, q.z, q.w]
            translation = np.array([t.x, t.y, t.z], dtype=float)
        except Exception:
            q_world_to_base = [0.0, 0.0, 0.0, 1.0]
            translation = np.zeros(3, dtype=float)

        q_base_to_world = [
            -q_world_to_base[0],
            -q_world_to_base[1],
            -q_world_to_base[2],
            q_world_to_base[3],
        ]
        objects = []
        for obj in self.static_objects:
            center = (
                quat_apply(q_world_to_base, obj["position"])
                + translation
            )
            objects.append({
                "name": obj["name"],
                "position": center,
                "q_inv": q_base_to_world,
                "half": obj["half"],
            })
        return objects

    def _target_surface_keep_mask(self, pts_base: np.ndarray):
        """Set Target 으로 고정된 작업대상 평면은 obstacle 중복 등록에서 제외."""
        if self.locked_target_point is None or self.locked_target_normal is None:
            return np.ones(pts_base.shape[0], dtype=bool)
        signed_dist = (
            (pts_base - self.locked_target_point) @ self.locked_target_normal
        )
        return np.abs(signed_dist) > TARGET_SURFACE_EXCLUSION_DIST

    def _robot_self_keep_mask(self, pts_base: np.ndarray):
        """ZED가 본 로봇 본체 표면을 dynamic obstacle에서 제외한다."""
        if pts_base.shape[0] == 0:
            return np.ones(0, dtype=bool)
        frames = self._lookup_robot_link_points()
        if len(frames) < 2:
            self.get_logger().warn(
                "robot/self filter TF 부족 — 로봇 자체 point 제거 skip")
            return np.ones(pts_base.shape[0], dtype=bool)

        keep = np.ones(pts_base.shape[0], dtype=bool)
        for pair, radius in ROBOT_LINK_CAPSULE_RADIUS.items():
            if pair[0] not in frames or pair[1] not in frames:
                continue
            a = frames[pair[0]]
            b = frames[pair[1]]
            d = self._distance_to_segment(pts_base, a, b)
            keep &= d > (radius + ROBOT_SELF_PADDING)
        keep &= self._eoat_self_keep_mask(pts_base)
        return keep

    def _lookup_robot_link_points(self):
        out = {}
        for frame in ROBOT_LINK_FRAMES:
            if frame == BASE_FRAME:
                out[frame] = np.zeros(3, dtype=float)
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    BASE_FRAME, frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            t = tf.transform.translation
            out[frame] = np.array([t.x, t.y, t.z], dtype=float)
        return out

    def _eoat_self_keep_mask(self, pts_base: np.ndarray):
        """AFT200+roller attached tool points are part of the robot, not obstacles."""
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, "tcp", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return np.ones(pts_base.shape[0], dtype=bool)

        t = tf.transform.translation
        q = tf.transform.rotation
        tcp = np.array([t.x, t.y, t.z], dtype=float)
        quat = [q.x, q.y, q.z, q.w]
        tool_axis = quat_apply(quat, EOAT_AXIS_LOCAL[None, :])[0]
        roller_axis = quat_apply(quat, EOAT_ROLLER_AXIS_LOCAL[None, :])[0]
        tip = tcp + tool_axis * EOAT_TIP_OFFSET

        keep = np.ones(pts_base.shape[0], dtype=bool)
        d_tool = self._distance_to_segment(pts_base, tcp, tip)
        keep &= d_tool > (EOAT_SELF_RADIUS + ROBOT_SELF_PADDING)

        a = tip - roller_axis * (EOAT_ROLLER_LENGTH / 2.0)
        b = tip + roller_axis * (EOAT_ROLLER_LENGTH / 2.0)
        d_roller = self._distance_to_segment(pts_base, a, b)
        keep &= d_roller > (EOAT_ROLLER_RADIUS + ROBOT_SELF_PADDING)

        rot = quat_to_matrix(quat)
        pts_tcp = (pts_base - tcp) @ rot
        d405_delta = np.abs(pts_tcp - D405_CENTER_LOCAL)
        d405_inside = np.all(
            d405_delta <= (D405_HALF_SIZE_LOCAL + ROBOT_SELF_PADDING), axis=1)
        keep &= ~d405_inside
        return keep

    @staticmethod
    def _distance_to_segment(points, a, b):
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-12:
            return np.linalg.norm(points - a, axis=1)
        t = np.clip(((points - a) @ ab) / denom, 0.0, 1.0)
        closest = a + t[:, None] * ab
        return np.linalg.norm(points - closest, axis=1)

    def _transform_points_to_base(self, pts: np.ndarray, frame_id: str):
        if frame_id == BASE_FRAME:
            return pts
        tf = self.tf_buffer.lookup_transform(
            BASE_FRAME, frame_id, rclpy.time.Time(),
            timeout=Duration(seconds=0.5))
        t = tf.transform.translation
        q = tf.transform.rotation
        return quat_apply([q.x, q.y, q.z, q.w], pts) + np.array(
            [t.x, t.y, t.z], dtype=float)

    # ── 평면 라벨링 ─────────────────────────────────────────
    def _classify(self, planes):
        """
        normal.z 기반 분류 + 'wall' / 'floor' 1개씩 선정.
        labels[i] = {"id":i, "type":"wall"|"floor"|"other", "size":[w,h], "n_inliers":N}
        """
        labels = []
        verts, horizs = [], []
        for i, p in enumerate(planes):
            nz = abs(float(p["normal"][2]))
            if nz < VERT_NZ_MAX:
                kind = "vertical"
                verts.append((i, p))
            elif nz > HORIZ_NZ_MIN:
                kind = "horizontal"
                horizs.append((i, p))
            else:
                kind = "other"
            w, h = _plane_size_on_axes(p["inlier_pts"], p["normal"])
            labels.append({"id": i, "type": "other",
                           "size": [round(w, 3), round(h, 3)],
                           "n_inliers": p["n_inliers"], "_kind": kind})

        # wall = 수직 중 최대 inlier
        if verts:
            best_id = max(verts, key=lambda x: x[1]["n_inliers"])[0]
            labels[best_id]["type"] = "wall"
        # floor = 수평 중 centroid.z 가장 낮은 것
        if horizs:
            best_id = min(horizs, key=lambda x: float(x[1]["centroid"][2]))[0]
            labels[best_id]["type"] = "floor"
        # 내부 키 정리
        for lbl in labels:
            lbl.pop("_kind", None)
        return labels

    # ── 잔여 점 voxel grid ─────────────────────────────────
    def _voxelize(self, pts: np.ndarray, voxel: float) -> np.ndarray:
        if pts.shape[0] == 0:
            return np.empty((0, 3), dtype=float)
        idx = np.floor(pts / voxel).astype(np.int64)
        # 고유 voxel 인덱스 추출 후 중심 좌표로 환산
        uniq = np.unique(idx, axis=0)
        return (uniq.astype(float) + 0.5) * voxel

    # ── 발행: planes (PoseArray) + labels (String) ─────────
    def _publish_planes(self, planes, labels, stamp):
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self._latest_frame_id
        for p in planes:
            pose = Pose()
            c = p["centroid"]
            pose.position.x = float(c[0])
            pose.position.y = float(c[1])
            pose.position.z = float(c[2])
            qx, qy, qz, qw = _normal_to_quaternion(p["normal"])
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pa.poses.append(pose)
        self.planes_pub.publish(pa)

        msg = String()
        msg.data = json.dumps({"planes": labels})
        self.labels_pub.publish(msg)

    # ── 발행: obstacles (MarkerArray) ──────────────────────
    def _publish_obstacles(self, voxel_centers: np.ndarray, stamp):
        ma = MarkerArray()
        # 이전 마커 클리어
        clear = Marker()
        clear.header.frame_id = self._latest_frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, c in enumerate(voxel_centers):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._latest_frame_id
            m.ns = "obstacle_voxels"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.pose.orientation.w = 1.0
            m.scale.x = self.obstacle_voxel_m
            m.scale.y = self.obstacle_voxel_m
            m.scale.z = self.obstacle_voxel_m
            m.color.r = 0.9
            m.color.g = 0.3
            m.color.b = 0.1
            m.color.a = 0.6
            ma.markers.append(m)

        self.obstacles_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentScannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
