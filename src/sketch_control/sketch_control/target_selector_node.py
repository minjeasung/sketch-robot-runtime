"""
target_selector_node — ZED 전역 이미지 위 사용자 스케치로 작업대상 표면 선택.

입력:
  /target_selection_pixels              geometry_msgs/PoseArray, frame_id="zed_raw"
  /zed/zed_node/depth/depth_registered  sensor_msgs/Image
  /zed/zed_node/depth/camera_info       sensor_msgs/CameraInfo

출력:
  /perception/target_surface            geometry_msgs/PoseStamped

사용자가 벽/판/물체 위에 대략 동그라미/박스를 그리면, 그 stroke 를 감싸는
픽셀 영역의 depth 를 3D point 로 변환하고 RANSAC plane 을 추정한다. 이 plane 이
이후 작업영역 projection 과 경로 생성의 기준 surface 가 된다.
"""
import json
import numpy as np
from std_msgs.msg import String
from sketch_control.multi_plane_geometry import polygon_mask, extract_planes
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from sketch_control.pointcloud_utils import ransac_plane


TARGET_SELECTION_TOPIC = "/target_selection_pixels"
DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered"
CAMERA_INFO_TOPIC = "/zed/zed_node/depth/camera_info"
TARGET_SURFACE_TOPIC = "/perception/target_surface"

ROI_PADDING_PX = 16
ROI_SAMPLE_STRIDE = 4
MIN_TARGET_POINTS = 80
RANSAC_DIST = 0.015
RANSAC_ITERS = 2000


LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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


class TargetSelectorNode(Node):
    def __init__(self):
        super().__init__("target_selector_node")
        self.catalog_pub = self.create_publisher(String, "/perception/target_planes", LATCHED_QOS)
        self.K = None
        self.latest_depth = None
        self.latest_depth_header = None

        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            PoseArray, TARGET_SELECTION_TOPIC, self._on_selection, 10)

        self.pub = self.create_publisher(
            PoseStamped, TARGET_SURFACE_TOPIC, LATCHED_QOS)

        self.get_logger().info(
            f"target_selector 시작: {TARGET_SELECTION_TOPIC} + depth -> "
            f"{TARGET_SURFACE_TOPIC}")

    def _on_info(self, msg: CameraInfo):
        self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)

    def _on_depth(self, msg: Image):
        try:
            self.latest_depth = self._decode_depth(msg)
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"depth decode 실패: {e}")

    def _on_selection(self, msg: PoseArray):
        if (msg.header.frame_id or "") != "zed_raw":
            self.get_logger().warn(
                f"target selection frame_id='{msg.header.frame_id}' skip")
            return
        if not self._stamp_is_valid(msg.header.stamp):
            # The selection stamp is the end-to-end request identity used by
            # the D405 refiner and browser.  Inventing a replacement stamp here
            # would make a stale refinement indistinguishable from this request.
            self.get_logger().warn("target selection stamp invalid/zero — skip")
            return
        if self.K is None or self.latest_depth is None:
            self.get_logger().warn("CameraInfo/depth 미수신 — target selection 보류")
            return
        if not msg.poses:
            self.get_logger().warn("target selection 비어있음")
            return

        try:
            polygons = {}
            for p in msg.poses:
                polygons.setdefault(int(round(p.position.z)), []).append([p.position.x, p.position.y])
            depth = self.latest_depth
            vv, uu = np.mgrid[0:depth.shape[0]:ROI_SAMPLE_STRIDE, 0:depth.shape[1]:ROI_SAMPLE_STRIDE]
            pixels = np.column_stack((uu.ravel(), vv.ravel()))
            z = depth[vv, uu].ravel()
            valid = polygon_mask(pixels, polygons.values()) & np.isfinite(z) & (z > .15) & (z < 5.)
            pixels, z = pixels[valid], z[valid]
            points = np.column_stack(((pixels[:,0]-self.K[0,2])*z/self.K[0,0],
                                      (pixels[:,1]-self.K[1,2])*z/self.K[1,1], z))
            planes = extract_planes(points, pixels)
            generation = str(msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec)
            for index, plane in enumerate(planes):
                plane["id"] = generation + ":" + str(index+1)
            payload = dict(generation=generation, frame_id=self.latest_depth_header.frame_id,
                           image_width=depth.shape[1], image_height=depth.shape[0], planes=planes)
        except (ValueError, TypeError, RuntimeError) as exc:
            payload = dict(generation=str(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec),
                           planes=[], error=str(exc))
        out = String()
        out.data = json.dumps(payload, allow_nan=False)
        self.catalog_pub.publish(out)

    def _points_from_roi(self, u0, u1, v0, v1):
        depth = self.latest_depth
        ys = np.arange(v0, v1 + 1, ROI_SAMPLE_STRIDE, dtype=np.float32)
        xs = np.arange(u0, u1 + 1, ROI_SAMPLE_STRIDE, dtype=np.float32)
        uu, vv = np.meshgrid(xs, ys)
        z = depth[vv.astype(int), uu.astype(int)]
        valid = np.isfinite(z) & (z > 0.15) & (z < 5.0)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        return np.column_stack((x[valid], y[valid], z[valid])).astype(np.float32)

    @staticmethod
    def _ransac_plane(points):
        return ransac_plane(points, RANSAC_DIST, RANSAC_ITERS)

    @staticmethod
    def _stamp_is_valid(stamp):
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        return sec >= 0 and 0 <= nanosec < 1_000_000_000 and (
            sec > 0 or nanosec > 0
        )

    @staticmethod
    def _decode_depth(msg: Image):
        h, w = msg.height, msg.width
        if msg.encoding in ("32FC1", "32fc1"):
            row_floats = msg.step // 4
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, row_floats)
            return arr[:, :w].copy()
        if msg.encoding in ("16UC1", "mono16"):
            row_uint16 = msg.step // 2
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, row_uint16)
            return arr[:, :w].astype(np.float32) * 0.001
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
