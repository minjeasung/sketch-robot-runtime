#!/usr/bin/env python3
"""Calibrate fixed ZED extrinsics using a wrist D405 and one AprilTag.

The tag is fixed in the scene.  Both cameras observe the same tag:

    T_World_ZED = T_World_link0 * T_link0_D405 * T_D405_tag * inv(T_ZED_tag)

The D405 is part of the robot model, so T_link0_D405 comes from TF.
"""

import json
import math
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException


DEFAULT_ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
DEFAULT_ZED_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"
DEFAULT_D405_IMAGE_TOPIC = "/d405/d405/color/image_raw"
DEFAULT_D405_INFO_TOPIC = "/d405/d405/color/camera_info"
DEFAULT_OUTPUT_PATH = str(
    Path.home() / "sketch_robot_ws" / "zed_d405_apriltag_calibration.json"
)


@dataclass
class TagDetection:
    stamp_sec: float
    frame_id: str
    corners_px: np.ndarray
    R_cam_tag: np.ndarray
    t_cam_tag: np.ndarray


class DualCameraAprilTagCalibrator(Node):
    def __init__(self):
        super().__init__("apriltag_dual_camera_calibrator")

        self.tag_id = int(self.declare_parameter("tag_id", 1).value)
        self.tag_size_m = float(
            self.declare_parameter("tag_size_m", 0.200).value)
        self.tag_family = str(
            self.declare_parameter("tag_family", "tag36h11").value)

        self.zed_image_topic = str(
            self.declare_parameter(
                "zed_image_topic", DEFAULT_ZED_IMAGE_TOPIC).value)
        self.zed_info_topic = str(
            self.declare_parameter(
                "zed_camera_info_topic", DEFAULT_ZED_INFO_TOPIC).value)
        self.d405_image_topic = str(
            self.declare_parameter(
                "d405_image_topic", DEFAULT_D405_IMAGE_TOPIC).value)
        self.d405_info_topic = str(
            self.declare_parameter(
                "d405_camera_info_topic", DEFAULT_D405_INFO_TOPIC).value)

        self.base_frame = str(
            self.declare_parameter("base_frame", "link0").value)
        self.world_frame = str(
            self.declare_parameter("world_frame", "World").value)
        self.d405_optical_frame = str(
            self.declare_parameter(
                "d405_optical_frame", "d405_color_optical_frame").value)
        self.zed_optical_frame = str(
            self.declare_parameter(
                "zed_optical_frame", "zed_left_camera_frame_optical").value)

        self.num_samples = int(
            self.declare_parameter("num_samples", 20).value)
        self.min_valid_samples = int(
            self.declare_parameter("min_valid_samples", 5).value)
        self.sample_period_s = float(
            self.declare_parameter("sample_period_s", 0.5).value)
        self.manual_sample = bool(
            self.declare_parameter("manual_sample", False).value)
        self.max_duration_s = float(
            self.declare_parameter("max_duration_s", 90.0).value)
        self.detection_fresh_s = float(
            self.declare_parameter("detection_fresh_s", 1.0).value)
        self.enable_outlier_rejection = bool(
            self.declare_parameter("enable_outlier_rejection", True).value)
        self.outlier_trim_ratio = float(
            self.declare_parameter("outlier_trim_ratio", 0.20).value)
        self.outlier_rotation_weight_mm_per_deg = float(
            self.declare_parameter(
                "outlier_rotation_weight_mm_per_deg", 20.0).value)
        self.output_path = str(
            self.declare_parameter("output_path", DEFAULT_OUTPUT_PATH).value)

        dictionary_id = self._dictionary_id(self.tag_family)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.zed_K = None
        self.zed_D = None
        self.d405_K = None
        self.d405_D = None
        self.latest_zed: Optional[TagDetection] = None
        self.latest_d405: Optional[TagDetection] = None
        self.last_tf_error = None

        self.create_subscription(
            CameraInfo, self.zed_info_topic, self._on_zed_info, 10)
        self.create_subscription(
            CameraInfo, self.d405_info_topic, self._on_d405_info, 10)
        self.create_subscription(
            Image, self.zed_image_topic, self._on_zed_image,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.d405_image_topic, self._on_d405_image,
            qos_profile_sensor_data)

        self.get_logger().info(
            "AprilTag dual-camera calibrator ready: "
            f"family={self.tag_family}, id={self.tag_id}, "
            f"size={self.tag_size_m:.3f}m")
        self.get_logger().info(
            f"ZED: {self.zed_image_topic} + {self.zed_info_topic}")
        self.get_logger().info(
            f"D405: {self.d405_image_topic} + {self.d405_info_topic}")

    @staticmethod
    def _dictionary_id(name: str):
        key = name.strip().lower().replace("_", "")
        mapping = {
            "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
            "apriltag36h11": cv2.aruco.DICT_APRILTAG_36h11,
            "dictapriltag36h11": cv2.aruco.DICT_APRILTAG_36h11,
            "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
        }
        if key not in mapping:
            raise ValueError(f"unsupported AprilTag family: {name}")
        return mapping[key]

    def _on_zed_info(self, msg: CameraInfo):
        self.zed_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.zed_D = np.asarray(msg.d, dtype=np.float64)

    def _on_d405_info(self, msg: CameraInfo):
        self.d405_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.d405_D = np.asarray(msg.d, dtype=np.float64)

    def _on_zed_image(self, msg: Image):
        if self.zed_K is None:
            return
        self.latest_zed = self._detect_from_image(
            msg, self.zed_K, self.zed_D, "ZED")

    def _on_d405_image(self, msg: Image):
        if self.d405_K is None:
            return
        self.latest_d405 = self._detect_from_image(
            msg, self.d405_K, self.d405_D, "D405")

    def _detect_from_image(self, msg: Image, K, D, label: str):
        try:
            gray = self._decode_gray(msg)
        except Exception as e:
            self.get_logger().warn(f"{label} image decode failed: {e}")
            return None

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        ids_flat = ids.reshape(-1)
        matches = np.where(ids_flat == self.tag_id)[0]
        if matches.size == 0:
            return None

        idx = int(matches[0])
        corners_px = np.asarray(corners[idx], dtype=np.float64).reshape(4, 2)
        pnp = self._solve_pnp(corners_px, K, D)
        if pnp is None:
            return None
        R_cam_tag, t_cam_tag = pnp
        return TagDetection(
            stamp_sec=self._stamp_to_sec(msg.header.stamp),
            frame_id=msg.header.frame_id,
            corners_px=corners_px,
            R_cam_tag=R_cam_tag,
            t_cam_tag=t_cam_tag,
        )

    def _solve_pnp(self, corners_px, K, D):
        half = self.tag_size_m / 2.0
        # OpenCV ArUco/AprilTag corner order: TL, TR, BR, BL.
        # Tag frame: +X right, +Y up, +Z out of tag plane.
        obj_pts = np.array([
            [-half, +half, 0.0],
            [+half, +half, 0.0],
            [+half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)
        dist_coeffs = D if D is not None and D.size else None
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            corners_px.astype(np.float64),
            K,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.reshape(3)

    def run(self):
        deadline = time.monotonic() + self.max_duration_s
        samples = []
        last_sample_t = 0.0
        self.get_logger().info(
            "Show the same tag to ZED and D405. Move the wrist to a few "
            "slightly different views if it is safe; this node only observes.")
        if self.manual_sample:
            self.get_logger().info(
                "Manual sampling: stop all motion, then press Enter in this "
                "terminal to capture each sample. Type 'done' or 'q' to finish "
                "early once enough samples are captured.")

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if len(samples) >= self.num_samples:
                break
            if not self._detections_ready():
                self._log_waiting_throttled()
                continue
            if self.manual_sample:
                manual_command = self._manual_sample_command()
                if manual_command == "finish":
                    if len(samples) >= self.min_valid_samples:
                        self.get_logger().info(
                            f"manual finish requested with {len(samples)} "
                            "samples")
                        break
                    self.get_logger().warn(
                        f"Need at least {self.min_valid_samples} samples before "
                        f"finishing; currently {len(samples)}")
                    continue
                if manual_command != "sample":
                    self._log_manual_waiting_throttled(len(samples))
                    continue
            elif now - last_sample_t < self.sample_period_s:
                continue

            sample = self._make_sample()
            if sample is None:
                self._log_waiting_throttled()
                continue
            sample["sample_index"] = len(samples) + 1
            samples.append(sample)
            last_sample_t = now
            t = sample["T_world_zed"][:3, 3]
            self.get_logger().info(
                f"sample {len(samples)}/{self.num_samples}: "
                f"World->ZED optical t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})")

        if not rclpy.ok():
            self.get_logger().info("calibration interrupted before completion")
            return False

        if len(samples) < self.min_valid_samples:
            self.get_logger().error(
                f"Only {len(samples)} valid samples; need >= "
                f"{self.min_valid_samples}")
            return False

        inlier_samples, rejected_samples, raw_metrics = (
            self._reject_outlier_samples(samples))
        if len(inlier_samples) < self.min_valid_samples:
            self.get_logger().error(
                f"Only {len(inlier_samples)} inlier samples after outlier "
                f"rejection; need >= {self.min_valid_samples}")
            return False

        T_world_zed = _average_transforms(
            [s["T_world_zed"] for s in inlier_samples])
        T_base_zed = _average_transforms(
            [s["T_base_zed"] for s in inlier_samples])
        self._print_and_save(
            inlier_samples,
            T_world_zed,
            T_base_zed,
            raw_samples=samples,
            rejected_samples=rejected_samples,
            raw_metrics=raw_metrics,
        )
        return True

    def _detections_ready(self):
        if self.latest_zed is None or self.latest_d405 is None:
            return False
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        return (
            abs(now_ros - self.latest_zed.stamp_sec) <= self.detection_fresh_s
            and abs(now_ros - self.latest_d405.stamp_sec) <= self.detection_fresh_s
        )

    def _make_sample(self):
        try:
            T_base_d405 = self._lookup_T(
                self.base_frame, self.d405_optical_frame)
            T_world_base = self._lookup_T(self.world_frame, self.base_frame)
        except TransformException as e:
            self.last_tf_error = str(e)
            self.get_logger().warn(f"TF lookup failed: {e}",
                                   throttle_duration_sec=1.0)
            return None

        T_d405_tag = _Rt_to_T(
            self.latest_d405.R_cam_tag, self.latest_d405.t_cam_tag)
        T_zed_tag = _Rt_to_T(
            self.latest_zed.R_cam_tag, self.latest_zed.t_cam_tag)
        T_base_tag = T_base_d405 @ T_d405_tag
        T_base_zed = T_base_tag @ np.linalg.inv(T_zed_tag)
        T_world_zed = T_world_base @ T_base_zed

        return {
            "T_base_d405": T_base_d405,
            "T_d405_tag": T_d405_tag,
            "T_zed_tag": T_zed_tag,
            "T_base_zed": T_base_zed,
            "T_world_zed": T_world_zed,
            "zed_frame_id": self.latest_zed.frame_id,
            "d405_frame_id": self.latest_d405.frame_id,
            "zed_corners_px": self.latest_zed.corners_px.tolist(),
            "d405_corners_px": self.latest_d405.corners_px.tolist(),
        }

    def _lookup_T(self, target_frame, source_frame):
        tf = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.3),
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = _quat_xyzw_to_R([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _reject_outlier_samples(self, samples):
        raw_T = _average_transforms([s["T_world_zed"] for s in samples])
        raw_metrics = self._sample_metrics(samples, raw_T)
        if not self.enable_outlier_rejection:
            return samples, [], raw_metrics

        trim_ratio = min(max(self.outlier_trim_ratio, 0.0), 0.49)
        max_trim = max(0, len(samples) - self.min_valid_samples)
        trim_count = min(int(math.floor(len(samples) * trim_ratio)), max_trim)
        if trim_count <= 0:
            return samples, [], raw_metrics

        by_score = sorted(raw_metrics, key=lambda m: m["score"], reverse=True)
        rejected_positions = {m["position"] for m in by_score[:trim_count]}
        inliers = [
            s for i, s in enumerate(samples)
            if i not in rejected_positions
        ]
        rejected = [
            samples[i] for i in sorted(rejected_positions)
        ]
        rejected_indices = [
            int(s.get("sample_index", i + 1))
            for i, s in enumerate(samples)
            if i in rejected_positions
        ]
        self.get_logger().info(
            "outlier rejection: "
            f"kept {len(inliers)}/{len(samples)} samples, "
            f"rejected sample indices={rejected_indices}, "
            f"trim_ratio={trim_ratio:.2f}")
        return inliers, rejected, raw_metrics

    def _sample_metrics(self, samples, T_reference):
        metrics = []
        for i, sample in enumerate(samples):
            trans_mm = float(
                np.linalg.norm(
                    sample["T_world_zed"][:3, 3] - T_reference[:3, 3])
                * 1000.0
            )
            rot_deg = _rotation_error_deg(
                sample["T_world_zed"][:3, :3],
                T_reference[:3, :3],
            )
            score = float(math.hypot(
                trans_mm,
                rot_deg * self.outlier_rotation_weight_mm_per_deg,
            ))
            metrics.append({
                "position": i,
                "sample_index": int(sample.get("sample_index", i + 1)),
                "translation_error_mm": trans_mm,
                "rotation_error_deg": float(rot_deg),
                "score": score,
            })
        return metrics

    def _print_and_save(
        self,
        samples,
        T_world_zed,
        T_base_zed,
        raw_samples=None,
        rejected_samples=None,
        raw_metrics=None,
    ):
        raw_samples = raw_samples or samples
        rejected_samples = rejected_samples or []
        q_world = _R_to_quat_xyzw(T_world_zed[:3, :3])
        q_base = _R_to_quat_xyzw(T_base_zed[:3, :3])
        t_world = T_world_zed[:3, 3]
        t_base = T_base_zed[:3, 3]

        trans_errors = [
            float(np.linalg.norm(s["T_world_zed"][:3, 3] - t_world))
            for s in samples
        ]
        rot_errors = [
            _rotation_error_deg(s["T_world_zed"][:3, :3], T_world_zed[:3, :3])
            for s in samples
        ]
        rejected_indices = [
            int(s.get("sample_index", 0)) for s in rejected_samples
        ]

        print()
        print("=" * 72)
        print("ZED extrinsic from D405 + AprilTag")
        print("=" * 72)
        print(f"samples: {len(samples)} used / {len(raw_samples)} captured")
        print(f"tag: family={self.tag_family}, id={self.tag_id}, "
              f"size={self.tag_size_m:.3f} m")
        if rejected_samples:
            print(
                "outlier rejection: "
                f"rejected {len(rejected_samples)} samples "
                f"{rejected_indices}")
        print()
        print("Use these Terminal 2 launch args:")
        print(f"  zed_x:={t_world[0]:.9f} \\")
        print(f"  zed_y:={t_world[1]:.9f} \\")
        print(f"  zed_z:={t_world[2]:.9f} \\")
        print(f"  zed_qx:={q_world[0]:.9f} \\")
        print(f"  zed_qy:={q_world[1]:.9f} \\")
        print(f"  zed_qz:={q_world[2]:.9f} \\")
        print(f"  zed_qw:={q_world[3]:.9f}")
        print()
        print("Debug, link0->ZED optical:")
        print(f"  t=({t_base[0]:+.6f}, {t_base[1]:+.6f}, {t_base[2]:+.6f})")
        print(f"  q=({q_base[0]:+.6f}, {q_base[1]:+.6f}, "
              f"{q_base[2]:+.6f}, {q_base[3]:+.6f})")
        print()
        if raw_metrics:
            raw_trans = [m["translation_error_mm"] for m in raw_metrics]
            raw_rot = [m["rotation_error_deg"] for m in raw_metrics]
            print("Raw sample spread before rejection:")
            print(f"  translation mean/max: "
                  f"{np.mean(raw_trans):.2f} / {np.max(raw_trans):.2f} mm")
            print(f"  rotation mean/max: "
                  f"{np.mean(raw_rot):.3f} / {np.max(raw_rot):.3f} deg")
            print()
        print("Sample spread after rejection:")
        print(f"  translation mean/max: "
              f"{np.mean(trans_errors) * 1000.0:.2f} / "
              f"{np.max(trans_errors) * 1000.0:.2f} mm")
        print(f"  rotation mean/max: "
              f"{np.mean(rot_errors):.3f} / {np.max(rot_errors):.3f} deg")
        print("=" * 72)

        out = {
            "method": "D405_ZED_SINGLE_APRILTAG",
            "tag": {
                "family": self.tag_family,
                "id": self.tag_id,
                "size_m": self.tag_size_m,
            },
            "sample_counts": {
                "captured": len(raw_samples),
                "used": len(samples),
                "rejected": len(rejected_samples),
            },
            "outlier_rejection": {
                "enabled": bool(self.enable_outlier_rejection),
                "trim_ratio": float(self.outlier_trim_ratio),
                "rotation_weight_mm_per_deg": float(
                    self.outlier_rotation_weight_mm_per_deg),
                "rejected_sample_indices": rejected_indices,
            },
            "frames": {
                "world_frame": self.world_frame,
                "base_frame": self.base_frame,
                "d405_optical_frame": self.d405_optical_frame,
                "zed_optical_frame": self.zed_optical_frame,
            },
            "T_world_zed_optical": {
                "translation": t_world.tolist(),
                "rotation_xyzw": list(q_world),
            },
            "T_base_zed_optical": {
                "translation": t_base.tolist(),
                "rotation_xyzw": list(q_base),
            },
            "spread": {
                "translation_mean_mm": float(np.mean(trans_errors) * 1000.0),
                "translation_max_mm": float(np.max(trans_errors) * 1000.0),
                "rotation_mean_deg": float(np.mean(rot_errors)),
                "rotation_max_deg": float(np.max(rot_errors)),
            },
            "raw_spread_before_rejection": (
                {
                    "translation_mean_mm": float(np.mean([
                        m["translation_error_mm"] for m in raw_metrics
                    ])),
                    "translation_max_mm": float(np.max([
                        m["translation_error_mm"] for m in raw_metrics
                    ])),
                    "rotation_mean_deg": float(np.mean([
                        m["rotation_error_deg"] for m in raw_metrics
                    ])),
                    "rotation_max_deg": float(np.max([
                        m["rotation_error_deg"] for m in raw_metrics
                    ])),
                }
                if raw_metrics else None
            ),
            "samples": [
                {
                    "sample_index": int(s.get("sample_index", i + 1)),
                    "T_world_zed": s["T_world_zed"].tolist(),
                    "T_base_zed": s["T_base_zed"].tolist(),
                    "T_d405_tag": s["T_d405_tag"].tolist(),
                    "T_zed_tag": s["T_zed_tag"].tolist(),
                    "zed_frame_id": s["zed_frame_id"],
                    "d405_frame_id": s["d405_frame_id"],
                    "zed_corners_px": s["zed_corners_px"],
                    "d405_corners_px": s["d405_corners_px"],
                }
                for i, s in enumerate(samples)
            ],
            "rejected_samples": [
                {
                    "sample_index": int(s.get("sample_index", i + 1)),
                    "T_world_zed": s["T_world_zed"].tolist(),
                    "T_base_zed": s["T_base_zed"].tolist(),
                }
                for i, s in enumerate(rejected_samples)
            ],
        }
        path = Path(self.output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(out, indent=2))
        self.get_logger().info(f"saved calibration JSON: {path}")

    def _log_waiting_throttled(self):
        now = time.monotonic()
        last = getattr(self, "_last_wait_log", 0.0)
        if now - last < 2.0:
            return
        self._last_wait_log = now
        missing = []
        if self.zed_K is None:
            missing.append("ZED camera_info")
        if self.d405_K is None:
            missing.append("D405 camera_info")
        if self.latest_zed is None:
            missing.append("ZED tag")
        if self.latest_d405 is None:
            missing.append("D405 tag")
        if missing:
            self.get_logger().info("waiting: " + ", ".join(missing))
        elif self.last_tf_error:
            self.get_logger().info("waiting: TF - " + self.last_tf_error)
        else:
            self.get_logger().info("waiting: fresh detections/TF")

    def _manual_sample_command(self):
        if not sys.stdin or not sys.stdin.isatty():
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        line = sys.stdin.readline().strip().lower()
        if line in ("done", "q", "quit", "finish", "end", "stop"):
            return "finish"
        if line:
            self.get_logger().warn(
                f"Unknown manual command '{line}'. Press Enter to sample, "
                "or type 'done' to finish.")
            return None
        return "sample"

    def _log_manual_waiting_throttled(self, sample_count):
        now = time.monotonic()
        last = getattr(self, "_last_manual_wait_log", 0.0)
        if now - last < 2.0:
            return
        self._last_manual_wait_log = now
        self.get_logger().info(
            f"waiting: press Enter to capture sample "
            f"{sample_count + 1}/{self.num_samples}, or type 'done' to finish")

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _decode_gray(msg: Image):
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()
        if enc in ("mono8", "8uc1"):
            row = msg.step
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, row)
            return arr[:, :w].copy()
        if enc in ("rgb8", "bgr8"):
            row = msg.step // 3
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, row, 3)
            img = arr[:, :w, :]
            code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
            return cv2.cvtColor(img, code)
        if enc in ("rgba8", "bgra8"):
            row = msg.step // 4
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, row, 4)
            img = arr[:, :w, :]
            code = cv2.COLOR_RGBA2GRAY if enc == "rgba8" else cv2.COLOR_BGRA2GRAY
            return cv2.cvtColor(img, code)
        raise ValueError(f"unsupported image encoding: {msg.encoding}")


def _Rt_to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _quat_xyzw_to_R(q):
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _R_to_quat_xyzw(R):
    R = np.asarray(R, dtype=np.float64)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q) + 1e-12
    if q[3] < 0.0:
        q = -q
    return [float(v) for v in q]


def _average_transforms(transforms):
    T_avg = np.eye(4)
    T_avg[:3, 3] = np.mean([T[:3, 3] for T in transforms], axis=0)
    R_sum = np.sum([T[:3, :3] for T in transforms], axis=0)
    U, _, Vt = np.linalg.svd(R_sum)
    R_avg = U @ Vt
    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1.0
        R_avg = U @ Vt
    T_avg[:3, :3] = R_avg
    return T_avg


def _rotation_error_deg(R_a, R_b):
    R_diff = np.asarray(R_a) @ np.asarray(R_b).T
    trace = float(np.clip(np.trace(R_diff), -1.0, 3.0))
    return float(np.degrees(
        np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))))


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraAprilTagCalibrator()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
