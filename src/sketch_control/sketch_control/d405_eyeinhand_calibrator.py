#!/usr/bin/env python3
"""D405 eye-in-hand calibration (AX=XB).

Setup
-----
- AprilTag: FIXED in the world (wall, stand, etc.)
- D405:     mounted on robot wrist (already in URDF)
- Robot:    operator moves to each pose manually, presses Enter to capture

What it solves
--------------
For each pose i, two things are measured:
  R_gripper2base[i], t_gripper2base[i]  -- FK from TF (link0 → tcp)
  R_target2cam[i],  t_target2cam[i]     -- tag pose in D405 frame via solvePnP

cv2.calibrateHandEye() solves AX=XB and returns:
  R_cam2gripper, t_cam2gripper  =  T_d405_optical → tcp

Result is compared with the URDF-defined TF to show the correction needed.

Run
---
source /opt/ros/jazzy/setup.bash && source ~/sketch_robot_ws/install/setup.bash
ros2 run sketch_control d405_eyeinhand_calibrator
  # optional params:
  #   --ros-args -p tag_id:=1 -p tag_size_m:=0.200 -p num_samples:=20
"""

import json
import math
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException


DEFAULT_D405_IMAGE_TOPIC = "/d405/d405/color/image_raw"
DEFAULT_D405_INFO_TOPIC  = "/d405/d405/color/camera_info"
DEFAULT_OUTPUT_PATH = str(
    Path.home() / "sketch_robot_ws" / "d405_eyeinhand_calibration.json"
)

HAND_EYE_METHODS: List[Tuple[str, int]] = [
    ("TSAI",       cv2.CALIB_HAND_EYE_TSAI),
    ("PARK",       cv2.CALIB_HAND_EYE_PARK),
    ("HORAUD",     cv2.CALIB_HAND_EYE_HORAUD),
    ("ANDREFF",    cv2.CALIB_HAND_EYE_ANDREFF),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
]
PRIMARY_METHOD = "TSAI"


@dataclass
class Sample:
    R_gripper2base: np.ndarray   # (3,3) from FK
    t_gripper2base: np.ndarray   # (3,)  from FK
    R_target2cam:   np.ndarray   # (3,3) from PnP
    t_target2cam:   np.ndarray   # (3,)  from PnP
    corners_px:     np.ndarray   # (4,2) detected pixel corners


class D405EyeInHandCalibrator(Node):

    def __init__(self):
        super().__init__("d405_eyeinhand_calibrator")

        # --- ROS parameters ------------------------------------------------
        self.tag_id       = int(self.declare_parameter("tag_id",      1).value)
        self.tag_size_m   = float(self.declare_parameter("tag_size_m", 0.200).value)
        self.tag_family   = str(self.declare_parameter("tag_family",  "tag36h11").value)

        self.d405_image_topic = str(self.declare_parameter(
            "d405_image_topic", DEFAULT_D405_IMAGE_TOPIC).value)
        self.d405_info_topic  = str(self.declare_parameter(
            "d405_camera_info_topic", DEFAULT_D405_INFO_TOPIC).value)

        self.base_frame         = str(self.declare_parameter("base_frame",         "link0").value)
        self.tcp_frame          = str(self.declare_parameter("tcp_frame",          "tcp").value)
        self.d405_optical_frame = str(self.declare_parameter(
            "d405_optical_frame", "d405_color_optical_frame").value)

        self.num_samples      = int(self.declare_parameter("num_samples",      20).value)
        self.min_valid_samples = int(self.declare_parameter("min_valid_samples", 10).value)
        self.output_path      = str(self.declare_parameter("output_path", DEFAULT_OUTPUT_PATH).value)

        # --- ArUco/AprilTag detector ----------------------------------------
        self.detector = _make_detector(self.tag_family)

        # --- TF -------------------------------------------------------------
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ----------------------------------------------------------
        self.d405_K: Optional[np.ndarray]              = None
        self.d405_D: Optional[np.ndarray]              = None
        self.latest_detection: Optional[Tuple]         = None  # (corners, R, t)
        self._last_wait_log: float                     = 0.0

        # --- Subscriptions --------------------------------------------------
        self.create_subscription(CameraInfo, self.d405_info_topic, self._on_info, 10)
        self.create_subscription(Image, self.d405_image_topic,
                                 self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"D405 eye-in-hand calibrator | "
            f"tag={self.tag_family} id={self.tag_id} size={self.tag_size_m:.3f}m")
        self.get_logger().info(
            f"  D405 image:  {self.d405_image_topic}")
        self.get_logger().info(
            f"  D405 info:   {self.d405_info_topic}")
        self.get_logger().info(
            "  Place the AprilTag at a FIXED position visible to D405.")
        self.get_logger().info(
            "  Move robot to each pose, stop fully, then press Enter to sample. "
            "Type 'done' to finish early.")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo):
        self.d405_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.d405_D = np.asarray(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image):
        if self.d405_K is None:
            return
        try:
            gray = _decode_gray(msg)
        except Exception as e:
            self.get_logger().warn(f"D405 image decode failed: {e}",
                                   throttle_duration_sec=2.0)
            return
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            self.latest_detection = None
            return
        ids_flat = ids.reshape(-1)
        matches = np.where(ids_flat == self.tag_id)[0]
        if matches.size == 0:
            self.latest_detection = None
            return
        idx = int(matches[0])
        corners_px = np.asarray(corners[idx], dtype=np.float64).reshape(4, 2)
        pnp = _solve_pnp(corners_px, self.tag_size_m, self.d405_K, self.d405_D)
        if pnp is None:
            self.latest_detection = None
            return
        R, t = pnp
        self.latest_detection = (corners_px, R, t)

    # -----------------------------------------------------------------------
    # Main flow
    # -----------------------------------------------------------------------
    def run(self) -> bool:
        # Wait for camera_info and TF
        self.get_logger().info("Waiting for D405 camera_info and TF...")
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.d405_K is not None
                    and self.tf_buffer.can_transform(
                        self.base_frame, self.tcp_frame, rclpy.time.Time())):
                break

        missing = []
        if self.d405_K is None:
            missing.append(f"D405 CameraInfo ({self.d405_info_topic})")
        if not self.tf_buffer.can_transform(
                self.base_frame, self.tcp_frame, rclpy.time.Time()):
            missing.append(f"TF ({self.base_frame} → {self.tcp_frame})")
        if missing:
            self.get_logger().error("Missing prerequisites: " + ", ".join(missing))
            return False

        self.get_logger().info(
            f"Ready. Target: {self.num_samples} samples "
            f"(minimum {self.min_valid_samples}).")

        samples: List[Sample] = []
        while rclpy.ok() and len(samples) < self.num_samples:
            rclpy.spin_once(self, timeout_sec=0.05)
            cmd = _read_stdin_nonblock()
            if cmd == "finish":
                if len(samples) >= self.min_valid_samples:
                    self.get_logger().info(
                        f"Finished early — {len(samples)} samples captured.")
                    break
                self.get_logger().warn(
                    f"Need ≥{self.min_valid_samples} samples; have {len(samples)}.")
                continue
            if cmd != "sample":
                self._log_waiting(len(samples))
                continue

            sample = self._capture()
            if sample is None:
                self.get_logger().warn(
                    "Capture failed — no tag detected or TF unavailable. "
                    "Reposition and press Enter again.")
                continue

            samples.append(sample)
            t = sample.t_target2cam
            self.get_logger().info(
                f"[{len(samples):2d}/{self.num_samples}] "
                f"tag in D405: t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m  "
                f"|t|={np.linalg.norm(t):.3f} m")

        if not rclpy.ok():
            self.get_logger().info("Interrupted before completion.")
            return False

        if len(samples) < self.min_valid_samples:
            self.get_logger().error(
                f"Only {len(samples)} valid samples; need ≥{self.min_valid_samples}. "
                "Aborting.")
            return False

        return self._calibrate_and_save(samples)

    def _capture(self) -> Optional[Sample]:
        # Flush stale detection and wait for a fresh one
        self.latest_detection = None
        deadline = time.monotonic() + 1.5
        while rclpy.ok() and self.latest_detection is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_detection is None:
            return None

        corners_px, R_cam_tag, t_cam_tag = self.latest_detection

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        tr = tf.transform.translation
        q  = tf.transform.rotation
        return Sample(
            R_gripper2base=_quat_to_R([q.x, q.y, q.z, q.w]),
            t_gripper2base=np.array([tr.x, tr.y, tr.z]),
            R_target2cam=R_cam_tag,
            t_target2cam=t_cam_tag,
            corners_px=corners_px,
        )

    # -----------------------------------------------------------------------
    # Calibration
    # -----------------------------------------------------------------------
    def _calibrate_and_save(self, samples: List[Sample]) -> bool:
        R_g2b = [s.R_gripper2base for s in samples]
        t_g2b = [s.t_gripper2base.reshape(3, 1) for s in samples]
        R_t2c = [s.R_target2cam for s in samples]
        t_t2c = [s.t_target2cam.reshape(3, 1) for s in samples]

        results: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for name, method in HAND_EYE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
                results[name] = (R, t.flatten())
            except Exception as e:
                self.get_logger().warn(f"Method {name} failed: {e}")

        if not results:
            self.get_logger().error("All hand-eye calibration methods failed.")
            return False

        primary = PRIMARY_METHOD if PRIMARY_METHOD in results else next(iter(results))
        R_c2g, t_c2g = results[primary]
        q_c2g = _R_to_quat(R_c2g)

        residuals     = self._residuals(samples, R_c2g, t_c2g)
        urdf_cmp      = self._compare_urdf(R_c2g, t_c2g)

        self._print_results(
            samples, primary, results, R_c2g, t_c2g, q_c2g, residuals, urdf_cmp)

        out = {
            "method":         "EYE_IN_HAND_AX_XB",
            "primary_solver": primary,
            "tag": {
                "family": self.tag_family,
                "id":     self.tag_id,
                "size_m": self.tag_size_m,
            },
            "num_samples": len(samples),
            "frames": {
                "base_frame":         self.base_frame,
                "tcp_frame":          self.tcp_frame,
                "d405_optical_frame": self.d405_optical_frame,
            },
            "T_d405_optical_to_tcp": {
                "translation":    t_c2g.tolist(),
                "rotation_xyzw":  list(q_c2g),
            },
            "residuals":  residuals,
            "urdf_comparison": urdf_cmp,
            "all_methods": {
                name: {
                    "translation":   ti.flatten().tolist(),
                    "rotation_xyzw": _R_to_quat(Ri),
                }
                for name, (Ri, ti) in results.items()
            },
            "samples": [
                {
                    "R_gripper2base": s.R_gripper2base.tolist(),
                    "t_gripper2base": s.t_gripper2base.tolist(),
                    "R_target2cam":   s.R_target2cam.tolist(),
                    "t_target2cam":   s.t_target2cam.tolist(),
                    "corners_px":     s.corners_px.tolist(),
                }
                for s in samples
            ],
        }
        path = Path(self.output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(out, indent=2))
        self.get_logger().info(f"Saved calibration JSON → {path}")
        return True

    def _residuals(
        self, samples: List[Sample],
        R_c2g: np.ndarray, t_c2g: np.ndarray,
    ) -> dict:
        """Tag position in base frame should be constant (tag is fixed)."""
        T_c2g = _Rt_to_T(R_c2g, t_c2g)
        positions = []
        for s in samples:
            T_base_tcp = _Rt_to_T(s.R_gripper2base, s.t_gripper2base)
            T_d405_tag = _Rt_to_T(s.R_target2cam, s.t_target2cam)
            # p_base = T_base_tcp @ T_c2g @ T_d405_tag @ [0,0,0,1]
            T_base_tag = T_base_tcp @ T_c2g @ T_d405_tag
            positions.append(T_base_tag[:3, 3])
        positions = np.array(positions)
        mean_pos = np.mean(positions, axis=0)
        errors_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000.0
        return {
            "tag_position_in_base_mean_m": mean_pos.tolist(),
            "per_sample_error_mm":         errors_mm.tolist(),
            "mean_error_mm":               float(np.mean(errors_mm)),
            "max_error_mm":                float(np.max(errors_mm)),
        }

    def _compare_urdf(
        self, R_cal: np.ndarray, t_cal: np.ndarray,
    ) -> dict:
        """Look up URDF TF (tcp → d405_color_optical_frame) and compare."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tcp_frame, self.d405_optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except TransformException as e:
            return {"error": str(e)}

        tr = tf.transform.translation
        q  = tf.transform.rotation
        R_urdf = _quat_to_R([q.x, q.y, q.z, q.w])
        t_urdf = np.array([tr.x, tr.y, tr.z])

        t_err_mm  = float(np.linalg.norm(t_cal - t_urdf)) * 1000.0
        R_diff    = R_cal @ R_urdf.T
        trace     = float(np.clip(np.trace(R_diff), -1.0, 3.0))
        R_err_deg = float(np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))))

        return {
            "urdf_translation":       t_urdf.tolist(),
            "urdf_rotation_xyzw":     _R_to_quat(R_urdf),
            "translation_error_mm":   t_err_mm,
            "rotation_error_deg":     R_err_deg,
        }

    def _print_results(
        self, samples, primary, all_results,
        R, t, q, residuals, urdf_cmp,
    ):
        print()
        print("=" * 72)
        print("D405 Eye-in-Hand Calibration Result")
        print("=" * 72)
        print(f"Samples: {len(samples)}  |  Primary method: {primary}")
        print(f"Tag: {self.tag_family}  id={self.tag_id}  size={self.tag_size_m:.3f} m")
        print()
        print("T_d405_optical → TCP  (D405 optical frame in TCP frame):")
        print(f"  translation [m]:  [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]")
        print(f"  rotation (xyzw):  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")
        print()
        print("Tag position consistency (residuals; ideally < 2 mm):")
        print(f"  mean: {residuals['mean_error_mm']:.2f} mm")
        print(f"  max:  {residuals['max_error_mm']:.2f} mm")

        if "error" in urdf_cmp:
            print(f"\nURDF comparison unavailable: {urdf_cmp['error']}")
        else:
            print()
            print("URDF vs calibrated:")
            print(f"  URDF t [m]: {[f'{v:+.4f}' for v in urdf_cmp['urdf_translation']]}")
            print(f"  Cal  t [m]: {[f'{v:+.4f}' for v in t.tolist()]}")
            print(f"  Translation error: {urdf_cmp['translation_error_mm']:.2f} mm")
            print(f"  Rotation error:    {urdf_cmp['rotation_error_deg']:.3f} deg")

        print()
        print("All methods:")
        for name, (Ri, ti) in all_results.items():
            qi = _R_to_quat(Ri)
            tf = ti.flatten()
            mark = " ←" if name == primary else ""
            print(f"  {name:12s}: t=[{tf[0]:+.4f},{tf[1]:+.4f},{tf[2]:+.4f}]"
                  f"  q=[{qi[0]:+.4f},{qi[1]:+.4f},{qi[2]:+.4f},{qi[3]:+.4f}]{mark}")

        print()
        print("Static TF to override/validate URDF:")
        print("ros2 run tf2_ros static_transform_publisher \\")
        print(f"  --x {t[0]:.9f} --y {t[1]:.9f} --z {t[2]:.9f} \\")
        print(f"  --qx {q[0]:.9f} --qy {q[1]:.9f}"
              f" --qz {q[2]:.9f} --qw {q[3]:.9f} \\")
        print(f"  --frame-id {self.tcp_frame}"
              f" --child-frame-id {self.d405_optical_frame}")
        print("=" * 72)

    def _log_waiting(self, n_captured: int):
        now = time.monotonic()
        if now - self._last_wait_log < 3.0:
            return
        self._last_wait_log = now
        if self.d405_K is None:
            self.get_logger().info("waiting: D405 camera_info")
        elif self.latest_detection is None:
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] "
                "tag NOT detected — reposition, then press Enter to sample")
        else:
            t = self.latest_detection[2]
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] "
                f"tag visible at |t|={np.linalg.norm(t):.3f} m — "
                "press Enter to sample, or move to next pose")


# =============================================================================
# Shared utilities
# =============================================================================

def _make_detector(family: str) -> cv2.aruco.ArucoDetector:
    key = family.strip().lower().replace("_", "")
    mapping = {
        "tag36h11":       cv2.aruco.DICT_APRILTAG_36h11,
        "apriltag36h11":  cv2.aruco.DICT_APRILTAG_36h11,
        "tag25h9":        cv2.aruco.DICT_APRILTAG_25h9,
        "tag16h5":        cv2.aruco.DICT_APRILTAG_16h5,
    }
    if key not in mapping:
        raise ValueError(f"Unsupported AprilTag family: {family}")
    dictionary = cv2.aruco.getPredefinedDictionary(mapping[key])
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def _decode_gray(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc in ("mono8", "8uc1"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
        return arr[:, :w].copy()
    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step // 3, 3)
        code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(arr[:, :w, :], code)
    if enc in ("rgba8", "bgra8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step // 4, 4)
        code = cv2.COLOR_RGBA2GRAY if enc == "rgba8" else cv2.COLOR_BGRA2GRAY
        return cv2.cvtColor(arr[:, :w, :], code)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def _solve_pnp(
    corners_px: np.ndarray,
    tag_size_m: float,
    K: np.ndarray,
    D: np.ndarray,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    half = tag_size_m / 2.0
    obj_pts = np.array([
        [-half, +half, 0.0],
        [+half, +half, 0.0],
        [+half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    dist_coeffs = D if D is not None and D.size else None
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, corners_px.astype(np.float64),
        K, dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)


def _read_stdin_nonblock() -> Optional[str]:
    if not sys.stdin or not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    line = sys.stdin.readline().strip().lower()
    if line in ("done", "q", "quit", "finish", "end", "stop"):
        return "finish"
    return "sample"  # any Enter (including empty) triggers capture


def _Rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3]  = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _quat_to_R(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),  2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ], dtype=np.float64)


def _R_to_quat(R: np.ndarray) -> List[float]:
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


# =============================================================================
# Entry
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = D405EyeInHandCalibrator()
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
