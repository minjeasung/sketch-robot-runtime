#!/usr/bin/env python3
"""ZED eye-to-hand calibration (AX=XB).

Setup
-----
- ZED:      FIXED in the scene (mounted on tripod / ceiling / wall)
- AprilTag: rigidly attached to robot TCP/EOAT
- Robot:    operator moves to each pose manually, presses Enter to capture

What it solves
--------------
For each pose i, two things are measured:
  T_base_tcp[i]   -- FK from TF (link0 → tcp)
  T_zed_tag[i]    -- tag pose in ZED frame via solvePnP

Eye-to-hand trick for cv2.calibrateHandEye:
  Pass INVERTED FK (T_tcp_base = inv(T_base_tcp)) as "gripper2base" input.
  cv2.calibrateHandEye() then returns X = T_zed_optical → base
  (camera expressed in robot base frame).

Result
------
  T_world_zed_optical — directly compatible with rb10_real_perception_sketch.launch.py
  JSON key: T_world_zed_optical  (same as apriltag_dual_camera_calibrator output).

Run
---
source /opt/ros/jazzy/setup.bash && source ~/sketch_robot_ws/install/setup.bash
ros2 run sketch_control zed_eyetohand_calibrator
  # optional params (set your tag details):
  #   --ros-args -p tag_id:=2 -p tag_size_m:=0.150 -p num_samples:=20
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


DEFAULT_ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEFAULT_ZED_INFO_TOPIC  = "/zed/zed_node/rgb/camera_info"
DEFAULT_OUTPUT_PATH = str(
    Path.home() / "sketch_robot_ws" / "zed_eyetohand_calibration.json"
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
    R_base_tcp:   np.ndarray   # (3,3) FK rotation
    t_base_tcp:   np.ndarray   # (3,)  FK translation
    R_zed_tag:    np.ndarray   # (3,3) tag in ZED frame from PnP
    t_zed_tag:    np.ndarray   # (3,)  tag in ZED frame from PnP
    corners_px:   np.ndarray   # (4,2) pixel corners


class ZedEyeToHandCalibrator(Node):

    def __init__(self):
        super().__init__("zed_eyetohand_calibrator")

        # --- ROS parameters ------------------------------------------------
        # Tag parameters MUST be set to match your actual calibration target.
        self.tag_id     = int(self.declare_parameter("tag_id",     2).value)
        self.tag_size_m = float(self.declare_parameter("tag_size_m", 0.150).value)
        self.tag_family = str(self.declare_parameter("tag_family", "tag36h11").value)

        self.zed_image_topic = str(self.declare_parameter(
            "zed_image_topic", DEFAULT_ZED_IMAGE_TOPIC).value)
        self.zed_info_topic  = str(self.declare_parameter(
            "zed_camera_info_topic", DEFAULT_ZED_INFO_TOPIC).value)

        self.base_frame        = str(self.declare_parameter("base_frame",        "link0").value)
        self.tcp_frame         = str(self.declare_parameter("tcp_frame",         "tcp").value)
        self.zed_optical_frame = str(self.declare_parameter(
            "zed_optical_frame", "zed_left_camera_optical_frame").value)

        self.num_samples       = int(self.declare_parameter("num_samples",       20).value)
        self.min_valid_samples = int(self.declare_parameter("min_valid_samples", 10).value)
        self.output_path       = str(self.declare_parameter(
            "output_path", DEFAULT_OUTPUT_PATH).value)

        # --- ArUco/AprilTag detector ----------------------------------------
        self.detector = _make_detector(self.tag_family)

        # --- TF -------------------------------------------------------------
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ----------------------------------------------------------
        self.zed_K: Optional[np.ndarray]      = None
        self.zed_D: Optional[np.ndarray]      = None
        self.latest_detection: Optional[Tuple] = None  # (corners, R, t)
        self._last_wait_log: float             = 0.0

        # --- Subscriptions --------------------------------------------------
        self.create_subscription(CameraInfo, self.zed_info_topic, self._on_info, 10)
        self.create_subscription(Image, self.zed_image_topic,
                                 self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"ZED eye-to-hand calibrator | "
            f"tag={self.tag_family} id={self.tag_id} size={self.tag_size_m:.3f} m")
        self.get_logger().info(f"  ZED image: {self.zed_image_topic}")
        self.get_logger().info(f"  ZED info:  {self.zed_info_topic}")
        self.get_logger().info(
            "  Attach the AprilTag RIGIDLY to the TCP/EOAT before starting.")
        self.get_logger().info(
            "  Move robot to each pose, stop fully, then press Enter to sample. "
            "Type 'done' to finish early.")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo):
        self.zed_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.zed_D = np.asarray(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image):
        if self.zed_K is None:
            return
        try:
            gray = _decode_gray(msg)
        except Exception as e:
            self.get_logger().warn(f"ZED image decode failed: {e}",
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
        pnp = _solve_pnp(corners_px, self.tag_size_m, self.zed_K, self.zed_D)
        if pnp is None:
            self.latest_detection = None
            return
        R, t = pnp
        self.latest_detection = (corners_px, R, t)

    # -----------------------------------------------------------------------
    # Main flow
    # -----------------------------------------------------------------------
    def run(self) -> bool:
        self.get_logger().info("Waiting for ZED camera_info and TF...")
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.zed_K is not None
                    and self.tf_buffer.can_transform(
                        self.base_frame, self.tcp_frame, rclpy.time.Time())):
                break

        missing = []
        if self.zed_K is None:
            missing.append(f"ZED CameraInfo ({self.zed_info_topic})")
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
            t = sample.t_zed_tag
            self.get_logger().info(
                f"[{len(samples):2d}/{self.num_samples}] "
                f"tag in ZED: t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m  "
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
        self.latest_detection = None
        deadline = time.monotonic() + 1.5
        while rclpy.ok() and self.latest_detection is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_detection is None:
            return None

        corners_px, R_zed_tag, t_zed_tag = self.latest_detection

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
            R_base_tcp=_quat_to_R([q.x, q.y, q.z, q.w]),
            t_base_tcp=np.array([tr.x, tr.y, tr.z]),
            R_zed_tag=R_zed_tag,
            t_zed_tag=t_zed_tag,
            corners_px=corners_px,
        )

    # -----------------------------------------------------------------------
    # Calibration (eye-to-hand via inverted FK trick)
    # -----------------------------------------------------------------------
    def _calibrate_and_save(self, samples: List[Sample]) -> bool:
        # Invert FK: pass T_tcp_base instead of T_base_tcp.
        # cv2.calibrateHandEye with inverted FK → X = T_zed_to_base.
        R_invFK, t_invFK = [], []
        for s in samples:
            R_b2g = s.R_base_tcp
            t_b2g = s.t_base_tcp
            R_g2b_inv = R_b2g.T                          # R_tcp_base
            t_g2b_inv = (-R_b2g.T @ t_b2g).reshape(3, 1)  # t_tcp_base
            R_invFK.append(R_g2b_inv)
            t_invFK.append(t_g2b_inv)

        R_t2c = [s.R_zed_tag for s in samples]
        t_t2c = [s.t_zed_tag.reshape(3, 1) for s in samples]

        results: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for name, method in HAND_EYE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(
                    R_invFK, t_invFK, R_t2c, t_t2c, method=method)
                results[name] = (R, t.flatten())
            except Exception as e:
                self.get_logger().warn(f"Method {name} failed: {e}")

        if not results:
            self.get_logger().error("All hand-eye calibration methods failed.")
            return False

        primary = PRIMARY_METHOD if PRIMARY_METHOD in results else next(iter(results))
        R_zed2base, t_zed2base = results[primary]
        q_zed2base = _R_to_quat(R_zed2base)

        residuals = self._residuals(samples, R_zed2base, t_zed2base)

        self._print_results(
            samples, primary, results,
            R_zed2base, t_zed2base, q_zed2base, residuals,
        )

        # Output JSON compatible with existing zed_d405_apriltag_calibration.json format.
        # T_world_zed_optical == T_base_zed_optical (world = link0 = base in our setup).
        out = {
            "method":         "ZED_EYE_TO_HAND_AX_XB",
            "primary_solver": primary,
            "tag": {
                "family": self.tag_family,
                "id":     self.tag_id,
                "size_m": self.tag_size_m,
            },
            "num_samples": len(samples),
            "frames": {
                "base_frame":        self.base_frame,
                "tcp_frame":         self.tcp_frame,
                "zed_optical_frame": self.zed_optical_frame,
            },
            # Primary result key — matches zed_calibration_pose_key default in launch.
            "T_world_zed_optical": {
                "translation":   t_zed2base.tolist(),
                "rotation_xyzw": list(q_zed2base),
            },
            # Alias for compatibility with apriltag_dual_camera_calibrator output.
            "T_base_zed_optical": {
                "translation":   t_zed2base.tolist(),
                "rotation_xyzw": list(q_zed2base),
            },
            "residuals": residuals,
            "all_methods": {
                name: {
                    "translation":   ti.flatten().tolist(),
                    "rotation_xyzw": _R_to_quat(Ri),
                }
                for name, (Ri, ti) in results.items()
            },
            "samples": [
                {
                    "R_base_tcp": s.R_base_tcp.tolist(),
                    "t_base_tcp": s.t_base_tcp.tolist(),
                    "R_zed_tag":  s.R_zed_tag.tolist(),
                    "t_zed_tag":  s.t_zed_tag.tolist(),
                    "corners_px": s.corners_px.tolist(),
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
        R_zed2base: np.ndarray, t_zed2base: np.ndarray,
    ) -> dict:
        """Tag position in base frame should be consistent with FK at each pose.

        Cross-check: T_base_tag_i via FK chain vs via ZED chain should agree
        up to unknown tcp→tag offset.  We report self-consistency of the ZED chain.
        """
        # ZED chain: T_base_tag_i = T_zed2base @ T_zed_tag_i
        T_zed2base = _Rt_to_T(R_zed2base, t_zed2base)
        positions = []
        for s in samples:
            T_zed_tag = _Rt_to_T(s.R_zed_tag, s.t_zed_tag)
            T_base_tag = T_zed2base @ T_zed_tag
            positions.append(T_base_tag[:3, 3])
        positions = np.array(positions)

        # These SHOULD vary (TCP moves). Report spread relative to a moving mean
        # is not directly useful; instead report residual via the AX=XB equation.
        # Simpler metric: check that for each pair, the cross-chain residual is small.
        residual_mm = []
        for i, si in enumerate(samples):
            for j, sj in enumerate(samples):
                if j <= i:
                    continue
                # FK relative motion A = inv(T_base_tcp_i) @ T_base_tcp_j
                T_i = _Rt_to_T(si.R_base_tcp, si.t_base_tcp)
                T_j = _Rt_to_T(sj.R_base_tcp, sj.t_base_tcp)
                A = np.linalg.inv(T_i) @ T_j  # relative motion in base frame via FK

                # ZED relative motion B = T_zed_tag_i @ inv(T_zed_tag_j)
                T_zi = _Rt_to_T(si.R_zed_tag, si.t_zed_tag)
                T_zj = _Rt_to_T(sj.R_zed_tag, sj.t_zed_tag)
                B = T_zi @ np.linalg.inv(T_zj)

                # If X = T_zed2base is correct: X @ A == B @ X  ⟹ X^-1 @ B @ X = A
                X = T_zed2base
                lhs = np.linalg.inv(X) @ B @ X
                err_t = float(np.linalg.norm((lhs - A)[:3, 3]) * 1000.0)
                residual_mm.append(err_t)

        residual_mm = np.array(residual_mm) if residual_mm else np.zeros(1)
        return {
            "ax_xb_residual_mean_mm": float(np.mean(residual_mm)),
            "ax_xb_residual_max_mm":  float(np.max(residual_mm)),
            "num_pairs":              len(residual_mm),
        }

    def _print_results(
        self, samples, primary, all_results,
        R, t, q, residuals,
    ):
        print()
        print("=" * 72)
        print("ZED Eye-to-Hand Calibration Result")
        print("=" * 72)
        print(f"Samples: {len(samples)}  |  Primary method: {primary}")
        print(f"Tag: {self.tag_family}  id={self.tag_id}  size={self.tag_size_m:.3f} m")
        print()
        print("T_zed_optical → base  (ZED optical frame in robot base frame):")
        print(f"  translation [m]:  [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]")
        print(f"  rotation (xyzw):  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")
        print()
        print("AX=XB residual (ideally < 5 mm):")
        print(f"  mean: {residuals['ax_xb_residual_mean_mm']:.2f} mm")
        print(f"  max:  {residuals['ax_xb_residual_max_mm']:.2f} mm")
        print()
        print("All methods:")
        for name, (Ri, ti) in all_results.items():
            qi = _R_to_quat(Ri)
            tf = ti.flatten()
            mark = " ←" if name == primary else ""
            print(f"  {name:12s}: t=[{tf[0]:+.4f},{tf[1]:+.4f},{tf[2]:+.4f}]"
                  f"  q=[{qi[0]:+.4f},{qi[1]:+.4f},{qi[2]:+.4f},{qi[3]:+.4f}]{mark}")

        print()
        print("Launch args for rb10_real_perception_sketch.launch.py:")
        print(f"  zed_x:={t[0]:.9f} \\")
        print(f"  zed_y:={t[1]:.9f} \\")
        print(f"  zed_z:={t[2]:.9f} \\")
        print(f"  zed_qx:={q[0]:.9f} \\")
        print(f"  zed_qy:={q[1]:.9f} \\")
        print(f"  zed_qz:={q[2]:.9f} \\")
        print(f"  zed_qw:={q[3]:.9f}")
        print()
        print("Or use the saved JSON with:")
        print(f"  use_zed_calibration_file:=true")
        print(f"  zed_calibration_file:={self.output_path}")
        print(f"  zed_calibration_pose_key:=T_world_zed_optical")
        print()
        print("Static TF to verify:")
        print("ros2 run tf2_ros static_transform_publisher \\")
        print(f"  --x {t[0]:.9f} --y {t[1]:.9f} --z {t[2]:.9f} \\")
        print(f"  --qx {q[0]:.9f} --qy {q[1]:.9f}"
              f" --qz {q[2]:.9f} --qw {q[3]:.9f} \\")
        print(f"  --frame-id {self.base_frame}"
              f" --child-frame-id {self.zed_optical_frame}")
        print("=" * 72)

    def _log_waiting(self, n_captured: int):
        now = time.monotonic()
        if now - self._last_wait_log < 3.0:
            return
        self._last_wait_log = now
        if self.zed_K is None:
            self.get_logger().info("waiting: ZED camera_info")
        elif self.latest_detection is None:
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] "
                "tag NOT detected — move TCP so ZED can see the tag, "
                "then press Enter to sample")
        else:
            t = self.latest_detection[2]
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] "
                f"tag visible at |t|={np.linalg.norm(t):.3f} m — "
                "press Enter to sample, or move to next pose")


# =============================================================================
# Shared utilities  (identical copies from d405_eyeinhand_calibrator.py)
# =============================================================================

def _make_detector(family: str) -> cv2.aruco.ArucoDetector:
    key = family.strip().lower().replace("_", "")
    mapping = {
        "tag36h11":      cv2.aruco.DICT_APRILTAG_36h11,
        "apriltag36h11": cv2.aruco.DICT_APRILTAG_36h11,
        "tag25h9":       cv2.aruco.DICT_APRILTAG_25h9,
        "tag16h5":       cv2.aruco.DICT_APRILTAG_16h5,
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
    return "sample"


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
    node = ZedEyeToHandCalibrator()
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
