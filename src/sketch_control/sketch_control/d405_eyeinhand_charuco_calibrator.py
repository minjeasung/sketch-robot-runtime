#!/usr/bin/env python3
"""D405 eye-in-hand calibration with a ChArUco board (AX=XB).

Setup
-----
- D405:    mounted on the wrist/EOAT (moves with the robot).
- ChArUco: FIXED in the scene (taped to the wall / table).
- Robot:   operator hand-guides (free-drive) to each pose, presses Enter to
           capture. MoveIt/move_group is NOT required — only /joint_states
           (joint_state_broadcaster) + robot_state_publisher for TF.

What it solves
--------------
Per pose i:
  R_gripper2base[i], t_gripper2base[i]  -- FK from TF (base_frame -> tcp_frame)
  R_target2cam[i],  t_target2cam[i]     -- ChArUco board pose in the D405 frame

cv2.calibrateHandEye() returns:
  R_cam2gripper, t_cam2gripper  =  T_d405_optical -> tcp
  (the D405 optical frame expressed in the TCP frame — same as the URDF fixed
  joint that mounts the D405 on the EOAT).

Result
------
  JSON key T_d405_optical_to_tcp. Compare against the URDF TF to sanity-check.

Run
---
source /opt/ros/jazzy/setup.bash && source ~/sketch_robot_ws/install/setup.bash
ros2 run sketch_control d405_eyeinhand_charuco_calibrator --ros-args \
  -p squares_x:=5 -p squares_y:=7 \
  -p square_length_m:=0.030 -p marker_length_m:=0.022 \
  -p aruco_dict:=DICT_5X5_100 -p num_samples:=20
"""

import json
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
from tf2_ros import Buffer, TransformException, TransformListener

from sketch_control.charuco_utils import (
    HAND_EYE_METHODS,
    PRIMARY_METHOD,
    R_to_quat,
    Rt_to_T,
    decode_gray,
    detect_charuco_pose,
    make_charuco,
    quat_to_R,
    read_stdin_nonblock,
)


DEFAULT_D405_IMAGE_TOPIC = "/d405/d405/color/image_raw"
DEFAULT_D405_INFO_TOPIC = "/d405/d405/color/camera_info"
DEFAULT_OUTPUT_PATH = str(
    Path.home() / "sketch_robot_ws" / "d405_eyeinhand_charuco_calibration.json")


@dataclass
class Sample:
    R_gripper2base: np.ndarray
    t_gripper2base: np.ndarray
    R_target2cam: np.ndarray
    t_target2cam: np.ndarray
    n_corners: int


class D405EyeInHandCharucoCalibrator(Node):

    def __init__(self):
        super().__init__("d405_eyeinhand_charuco_calibrator")

        self.squares_x = int(self.declare_parameter("squares_x", 5).value)
        self.squares_y = int(self.declare_parameter("squares_y", 7).value)
        self.square_length_m = float(
            self.declare_parameter("square_length_m", 0.030).value)
        self.marker_length_m = float(
            self.declare_parameter("marker_length_m", 0.022).value)
        self.aruco_dict = str(
            self.declare_parameter("aruco_dict", "DICT_5X5_100").value)
        self.min_corners = int(self.declare_parameter("min_corners", 8).value)

        self.d405_image_topic = str(self.declare_parameter(
            "d405_image_topic", DEFAULT_D405_IMAGE_TOPIC).value)
        self.d405_info_topic = str(self.declare_parameter(
            "d405_camera_info_topic", DEFAULT_D405_INFO_TOPIC).value)

        self.base_frame = str(self.declare_parameter("base_frame", "link0").value)
        self.tcp_frame = str(self.declare_parameter("tcp_frame", "tcp").value)
        self.d405_optical_frame = str(self.declare_parameter(
            "d405_optical_frame", "d405_color_optical_frame").value)

        self.num_samples = int(self.declare_parameter("num_samples", 20).value)
        self.min_valid_samples = int(
            self.declare_parameter("min_valid_samples", 10).value)
        self.reject_outliers = bool(
            self.declare_parameter("reject_outliers", True).value)
        self.outlier_min_error_mm = float(
            self.declare_parameter("outlier_min_error_mm", 3.0).value)
        self.outlier_mad_scale = float(
            self.declare_parameter("outlier_mad_scale", 3.5).value)
        self.outlier_max_iterations = int(
            self.declare_parameter("outlier_max_iterations", 2).value)
        self.output_path = str(
            self.declare_parameter("output_path", DEFAULT_OUTPUT_PATH).value)

        self.board, self.detector = make_charuco(
            self.squares_x, self.squares_y,
            self.square_length_m, self.marker_length_m, self.aruco_dict)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.d405_K: Optional[np.ndarray] = None
        self.d405_D: Optional[np.ndarray] = None
        self.latest_detection: Optional[Tuple] = None
        self._last_wait_log = 0.0

        self.create_subscription(CameraInfo, self.d405_info_topic, self._on_info, 10)
        self.create_subscription(
            Image, self.d405_image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            "D405 eye-in-hand ChArUco calibrator | "
            f"board {self.squares_x}x{self.squares_y} "
            f"square={self.square_length_m*1000:.0f}mm "
            f"marker={self.marker_length_m*1000:.0f}mm dict={self.aruco_dict}")
        self.get_logger().info(f"  D405 image: {self.d405_image_topic}")
        self.get_logger().info(f"  D405 info:  {self.d405_info_topic}")
        self.get_logger().info(
            "  Fix the ChArUco board in the scene. Hand-guide the wrist to varied "
            "poses (vary ORIENTATION, keep the whole board in view), stop, then "
            "press Enter to sample. Type 'done' to finish.")

    def _on_info(self, msg: CameraInfo):
        self.d405_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.d405_D = np.asarray(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image):
        if self.d405_K is None:
            return
        try:
            gray = decode_gray(msg)
        except Exception as e:
            self.get_logger().warn(f"D405 image decode failed: {e}",
                                   throttle_duration_sec=2.0)
            return
        det = detect_charuco_pose(
            gray, self.board, self.detector, self.d405_K, self.d405_D,
            min_corners=self.min_corners)
        if det is None:
            self.latest_detection = None
            return
        _cc, _ci, R, t, n = det
        self.latest_detection = (R, t, n)

    def run(self) -> bool:
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
            missing.append(f"TF ({self.base_frame} -> {self.tcp_frame})")
        if missing:
            self.get_logger().error("Missing prerequisites: " + ", ".join(missing))
            return False

        self.get_logger().info(
            f"Ready. Target: {self.num_samples} samples "
            f"(minimum {self.min_valid_samples}).")

        samples: List[Sample] = []
        while rclpy.ok() and len(samples) < self.num_samples:
            rclpy.spin_once(self, timeout_sec=0.05)
            cmd = read_stdin_nonblock()
            if cmd == "finish":
                if len(samples) >= self.min_valid_samples:
                    self.get_logger().info(
                        f"Finished early — {len(samples)} samples.")
                    break
                self.get_logger().warn(
                    f"Need >= {self.min_valid_samples}; have {len(samples)}.")
                continue
            if cmd != "sample":
                self._log_waiting(len(samples))
                continue

            sample = self._capture()
            if sample is None:
                self.get_logger().warn(
                    "Capture failed — board not detected or TF unavailable. "
                    "Reposition and press Enter again.")
                continue
            samples.append(sample)
            t = sample.t_target2cam
            self.get_logger().info(
                f"[{len(samples):2d}/{self.num_samples}] board in D405: "
                f"t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m "
                f"|t|={np.linalg.norm(t):.3f} m  corners={sample.n_corners}")

        if not rclpy.ok():
            self.get_logger().info("Interrupted before completion.")
            return False
        if len(samples) < self.min_valid_samples:
            self.get_logger().error(
                f"Only {len(samples)} valid samples; need "
                f">= {self.min_valid_samples}. Aborting.")
            return False
        return self._calibrate_and_save(samples)

    def _capture(self) -> Optional[Sample]:
        self.latest_detection = None
        deadline = time.monotonic() + 1.5
        while (rclpy.ok() and self.latest_detection is None
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_detection is None:
            return None
        R_t2c, t_t2c, n = self.latest_detection
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        return Sample(
            R_gripper2base=quat_to_R([q.x, q.y, q.z, q.w]),
            t_gripper2base=np.array([tr.x, tr.y, tr.z]),
            R_target2cam=R_t2c,
            t_target2cam=t_t2c,
            n_corners=n,
        )

    def _calibrate_and_save(self, samples: List[Sample]) -> bool:
        raw_samples = list(samples)
        indexed_samples = list(enumerate(raw_samples))
        rejected = []

        if self.reject_outliers:
            for _ in range(max(0, self.outlier_max_iterations)):
                active_samples = [sample for _, sample in indexed_samples]
                results = self._solve_methods(active_samples)
                if not results:
                    self.get_logger().error("All hand-eye methods failed.")
                    return False
                primary, method_residuals = self._choose_primary(
                    active_samples, results)
                residuals = method_residuals[primary]
                reject_now = self._outlier_indices(residuals)
                if not reject_now:
                    break
                if len(indexed_samples) - len(reject_now) < self.min_valid_samples:
                    self.get_logger().warn(
                        "Outlier rejection skipped because it would leave fewer "
                        f"than {self.min_valid_samples} samples.")
                    break
                errors = residuals["per_sample_error_mm"]
                reject_set = set(reject_now)
                for local_i in reject_now:
                    raw_i, sample = indexed_samples[local_i]
                    rejected.append({
                        "sample_index": raw_i + 1,
                        "error_mm": float(errors[local_i]),
                        "n_corners": sample.n_corners,
                    })
                indexed_samples = [
                    pair for local_i, pair in enumerate(indexed_samples)
                    if local_i not in reject_set
                ]

        samples = [sample for _, sample in indexed_samples]
        if len(samples) < self.min_valid_samples:
            self.get_logger().error(
                f"Only {len(samples)} samples remain after outlier rejection; "
                f"need >= {self.min_valid_samples}. Aborting.")
            return False

        results = self._solve_methods(samples)
        if not results:
            self.get_logger().error("All hand-eye methods failed.")
            return False
        primary, method_residuals = self._choose_primary(samples, results)
        R_c2g, t_c2g = results[primary]
        q_c2g = R_to_quat(R_c2g)
        residuals = method_residuals[primary]
        urdf_cmp = self._compare_urdf(R_c2g, t_c2g)
        self._print_results(samples, len(raw_samples), primary, results,
                            method_residuals, t_c2g, q_c2g, residuals,
                            urdf_cmp, rejected)

        out = {
            "method": "D405_EYE_IN_HAND_CHARUCO_AX_XB",
            "primary_solver": primary,
            "charuco": {
                "squares_x": self.squares_x, "squares_y": self.squares_y,
                "square_length_m": self.square_length_m,
                "marker_length_m": self.marker_length_m,
                "aruco_dict": self.aruco_dict,
            },
            "num_samples": len(samples),
            "num_samples_raw": len(raw_samples),
            "num_samples_used": len(samples),
            "outlier_rejection": {
                "enabled": self.reject_outliers,
                "min_error_mm": self.outlier_min_error_mm,
                "mad_scale": self.outlier_mad_scale,
                "max_iterations": self.outlier_max_iterations,
                "rejected_samples": rejected,
            },
            "frames": {
                "base_frame": self.base_frame,
                "tcp_frame": self.tcp_frame,
                "d405_optical_frame": self.d405_optical_frame,
            },
            "T_d405_optical_to_tcp": {
                "translation": t_c2g.tolist(),
                "rotation_xyzw": list(q_c2g),
            },
            "residuals": residuals,
            "urdf_comparison": urdf_cmp,
            "all_method_residuals": method_residuals,
            "all_methods": {
                name: {"translation": ti.flatten().tolist(),
                       "rotation_xyzw": R_to_quat(Ri)}
                for name, (Ri, ti) in results.items()
            },
            "samples": [
                {
                    "R_gripper2base": s.R_gripper2base.tolist(),
                    "t_gripper2base": s.t_gripper2base.tolist(),
                    "R_target2cam": s.R_target2cam.tolist(),
                    "t_target2cam": s.t_target2cam.tolist(),
                    "n_corners": s.n_corners,
                }
                for s in samples
            ],
        }
        path = Path(self.output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(out, indent=2))
        self.get_logger().info(f"Saved calibration JSON -> {path}")
        return True

    def _solve_methods(self, samples: List[Sample]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
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
        return results

    def _choose_primary(self, samples: List[Sample],
                        results: Dict[str, Tuple[np.ndarray, np.ndarray]]):
        method_residuals = {
            name: self._residuals(samples, R, t)
            for name, (R, t) in results.items()
        }
        primary = min(
            method_residuals,
            key=lambda name: method_residuals[name]["mean_error_mm"])
        return primary, method_residuals

    def _outlier_indices(self, residuals: dict) -> List[int]:
        errors = np.asarray(residuals["per_sample_error_mm"], dtype=np.float64)
        if errors.size < self.min_valid_samples + 1:
            return []
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        robust_sigma = 1.4826 * mad
        threshold = max(
            self.outlier_min_error_mm,
            median + self.outlier_mad_scale * robust_sigma)
        return [
            int(i) for i, error in enumerate(errors)
            if float(error) > threshold
        ]

    def _residuals(self, samples, R_c2g, t_c2g) -> dict:
        """Fixed board position in base should be constant across poses."""
        T_c2g = Rt_to_T(R_c2g, t_c2g)
        positions = []
        for s in samples:
            T_base_tcp = Rt_to_T(s.R_gripper2base, s.t_gripper2base)
            T_d405_board = Rt_to_T(s.R_target2cam, s.t_target2cam)
            T_base_board = T_base_tcp @ T_c2g @ T_d405_board
            positions.append(T_base_board[:3, 3])
        positions = np.array(positions)
        mean_pos = np.mean(positions, axis=0)
        errors_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000.0
        return {
            "board_position_in_base_mean_m": mean_pos.tolist(),
            "per_sample_error_mm": errors_mm.tolist(),
            "mean_error_mm": float(np.mean(errors_mm)),
            "max_error_mm": float(np.max(errors_mm)),
        }

    def _compare_urdf(self, R_cal, t_cal) -> dict:
        """Compare calibrated T_d405_optical->tcp against the URDF fixed TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tcp_frame, self.d405_optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except TransformException as e:
            return {"error": str(e)}
        tr = tf.transform.translation
        q = tf.transform.rotation
        R_urdf = quat_to_R([q.x, q.y, q.z, q.w])
        t_urdf = np.array([tr.x, tr.y, tr.z])
        t_err_mm = float(np.linalg.norm(t_cal - t_urdf)) * 1000.0
        R_diff = R_cal @ R_urdf.T
        trace = float(np.clip(np.trace(R_diff), -1.0, 3.0))
        R_err_deg = float(np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))))
        return {
            "urdf_translation_m": t_urdf.tolist(),
            "urdf_rotation_xyzw": R_to_quat(R_urdf),
            "translation_error_mm": t_err_mm,
            "rotation_error_deg": R_err_deg,
        }

    def _print_results(self, samples, raw_sample_count, primary, all_results,
                       all_residuals, t, q, residuals, urdf_cmp, rejected):
        print()
        print("=" * 72)
        print("D405 Eye-in-Hand ChArUco Calibration Result")
        print("=" * 72)
        print(f"Samples: {len(samples)}/{raw_sample_count} used"
              f"  |  Primary method: {primary}")
        print()
        print("T_d405_optical -> TCP  (D405 optical frame in the TCP frame):")
        print(f"  translation [m]:  [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]")
        print(f"  rotation (xyzw):  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")
        print()
        print("Fixed-board consistency (ideally < 3 mm):")
        print(f"  mean: {residuals['mean_error_mm']:.2f} mm  "
              f"max: {residuals['max_error_mm']:.2f} mm")
        if rejected:
            rejected_txt = ", ".join(
                f"#{r['sample_index']} ({r['error_mm']:.2f} mm)"
                for r in rejected)
            print(f"  rejected: {rejected_txt}")
        else:
            print("  rejected: none")
        print()
        if "error" in urdf_cmp:
            print(f"URDF comparison unavailable: {urdf_cmp['error']}")
        else:
            print("vs URDF (tcp -> d405_color_optical_frame):")
            print(f"  translation error: {urdf_cmp['translation_error_mm']:.2f} mm")
            print(f"  rotation error:    {urdf_cmp['rotation_error_deg']:.3f} deg")
        print()
        print("All methods:")
        for name, (Ri, ti) in all_results.items():
            qi = R_to_quat(Ri)
            tf = ti.flatten()
            method_res = all_residuals[name]
            mark = " <-" if name == primary else ""
            print(f"  {name:12s}: t=[{tf[0]:+.4f},{tf[1]:+.4f},{tf[2]:+.4f}]"
                  f"  q=[{qi[0]:+.4f},{qi[1]:+.4f},{qi[2]:+.4f},{qi[3]:+.4f}]"
                  f"  mean={method_res['mean_error_mm']:.2f}mm{mark}")
        print()
        print("Static TF to verify:")
        print("ros2 run tf2_ros static_transform_publisher \\")
        print(f"  --x {t[0]:.9f} --y {t[1]:.9f} --z {t[2]:.9f} \\")
        print(f"  --qx {q[0]:.9f} --qy {q[1]:.9f} --qz {q[2]:.9f} --qw {q[3]:.9f} \\")
        print(f"  --frame-id {self.tcp_frame} --child-frame-id {self.d405_optical_frame}")
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
                f"[{n_captured}/{self.num_samples}] ChArUco NOT detected — "
                "aim the D405 at the fixed board, then press Enter")
        else:
            t = self.latest_detection[1]
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] board visible "
                f"|t|={np.linalg.norm(t):.3f} m — press Enter to sample")


def main(args=None):
    rclpy.init(args=args)
    node = D405EyeInHandCharucoCalibrator()
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
