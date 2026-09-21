#!/usr/bin/env python3
"""ZED eye-to-hand calibration with a ChArUco board (AX=XB).

Setup
-----
- ZED:     FIXED in the scene (tripod / ceiling / wall).
- ChArUco: rigidly attached to the robot TCP/EOAT (moves with the robot).
- Robot:   operator hand-guides (free-drive) to each pose, presses Enter to
           capture. MoveIt/move_group is NOT required — only /joint_states
           (joint_state_broadcaster) + robot_state_publisher for TF.

What it solves
--------------
Per pose i:
  T_base_tcp[i]   -- FK from TF (base_frame → tcp_frame)
  T_zed_board[i]  -- ChArUco board pose in the ZED optical frame (solvePnP)

Eye-to-hand trick for cv2.calibrateHandEye:
  Pass INVERTED FK (T_tcp_base = inv(T_base_tcp)) as the "gripper2base" input.
  cv2.calibrateHandEye() then returns X = T_zed_optical → base
  (the ZED optical frame expressed in the robot base frame).

Result
------
  JSON key T_world_zed_optical — directly usable by
  rb10_real_perception_sketch.launch.py (world = link0 = base here).

Run
---
source /opt/ros/jazzy/setup.bash && source ~/sketch_robot_ws/install/setup.bash
ros2 run sketch_control zed_eyetohand_charuco_calibrator --ros-args \
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


DEFAULT_ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEFAULT_ZED_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"
DEFAULT_OUTPUT_PATH = str(
    Path.home() / "sketch_robot_ws" / "zed_eyetohand_charuco_calibration.json")


@dataclass
class Sample:
    R_base_tcp: np.ndarray
    t_base_tcp: np.ndarray
    R_zed_board: np.ndarray
    t_zed_board: np.ndarray
    n_corners: int


class ZedEyeToHandCharucoCalibrator(Node):

    def __init__(self):
        super().__init__("zed_eyetohand_charuco_calibrator")

        # --- ChArUco geometry (MUST match your printed board) ---------------
        self.squares_x = int(self.declare_parameter("squares_x", 5).value)
        self.squares_y = int(self.declare_parameter("squares_y", 7).value)
        self.square_length_m = float(
            self.declare_parameter("square_length_m", 0.030).value)
        self.marker_length_m = float(
            self.declare_parameter("marker_length_m", 0.022).value)
        self.aruco_dict = str(
            self.declare_parameter("aruco_dict", "DICT_5X5_100").value)
        self.min_corners = int(self.declare_parameter("min_corners", 8).value)

        self.zed_image_topic = str(self.declare_parameter(
            "zed_image_topic", DEFAULT_ZED_IMAGE_TOPIC).value)
        self.zed_info_topic = str(self.declare_parameter(
            "zed_camera_info_topic", DEFAULT_ZED_INFO_TOPIC).value)

        self.base_frame = str(self.declare_parameter("base_frame", "link0").value)
        self.tcp_frame = str(self.declare_parameter("tcp_frame", "tcp").value)
        self.zed_optical_frame = str(self.declare_parameter(
            "zed_optical_frame", "zed_left_camera_optical_frame").value)

        self.num_samples = int(self.declare_parameter("num_samples", 20).value)
        self.min_valid_samples = int(
            self.declare_parameter("min_valid_samples", 10).value)
        self.output_path = str(
            self.declare_parameter("output_path", DEFAULT_OUTPUT_PATH).value)

        self.board, self.detector = make_charuco(
            self.squares_x, self.squares_y,
            self.square_length_m, self.marker_length_m, self.aruco_dict)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.zed_K: Optional[np.ndarray] = None
        self.zed_D: Optional[np.ndarray] = None
        self.latest_detection: Optional[Tuple] = None
        self._last_wait_log = 0.0

        self.create_subscription(CameraInfo, self.zed_info_topic, self._on_info, 10)
        self.create_subscription(
            Image, self.zed_image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            "ZED eye-to-hand ChArUco calibrator | "
            f"board {self.squares_x}x{self.squares_y} "
            f"square={self.square_length_m*1000:.0f}mm "
            f"marker={self.marker_length_m*1000:.0f}mm dict={self.aruco_dict}")
        self.get_logger().info(f"  ZED image: {self.zed_image_topic}")
        self.get_logger().info(f"  ZED info:  {self.zed_info_topic}")
        self.get_logger().info(
            "  Attach the ChArUco board RIGIDLY to the TCP/EOAT. Hand-guide the "
            "robot to varied poses (vary ORIENTATION, not just position), stop, "
            "then press Enter to sample. Type 'done' to finish.")

    def _on_info(self, msg: CameraInfo):
        self.zed_K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.zed_D = np.asarray(msg.d, dtype=np.float64)

    def _on_image(self, msg: Image):
        if self.zed_K is None:
            return
        try:
            gray = decode_gray(msg)
        except Exception as e:
            self.get_logger().warn(f"ZED image decode failed: {e}",
                                   throttle_duration_sec=2.0)
            return
        det = detect_charuco_pose(
            gray, self.board, self.detector, self.zed_K, self.zed_D,
            min_corners=self.min_corners)
        if det is None:
            self.latest_detection = None
            return
        _cc, _ci, R, t, n = det
        self.latest_detection = (R, t, n)

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
            t = sample.t_zed_board
            self.get_logger().info(
                f"[{len(samples):2d}/{self.num_samples}] board in ZED: "
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
        R_zed_board, t_zed_board, n = self.latest_detection
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
            R_base_tcp=quat_to_R([q.x, q.y, q.z, q.w]),
            t_base_tcp=np.array([tr.x, tr.y, tr.z]),
            R_zed_board=R_zed_board,
            t_zed_board=t_zed_board,
            n_corners=n,
        )

    def _calibrate_and_save(self, samples: List[Sample]) -> bool:
        # Eye-to-hand: pass inverted FK (T_tcp_base) as gripper2base.
        R_invFK, t_invFK = [], []
        for s in samples:
            R_invFK.append(s.R_base_tcp.T)
            t_invFK.append((-s.R_base_tcp.T @ s.t_base_tcp).reshape(3, 1))
        R_t2c = [s.R_zed_board for s in samples]
        t_t2c = [s.t_zed_board.reshape(3, 1) for s in samples]

        results: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for name, method in HAND_EYE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(
                    R_invFK, t_invFK, R_t2c, t_t2c, method=method)
                results[name] = (R, t.flatten())
            except Exception as e:
                self.get_logger().warn(f"Method {name} failed: {e}")
        if not results:
            self.get_logger().error("All hand-eye methods failed.")
            return False

        method_residuals = {
            name: self._residuals(samples, R, t)
            for name, (R, t) in results.items()
        }
        primary = min(
            method_residuals,
            key=lambda name: method_residuals[name]["mean_error_mm"])
        R_zed2base, t_zed2base = results[primary]
        q_zed2base = R_to_quat(R_zed2base)
        residuals = method_residuals[primary]
        self._print_results(samples, primary, results, method_residuals,
                            t_zed2base, q_zed2base, residuals)

        out = {
            "method": "ZED_EYE_TO_HAND_CHARUCO_AX_XB",
            "primary_solver": primary,
            "charuco": {
                "squares_x": self.squares_x, "squares_y": self.squares_y,
                "square_length_m": self.square_length_m,
                "marker_length_m": self.marker_length_m,
                "aruco_dict": self.aruco_dict,
            },
            "num_samples": len(samples),
            "frames": {
                "base_frame": self.base_frame,
                "tcp_frame": self.tcp_frame,
                "zed_optical_frame": self.zed_optical_frame,
            },
            "T_world_zed_optical": {
                "translation": t_zed2base.tolist(),
                "rotation_xyzw": list(q_zed2base),
            },
            "T_base_zed_optical": {
                "translation": t_zed2base.tolist(),
                "rotation_xyzw": list(q_zed2base),
            },
            "residuals": residuals,
            "all_method_residuals": method_residuals,
            "all_methods": {
                name: {"translation": ti.flatten().tolist(),
                       "rotation_xyzw": R_to_quat(Ri)}
                for name, (Ri, ti) in results.items()
            },
            "samples": [
                {
                    "R_base_tcp": s.R_base_tcp.tolist(),
                    "t_base_tcp": s.t_base_tcp.tolist(),
                    "R_zed_board": s.R_zed_board.tolist(),
                    "t_zed_board": s.t_zed_board.tolist(),
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

    def _residuals(self, samples, R_zed2base, t_zed2base) -> dict:
        """Board attached to TCP should be constant in the TCP frame."""
        T_zed2base = Rt_to_T(R_zed2base, t_zed2base)
        positions = []
        for s in samples:
            T_base_tcp = Rt_to_T(s.R_base_tcp, s.t_base_tcp)
            T_zed_board = Rt_to_T(s.R_zed_board, s.t_zed_board)
            T_base_board = T_zed2base @ T_zed_board
            T_tcp_board = np.linalg.inv(T_base_tcp) @ T_base_board
            positions.append(T_tcp_board[:3, 3])
        positions = np.array(positions)
        mean_pos = np.mean(positions, axis=0)
        errors_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000.0
        return {
            "board_position_in_tcp_mean_m": mean_pos.tolist(),
            "per_sample_error_mm": errors_mm.tolist(),
            "mean_error_mm": float(np.mean(errors_mm)),
            "max_error_mm": float(np.max(errors_mm)),
        }

    def _print_results(self, samples, primary, all_results, all_residuals,
                       t, q, residuals):
        print()
        print("=" * 72)
        print("ZED Eye-to-Hand ChArUco Calibration Result")
        print("=" * 72)
        print(f"Samples: {len(samples)}  |  Primary method: {primary}")
        print()
        print("T_zed_optical -> base  (ZED optical frame in robot base frame):")
        print(f"  translation [m]:  [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]")
        print(f"  rotation (xyzw):  [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")
        print()
        print("TCP-board consistency (ideally < 3 mm):")
        print(f"  mean: {residuals['mean_error_mm']:.2f} mm")
        print(f"  max:  {residuals['max_error_mm']:.2f} mm")
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
        print("Launch args for rb10_real_perception_sketch.launch.py:")
        print(f"  zed_x:={t[0]:.9f} zed_y:={t[1]:.9f} zed_z:={t[2]:.9f} \\")
        print(f"  zed_qx:={q[0]:.9f} zed_qy:={q[1]:.9f} "
              f"zed_qz:={q[2]:.9f} zed_qw:={q[3]:.9f}")
        print()
        print("Or use the saved JSON:")
        print("  use_zed_calibration_file:=true \\")
        print(f"  zed_calibration_file:={self.output_path} \\")
        print("  zed_calibration_pose_key:=T_world_zed_optical")
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
                f"[{n_captured}/{self.num_samples}] ChArUco NOT detected — "
                "aim the board at the ZED, then press Enter")
        else:
            t = self.latest_detection[1]
            self.get_logger().info(
                f"[{n_captured}/{self.num_samples}] board visible "
                f"|t|={np.linalg.norm(t):.3f} m — press Enter to sample")


def main(args=None):
    rclpy.init(args=args)
    node = ZedEyeToHandCharucoCalibrator()
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
