#!/usr/bin/env python3
"""Shared ChArUco detection + math utilities for hand-eye calibration.

Used by:
  - zed_eyetohand_charuco_calibrator.py   (eye-to-hand, fixed ZED)
  - d405_eyeinhand_charuco_calibrator.py  (eye-in-hand, wrist D405)

A ChArUco board (chessboard + embedded ArUco markers) is more robust than a
single AprilTag: sub-pixel chessboard corners, partial-occlusion tolerant, and
an unambiguous board pose from many correspondences.

OpenCV 4.7+ API only (CharucoDetector + board.matchImagePoints + solvePnP).
"""

import math
import select
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np


# --- ArUco dictionary name → id ----------------------------------------------
_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


def dict_id(name: str) -> int:
    key = name.strip().upper()
    if not key.startswith("DICT_"):
        key = "DICT_" + key
    if key not in _DICT_MAP:
        raise ValueError(
            f"unsupported aruco dict '{name}'. one of {sorted(_DICT_MAP)}")
    return _DICT_MAP[key]


def make_charuco(squares_x: int, squares_y: int,
                 square_length_m: float, marker_length_m: float,
                 dict_name: str):
    """Return (board, detector) for the given ChArUco geometry."""
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id(dict_name))
    board = cv2.aruco.CharucoBoard(
        (int(squares_x), int(squares_y)),
        float(square_length_m), float(marker_length_m), dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector


def detect_charuco_pose(gray, board, detector, K, D, min_corners: int = 6):
    """Detect the ChArUco board and solve its pose in the camera frame.

    Returns (charuco_corners, charuco_ids, R_cam_board, t_cam_board, n_corners)
    or None if the board is not seen with enough corners / PnP fails.
    """
    charuco_corners, charuco_ids, _m_corners, _m_ids = detector.detectBoard(gray)
    if charuco_corners is None or len(charuco_corners) < max(4, min_corners):
        return None
    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_pts is None or len(obj_pts) < 4:
        return None
    dist = D if (D is not None and np.asarray(D).size) else None
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return charuco_corners, charuco_ids, R, tvec.reshape(3), int(len(charuco_corners))


# --- image / io helpers ------------------------------------------------------
def decode_gray(msg) -> np.ndarray:
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
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def read_stdin_nonblock() -> Optional[str]:
    """Non-blocking terminal read. Enter -> 'sample'; done/q -> 'finish'."""
    if not sys.stdin or not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    line = sys.stdin.readline().strip().lower()
    if line in ("done", "q", "quit", "finish", "end", "stop"):
        return "finish"
    return "sample"


# --- SE(3) / quaternion math -------------------------------------------------
def Rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def R_to_quat(R: np.ndarray) -> List[float]:
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


HAND_EYE_METHODS: List[Tuple[str, int]] = [
    ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
    ("PARK", cv2.CALIB_HAND_EYE_PARK),
    ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
    ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
]
PRIMARY_METHOD = "TSAI"
