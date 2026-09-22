"""
MoveIt Executor - 4-stage 용접 모션 (Plan + FollowJointTrajectory 실행)

대상 로봇: Rainbow Robotics RB10-1300E / RB20-1900ES (rbpodo_ros2 driver)
EE link: tcp / Base frame: link0 / Planning group: manipulator

4-stage 파이프라인:
  Stage 1: 자유 plan (현재 자세 → safety pose)  [OMPL via /move_action]
  Stage 2: cartesian (safety → 표면 첫 점)        [/compute_cartesian_path]
  Stage 3: cartesian path (표면 위 스케치 추종)
  Stage 4: cartesian (표면 끝 → retreat pose)
  Stage 5: 자유 plan (retreat → READY_POSE)

EoAT 체인:
  tcp -> AFT200 force/torque sensor -> RR-00A_B EOAT(no-camera) -> D405

제공된 AFT200 URDF/roller STEP 은 CAD local +Z 방향으로 뻗지만, 실제 장착은
TCP local -Y 방향이다. 따라서 planning 에서는 CAD +Z 를 TCP -Y 로 해석한다.
"""
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point, PoseArray, Pose, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from sketch_control.robot_models import DEFAULT_MODEL, validate_model, model_joint_limits, model_srdf
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionIK,
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    PositionIKRequest, RobotState,
    CollisionObject, AttachedCollisionObject,
    PlanningScene, PlanningSceneWorld,
    Constraints, OrientationConstraint, PositionConstraint, JointConstraint,
    AllowedCollisionEntry, AllowedCollisionMatrix, PlanningSceneComponents,
)
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

from sketch_control.targets import (
    load_objects_config, get_surface_plane, get_target, ee_quat_for_target,
)
from rbpodo_painting_control.segment_path import (
    CLEARANCE_MODES,
    CONTACT_MOTION_MODES,
    MOTION_MODES,
    ZERO_FORCE_MODES,
    ZERO_FORCE_MOTION_MODES,
    SegmentPath,
    SegmentPathError,
    build_execution_steps,
    parse_segment_path,
    rotation_from_surface_path,
    segment_waypoint_position,
    transform_segment_path,
    validate_segment_path_for_real_execution,
)
from sketch_control.painting_execution import (
    ContactSearchConfig,
    evaluate_contact_search,
    evaluate_stationary_paint_hold,
    follow_joint_watchdog_reason,
    joint_velocities_allow_stationary,
    ramp_down_handshake_action,
    real_plan_gate_blockers,
)
from sketch_control.work_area_geometry import outside_quad_3d_indices
from sketch_control.rotation_utils import (
    quat_apply, quat_from_matrix, quat_from_two_vectors, quat_multiply,
    quat_to_matrix,
)

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - non-ROS import paths
    get_package_share_directory = None


# Current sketch SRDF and rbpodo_moveit_config use the corrected group name.
PLANNING_GROUP = "manipulator"
EE_LINK = "tcp"
BASE_FRAME = "link0"
CARTESIAN_COMPLETE_FRACTION = 1.0 - 1e-6

CONTACT_CLEARANCE = 0.005  # MoveIt 상에서는 벽과 5mm clearance 유지
CONTACT_PLANE_TOL = 0.015  # perception/TF noise 허용 범위
WORK_AREA_W = 0.50
WORK_AREA_H = 0.40
WORK_AREA_MARGIN = 0.02
D405_PREFLIGHT_SCAN_ENABLED = True
D405_PREFLIGHT_REQUIRE_REFINED = False
D405_PREFLIGHT_SCAN_STANDOFF = 0.32
D405_PREFLIGHT_SCAN_SETTLE_SEC = 1.0
D405_PREFLIGHT_SCAN_TIMEOUT_SEC = 4.0
D405_PREFLIGHT_SCAN_INSET_M = 0.08
D405_PREFLIGHT_SPEED_SCALE = 0.0175
# D405 prescan 은 현재 자세에서 가까운 점부터 작은 tangent probing 을 수행한다.
# Full URDF/SRDF PlanningScene collision checking is authoritative.  The
# remaining joint-space hard guard only catches a discontinuity between adjacent
# command points; smooth total rotation, path length and point count are ranked
# or logged rather than treated as collision evidence.
D405_PREFLIGHT_SCAN_MAX_POSES = 5
D405_PREFLIGHT_PROBE_OFFSET_M = 0.06
D405_PREFLIGHT_MAX_JOINT_PATH_RAD = 1.8
# Large start-to-goal rotation is diagnostic.  Both task-equivalent D405
# positions remain eligible after endpoint and complete-path collision checks.
D405_PREFLIGHT_LARGE_START_GOAL_WARN_RAD = math.radians(85.0)
D405_PREFLIGHT_MAX_PLAN_POINTS = 260
D405_PREFLIGHT_CARTESIAN_FRACTION = CARTESIAN_COMPLETE_FRACTION
D405_PREFLIGHT_ADAPTIVE_PROBE_SCALES = (0.75, 0.50)
D405_PREFLIGHT_ADAPTIVE_MIN_MOVE_M = 0.015
D405_PREFLIGHT_FINAL_JOINT_TOL_RAD = 0.02
D405_PREFLIGHT_START_STATE_TOL_RAD = 0.01
D405_PREFLIGHT_ARRIVAL_VERIFY_TIMEOUT_S = 2.0
D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S = 0.20
ACTIVE_SURFACE_DEDUPE_POINT_TOL_M = 1e-6
ACTIVE_SURFACE_DEDUPE_NORMAL_TOL = 1e-6
MOTION_ABORT_TOPIC = "/motion_abort"
EOAT_SEGMENTS_TOPIC = "/sketch_eoat_segments"
PAINTING_ABORT_TOPIC = "/painting_admittance/abort"
PAINTING_MODE_TOPIC = "/painting_admittance/mode"
PAINTING_FORCE_TOPIC = "/painting_admittance/desired_force_n"
PAINTING_ENABLE_TOPIC = "/painting_admittance/enable_force"
PAINTING_RAMP_COMPLETE_TOPIC = "/painting_admittance/ramp_complete"
PAINTING_RAMP_STATUS_TOPIC = "/painting_admittance/ramp_status"
PAINTING_CONTACT_TOPIC = "/painting_admittance/contact_confirmed"
D405_REFINE_CAPTURE_TOPIC = "/d405/refine_capture"
D405_REFINEMENT_STATUS_TOPIC = "/perception/d405_surface_refinement_status"
D405_REFINEMENT_PROGRESS_STATES = frozenset(
    {
        "capture_armed",
        "evaluating",
        "waiting_for_tf",
        # These reject a duplicate/control request, not the currently accepted
        # plane or the capture already owned by the prescan state machine.
        "capture_rejected",
        "capture_ignored",
        "paint_locked",
        "work_area_state_rejected",
        "work_area_corners_rejected",
    }
)
WORK_AREA_STATE_TOPIC = "/painting_system/work_area_state"
PLAN_STATUS_TOPIC = "/painting_system/plan_status"
READINESS_TOPIC = "/painting_system/readiness"
EXECUTION_STATUS_TOPIC = "/painting_system/execution_status"
EXECUTOR_HEARTBEAT_TOPIC = "/painting_admittance/executor_heartbeat"
TRAJECTORY_ACTIVE_TOPIC = "/painting_admittance/trajectory_active"
HARDWARE_MOTION_INHIBITED_TOPIC = "/painting_system/hardware_motion_inhibited"
ROBOT_STATIONARY_TOPIC = "/painting_admittance/robot_stationary"
FREE_SPACE_CONFIRMED_TOPIC = "/painting_admittance/free_space_confirmed"
SAFETY_STATUS_TOPIC = "/painting_admittance/safety_status"
WRENCH_GUARD_STATUS_TOPIC = "/painting_admittance/wrench_guard_status"
CONTROLLER_FAULT_TOPIC = "/painting_admittance/controller_fault"
RUNTIME_FT_TARE_SERVICE = "/rbpodo_ft_tare/runtime_free_space_tare"
FORCE_SAFETY_RESET_SERVICE = "/painting_admittance/reset_safety"
WRENCH_GUARD_RESET_SERVICE = "/painting_admittance/reset_wrench_guard"
ROLLER_CONTACT_LINK = "paint_eoat_no_camera_roller_contact_link"
# Planning-scene apply/restore round trip.  A timeout here latches the ACM
# state as unknown and forces a full stack relaunch, so it must clear the
# worst-case scene service latency: a 0.85 x 0.79 m work area overran the
# original 2.0 s while aborting and cost a relaunch (2026-08-14).
ACM_TRANSACTION_TIMEOUT_S = 8.0
ACM_HEALTH_REFRESH_S = 0.5
ACM_HEALTH_MAX_AGE_S = 1.5
PRECONTACT_DEFERRED_SAFETY_REASONS = {
    "FT_STALE",
    "FT_NONFINITE",
    "TF_INVALID",
}
FORCE_GUARD_STATUS_TIMEOUT_S = 0.50
FORCE_GUARD_MODE_ACK_TIMEOUT_S = 1.00
PAINT_CARTESIAN_PLANNING_TIMEOUT_S = 3.00

SAFETY_OFFSET = 0.08   # 접촉 전 normal 방향 안전거리
RETREAT_OFFSET = 0.08  # Stage 4 후퇴점: 표면 normal 방향 8cm
MIN_APPROACH_NORMAL_ALIGN = 0.97
WORLD_COLLISION_PADDING = 0.02
TARGET_COLLISION_MARGIN = 0.20
TARGET_COLLISION_MIN_THICKNESS = 0.02
TARGET_COLLISION_MAX_THICKNESS = 0.08
TARGET_COLLISION_USE_NOMINAL_EXTENT_AFTER_D405 = False
OBSTACLES_TOPIC = "/perception/obstacles"
PLANES_TOPIC = "/perception/planes"
PLANE_LABELS_TOPIC = "/perception/plane_labels"
WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane"
WORK_AREA_REFINED_PLANE_TOPIC = "/perception/work_area_plane_refined"
WORK_AREA_CORNERS_TOPIC = "/perception/work_area_corners"
REFINE_WORK_AREA_TOPIC = "/refine_work_area"
WORK_AREA_REFINE_STATUS_TOPIC = "/work_area_refine_status"
FT_STATUS_TOPIC = "/ft/status"
FT_ZERO_TOPIC = "/ft/zero"
DEFAULT_JOINT_COMMAND_TOPIC = "/isaac_joint_command"
DEFAULT_FOLLOW_JOINT_TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
MAX_DYNAMIC_OBSTACLES = 80
DYNAMIC_OBSTACLE_PREFIX = "zed_obstacle_"
ROBOT_SELF_FILTER_PADDING = 0.10
ROBOT_LINK_FRAMES = ["link0", "link1", "link2", "link3", "link4", "link5", "link6", "tcp"]
ROBOT_LINK_CAPSULE_RADIUS = {
    ("link0", "link1"): 0.20,
    ("link1", "link2"): 0.20,
    ("link2", "link3"): 0.18,
    ("link3", "link4"): 0.16,
    ("link4", "link5"): 0.15,
    ("link5", "link6"): 0.15,
    ("link6", "tcp"): 0.20,
}
PLANNER_ID = "RRTConnect"
ALLOWED_PLANNING_TIME = 5.0
PLANNING_ATTEMPTS = 5
SCENE_WAIT_TIMEOUT_SEC = 5.0
SCENE_WAIT_PERIOD_SEC = 0.2

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Runtime speed policy.
# Stage 1 is intentionally slow: it is the longest free-space move near humans
# and can otherwise look abrupt on the real RB10.
STAGE1_SPEED_SCALE = 0.025
STAGE2_SPEED_SCALE = 0.020
STAGE3_SPEED_SCALE = 0.060
STAGE4_SPEED_SCALE = 0.025
STAGE5_SPEED_SCALE = 0.040
STAGE1_IK_TIMEOUT_S = 1.0
STAGE1_DUAL_IK_RESPONSE_TIMEOUT_S = 3.0
STAGE1_JOINT_GOAL_TOL = 0.02
STAGE1_LARGE_JOINT_DELTA_WARN_RAD = 1.2
# A collision-free 180-degree task-symmetry branch is allowed when the closer
# branch is blocked.  Only a discontinuous jump *between adjacent commanded
# points* remains a hard guard; total start-to-goal rotation is ranked/logged.
MAX_COMMAND_SEGMENT_JOINT_DELTA_RAD = math.radians(85.0)
STAGE1_MAX_JOINT_PATH_RAD = 2.4
STAGE1_MAX_PLAN_POINTS = 300
STAGE1_CARTESIAN_FRACTION = CARTESIAN_COMPLETE_FRACTION
FT_FORCE_STALE_SEC = 0.75
FT_ABORT_FORCE_N = 30.0
FT_AUTO_ZERO_BEFORE_SKETCH = True
FT_AUTO_ZERO_TIMEOUT_SEC = 6.0
FT_AUTO_ZERO_STALE_SEC = 1.0
FT_AUTO_ZERO_CHECK_PERIOD = 0.05

# 스케치 시작 전 READY_POSE 강제 복귀는 큰 관절 회전을 만들 수 있다.
# 현재 자세에서 첫 safety pose 로 바로 collision-aware plan 한다.
START_FROM_READY_BEFORE_SKETCH = False

# 작업 종료 후에는 READY_POSE 로 크게 복귀하지 않는다.
# Stage 4 에서 작업 위치 근처에서 벽 normal 방향으로만 짧게 빠진다.
RETURN_TO_READY_AFTER_SKETCH = False

# Stage 1 단독 디버그용. production 의 SAFETY_OFFSET 과 분리.
# 토치 미장착 + 첫 실로봇 검증이라 일반보다 보수적으로 잡음.
DEBUG_STAGE1_OFFSET = 0.15  # meters, surface normal 방향 후퇴 거리
# 실제 RB10 pendant/base 와 rbpodo URDF link0 는 Z축 기준 90도 차이가 있다.
# launch 의 world->link0 static TF(+90deg)가 실제 base(world) 좌표를 URDF link0
# 좌표로 변환한다. 디버그 평면은 실제 base/world 기준 x=+0.80 이다.
DEBUG_STAGE1_SURFACE_NORMAL = (-1.0, 0.0, 0.0)

# Joint-space jog 디버그용. motion pipeline 단독 검증.
# 한 joint 만 작게 움직여 OMPL / IK / Cartesian goal 의존성 모두 우회.
JOG_JOINT_INDEX = 5      # wrist3 (가장 국소적, 충돌 위험 최소)
JOG_DELTA_RAD = 0.05     # ≈ 2.9°. 시각적으로 보이지만 무시할 수준
JOG_DURATION_SEC = 10.0  # 10초에 걸쳐 움직임 → 인간 반응 충분
JOG_NUM_POINTS = 50      # 0.2초 간격 보간

# Isaac Sim 의 RB10 OmniGraph 는 /isaac_joint_command(sensor_msgs/JointState)를
# 직접 구독한다. 실제 로봇/ros2_control 이 쓰는 /joint_command 와 분리해서
# 두 publisher 가 같은 Isaac 로봇을 동시에 제어하는 상황을 피한다.
DEFAULT_EXECUTION_BACKEND = "joint_command"  # "joint_command" or "follow_joint_trajectory"
# Isaac Sim 에는 FollowJointTrajectory 대신 joint_command_topic 을 재생한다.
# 100Hz + trajectory time interpolation 으로 계단형 target jump 를 줄인다.
JOINT_COMMAND_TIMER_PERIOD = 0.01
JOINT_COMMAND_APPROACH_MAX_SPEED_RAD_S = 0.040
JOINT_COMMAND_CONTACT_MAX_SPEED_RAD_S = 0.070
JOINT_COMMAND_DEFAULT_MAX_SPEED_RAD_S = 0.050
JOINT_COMMAND_MIN_SEGMENT_DT = 0.05
JOINT_COMMAND_INSERT_START_TOL_RAD = 0.003
JOINT_COMMAND_SLEW_MULTIPLIER = 1.2
JOINT_COMMAND_FINAL_TOL_RAD = 0.002

# RB10 joint 운동학 순서 (URDF 기준).
# 주의: /joint_states 토픽은 알파벳 순으로 발행됨 (base, elbow, shoulder, wrist1, wrist2, wrist3) —
# 이 dict 는 이름 매핑이라 순서 무관, 안전.
# 실제 RB10 pendant/driver 에서 읽은 작업 시작 자세.
READY_POSE_JOINTS = {
    "base":     0.0005,   # pendant: +0.03 deg
    "shoulder": -0.9343,  # pendant: -53.53 deg
    "elbow":    2.4246,   # pendant: +138.92 deg
    "wrist1":  -1.6293,   # pendant: -93.35 deg
    "wrist2":   1.5675,   # pendant: +89.81 deg
    "wrist3":   0.0000,
}

# 실제 RB10 joint.yaml / MoveIt URDF 기준. Isaac 에도 같은 범위 안의 명령만 보낸다.
JOINT_LIMITS = {
    # These bounds match the robot_description produced by the active
    # rb10_1300e_u xacro plus MoveIt's wrist3 override.  MoveIt remains the
    # collision/joint-limit authority; this is the last command-boundary
    # duplicate guard.
    "base": (-3.14, 3.14),
    "shoulder": (-3.14, 3.14),
    "elbow": (-3.14, 3.14),
    "wrist1": (-3.14, 3.14),
    "wrist2": (-3.14, 3.14),
    "wrist3": (-6.28, 6.28),
}
JOINT_LIMIT_MARGIN = 1e-4

CALIBRATION_POSE_JOINTS = dict(READY_POSE_JOINTS)
# MoveIt RB10 URDF currently limits wrist1 to about -pi. The original Isaac
# calibration candidate is slightly below that, so keep the operator preset
# inside MoveIt bounds.
CALIBRATION_POSE_JOINTS["wrist1"] = -3.13

PRESET_POSES = {
    "ready": ("READY_POSE", READY_POSE_JOINTS, STAGE5_SPEED_SCALE),
    "work": ("READY_POSE", READY_POSE_JOINTS, STAGE5_SPEED_SCALE),
    "view": ("READY_POSE", READY_POSE_JOINTS, STAGE5_SPEED_SCALE),
    "calib": ("CALIB_POSE", CALIBRATION_POSE_JOINTS, STAGE5_SPEED_SCALE),
    "calibration": ("CALIB_POSE", CALIBRATION_POSE_JOINTS, STAGE5_SPEED_SCALE),
}

# MoveIt 내부 planning frame 은 URDF link0 이지만, world/World 는 실제 RB10 base.
# launch static TF(world->link0 +90deg) 로 둘을 연결한다.
ROBOT_ORIGIN = (0.0, 0.0, 0.0)

# ---- EoAT 형상 (tcp -> AFT200 -> roller) --------------------------------------
# CAD 는 +Z 로 뻗지만 실제 장착은 TCP local -Y.
TOOL_AXIS = "-y"

# AFT200 collision STL 기반. CAD +Z 52.2mm 를 TCP -Y 로 회전해 사용.
AFT200_LENGTH = 0.0522
if get_package_share_directory is not None:
    try:
        _EOAT_SHARE = Path(get_package_share_directory("eoat_description"))
    except Exception:
        _EOAT_SHARE = Path.home() / "sketch_robot_ws" / "src" / "eoat_description"
else:
    _EOAT_SHARE = Path.home() / "sketch_robot_ws" / "src" / "eoat_description"
AFT200_COLLISION_STL_PATH = str(_EOAT_SHARE / "meshes" / "aft200_collision.stl")
AFT200_SIZE = (0.104, AFT200_LENGTH, 0.082)  # TCP frame box size: x, y, z
AFT200_CENTER = (-0.0116, -AFT200_LENGTH / 2.0, 0.0)

# RR-00A_B__EOAT.step 의 CAD +Z forward reach.
EOAT_NO_CAMERA_COLLISION_STL_PATH = str(
    _EOAT_SHARE / "meshes" / "rr_00a_b_eoat_no_camera_collision_roller52.stl"
)
EOAT_MESH_FORWARD_LENGTH = 0.2305
EOAT_MESH_CENTER_OFFSET = np.array([
    0.0,
    -(AFT200_LENGTH + EOAT_MESH_FORWARD_LENGTH / 2.0),
    0.0,
], dtype=float)
ROLLER_SUPPORT_RADIUS = 0.012
ROLLER_LENGTH = 0.175
ROLLER_RADIUS = 0.026
ROLLER_LONG_AXIS = "+x"  # 벽면 가로(real base Y) 방향.
ROLLER_FORWARD_REACH = 0.192

# TCP → 롤러 회전축 중심까지의 거리.
# Cartesian / IK 가 "tip" 으로 삼는 점 = 롤러 회전축 중심.
# 2026-08 CAD/STL vertex comparison: the rescaled roller axis is at
# EOAT-local z=0.192 m, hence tcp -> roller axis = 52.2 + 192.0 mm.
EOAT_TIP_OFFSET = 0.2442
EOAT_TOTAL_REACH = EOAT_TIP_OFFSET

# Intel RealSense D405 attached on the roller EOAT.
# Camera front(+X in RealSense camera_link) points along TCP local -Y.
D405_SIZE = (0.042, 0.023, 0.042)  # TCP frame bbox: x(width), y(depth), z(height)
D405_COLLISION_CENTER = (0.0, -0.06870, 0.04375)

# EOAT is now part of robot_description as fixed robot links. Keep the runtime
# attached-object path disabled to avoid duplicate self-collision with the URDF
# EOAT links.
PUBLISH_EOAT_ATTACHED_OBJECT = False
EOAT_TOUCH_LINKS = ["tcp", "link6"]


def _cylinder_axis_quat(axis):
    """SolidPrimitive.CYLINDER (default +z) 를 axis 방향으로 회전시키는 quaternion (x,y,z,w).
    shortest-arc quaternion 으로 동적 계산하여 부호 실수 방지."""
    target = {"+x": [1, 0, 0], "-x": [-1, 0, 0],
              "+y": [0, 1, 0], "-y": [0, -1, 0],
              "+z": [0, 0, 1], "-z": [0, 0, -1]}[axis]
    src = np.array([0.0, 0.0, 1.0])
    tgt = np.array(target, dtype=float)
    if np.allclose(src, tgt):
        return (0.0, 0.0, 0.0, 1.0)
    if np.allclose(src, -tgt):
        # 180도 뒤집기 — 회전축은 src 와 직교한 임의 축. X 선택.
        return (1.0, 0.0, 0.0, 0.0)
    q = quat_from_two_vectors(src, tgt)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _axis_offset(axis, distance):
    """axis ('+x'/'-y'/...) 방향으로 distance 만큼 떨어진 점 (x, y, z)."""
    sign = -1.0 if axis.startswith("-") else 1.0
    a = axis[1]
    d = distance * sign
    if a == "x":
        return (d, 0.0, 0.0)
    if a == "y":
        return (0.0, d, 0.0)
    if a == "z":
        return (0.0, 0.0, d)
    raise ValueError(f"unknown axis: {axis}")


_AFT200_MESH_CACHE = None
_EOAT_NO_CAMERA_MESH_CACHE = None


def _cad_z_to_tcp_minus_y_np(x, y, z):
    """CAD +Z forward mesh vertex -> TCP local -Y frame."""
    return np.array([float(x), float(-z), float(y)], dtype=float)


def _load_stl_mesh(stl_path, transform_vertex):
    with open(stl_path, "rb") as f:
        data = f.read()
    if len(data) < 84:
        raise RuntimeError(f"STL 파일이 너무 짧음: {stl_path}")

    tri_count = struct.unpack("<I", data[80:84])[0]
    expected_len = 84 + tri_count * 50
    if expected_len != len(data):
        raise RuntimeError(f"binary STL 길이 불일치: {stl_path}")

    mesh = Mesh()
    off = 84
    for _ in range(tri_count):
        off += 12  # normal
        idx = []
        for _v in range(3):
            x, y, z = struct.unpack("<fff", data[off:off + 12])
            off += 12
            p = transform_vertex(x, y, z)
            mesh.vertices.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            idx.append(len(mesh.vertices) - 1)
        off += 2
        tri = MeshTriangle()
        tri.vertex_indices = idx
        mesh.triangles.append(tri)
    return mesh


def _load_aft200_mesh():
    """Load AFT200 collision STL as a MoveIt mesh in tcp frame."""
    global _AFT200_MESH_CACHE
    if _AFT200_MESH_CACHE is not None:
        return copy.deepcopy(_AFT200_MESH_CACHE)

    mesh = _load_stl_mesh(AFT200_COLLISION_STL_PATH, _cad_z_to_tcp_minus_y_np)
    _AFT200_MESH_CACHE = mesh
    return copy.deepcopy(mesh)


def _load_eoat_no_camera_mesh():
    """Load no-camera EOAT collision STL as a MoveIt mesh in tcp frame."""
    global _EOAT_NO_CAMERA_MESH_CACHE
    if _EOAT_NO_CAMERA_MESH_CACHE is not None:
        return copy.deepcopy(_EOAT_NO_CAMERA_MESH_CACHE)

    mesh = _load_stl_mesh(
        EOAT_NO_CAMERA_COLLISION_STL_PATH,
        lambda x, y, z: _cad_z_to_tcp_minus_y_np(x, y, z) + EOAT_MESH_CENTER_OFFSET,
    )
    _EOAT_NO_CAMERA_MESH_CACHE = mesh
    return copy.deepcopy(mesh)


from sketch_control.spray_execution import SprayExecutionMixin
from sketch_control.multi_surface_execution import MultiSurfaceMixin
from sketch_control.d405_view_geometry import measurement_samples, camera_view


class MoveItExecutor(SprayExecutionMixin, MultiSurfaceMixin, Node):
    def __init__(self):
        super().__init__("moveit_executor")
        self.model_id = validate_model(self.declare_parameter(
            "model_id", DEFAULT_MODEL, ParameterDescriptor(read_only=True)
        ).value)
        self._robot_joint_limits = model_joint_limits(self.model_id)

        self.execution_backend = str(
            self.declare_parameter(
                "execution_backend", DEFAULT_EXECUTION_BACKEND
            ).value
        ).strip()
        if self.execution_backend not in ("joint_command", "follow_joint_trajectory"):
            self.get_logger().warn(
                f"unknown execution_backend={self.execution_backend!r}; "
                f"fallback to {DEFAULT_EXECUTION_BACKEND!r}")
            self.execution_backend = DEFAULT_EXECUTION_BACKEND
        self.joint_command_topic = str(
            self.declare_parameter(
                "joint_command_topic", DEFAULT_JOINT_COMMAND_TOPIC
            ).value
        ).strip()
        self.follow_joint_trajectory_action = str(
            self.declare_parameter(
                "follow_joint_trajectory_action",
                DEFAULT_FOLLOW_JOINT_TRAJECTORY_ACTION,
            ).value
        ).strip()
        self.use_eoat_segments = bool(
            self.declare_parameter("use_eoat_segments", True).value
        )
        self.real_painting_enabled = bool(
            self.declare_parameter("real_painting_enabled", False).value
        )
        self.dry_run = bool(self.declare_parameter("dry_run", True).value)
        requested_force_enable = bool(
            self.declare_parameter("painting_force_enabled", False).value
        )
        # A non-zero controller command requires all three explicit switches.
        self.painting_force_enabled = bool(
            requested_force_enable
            and self.real_painting_enabled
            and not self.dry_run
        )
        if requested_force_enable and not self.painting_force_enabled:
            self.get_logger().warn(
                "painting_force_enabled 요청을 차단함: non-zero force에는 "
                "real_painting_enabled=true, dry_run=false가 모두 필요합니다"
            )
        self.require_contact_before_paint = bool(
            self.declare_parameter("require_contact_before_paint", True).value
        )
        self.segment_contact_offset_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "segment_contact_offset_m", ROLLER_RADIUS + CONTACT_CLEARANCE
                ).value
            ),
        )
        self.minimum_travel_clearance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "minimum_travel_clearance_m", 0.005
                ).value
            ),
        )
        self.max_paint_force_n = max(
            0.0,
            float(self.declare_parameter("max_paint_force_n", 20.0).value),
        )
        self.painting_ramp_feedback_timeout_s = max(
            0.1,
            float(
                self.declare_parameter(
                    "painting_ramp_feedback_timeout_s", 5.0
                ).value
            ),
        )
        self.painting_ramp_settle_s = max(
            0.0,
            float(self.declare_parameter("painting_ramp_settle_s", 0.10).value),
        )
        self.stationary_paint_hold_s = float(
            self.declare_parameter("stationary_paint_hold_s", 0.0).value
        )
        if (
            not math.isfinite(self.stationary_paint_hold_s)
            or self.stationary_paint_hold_s < 0.0
        ):
            raise ValueError("stationary_paint_hold_s must be finite and non-negative")
        self.contact_geometry_offset_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "contact_geometry_offset_m", ROLLER_RADIUS
                ).value
            ),
        )
        self.precontact_clearance_m = max(
            0.0,
            float(
                self.declare_parameter("precontact_clearance_m", 0.010).value
            ),
        )
        self.travel_clearance_m = max(
            self.minimum_travel_clearance_m,
            float(self.declare_parameter("travel_clearance_m", 0.010).value),
        )
        self.safety_approach_offset_m = max(
            self.precontact_clearance_m,
            float(
                self.declare_parameter("safety_approach_offset_m", 0.080).value
            ),
        )
        self.final_retreat_offset_m = max(
            self.travel_clearance_m,
            float(
                self.declare_parameter("final_retreat_offset_m", 0.080).value
            ),
        )
        self.contact_search_config = ContactSearchConfig(
            step_m=float(
                self.declare_parameter("contact_search_step_m", 0.0005).value
            ),
            max_distance_m=float(
                self.declare_parameter(
                    "contact_search_max_distance_m", 0.015
                ).value
            ),
            timeout_s=float(
                self.declare_parameter("contact_search_timeout_s", 10.0).value
            ),
        )
        self.contact_search_speed_mps = max(
            0.0001,
            float(
                self.declare_parameter("contact_search_speed_mps", 0.002).value
            ),
        )
        self.contact_search_mode_ack_timeout_s = max(
            0.2,
            float(
                self.declare_parameter(
                    "contact_search_mode_ack_timeout_s", 1.0
                ).value
            ),
        )
        self.force_guard_status_timeout_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "force_guard_status_timeout_s",
                    FORCE_GUARD_STATUS_TIMEOUT_S,
                ).value
            ),
        )
        self.force_guard_mode_ack_timeout_s = max(
            0.20,
            float(
                self.declare_parameter(
                    "force_guard_mode_ack_timeout_s",
                    FORCE_GUARD_MODE_ACK_TIMEOUT_S,
                ).value
            ),
        )
        self.paint_cartesian_planning_timeout_s = max(
            0.20,
            float(
                self.declare_parameter(
                    "paint_cartesian_planning_timeout_s",
                    PAINT_CARTESIAN_PLANNING_TIMEOUT_S,
                ).value
            ),
        )
        self.ft_required_timeout_s = max(
            0.01, float(self.declare_parameter("ft_timeout_s", 0.20).value)
        )
        self.fjt_result_timeout_margin_s = max(
            0.5,
            float(
                self.declare_parameter(
                    "fjt_result_timeout_margin_s", 5.0
                ).value
            ),
        )
        self.fjt_cancel_timeout_s = max(
            0.5,
            float(
                self.declare_parameter("fjt_cancel_timeout_s", 2.0).value
            ),
        )
        self.stationary_joint_velocity_rad_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "stationary_joint_velocity_rad_s", 0.01
                ).value
            ),
        )
        self.stationary_joint_delta_rad = max(
            0.0,
            float(
                self.declare_parameter("stationary_joint_delta_rad", 0.0002).value
            ),
        )
        self.stationary_hold_s = max(
            0.1,
            float(self.declare_parameter("stationary_hold_s", 0.5).value),
        )
        self.runtime_tare_enabled = bool(
            self.declare_parameter("runtime_tare_enabled", True).value
        )
        self.runtime_tare_service = str(
            self.declare_parameter(
                "runtime_tare_service", RUNTIME_FT_TARE_SERVICE
            ).value
        ).strip()
        self.runtime_tare_timeout_s = max(
            2.0,
            float(self.declare_parameter("runtime_tare_timeout_s", 20.0).value),
        )
        self.runtime_tare_quiet_s = max(
            0.5,
            float(self.declare_parameter("runtime_tare_quiet_s", 3.0).value),
        )
        self.runtime_tare_min_clearance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "runtime_tare_min_clearance_m", 0.010
                ).value
            ),
        )
        tare_geometry_descriptor = ParameterDescriptor(read_only=True)
        self.runtime_tare_min_actual_clearance_m = float(
            self.declare_parameter(
                "runtime_tare_min_actual_clearance_m",
                0.007,
                tare_geometry_descriptor,
            ).value
        )
        self.runtime_tare_max_tcp_position_error_m = float(
            self.declare_parameter(
                "runtime_tare_max_tcp_position_error_m",
                0.003,
                tare_geometry_descriptor,
            ).value
        )
        self.runtime_tare_max_tcp_orientation_error_deg = float(
            self.declare_parameter(
                "runtime_tare_max_tcp_orientation_error_deg",
                3.0,
                tare_geometry_descriptor,
            ).value
        )
        self.runtime_tare_max_tcp_tf_age_s = float(
            self.declare_parameter(
                "runtime_tare_max_tcp_tf_age_s",
                0.20,
                tare_geometry_descriptor,
            ).value
        )
        for name, value, minimum in (
            (
                "runtime_tare_min_actual_clearance_m",
                self.runtime_tare_min_actual_clearance_m,
                0.0,
            ),
            (
                "runtime_tare_max_tcp_position_error_m",
                self.runtime_tare_max_tcp_position_error_m,
                1e-6,
            ),
            (
                "runtime_tare_max_tcp_orientation_error_deg",
                self.runtime_tare_max_tcp_orientation_error_deg,
                1e-6,
            ),
            (
                "runtime_tare_max_tcp_tf_age_s",
                self.runtime_tare_max_tcp_tf_age_s,
                1e-6,
            ),
        ):
            if not math.isfinite(value) or value < minimum:
                raise ValueError(f"{name} must be finite and >= {minimum}")
        self.runtime_tare_verify_duration_s = max(
            0.3,
            float(
                self.declare_parameter(
                    "runtime_tare_verify_duration_s", 0.40
                ).value
            ),
        )
        self.runtime_tare_max_force_norm_n = max(
            0.0,
            float(
                self.declare_parameter(
                    "runtime_tare_max_force_norm_n", 0.75
                ).value
            ),
        )
        self.runtime_tare_max_torque_norm_nm = max(
            0.0,
            float(
                self.declare_parameter(
                    "runtime_tare_max_torque_norm_nm", 0.10
                ).value
            ),
        )

        # I/O
        self.create_subscription(PoseArray, "/sketch_waypoints", self.on_waypoints, 10)
        self.create_subscription(String, EOAT_SEGMENTS_TOPIC, self.on_eoat_segments, 10)
        self.create_subscription(Bool, "/sketch_execute", self.on_execute, 10)
        self.create_subscription(Bool, MOTION_ABORT_TOPIC, self.on_motion_abort, 10)
        self.create_subscription(
            Bool, PAINTING_ABORT_TOPIC, self.on_painting_abort, 10
        )
        self.create_subscription(
            Bool,
            PAINTING_RAMP_COMPLETE_TOPIC,
            self.on_painting_ramp_complete,
            10,
        )
        self.create_subscription(
            String,
            PAINTING_RAMP_STATUS_TOPIC,
            self.on_painting_ramp_status,
            10,
        )
        self.create_subscription(
            Bool, PAINTING_CONTACT_TOPIC, self.on_painting_contact, 10
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            MarkerArray, OBSTACLES_TOPIC, self.on_dynamic_obstacles, 10)
        self.create_subscription(
            PoseArray, PLANES_TOPIC, self.on_perception_planes, 10)
        self.create_subscription(
            String, PLANE_LABELS_TOPIC, self.on_plane_labels, 10)
        self.create_subscription(
            PoseStamped, WORK_AREA_PLANE_TOPIC, self.on_active_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, WORK_AREA_REFINED_PLANE_TOPIC,
            self.on_refined_active_surface, LATCHED_QOS)
        self.create_subscription(
            PoseArray, WORK_AREA_CORNERS_TOPIC, self.on_work_area_corners, LATCHED_QOS)
        self.create_subscription(
            Bool, REFINE_WORK_AREA_TOPIC, self.on_refine_work_area, 10)
        self.create_subscription(String, FT_STATUS_TOPIC, self.on_ft_status, 10)
        self.create_subscription(
            String,
            D405_REFINEMENT_STATUS_TOPIC,
            self.on_d405_refinement_status,
            LATCHED_QOS,
        )
        self.create_subscription(
            String, WORK_AREA_STATE_TOPIC, self.on_work_area_state, LATCHED_QOS
        )
        self.create_subscription(
            String, PLAN_STATUS_TOPIC, self.on_plan_status, LATCHED_QOS
        )
        self.create_subscription(
            String, SAFETY_STATUS_TOPIC, self.on_safety_status, 10
        )
        self.create_subscription(
            String, WRENCH_GUARD_STATUS_TOPIC, self.on_wrench_guard_status, 10
        )
        self.create_subscription(
            Bool, CONTROLLER_FAULT_TOPIC, self.on_controller_fault, 10
        )
        self.create_subscription(
            Bool,
            HARDWARE_MOTION_INHIBITED_TOPIC,
            self.on_hardware_motion_inhibited,
            10,
        )
        self.create_subscription(
            Bool,
            FREE_SPACE_CONFIRMED_TOPIC,
            self.on_free_space_confirmed,
            10,
        )
        # 디버그용 — 실로봇 검증 시 Stage 5 단독 호출용
        self.create_subscription(
            Bool, "/debug_trigger_stage5", self.on_debug_trigger_stage5, 10)
        # 디버그용 — 실로봇 검증 시 Stage 1 단독 호출용
        self.create_subscription(
            Bool, "/debug_trigger_stage1", self.on_debug_trigger_stage1, 10)
        # 디버그용 — motion pipeline 단독 검증 (OMPL/IK 우회 jog)
        self.create_subscription(
            Bool, "/debug_trigger_jog", self.on_debug_trigger_jog, 10)
        # 수동 프리셋 이동 — perception 전에 카메라 시야 확보/캘리브 자세 복귀용.
        self.create_subscription(
            String, "/robot_pose_preset", self.on_robot_pose_preset, 10)
        self.create_subscription(
            Bool, "/go_ready_pose", self.on_go_ready_pose, 10)
        self.create_subscription(
            Bool, "/go_calibration_pose", self.on_go_calibration_pose, 10)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.joint_cmd_pub = self.create_publisher(
            JointState, self.joint_command_topic, 10)
        self.ft_zero_pub = self.create_publisher(Bool, FT_ZERO_TOPIC, 10)
        self.d405_capture_pub = self.create_publisher(
            Bool, D405_REFINE_CAPTURE_TOPIC, 10)
        self.work_area_refine_status_pub = self.create_publisher(
            String, WORK_AREA_REFINE_STATUS_TOPIC, 10)
        self.painting_mode_pub = self.create_publisher(
            String, PAINTING_MODE_TOPIC, 10
        )
        self.painting_force_pub = self.create_publisher(
            Float64, PAINTING_FORCE_TOPIC, 10
        )
        self.painting_enable_pub = self.create_publisher(
            Bool, PAINTING_ENABLE_TOPIC, 10
        )
        self.motion_abort_pub = self.create_publisher(Bool, MOTION_ABORT_TOPIC, 10)
        self.executor_heartbeat_pub = self.create_publisher(
            Bool, EXECUTOR_HEARTBEAT_TOPIC, 10
        )
        self.trajectory_active_pub = self.create_publisher(
            Bool, TRAJECTORY_ACTIVE_TOPIC, 10
        )
        self.robot_stationary_pub = self.create_publisher(
            Bool, ROBOT_STATIONARY_TOPIC, 10
        )
        self.free_space_confirmed_pub = self.create_publisher(
            Bool, FREE_SPACE_CONFIRMED_TOPIC, 10
        )
        self.readiness_pub = self.create_publisher(
            String, READINESS_TOPIC, LATCHED_QOS
        )
        self.execution_status_pub = self.create_publisher(
            String, EXECUTION_STATUS_TOPIC, LATCHED_QOS
        )
        self.create_service(
            Trigger,
            "/painting_system/reset_execution_abort",
            self.on_reset_execution_abort,
        )

        # MoveIt endpoints (계획만 사용)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik")

        # MoveGroup action client (Stage 1: 자유 경로 planning)
        self.move_action_client = ActionClient(self, MoveGroup, "/move_action")

        # ApplyPlanningScene service client (실제 MoveIt collision detection 등록)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene")
        self.list_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.runtime_ft_tare_client = self.create_client(
            Trigger, self.runtime_tare_service
        )
        self.force_safety_reset_client = self.create_client(
            Trigger, FORCE_SAFETY_RESET_SERVICE
        )
        self.wrench_guard_reset_client = self.create_client(
            Trigger, WRENCH_GUARD_RESET_SERVICE
        )

        # FollowJointTrajectory action client (RB10 driver 의 joint_trajectory_controller)
        self.traj_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.follow_joint_trajectory_action,
        )
        self.get_logger().info(
            f"execution_backend={self.execution_backend}, "
            f"joint_command_topic={self.joint_command_topic}, "
            f"follow_joint_trajectory_action={self.follow_joint_trajectory_action}")

        # 다음 stage 로 넘기는 데 쓰는 상태
        self._safety_tcp_pose = None
        self._retreat_tcp_pose = None
        self._stage3_tcp_wps = None  # Stage 2 끝났을 때 stage 3 가 쓸 waypoints
        self._stage3_tip_wps = None
        self._stage1_retried = False
        self._stage1_goal_constraints = None
        self._stage1_scene_wait_timer = None
        self._stage1_scene_wait_start = None
        self._stage1_ik_candidate_generation = 0
        self._stage1_ik_candidate_results = {}
        self._stage1_ik_candidates_finalized_generation = -1
        self._stage1_ik_candidate_timer = None
        self._stage1_ik_seed_state = None
        self._stage1_attempt_token = None
        self._stage1_orientation_candidates = ()
        self._stage1_orientation_ranked = []
        self._stage1_orientation_rank_index = -1
        self._selected_segment_orientation_branch = ""
        self._stage1_orientation_branch_frozen = False
        self._joint_goal_context = None
        self._pose_goal_context = None
        self._d405_cartesian_context = None
        self._d405_prescan_active = False
        self._d405_prescan_queue = []
        self._d405_prescan_surface_points = []
        self._d405_prescan_surface_normal = None
        self._d405_prescan_index = 0
        self._d405_prescan_timer = None
        self._d405_prescan_wait_start = None
        self._d405_prescan_arrived_time = 0.0
        self._d405_prescan_capture_sent = False
        self._d405_prescan_capture_time = 0.0
        self._d405_prescan_mode = "sketch"
        self._d405_prescan_token = None
        self._d405_scene_wait_timer = None
        self._d405_scene_wait_start = None
        self._d405_scene_wait_revision = None
        self._d405_orientation_generation = 0
        self._d405_orientation_context = None
        self._d405_orientation_timer = None
        self._d405_arrival_timer = None
        self._d405_selected_orientation_branch = ""
        self._d405_refined_generation = 0
        self._d405_refined_lock_active = False
        self._d405_refined_lock_time = 0.0
        self._d405_refined_lock_signature = None
        self._d405_refined_allow_corner_correction = False
        self._d405_refined_pose_armed = False
        self._d405_refined_pose_generation_id = ""
        self._work_area_refine_wait_timer = None
        self._work_area_refine_request_time = 0.0
        self._work_area_refine_previous_corner_signature = None
        self._motion_abort_requested = False
        self._execution_abort_reason = ""
        self._active_trajectory_goal_handle = None
        self._active_trajectory_goal_token = None
        self._active_trajectory_label = ""
        self._active_trajectory_result_future = None
        self._active_trajectory_result_consumer = None
        self._active_trajectory_terminal_committed = False
        self._active_trajectory_cancel_requested = False
        self._active_trajectory_cancel_reason = ""
        self._active_trajectory_result_deadline = 0.0
        self._active_trajectory_cancel_deadline = 0.0
        self._fjt_guard_timer = None
        self._fjt_motion_state_unknown = False
        self._last_dispatch_inhibit_reason = ""
        self._last_trajectory_failure_phase = ""
        if getattr(self, "process_mode", "paint") == "spray":
            blockers = self._spray_motion_blockers()
            if blockers:
                self._last_dispatch_inhibit_reason = ",".join(blockers)
                self._spray_off()
                return False
            complete, failure, rejected = on_complete, on_failure, on_rejected
            def finish(callback):
                self._spray_off()
                if callback is not None:
                    callback()
            on_complete = lambda: finish(complete)
            on_failure = lambda: finish(failure)
            on_rejected = lambda: finish(rejected)
        self._hardware_motion_inhibited = False
        self._hardware_motion_inhibit_time = 0.0
        self._segment_path = None
        self._segment_path_received_at = 0.0
        self._waypoints_received_at = 0.0
        self._waypoints_path_id = ""
        self._active_segment_path = None
        # A real Run consumes one immutable link0 geometry snapshot.  D405 is
        # eye-in-hand, so camera-frame observations may continue to change as
        # the arm moves; they must never rewrite the accepted execution.
        self._execution_snapshot = None
        self._execution_candidate_invalidated = False
        self._execution_candidate_invalidation_reason = ""
        self._process_steps = []
        self._process_step_index = 0
        self._process_row_tcp_poses = {}
        self._process_last_tcp_pose = None
        self._process_force_ready = False
        self._contact_search_confirmed = False
        self._contact_search_distance_m = 0.0
        self._process_timer = None
        self._paint_entry_context = None
        # Force/compliance ownership is independent from planning and FJT
        # timers.  It starts at the RAMP_UP enable edge and remains alive until
        # a source-stamped post-disable RAMP_DOWN zero ACK is observed.
        self._force_phase_lease_context = None
        self._force_phase_watchdog_timer = None
        self._segment_cartesian_timeout_timer = None
        # The selective roller<->wall ACM allowance must remain active while
        # leaving a physical contact state.  It is restored only after the
        # hashed, normal-outward RETRACT/FINAL_RETRACT has completed.
        self._contact_escape_context = None
        self._stationary_paint_hold_started_at = 0.0
        self._stationary_paint_hold_samples = []
        self._segment_cartesian_context = None
        self._painting_ramp_complete = False
        self._painting_ramp_feedback_seq = 0
        self._painting_ramp_feedback_time = 0.0
        self._painting_ramp_status = {}
        self._painting_ramp_status_time = 0.0
        self._painting_ramp_status_source_time = 0.0
        self._painting_ramp_status_sequence = 0
        self._painting_contact_confirmed = False
        self._painting_contact_feedback_time = 0.0
        self._painting_command_mode = "IDLE"
        self._painting_command_force_n = 0.0
        self._painting_command_enable = False
        self._painting_command_generation = 0
        self._painting_command_context = None
        self._contact_search_command_context = None
        self._contact_search_started_at = 0.0
        self._contact_search_distance_m = 0.0
        self._contact_search_step_active = False
        self._contact_search_confirmed = False
        self._contact_search_context = None
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        self._contact_search_mode_published_at = 0.0
        self._contact_collision_allowed = False
        self._contact_collision_baseline = None
        self._contact_collision_target_name = ""
        self._acm_update_seq = 0
        self._acm_update_pending = None
        self._acm_update_timer = None
        self._acm_state_unknown = False
        # A contact transaction validates the live MoveIt ACM again, but that
        # is too late for Stage 1 / APPROACH_PRECONTACT.  Verify the clean
        # SRDF baseline asynchronously as soon as the executor starts and keep
        # every physical dispatch blocked until the authoritative
        # GetPlanningScene response has been checked.
        self._acm_baseline_verified = False
        self._acm_baseline_verified_target_name = ""
        self._acm_baseline_verified_time = 0.0
        self._acm_health_query_seq = 0
        self._acm_health_query_pending = None
        self._acm_health_query_timer = None
        self._latest_allowed_collision_matrix = None
        self._required_acm_allowed_pairs = (
            MoveItExecutor._load_srdf_allowed_collision_pairs(self.model_id)
        )
        if not self._required_acm_allowed_pairs:
            self.get_logger().error(
                "[ACM] SRDF disabled-collision baseline unavailable; "
                "roller contact will remain fail-closed"
            )
        self._execution_state = "IDLE"

        self.current_waypoints = []
        self.current_joint_state = None
        self.current_joint_state_time = 0.0
        self._joint_motion_last_time = time.monotonic()
        self._free_space_confirmed = False
        self._free_space_confirmed_time = 0.0
        self._execution_free_space_confirmed = False
        self._execution_tare_ready = False
        self._runtime_tare_phase = "IDLE"
        self._runtime_tare_token = None
        self._runtime_tare_timer = None
        self._runtime_tare_started_at = 0.0
        self._runtime_tare_quiet_started_at = 0.0
        self._runtime_tare_phase_started_at = 0.0
        self._runtime_tare_verify_started_at = 0.0
        self._runtime_tare_context = None
        self._runtime_tare_actual_metrics = {}
        self._joint_command_timer = None
        self.scene_initialized = False
        self.scene_confirmed = False
        self._scene_revision = 0
        self._scene_confirmed_revision = -1
        self._scene_apply_inflight_revision = None
        self.executing = False
        self.dynamic_obstacles = []
        self._dynamic_obstacle_ids = set()
        self._stale_dynamic_obstacle_ids = set()
        self._dynamic_obstacle_signature = None
        self.perception_planes = []
        self.perception_plane_labels = []
        self.dynamic_surface_point = None
        self.dynamic_surface_normal = None
        self.dynamic_surface_source = "fallback"
        self.dynamic_surface_source_time = 0.0
        self.dynamic_work_area_corners = None
        self._work_area_corners_signature = None
        self._pending_surface_msg = None
        self._pending_surface_source = "zed"
        self._pending_corners_msg = None
        self.ft_normal_force_n = None
        self.ft_contact = False
        self.ft_bias_ready = False
        self.ft_abort_force_n = FT_ABORT_FORCE_N
        self.ft_status_time = 0.0
        self.ft_state = "unknown"
        self._ft_auto_zero_timer = None
        self._current_work_area_id = ""
        self._current_plane_generation_id = ""
        self._d405_plane_accepted = False
        self._d405_status_time = 0.0
        self._accepted_plan_hash = ""
        self._accepted_plan_path_id = ""
        self._plan_status_time = 0.0
        self._safety_status = {}
        self._safety_status_time = 0.0
        self._guard_status = {}
        self._guard_status_time = 0.0
        self._guard_status_source_time = 0.0
        self._guard_status_sequence = 0
        self._controller_fault = True
        self._controller_fault_time = 0.0
        self._controller_states = {}
        self._controller_states_time = 0.0
        self._controller_query_pending = False

        # ---- Target registry ----
        self.cfg = load_objects_config()
        self.active_target_name = self.cfg.get("active_target", "wall")
        self._enabled_ids = set(
            o["name"] for o in self.cfg["objects"] if o.get("enabled", True)
        )

        # PlanningScene 모니터링
        self.create_subscription(
            PlanningScene, "/monitored_planning_scene",
            self.on_scene_update, 10)

        self.create_timer(1.0, self.publish_scene_periodic)

        # ---- 진단: 현재 tcp TF 를 3초마다 출력 (수동 캘리브레이션 용) ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self._retry_pending_surface_tf)
        self.create_timer(3.0, self._log_current_tcp)
        self.create_timer(0.05, self._publish_executor_heartbeat)
        self._init_spray()
        self._init_multi_surface()
        self.create_timer(0.5, self._update_and_publish_readiness)

        self.get_logger().info(
            f"MoveIt Executor 노드 시작 "
            f"(planning=MoveIt, execution={self.execution_backend}, "
            f"segment_path={self.use_eoat_segments}, "
            f"painting_force={self.painting_force_enabled})")

    def _log_current_tcp(self):
        """world → tcp TF 를 주기적으로 로그. 수동 캘리브레이션 시 사용."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", EE_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
            )
        except Exception:
            # world 프레임 없으면 World 대문자 시도
            try:
                tf = self.tf_buffer.lookup_transform(
                    "World", EE_LINK, rclpy.time.Time(),
                    timeout=Duration(seconds=0.3),
                )
            except Exception:
                return

        q = tf.transform.rotation
        p = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        local_x = (1 - 2 * (y * y + z * z),
                   2 * (x * y + z * w),
                   2 * (x * z - y * w))
        local_y = (2 * (x * y - z * w),
                   1 - 2 * (x * x + z * z),
                   2 * (y * z + x * w))
        local_z = (2 * (x * z + y * w),
                   2 * (y * z - x * w),
                   1 - 2 * (x * x + y * y))
        self.get_logger().info(
            f"[TCP_NOW] pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) "
            f"quat=({q.x:+.3f},{q.y:+.3f},{q.z:+.3f},{q.w:+.3f})"
        )
        self.get_logger().info(
            f"          local_X_in_world=({local_x[0]:+.2f},{local_x[1]:+.2f},{local_x[2]:+.2f})"
        )
        self.get_logger().info(
            f"          local_Y_in_world=({local_y[0]:+.2f},{local_y[1]:+.2f},{local_y[2]:+.2f})"
        )
        self.get_logger().info(
            f"          local_Z_in_world=({local_z[0]:+.2f},{local_z[1]:+.2f},{local_z[2]:+.2f})"
        )

    # ---- callbacks ----------------------------------------------------------
    @staticmethod
    def _execution_snapshot_updates_locked(executor, source="perception"):
        """Return True while candidate updates must not touch the active Run."""

        snapshot = getattr(executor, "_execution_snapshot", None)
        if not isinstance(snapshot, dict):
            return False
        get_logger = getattr(executor, "get_logger", None)
        if callable(get_logger):
            message = (
                f"[EXECUTION SNAPSHOT] {source} 갱신 무시 — "
                "Run 시작 시 확정한 link0 평면/작업영역/경로 유지"
            )
            try:
                get_logger().warn(message, throttle_duration_sec=2.0)
            except TypeError:
                # Lightweight unit-test loggers do not expose rclpy kwargs.
                get_logger().warn(message)
        return True

    @staticmethod
    def _defer_candidate_invalidation(executor, reason):
        """Block reuse after this Run without mutating its frozen snapshot."""

        executor._execution_candidate_invalidated = True
        executor._execution_candidate_invalidation_reason = str(reason)

    def _capture_execution_snapshot(self, path):
        """Freeze the accepted v3 plan and its link0 geometry for this Run."""

        if path is None or getattr(path, "version", 0) < 3:
            return "immutable v3 segment path unavailable"
        if str(getattr(path, "frame_id", "")) != BASE_FRAME:
            return f"segment frame must be {BASE_FRAME}"
        try:
            point = np.asarray(self.dynamic_surface_point, dtype=float)
            normal = np.asarray(self.dynamic_surface_normal, dtype=float)
            corners = np.asarray(self.dynamic_work_area_corners, dtype=float)
        except (TypeError, ValueError):
            return "accepted link0 plane/work-area geometry unavailable"
        normal_norm = float(np.linalg.norm(normal))
        if (
            point.shape != (3,)
            or normal.shape != (3,)
            or corners.shape != (4, 3)
            or not np.all(np.isfinite(point))
            or not np.all(np.isfinite(normal))
            or not np.all(np.isfinite(corners))
            or not math.isfinite(normal_norm)
            or normal_norm < 1e-9
        ):
            return "accepted link0 plane/work-area geometry is invalid"
        frozen_path = copy.deepcopy(path)
        self._execution_snapshot = {
            "segment_path": frozen_path,
            "waypoints": tuple(copy.deepcopy(self.current_waypoints)),
            "surface_point": point.copy(),
            "surface_normal": (normal / normal_norm).copy(),
            "work_area_corners": corners.copy(),
            "surface_source": str(self.dynamic_surface_source),
            "path_id": str(frozen_path.path_id),
            "plan_hash": str(frozen_path.plan_hash),
            "work_area_id": str(frozen_path.work_area_id),
            "plane_generation_id": str(frozen_path.plane_generation_id),
        }
        self._active_segment_path = frozen_path
        self._execution_candidate_invalidated = False
        self._execution_candidate_invalidation_reason = ""
        # Do not let a pre-Run TF retry overwrite the next candidate after the
        # accepted snapshot has finished.
        self._pending_surface_msg = None
        self._pending_corners_msg = None
        self.current_waypoints = list(copy.deepcopy(
            self._execution_snapshot["waypoints"]
        ))
        return ""

    def _clear_execution_snapshot(self):
        deferred = bool(getattr(self, "_execution_candidate_invalidated", False))
        self._execution_snapshot = None
        if deferred:
            # The current Run is over.  A lifecycle change seen while it was
            # locked now invalidates only the *next* candidate, forcing an
            # explicit work-area refine and path regeneration before reuse.
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
            self._d405_plane_accepted = False
        self._execution_candidate_invalidated = False
        self._execution_candidate_invalidation_reason = ""

    def _execution_surface_geometry(self):
        snapshot = getattr(self, "_execution_snapshot", None)
        if isinstance(snapshot, dict):
            return (
                np.asarray(snapshot["surface_point"], dtype=float).copy(),
                np.asarray(snapshot["surface_normal"], dtype=float).copy(),
                np.asarray(
                    snapshot["work_area_corners"], dtype=float
                ).copy(),
            )
        return (
            getattr(self, "dynamic_surface_point", None),
            getattr(self, "dynamic_surface_normal", None),
            getattr(self, "dynamic_work_area_corners", None),
        )

    def on_waypoints(self, msg: PoseArray):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "waypoints"
        ):
            MoveItExecutor._defer_candidate_invalidation(
                self, "NEW_WAYPOINTS_DURING_EXECUTION"
            )
            return
        if self.executing:
            self.get_logger().error(
                "/sketch_waypoints 실행 중 수신 -> 현재 plan에 반영하지 않음"
            )
            self._publish_execution_status(
                "PATH_REJECTED", "NEW_WAYPOINTS_DURING_EXECUTION"
            )
            return
        # sketch_to_waypoints 는 Isaac World 좌표로 waypoint 를 만든다.
        # MoveIt planning frame 은 URDF link0 이다. World/world 는 실제 RB10 base 이므로
        # static TF(world->link0 +90deg) 변환을 거쳐 보관한다.
        frame = self._canonical_world_frame(msg.header.frame_id or BASE_FRAME)
        transform = None
        if frame != BASE_FRAME:
            transform = self._lookup_transform_to_base(frame, timeout_s=0.5)
            if transform is None:
                self.get_logger().error(
                    f"/sketch_waypoints TF 실패 ({BASE_FRAME}<-{frame}) "
                    "-> waypoint 폐기")
                return

        converted = []
        for p in msg.poses:
            bp = self._transform_pose_msg_to_base(p, transform)
            converted.append(bp)
        self.current_waypoints = converted
        self._waypoints_received_at = time.monotonic()
        received_path_id = self._stamp_path_id(msg.header.stamp)
        # sketch_to_waypoints publishes segment -> plan status -> PoseArray.
        # Preserve an already accepted status only when this PoseArray carries
        # the exact same path identity; otherwise invalidate the old plan.
        if (
            self._accepted_plan_path_id
            and received_path_id != self._accepted_plan_path_id
        ):
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
        self._waypoints_path_id = received_path_id
        if not self.current_waypoints:
            self.get_logger().warn("빈 /sketch_waypoints 수신")
            return
        self.get_logger().info(
            f"{len(self.current_waypoints)}개 웨이포인트 수신 "
            f"({frame}->{BASE_FRAME}: 첫 점 "
            f"x={self.current_waypoints[0].position.x:.2f} "
            f"y={self.current_waypoints[0].position.y:.2f} "
            f"z={self.current_waypoints[0].position.z:.2f})")

    @staticmethod
    def _stamp_path_id(stamp) -> str:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return str(stamp_ns) if stamp_ns > 0 else ""

    def on_eoat_segments(self, msg: String):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "segment path"
        ):
            MoveItExecutor._defer_candidate_invalidation(
                self, "NEW_PATH_DURING_EXECUTION"
            )
            return
        if self.executing:
            self.get_logger().error(
                "[PAINT PATH] 실행 중 새 segment path 수신 -> 현재 실행에 반영하지 않음"
            )
            self._publish_execution_status(
                "PATH_REJECTED", "NEW_PATH_DURING_EXECUTION"
            )
            return
        received_at = time.monotonic()
        try:
            path = parse_segment_path(
                msg.data,
                default_contact_offset_m=self.segment_contact_offset_m,
                max_force_n=self.max_paint_force_n,
                minimum_clearance_m=self.minimum_travel_clearance_m,
                warn=lambda text: self.get_logger().warn(
                    f"[PAINT PATH] {text}"
                ),
                allow_legacy=not self.real_painting_enabled,
            )
            if path.process_mode != getattr(self, "process_mode", "paint"):
                raise SegmentPathError("path process mode differs from executor")
            if path.frame_id != BASE_FRAME:
                frame = self._canonical_world_frame(path.frame_id)
                transform = self._lookup_transform_to_base(frame, timeout_s=0.5)
                if transform is None:
                    raise SegmentPathError(
                        f"TF unavailable: {BASE_FRAME} <- {frame}"
                    )
                q = transform.transform.rotation
                t = transform.transform.translation
                path = transform_segment_path(
                    path,
                    (q.x, q.y, q.z, q.w),
                    (t.x, t.y, t.z),
                    BASE_FRAME,
                )
            unsafe_clearance = [
                row
                for row in path.rows
                if row.mode in CLEARANCE_MODES
                and row.offset_m < self.minimum_travel_clearance_m
            ]
            if unsafe_clearance:
                first = unsafe_clearance[0]
                raise SegmentPathError(
                    f"row {first.row_number} {first.mode} clearance "
                    f"{first.offset_m:.4f} m is below required "
                    f"{self.minimum_travel_clearance_m:.4f} m"
                )
            if self.real_painting_enabled:
                validate_segment_path_for_real_execution(path)
                expected_geometry = (
                    (
                        "contact_geometry_offset_m",
                        path.contact_geometry_offset_m,
                        self.contact_geometry_offset_m,
                    ),
                    (
                        "precontact_clearance_m",
                        path.precontact_clearance_m,
                        self.precontact_clearance_m,
                    ),
                    (
                        "travel_clearance_m",
                        path.travel_clearance_m,
                        self.travel_clearance_m,
                    ),
                    (
                        "safety_approach_offset_m",
                        path.safety_approach_offset_m,
                        self.safety_approach_offset_m,
                    ),
                    (
                        "final_retreat_offset_m",
                        path.final_retreat_offset_m,
                        self.final_retreat_offset_m,
                    ),
                )
                for field, actual, expected in expected_geometry:
                    if path.process_mode == "spray" and field != "contact_geometry_offset_m":
                        expected = 0.5
                    if abs(actual - expected) > 1e-6:
                        raise SegmentPathError(
                            f"{field} differs from executor config: "
                            f"{actual:.6f} != {expected:.6f}"
                        )
        except SegmentPathError as exc:
            self._segment_path = None
            self._segment_path_received_at = received_at
            self.get_logger().error(f"[PAINT PATH] rejected: {exc}")
            self._publish_execution_status("PATH_REJECTED", str(exc))
            return

        if (
            self._accepted_plan_path_id
            and (
                path.path_id != self._accepted_plan_path_id
                or path.plan_hash != self._accepted_plan_hash
            )
        ):
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
        self._segment_path = path
        self._segment_path_received_at = received_at
        self.get_logger().info(
            f"[PAINT PATH] {len(path.rows)} rows accepted "
            f"(version={path.version}, frame={path.frame_id}, "
            f"path_id={path.path_id or 'legacy'})"
        )
        self._publish_execution_status(
            "PATH_RECEIVED",
            "",
            path_id=path.path_id,
            plan_hash=path.plan_hash,
        )

    def on_painting_abort(self, msg: Bool):
        if msg.data and getattr(self, "process_mode", "paint") != "spray":
            self._request_motion_abort("painting_force_monitor")

    def on_painting_ramp_complete(self, msg: Bool):
        # Legacy diagnostic only. Physical ramp completion is accepted solely
        # from the source-stamped `/painting_admittance/ramp_status` contract.
        self._painting_ramp_complete = bool(msg.data)
        self._painting_ramp_feedback_time = time.monotonic()

    def on_painting_ramp_status(self, msg: String):
        received_at = time.monotonic()
        try:
            payload = json.loads(msg.data or "{}")
            if not isinstance(payload, dict):
                raise TypeError("ramp status must be a JSON object")
            source_time = float(payload.get("published_monotonic_s"))
            sequence = payload.get("status_sequence")
            valid = bool(
                math.isfinite(source_time)
                and source_time > 0.0
                and source_time <= received_at + 0.05
                and isinstance(sequence, int)
                and not isinstance(sequence, bool)
                and sequence > 0
                and isinstance(payload.get("mode"), str)
                and payload.get("ramp_complete") in {True, False}
                and payload.get("force_enable") in {True, False}
            )
        except (TypeError, ValueError, OverflowError, json.JSONDecodeError):
            payload = {}
            source_time = 0.0
            sequence = 0
            valid = False
        if not valid or source_time <= self._painting_ramp_status_source_time:
            return
        self._painting_ramp_status = payload
        self._painting_ramp_status_time = received_at
        self._painting_ramp_status_source_time = source_time
        self._painting_ramp_status_sequence = int(sequence)
        self._painting_ramp_complete = payload["ramp_complete"] is True
        self._painting_ramp_feedback_seq += 1
        self._painting_ramp_feedback_time = received_at

    def on_painting_contact(self, msg: Bool):
        self._painting_contact_confirmed = bool(msg.data)
        self._painting_contact_feedback_time = time.monotonic()

    def on_joint_state(self, msg: JointState):
        previous = self.current_joint_state
        now = time.monotonic()
        moved = previous is None
        if previous is not None:
            old_by_name = {
                name: float(position)
                for name, position in zip(previous.name, previous.position)
                if math.isfinite(float(position))
            }
            compared = 0
            for name, position in zip(msg.name, msg.position):
                value = float(position)
                if not math.isfinite(value) or name not in old_by_name:
                    moved = True
                    break
                compared += 1
                if abs(value - old_by_name[name]) > self.stationary_joint_delta_rad:
                    moved = True
                    break
            if compared == 0:
                moved = True
        if not joint_velocities_allow_stationary(
            msg.velocity,
            expected_count=len(msg.position),
            max_abs_velocity_rad_s=self.stationary_joint_velocity_rad_s,
        ):
            moved = True
        if moved:
            self._joint_motion_last_time = now
        self.current_joint_state = msg
        self.current_joint_state_time = now

    def on_free_space_confirmed(self, msg: Bool):
        self._free_space_confirmed = bool(msg.data)
        self._free_space_confirmed_time = time.monotonic()

    def on_ft_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.ft_state = str(payload.get("state", "unknown"))
        self.ft_status_time = time.monotonic()
        if "bias_ready" in payload:
            self.ft_bias_ready = bool(payload.get("bias_ready", False))
        if not payload.get("ok", False):
            self.ft_normal_force_n = None
            self.ft_contact = False
            return
        self.ft_normal_force_n = float(payload.get("normal_force_n", 0.0))
        self.ft_contact = bool(payload.get("contact", False))
        self.ft_abort_force_n = float(
            payload.get("abort_force_n", FT_ABORT_FORCE_N))

    def _abort_active_plan_invalidation(self, reason):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, f"plan invalidation ({reason})"
        ):
            return
        if (
            not self.real_painting_enabled
            or not self.executing
            or self._motion_abort_requested
        ):
            return
        self._set_contact_collision_allowed(False)
        self._request_motion_abort(f"PLAN_INVALIDATED:{reason}")

    def on_work_area_state(self, msg: String):
        if getattr(self, "_multi_current", None) is not None or getattr(self, "_multi_queue", []):
            # Target scanning owns an immutable candidate plane; delayed
            # work-area invalidations belong to the previous drawing.
            return
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "work-area state"
        ):
            try:
                pending = json.loads(msg.data or "{}")
            except (TypeError, ValueError, json.JSONDecodeError):
                pending = {}
            snapshot = self._execution_snapshot
            if (
                not isinstance(pending, dict)
                or pending.get("selected") is not True
                or str(pending.get("work_area_id", "")).strip()
                != str(snapshot.get("work_area_id", ""))
            ):
                MoveItExecutor._defer_candidate_invalidation(
                    self, "WORK_AREA_CHANGED_DURING_EXECUTION"
                )
            return
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        selected = bool(payload.get("selected", False))
        work_area_id = str(payload.get("work_area_id", "")).strip()
        if not selected or not work_area_id:
            self._free_space_confirmed = False
            self._reset_d405_refined_lock("work area cleared", clear_surface=True)
            self._current_work_area_id = ""
            self._current_plane_generation_id = ""
            self._d405_plane_accepted = False
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
            self._publish_execution_status("PLAN_INVALIDATED", "WORK_AREA_CLEARED")
            self._abort_active_plan_invalidation("WORK_AREA_CLEARED")
            return
        if work_area_id != self._current_work_area_id:
            self._free_space_confirmed = False
            self._reset_d405_refined_lock("work area changed", clear_surface=True)
            self._current_work_area_id = work_area_id
            self._current_plane_generation_id = ""
            self._d405_plane_accepted = False
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
            self._publish_execution_status("PLAN_INVALIDATED", "WORK_AREA_CHANGED")
            self._abort_active_plan_invalidation("WORK_AREA_CHANGED")

    def on_d405_refinement_status(self, msg: String):
        if getattr(self, "_multi_current", None) is not None or getattr(self, "_multi_queue", []):
            # Target scanning owns an immutable candidate plane; delayed
            # work-area invalidations belong to the previous drawing.
            return
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "D405 refinement status"
        ):
            try:
                pending = json.loads(msg.data or "{}")
            except (TypeError, ValueError, json.JSONDecodeError):
                pending = {}
            snapshot = self._execution_snapshot
            pending_mode = (
                str(pending.get("mode", "")).strip().lower()
                if isinstance(pending, dict)
                else ""
            )
            if pending_mode and pending_mode != "work_area":
                return
            is_same_accepted_generation = bool(
                isinstance(pending, dict)
                and pending_mode == "work_area"
                and pending.get("accepted") is True
                and str(pending.get("work_area_id", "")).strip()
                == str(snapshot.get("work_area_id", ""))
                and str(pending.get("plane_generation_id", "")).strip()
                == str(snapshot.get("plane_generation_id", ""))
            )
            if not is_same_accepted_generation:
                MoveItExecutor._defer_candidate_invalidation(
                    self, "D405_GENERATION_CHANGED_DURING_EXECUTION"
                )
            return
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            self._d405_plane_accepted = False
            self._d405_status_time = time.monotonic()
            self._abort_active_plan_invalidation("D405_STATUS_INVALID")
            return
        if (
            not isinstance(payload, dict)
            or str(payload.get("mode", "")).strip().lower()
            != "work_area"
        ):
            return
        self._d405_status_time = time.monotonic()
        accepted = payload.get("accepted") is True
        state = str(payload.get("state", "")).strip().lower()
        work_area_id = str(payload.get("work_area_id", "")).strip()
        generation_id = str(payload.get("plane_generation_id", "")).strip()
        prescan_active = bool(getattr(self, "_d405_prescan_active", False))

        # A D405 capture is a multi-message transaction.  In particular,
        # capture_armed is intentionally published with accepted=false before
        # the next cloud is evaluated.  These states must not invalidate the
        # last committed plane or trip the global motion-abort latch.
        if not accepted and state in D405_REFINEMENT_PROGRESS_STATES:
            return

        # Transient-local delivery can replay an accepted result belonging to
        # the previous work area.  It is stale candidate data, not a rejection
        # of the current work area.
        if accepted and work_area_id != self._current_work_area_id:
            return
        if (
            accepted
            and work_area_id
            and generation_id
            and work_area_id == self._current_work_area_id
        ):
            changed = generation_id != self._current_plane_generation_id
            if changed:
                self._reset_d405_refined_lock(
                    "new accepted plane generation", clear_surface=True
                )
            self._d405_plane_accepted = True
            self._current_plane_generation_id = generation_id
            if changed or not self._d405_refined_lock_active:
                self._d405_refined_pose_armed = True
                self._d405_refined_pose_generation_id = generation_id
            if changed and self._accepted_plan_hash:
                self._accepted_plan_hash = ""
                self._accepted_plan_path_id = ""
                self._publish_execution_status(
                    "PLAN_INVALIDATED", "PLANE_GENERATION_CHANGED"
                )
            if changed and not prescan_active:
                self._abort_active_plan_invalidation(
                    "PLANE_GENERATION_CHANGED"
                )
            return
        self._d405_plane_accepted = False
        self._d405_refined_pose_armed = False
        self._d405_refined_pose_generation_id = ""
        if work_area_id == self._current_work_area_id:
            self._current_plane_generation_id = generation_id
        self._accepted_plan_hash = ""
        self._accepted_plan_path_id = ""
        # During D405 prescan the executor itself owns this perception update.
        # A rejected capture is handled by its existing timeout/next-pose
        # retry; a newly accepted generation is the expected success result.
        # External changes during a painting Run remain protected by the
        # execution-snapshot branch above and by on_work_area_state().
        if not prescan_active:
            self._abort_active_plan_invalidation("D405_PLANE_REJECTED")

    def on_plan_status(self, msg: String):
        if getattr(self, "_multi_current", None) is not None or getattr(self, "_multi_queue", []):
            # Target scanning owns an immutable candidate plane; delayed
            # work-area invalidations belong to the previous drawing.
            return
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "plan status"
        ):
            try:
                pending = json.loads(msg.data or "{}")
            except (TypeError, ValueError, json.JSONDecodeError):
                pending = {}
            snapshot = self._execution_snapshot
            is_same_generated_plan = bool(
                isinstance(pending, dict)
                and str(pending.get("state", "")).strip().lower()
                == "generated"
                and str(pending.get("path_id", "")).strip()
                == str(snapshot.get("path_id", ""))
                and str(pending.get("plan_hash", "")).strip()
                == str(snapshot.get("plan_hash", ""))
            )
            if not is_same_generated_plan:
                MoveItExecutor._defer_candidate_invalidation(
                    self, "PLAN_CHANGED_DURING_EXECUTION"
                )
            return
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
            self._abort_active_plan_invalidation("PLAN_STATUS_INVALID")
            return
        if not isinstance(payload, dict):
            self._abort_active_plan_invalidation("PLAN_STATUS_INVALID")
            return
        self._plan_status_time = time.monotonic()
        state = str(payload.get("state", "")).strip().lower()
        if state == "generated":
            plan_hash = str(payload.get("plan_hash", "")).strip()
            path_id = str(payload.get("path_id", "")).strip()
            active = self._active_segment_path
            changed = bool(
                self.executing
                and active is not None
                and (
                    plan_hash != str(active.plan_hash)
                    or path_id != str(active.path_id)
                )
            )
            self._accepted_plan_hash = plan_hash
            self._accepted_plan_path_id = path_id
            if changed:
                self._abort_active_plan_invalidation("PLAN_IDENTITY_CHANGED")
        elif state in {"rejected", "invalidated"}:
            self._accepted_plan_hash = ""
            self._accepted_plan_path_id = ""
            self._abort_active_plan_invalidation(
                f"PLAN_STATUS_{state.upper()}"
            )

    def on_safety_status(self, msg: String):
        self._safety_status_time = time.monotonic()
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        self._safety_status = payload if isinstance(payload, dict) else {}

    def on_wrench_guard_status(self, msg: String):
        received_at = time.monotonic()
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        if not isinstance(payload, dict):
            payload = {}
        try:
            source_time = float(payload.get("published_monotonic_s"))
            sequence = payload.get("status_sequence")
            provenance_valid = bool(
                math.isfinite(source_time)
                and source_time > 0.0
                and source_time <= received_at + 0.05
                and isinstance(sequence, int)
                and not isinstance(sequence, bool)
                and sequence > 0
            )
        except (TypeError, ValueError, OverflowError):
            source_time = 0.0
            sequence = 0
            provenance_valid = False

        if not provenance_valid:
            # A malformed status must not refresh the guard lease.  Clearing
            # the locally cached sample makes all force-capable paths fail
            # closed instead of treating a JSON/parser error as fresh health.
            self._guard_status = {}
            self._guard_status_time = 0.0
            self._guard_status_source_time = 0.0
            self._guard_status_sequence = 0
            self._controller_fault = True
            self._controller_fault_time = received_at
            return
        if source_time <= float(self._guard_status_source_time):
            # DDS may deliver an older queued sample after a newer one.  Do
            # not let it roll status or edge identity backwards.  A restarted
            # guard may reset its sequence, but its monotonic source time is
            # still newer and is accepted by this rule.
            return

        self._guard_status = payload
        self._guard_status_time = received_at
        self._guard_status_source_time = source_time
        self._guard_status_sequence = int(sequence)
        self._controller_fault = payload.get("controller_fault") is not False
        self._controller_fault_time = received_at

    def on_controller_fault(self, msg: Bool):
        self._controller_fault = bool(msg.data)
        self._controller_fault_time = time.monotonic()

    def _force_guard_status_blockers(
        self,
        *,
        expected_mode=None,
        active=None,
        command_context=None,
        now=None,
    ):
        """Validate one source-stamped wrench-guard state fail-closed.

        Guard status is produced by another process on the same robot host,
        so ``time.monotonic()`` is a shared ordering and age clock.  Both the
        source timestamp and local receipt timestamp are checked: a queued
        pre-command sample cannot become a valid ACK merely because this
        single-threaded executor received it late.
        """

        if not getattr(self, "painting_force_enabled", False):
            return ()
        now = time.monotonic() if now is None else float(now)
        timeout_s = float(
            getattr(
                self,
                "force_guard_status_timeout_s",
                FORCE_GUARD_STATUS_TIMEOUT_S,
            )
        )
        status = getattr(self, "_guard_status", {})
        received_at = float(getattr(self, "_guard_status_time", 0.0))
        source_time = float(
            getattr(self, "_guard_status_source_time", 0.0)
        )
        sequence = getattr(self, "_guard_status_sequence", 0)
        blockers = []
        if (
            not isinstance(status, dict)
            or not math.isfinite(now)
            or not math.isfinite(timeout_s)
            or timeout_s <= 0.0
            or not math.isfinite(received_at)
            or not math.isfinite(source_time)
            or received_at <= 0.0
            or source_time <= 0.0
            or not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or sequence <= 0
            or now - received_at < 0.0
            or now - source_time < 0.0
            or now - received_at > timeout_s
            or now - source_time > timeout_s
        ):
            blockers.append("WRENCH_GUARD_STALE")

        if status.get("ft_valid") is not True:
            blockers.append("GUARD_FT_INVALID")
        if status.get("tf_valid") is not True:
            blockers.append("GUARD_TF_INVALID")
        if status.get("abort_latched") is not False:
            blockers.append("GUARD_ABORT_LATCHED")
        if status.get("controller_fault") is not False:
            blockers.append("GUARD_CONTROLLER_FAULT")

        normalized_mode = (
            str(expected_mode).strip().upper()
            if expected_mode is not None
            else ""
        )
        if normalized_mode:
            actual_mode = str(status.get("mode", "")).strip().upper()
            if actual_mode != normalized_mode:
                blockers.append(
                    f"GUARD_MODE_ACK:{actual_mode or 'MISSING'}"
                )

        if active is True:
            if status.get("force_enable") is not True:
                blockers.append("GUARD_FORCE_NOT_ENABLED")
            if status.get("forwarding") is not True:
                blockers.append("GUARD_NOT_FORWARDING")
            if status.get("compliance_enabled") is not True:
                blockers.append("GUARD_COMPLIANCE_NOT_ENABLED")
            if status.get("compliance_active") is not True:
                blockers.append("GUARD_COMPLIANCE_NOT_ACTIVE")
        elif active is False:
            if status.get("force_enable") is not False:
                blockers.append("GUARD_FORCE_NOT_OFF")
            if status.get("forwarding") is not False:
                blockers.append("GUARD_FORWARDING_NOT_OFF")
            if status.get("compliance_enabled") is not False:
                blockers.append("GUARD_COMPLIANCE_NOT_OFF")
            if status.get("compliance_active") is not False:
                blockers.append("GUARD_COMPLIANCE_ACTIVE")

        if command_context is not None:
            try:
                command_time = float(command_context["published_at_s"])
                edge_source_time = float(
                    command_context["guard_source_timestamp_s"]
                )
                edge_sequence = command_context["guard_status_sequence"]
                generation = int(command_context["generation"])
            except (KeyError, TypeError, ValueError, OverflowError):
                blockers.append("GUARD_COMMAND_EDGE_INVALID")
            else:
                if (
                    not math.isfinite(command_time)
                    or not math.isfinite(edge_source_time)
                    or not isinstance(edge_sequence, int)
                    or isinstance(edge_sequence, bool)
                    or generation
                    != int(getattr(self, "_painting_command_generation", -1))
                    or command_context
                    is not getattr(self, "_painting_command_context", None)
                ):
                    blockers.append("GUARD_COMMAND_EDGE_INVALID")
                elif (
                    received_at <= command_time
                    or source_time <= command_time
                    or source_time <= edge_source_time
                    or sequence <= edge_sequence
                ):
                    blockers.append("GUARD_COMMAND_ACK_PENDING")
        return tuple(dict.fromkeys(blockers))

    def _acm_baseline_is_fresh(self):
        if not MoveItExecutor._acm_baseline_is_verified(self):
            return False
        verified_at = float(
            getattr(self, "_acm_baseline_verified_time", 0.0)
        )
        age = time.monotonic() - verified_at
        return bool(verified_at > 0.0 and 0.0 <= age <= ACM_HEALTH_MAX_AGE_S)

    def _acm_baseline_is_verified(self):
        """Return whether this executor verified the current target's ACM.

        Verification age is intentionally not a physical-dispatch deadline.
        This node is the sole ACM writer in the painting stack and refreshes
        the matrix in the background.  Treating a delayed callback as a
        collision fault previously stopped a valid trajectory 22 ms after an
        arbitrary 1.5 s deadline and escalated it to a hardware relaunch.
        Actual malformed/contaminated responses still latch
        ``_acm_state_unknown`` and hard-stop immediately.
        """

        return bool(
            getattr(self, "_acm_baseline_verified", False)
            and getattr(self, "_acm_baseline_verified_target_name", "")
            == getattr(self, "active_target_name", "")
            and float(getattr(self, "_acm_baseline_verified_time", 0.0)) > 0.0
        )

    def _motion_dispatch_inhibited_reason(
        self, *, requires_contact_acm=False, contact_acm_context=None
    ):
        if getattr(self, "_hardware_motion_inhibited", False):
            return "HARDWARE_MOTION_INHIBITED_RELAUNCH_REQUIRED"
        if getattr(self, "_fjt_motion_state_unknown", False):
            return "FJT_MOTION_STATE_UNKNOWN_RELAUNCH_REQUIRED"
        if getattr(self, "_acm_state_unknown", False):
            return "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
        physical_execution = not bool(getattr(self, "dry_run", True))
        # Collision safety comes from the verified full SRDF/MoveIt matrix and
        # collision-checked plan.  Background refresh age is health telemetry,
        # not proof that the matrix changed.
        baseline_ready = MoveItExecutor._acm_baseline_is_verified(self)
        if physical_execution and not requires_contact_acm and not baseline_ready:
            return "ACM_BASELINE_NOT_VERIFIED"
        if physical_execution and requires_contact_acm and (
            not getattr(self, "_acm_baseline_verified", False)
            or getattr(self, "_acm_baseline_verified_target_name", "")
            != getattr(self, "active_target_name", "")
        ):
            return "ACM_BASELINE_NOT_VERIFIED"
        if getattr(self, "_acm_update_pending", None) is not None:
            return "ACM_TRANSACTION_PENDING"
        contact_allowed = bool(
            getattr(self, "_contact_collision_allowed", False)
        )
        if requires_contact_acm:
            if not contact_allowed:
                return "CONTACT_ACM_NOT_ACTIVE"
            base_context_valid = bool(
                getattr(self, "_contact_collision_baseline", None) is not None
                and getattr(self, "_contact_collision_target_name", "")
                == getattr(self, "active_target_name", "")
                and getattr(self, "executing", False)
                and not getattr(self, "_motion_abort_requested", False)
            )
            state = getattr(self, "_execution_state", "")
            normal_contact = state in {"CONTACT_SEARCH", "PAINT"}
            contact_escape = bool(
                state in {"RETRACT", "FINAL_RETRACT"}
                and contact_acm_context is not None
                and MoveItExecutor._contact_escape_is_current(
                    self,
                    contact_acm_context,
                    step=(
                        self._process_steps[self._process_step_index]
                        if 0 <= self._process_step_index < len(self._process_steps)
                        else None
                    ),
                    require_zero_ack=True,
                )
                and not getattr(self, "_painting_command_enable", True)
                and not getattr(self, "_process_force_ready", True)
            )
            if not base_context_valid or not (normal_contact or contact_escape):
                return "CONTACT_ACM_CONTEXT_INVALID"
        elif contact_allowed:
            return "CONTACT_ACM_ACTIVE"
        return ""

    def _publish_motion_abort_latch(self):
        message = Bool()
        message.data = True
        self.motion_abort_pub.publish(message)

    def on_hardware_motion_inhibited(self, msg: Bool):
        self._hardware_motion_inhibit_time = time.monotonic()
        if not msg.data:
            # Hardware inhibit is activation-scoped and cannot be cleared by a
            # later false sample.  A full stack relaunch constructs a new
            # executor and hardware activation together.
            return
        first_latch = not self._hardware_motion_inhibited
        self._hardware_motion_inhibited = True
        if first_latch:
            self.get_logger().error(
                "[HARD STOP] hardware motion inhibit acknowledged; full stack "
                "relaunch is required"
            )
        if not self._motion_abort_requested:
            self._request_motion_abort("HARDWARE_MOTION_INHIBITED")
        else:
            self._publish_motion_abort_latch()
        if first_latch:
            self._publish_execution_status(
                "HARDWARE_MOTION_INHIBITED",
                "hardware stop latched; full stack relaunch required",
            )

    def _publish_executor_heartbeat(self):
        heartbeat = Bool()
        heartbeat.data = bool(self.executing and not self._motion_abort_requested)
        self.executor_heartbeat_pub.publish(heartbeat)

        trajectory_active = Bool()
        # `executing` means that the multi-stage workflow owns the executor;
        # it remains true while the arm is stopped between two trajectories.
        # The force monitor needs the narrower physical-command state so it
        # can collect a stationary bias at the pre-contact pose.
        trajectory_active.data = self._trajectory_command_active()
        self.trajectory_active_pub.publish(trajectory_active)
        if (
            self._motion_abort_requested
            or self._fjt_motion_state_unknown
            or self._hardware_motion_inhibited
        ):
            self._publish_motion_abort_latch()
        stationary = Bool()
        stationary.data = self._robot_stationary_for_bias()
        self.robot_stationary_pub.publish(stationary)

        # The guard intentionally times out each independent input.  Republish
        # the current command atomically at 20 Hz even when a PAINT trajectory
        # lasts much longer than its 200 ms watchdog.
        mode = String()
        mode.data = self._painting_command_mode
        force = Float64()
        force.data = float(self._painting_command_force_n)
        enable = Bool()
        enable.data = bool(self._painting_command_enable)
        self.painting_force_pub.publish(force)
        self.painting_enable_pub.publish(enable)
        self.painting_mode_pub.publish(mode)

    def _trajectory_command_active(self):
        return bool(
            getattr(self, "_active_trajectory_goal_token", None) is not None
            or getattr(self, "_joint_command_timer", None) is not None
        )

    def _robot_stationary_for_bias(self):
        now = time.monotonic()
        if (
            self._trajectory_command_active()
            or self.current_joint_state is None
            or now - self.current_joint_state_time > 0.2
            or now - self._joint_motion_last_time < self.stationary_hold_s
        ):
            return False
        positions = [float(value) for value in self.current_joint_state.position]
        if not positions or not all(math.isfinite(value) for value in positions):
            return False
        velocities = [float(value) for value in self.current_joint_state.velocity]
        if not joint_velocities_allow_stationary(
            velocities,
            expected_count=len(positions),
            max_abs_velocity_rad_s=self.stationary_joint_velocity_rad_s,
        ):
            return False
        return True

    def _query_controller_states(self):
        if self._controller_query_pending:
            return
        if not self.list_controllers_client.service_is_ready():
            self.list_controllers_client.wait_for_service(timeout_sec=0.0)
            return
        self._controller_query_pending = True
        future = self.list_controllers_client.call_async(ListControllers.Request())

        def done(result_future):
            self._controller_query_pending = False
            try:
                response = result_future.result()
                self._controller_states = {
                    str(controller.name): str(controller.state)
                    for controller in response.controller
                }
                self._controller_states_time = time.monotonic()
            except Exception:
                self._controller_states = {}
                self._controller_states_time = 0.0

        future.add_done_callback(done)

    def _current_real_plan_blockers(self):
        snapshot = getattr(self, "_execution_snapshot", None)
        if isinstance(snapshot, dict):
            path = snapshot.get("segment_path")
            blockers = []
            if path is None:
                return ("EXECUTION_SNAPSHOT_PATH_MISSING",)
            if path is not self._active_segment_path:
                blockers.append("ACTIVE_PATH_NOT_EXECUTION_SNAPSHOT")
            if str(getattr(path, "frame_id", "")) != BASE_FRAME:
                blockers.append("EXECUTION_SNAPSHOT_FRAME_INVALID")
            try:
                validate_segment_path_for_real_execution(
                    path,
                    expected_path_id=str(snapshot.get("path_id", "")),
                    expected_plan_hash=str(snapshot.get("plan_hash", "")),
                    expected_work_area_id=str(
                        snapshot.get("work_area_id", "")
                    ),
                    expected_plane_generation_id=str(
                        snapshot.get("plane_generation_id", "")
                    ),
                )
            except SegmentPathError as exc:
                blockers.append(f"EXECUTION_SNAPSHOT_INVALID:{exc}")
            return tuple(blockers)

        path = self._segment_path
        blockers = list(
            real_plan_gate_blockers(
                segment_present=path is not None,
                segment_version=getattr(path, "version", None),
                segment_path_id=getattr(path, "path_id", ""),
                waypoint_path_id=self._waypoints_path_id,
                segment_plan_hash=getattr(path, "plan_hash", ""),
                accepted_plan_hash=self._accepted_plan_hash,
                accepted_plan_path_id=self._accepted_plan_path_id,
                segment_work_area_id=getattr(path, "work_area_id", ""),
                current_work_area_id=self._current_work_area_id,
                segment_plane_generation_id=getattr(
                    path, "plane_generation_id", ""
                ),
                current_plane_generation_id=self._current_plane_generation_id,
                d405_plane_accepted=self._d405_plane_accepted,
            )
        )
        if path is not None and path.process_mode != getattr(self, "process_mode", "paint"):
            blockers.append("PROCESS_MODE_MISMATCH")
        if path is not None and not blockers:
            try:
                validate_segment_path_for_real_execution(
                    path,
                    expected_path_id=self._waypoints_path_id,
                    expected_plan_hash=self._accepted_plan_hash or None,
                    expected_work_area_id=self._current_work_area_id,
                    expected_plane_generation_id=self._current_plane_generation_id,
                )
            except SegmentPathError as exc:
                blockers.append(f"SEGMENT_INVALID:{exc}")
        return tuple(blockers)

    def _precontact_tare_pending(self):
        return bool(
            self.painting_force_enabled
            and self.runtime_tare_enabled
            and not self._execution_tare_ready
            and not self._motion_abort_requested
        )

    def _runtime_tare_services_ready(self):
        return bool(
            self.runtime_tare_enabled
            and self.runtime_ft_tare_client.service_is_ready()
            and self.force_safety_reset_client.service_is_ready()
            and self.wrench_guard_reset_client.service_is_ready()
        )

    def _precontact_safety_fault_is_deferred(self):
        if not self._precontact_tare_pending():
            return False
        reason = str(self._safety_status.get("reason", "")).strip().upper()
        return bool(
            self._safety_status.get("abort_latched") is True
            and reason in PRECONTACT_DEFERRED_SAFETY_REASONS
        )

    def _real_geometry_runtime_blockers(self):
        """Dependencies required before the robot may approach pre-contact.

        Corrected F/T, monitor bias, and guard forwarding are deliberately not
        in this gate.  They are established at the verified 10 mm free-space
        pose and remain mandatory before CONTACT_SEARCH.
        """
        now = time.monotonic()
        blockers = []
        dispatch_inhibit = MoveItExecutor._motion_dispatch_inhibited_reason(self)
        if dispatch_inhibit:
            blockers.append(dispatch_inhibit)
        joint_age = now - self.current_joint_state_time
        if (
            self.current_joint_state is None
            or self.current_joint_state_time <= 0.0
            or joint_age < 0.0
            or joint_age > 1.0
        ):
            blockers.append("JOINT_STATE_STALE")
        if self._motion_abort_requested:
            blockers.append("EXECUTION_ABORT_LATCHED")
        safety_status_fresh = (
            self._safety_status_time > 0.0
            and now - self._safety_status_time <= 0.5
        )
        if self.painting_force_enabled and not safety_status_fresh:
            blockers.append("SAFETY_STATUS_STALE")
        if (
            self.painting_force_enabled
            and safety_status_fresh
            and bool(self._safety_status.get("abort_latched", True))
            and not self._precontact_safety_fault_is_deferred()
        ):
            blockers.append(
                "ABORT_LATCHED:"
                + str(self._safety_status.get("reason", "UNKNOWN"))
            )
        if self.painting_force_enabled and (
            self._guard_status_time <= 0.0
            or float(getattr(self, "_guard_status_source_time", 0.0)) <= 0.0
            or now - self._guard_status_time
            > float(
                getattr(
                    self,
                    "force_guard_status_timeout_s",
                    FORCE_GUARD_STATUS_TIMEOUT_S,
                )
            )
            or now - float(getattr(self, "_guard_status_source_time", 0.0))
            > float(
                getattr(
                    self,
                    "force_guard_status_timeout_s",
                    FORCE_GUARD_STATUS_TIMEOUT_S,
                )
            )
        ):
            blockers.append("WRENCH_GUARD_STALE")
        if self.painting_force_enabled:
            if not self._free_space_confirmed:
                blockers.append("FREE_SPACE_NOT_CONFIRMED")
            if not self._runtime_tare_services_ready():
                blockers.append("RUNTIME_TARE_SERVICES_UNAVAILABLE")
            if (
                self._controller_fault_time <= 0.0
                or now - self._controller_fault_time > 0.5
            ):
                blockers.append("CONTROLLER_STATUS_STALE")
            elif self._controller_fault:
                blockers.append("CONTROLLER_FAULT")
        if self._current_tcp_pose_np() is None:
            blockers.append("TCP_TF_INVALID")
        required_controllers = ["joint_trajectory_controller"]
        if self.painting_force_enabled:
            required_controllers.append("admittance_controller")
        for required_controller in required_controllers:
            if self._controller_states.get(required_controller) != "active":
                blockers.append(f"CONTROLLER_NOT_ACTIVE:{required_controller}")
        if (
            self.execution_backend == "follow_joint_trajectory"
            and not self.traj_action_client.server_is_ready()
        ):
            blockers.append("FOLLOW_JOINT_TRAJECTORY_UNAVAILABLE")
        return tuple(blockers)

    def _real_force_runtime_blockers(self):
        """Dependencies that must hold before CONTACT_SEARCH can move."""
        now = time.monotonic()
        blockers = []
        if not self._execution_tare_ready:
            blockers.append("EXECUTION_TARE_NOT_READY")
        if (
            self._safety_status_time <= 0.0
            or now - self._safety_status_time > self.ft_required_timeout_s
            or self._safety_status.get("ft_valid") is not True
        ):
            blockers.append("FT_STALE")
        if self._safety_status.get("tf_valid") is not True:
            blockers.append("FT_TF_INVALID")
        if self._safety_status.get("bias_ready") is not True:
            blockers.append("FT_BIAS_NOT_READY")
        if bool(self._safety_status.get("abort_latched", True)):
            blockers.append(
                "ABORT_LATCHED:"
                + str(self._safety_status.get("reason", "UNKNOWN"))
            )
        blockers.extend(
            MoveItExecutor._force_guard_status_blockers(self, now=now)
        )
        if (
            self._controller_fault_time <= 0.0
            or now - self._controller_fault_time > 0.5
        ):
            blockers.append("CONTROLLER_STATUS_STALE")
        elif self._controller_fault:
            blockers.append("CONTROLLER_FAULT")
        return tuple(blockers)

    def _real_runtime_blockers(self, require_force_ready=True):
        blockers = list(self._real_geometry_runtime_blockers())
        if getattr(self, "process_mode", "paint") == "spray":
            blockers.extend(self._spray_motion_blockers())
        if require_force_ready and self.painting_force_enabled:
            blockers.extend(self._real_force_runtime_blockers())
        return tuple(dict.fromkeys(blockers))

    def _stage1_pre_motion_blockers(self):
        """Revalidate real-hardware approach dependencies after async waits."""

        if not self.real_painting_enabled:
            return ()
        blockers = [
            blocker
            for blocker in self._real_geometry_runtime_blockers()
            if blocker != "FREE_SPACE_NOT_CONFIRMED"
        ]
        if not self._execution_free_space_confirmed and getattr(self, "process_mode", "paint") != "spray":
            blockers.append("EXECUTION_FREE_SPACE_NOT_CONFIRMED")
        if getattr(self, "process_mode", "paint") == "spray":
            blockers.extend(self._spray_motion_blockers())
        revision = int(getattr(self, "_scene_revision", 0))
        if (
            not self.scene_confirmed
            or int(getattr(self, "_scene_confirmed_revision", -1)) != revision
        ):
            blockers.append("PLANNING_SCENE_REVISION_UNCONFIRMED")
        return tuple(dict.fromkeys(blockers))

    def _update_and_publish_readiness(self):
        self._query_controller_states()
        self._request_acm_baseline_health_check()
        now = time.monotonic()
        joint_fresh = (
            self.current_joint_state is not None
            and now - self.current_joint_state_time <= 1.0
        )
        ft_fresh = (
            self._safety_status_time > 0.0
            and now - self._safety_status_time <= self.ft_required_timeout_s
            and self._safety_status.get("ft_valid") is True
            and self._safety_status.get("tf_valid") is True
        )
        guard_fresh = (
            self._guard_status_time > 0.0
            and float(getattr(self, "_guard_status_source_time", 0.0)) > 0.0
            and now - self._guard_status_time
            <= float(
                getattr(
                    self,
                    "force_guard_status_timeout_s",
                    FORCE_GUARD_STATUS_TIMEOUT_S,
                )
            )
            and now - float(getattr(self, "_guard_status_source_time", 0.0))
            <= float(
                getattr(
                    self,
                    "force_guard_status_timeout_s",
                    FORCE_GUARD_STATUS_TIMEOUT_S,
                )
            )
        )
        controller_fresh = (
            self._controller_states_time > 0.0
            and now - self._controller_states_time <= 2.0
        )
        required_controllers = ["joint_trajectory_controller"]
        if self.painting_force_enabled:
            required_controllers.append("admittance_controller")
        controller_active = (
            controller_fresh
            and all(
                self._controller_states.get(name) == "active"
                for name in required_controllers
            )
        )
        action_available = (
            self.execution_backend != "follow_joint_trajectory"
            or self.traj_action_client.server_is_ready()
        )
        tf_valid = self._current_tcp_pose_np() is not None
        plan_blockers = self._current_real_plan_blockers()
        plan_valid = not plan_blockers
        safety_status_fresh = (
            self._safety_status_time > 0.0
            and now - self._safety_status_time <= 0.5
        )
        precontact_tare_pending = self._precontact_tare_pending()
        deferred_safety_fault = self._precontact_safety_fault_is_deferred()
        safety_abort_latched = bool(
            self._safety_status.get("abort_latched", True)
        )
        startup_abort_clear = bool(
            not self._motion_abort_requested
            and (
                not self.painting_force_enabled
                or (safety_status_fresh and not safety_abort_latched)
                or deferred_safety_fault
            )
        )
        runtime_tare_services_ready = (
            not self.painting_force_enabled
            or self._runtime_tare_services_ready()
        )
        force_status_deferred = bool(
            precontact_tare_pending
            and safety_status_fresh
            and (not safety_abort_latched or deferred_safety_fault)
        )
        actual_controller_fault_clear = bool(
            self._controller_fault_time > 0.0
            and now - self._controller_fault_time <= 0.5
            and not self._controller_fault
        )
        checks = {
            "hardware_connected": joint_fresh,
            "required_controller_active": controller_active,
            "follow_joint_trajectory_available": action_available,
            "zed_surface_valid": self.dynamic_surface_point is not None,
            "d405_plane_accepted": self._d405_plane_accepted,
            "required_tf_valid": tf_valid,
            "safety_status_available": (
                not self.painting_force_enabled or safety_status_fresh
            ),
            # These legacy readiness keys describe whether the dependency is
            # either ready now or is safely deferred to the mandatory
            # pre-contact tare.  Actual contact readiness is exposed below in
            # `contact_checks`; false there never masquerades as ready force.
            "ft_valid_and_fresh": (
                not self.painting_force_enabled
                or ft_fresh
                or force_status_deferred
            ),
            "wrench_guard_active": (
                not self.painting_force_enabled or guard_fresh
            ),
            "controller_fault_clear": (
                not self.painting_force_enabled
                or actual_controller_fault_clear
            ),
            "ft_bias_ready": (
                not self.painting_force_enabled
                or self._safety_status.get("bias_ready") is True
                or force_status_deferred
            ),
            "free_space_confirmed": (
                not self.painting_force_enabled or self._free_space_confirmed
            ),
            "runtime_tare_services_ready": runtime_tare_services_ready,
            "precontact_tare_ready": (
                not self.painting_force_enabled
                or self._execution_tare_ready
                or (precontact_tare_pending and runtime_tare_services_ready)
            ),
            "abort_not_latched": startup_abort_clear,
            "fjt_motion_state_known": not self._fjt_motion_state_unknown,
            "hardware_motion_not_inhibited": (
                not self._hardware_motion_inhibited
            ),
            "acm_baseline_verified": bool(
                self.dry_run or self._acm_baseline_is_verified()
            ),
            "work_area_refined": bool(
                self._current_work_area_id
                and self._current_plane_generation_id
                and self._d405_plane_accepted
            ),
            "current_plan_validated": plan_valid,
        }
        if getattr(self, "process_mode", "paint") == "spray":
            checks["spray_output_ready"] = not self._spray_motion_blockers()
        ready = all(checks.values()) if self.real_painting_enabled else plan_valid
        if getattr(self, "process_mode", "paint") == "spray":
            ready = ready and checks["spray_output_ready"]
        if not self.dry_run:
            ready = bool(ready and checks["acm_baseline_verified"])
        if self._motion_abort_requested:
            safety_reason = str(
                self._safety_status.get("reason", "")
            ).strip()
            abort_reason = (
                self._execution_abort_reason
                or (
                    safety_reason
                    if safety_reason.upper() not in {"", "NONE"}
                    else "EXECUTION_ABORT"
                )
            )
        elif safety_abort_latched and not deferred_safety_fault and getattr(self, "process_mode", "paint") != "spray":
            abort_reason = str(self._safety_status.get("reason", "UNKNOWN"))
        else:
            abort_reason = "NONE"
        snapshot = getattr(self, "_execution_snapshot", None)
        snapshot_locked = isinstance(snapshot, dict)
        status_path = (
            snapshot.get("segment_path")
            if snapshot_locked
            else self._segment_path
        )
        payload = {
            "ready": bool(ready),
            "process_mode": getattr(self, "process_mode", "paint"),
            "model_id": getattr(self, "model_id", DEFAULT_MODEL),
            "real_painting_enabled": bool(self.real_painting_enabled),
            "dry_run": bool(self.dry_run),
            "running": bool(self.executing),
            "state": self._execution_state,
            "checks": checks,
            "precontact_tare_pending": bool(precontact_tare_pending),
            "precontact_tare_phase": self._runtime_tare_phase,
            "contact_checks": {
                "ft_valid_and_fresh": bool(ft_fresh),
                "ft_bias_ready": bool(
                    self._safety_status.get("bias_ready") is True
                ),
                "safety_abort_clear": bool(not safety_abort_latched),
                "controller_fault_clear": actual_controller_fault_clear,
                "execution_tare_ready": bool(self._execution_tare_ready),
            },
            "required_controllers": {
                name: self._controller_states.get(name, "missing")
                for name in required_controllers
            },
            "plan_blockers": list(plan_blockers),
            "work_area_id": (
                str(snapshot.get("work_area_id", ""))
                if snapshot_locked
                else self._current_work_area_id
            ),
            "plane_generation_id": (
                str(snapshot.get("plane_generation_id", ""))
                if snapshot_locked
                else self._current_plane_generation_id
            ),
            "plan_hash": getattr(status_path, "plan_hash", "")
            if status_path is not None
            else "",
            "path_id": getattr(status_path, "path_id", "")
            if status_path is not None
            else "",
            "execution_snapshot_locked": snapshot_locked,
            "abort_reason": abort_reason,
            "target_force_n": float(
                max(
                    [0.0]
                    + [
                        row.force_n
                        for row in status_path.rows
                        if row.mode in CONTACT_MOTION_MODES
                    ]
                )
                if status_path is not None
                else 0.0
            ),
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
        self.readiness_pub.publish(message)

    def _publish_execution_status(self, state, reason="", **fields):
        self._execution_state = str(state)
        snapshot = getattr(self, "_execution_snapshot", None)
        snapshot_locked = isinstance(snapshot, dict)
        payload = {
            "state": self._execution_state,
            "reason": str(reason),
            "timestamp_ns": int(self.get_clock().now().nanoseconds),
            "plan_hash": getattr(self._active_segment_path, "plan_hash", "")
            if self._active_segment_path is not None
            else getattr(self._segment_path, "plan_hash", "")
            if self._segment_path is not None
            else "",
            "plane_generation_id": (
                str(snapshot.get("plane_generation_id", ""))
                if snapshot_locked
                else self._current_plane_generation_id
            ),
            "work_area_id": (
                str(snapshot.get("work_area_id", ""))
                if snapshot_locked
                else self._current_work_area_id
            ),
            "execution_snapshot_locked": snapshot_locked,
            "normal_force_n": self.ft_normal_force_n,
            "contact_confirmed": bool(self._painting_contact_confirmed),
        }
        payload.update(fields)
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
        self.execution_status_pub.publish(message)

    def on_active_surface(self, msg: PoseStamped):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "ZED surface"
        ):
            return
        if (
            self._d405_refined_lock_active
            and self.dynamic_surface_source == "d405_refined"
        ):
            self.get_logger().info(
                "[SURFACE] D405 refined plane lock 유지 -> ZED surface 갱신 무시",
                throttle_duration_sec=2.0)
            return
        if self.executing:
            self.get_logger().warn(
                "[SURFACE] 실행 중 ZED surface 갱신 무시 "
                "(현재 plan/collision 기준 고정)",
                throttle_duration_sec=2.0)
            return
        self._set_active_surface(msg, "zed")

    def on_refined_active_surface(self, msg: PoseStamped):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "D405 refined surface"
        ):
            return
        if (
            not self._d405_plane_accepted
            or not self._current_work_area_id
            or not self._current_plane_generation_id
            or not self._d405_refined_pose_armed
            or self._d405_refined_pose_generation_id
            != self._current_plane_generation_id
        ):
            self.get_logger().warn(
                "[SURFACE] 현재 generation에 arm되지 않은 refined PoseStamped 무시",
                throttle_duration_sec=2.0,
            )
            return
        if self._d405_refined_lock_active and not self._d405_prescan_active:
            self.get_logger().info(
                "[SURFACE] D405 refined plane 이미 lock 됨 -> 추가 refined 갱신 무시",
                throttle_duration_sec=2.0)
            return
        if self.executing and not self._d405_prescan_active:
            self.get_logger().warn(
                "[SURFACE] 실행 중 D405 refined surface 갱신 무시 "
                "(다음 plan부터 적용)",
                throttle_duration_sec=2.0)
            return
        before_time = self.dynamic_surface_source_time
        self._set_active_surface(msg, "d405_refined")
        self._finish_refined_surface_update(before_time)

    def _finish_refined_surface_update(self, before_time):
        if (
            self.dynamic_surface_source == "d405_refined"
            and self.dynamic_surface_source_time != before_time
        ):
            self._d405_refined_pose_armed = False
            self._d405_refined_pose_generation_id = ""
            self._d405_refined_generation += 1
            self._d405_refined_lock_active = True
            self._d405_refined_lock_time = self.dynamic_surface_source_time
            self._d405_refined_lock_signature = self._work_area_corners_signature
            self._d405_refined_allow_corner_correction = True
            self.get_logger().info(
                "[SURFACE] D405 refined plane lock 활성화 "
                "(새 Work Area refine 전까지 ZED plane 으로 덮지 않음)")

    @staticmethod
    def _mark_scene_dirty(node):
        node._scene_revision = int(getattr(node, "_scene_revision", 0)) + 1
        node.scene_confirmed = False
        node.scene_initialized = False

    @staticmethod
    def _canonical_surface_plane(point, normal, source):
        """Canonical base-frame plane used only for replay deduplication.

        The nearest-origin point removes irrelevant tangential shifts in a
        PoseStamped plane point.  The unit normal remains signed because n -> -n
        reverses the robot approach/offset direction and is therefore a real
        geometry change.  ``source`` remains part of the identity: a ZED ->
        D405 handoff is also a real state transition even when planes coincide.
        """

        point = np.asarray(point, dtype=float)
        normal = np.asarray(normal, dtype=float)
        if (
            point.shape != (3,)
            or normal.shape != (3,)
            or not np.all(np.isfinite(point))
            or not np.all(np.isfinite(normal))
        ):
            return None
        magnitude = float(np.linalg.norm(normal))
        if not math.isfinite(magnitude) or magnitude < 1e-9:
            return None
        normal = normal / magnitude
        canonical_point = normal * float(np.dot(point, normal))
        return str(source), canonical_point, normal

    def _set_active_surface(self, msg: PoseStamped, source: str):
        frame = msg.header.frame_id or BASE_FRAME
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        q_msg = msg.pose.orientation
        normal_local = quat_apply([
            q_msg.x, q_msg.y, q_msg.z, q_msg.w
        ], [0.0, 0.0, 1.0])

        frame = self._canonical_world_frame(frame)
        if frame == BASE_FRAME:
            point = pos
            normal = normal_local
        else:
            tf = self._lookup_transform_to_base(frame, timeout_s=0.2)
            if tf is None:
                self._pending_surface_msg = msg
                self._pending_surface_source = source
                self.get_logger().warn(
                    f"[SURFACE] TF 실패 ({BASE_FRAME}<-{frame})",
                    throttle_duration_sec=2.0)
                return
            t = tf.transform.translation
            q = tf.transform.rotation
            q_tf = [q.x, q.y, q.z, q.w]
            point = quat_apply(q_tf, pos) + np.array([t.x, t.y, t.z])
            normal = quat_apply(q_tf, normal_local)

        canonical = MoveItExecutor._canonical_surface_plane(
            point,
            normal,
            source,
        )
        if canonical is None:
            self.get_logger().warn(
                f"[SURFACE] invalid/non-finite plane ignored ({source})",
                throttle_duration_sec=2.0,
            )
            return False

        previous = None
        previous_normal = getattr(self, "dynamic_surface_normal", None)
        if (
            getattr(self, "dynamic_surface_point", None) is not None
            and previous_normal is not None
        ):
            previous = MoveItExecutor._canonical_surface_plane(
                self.dynamic_surface_point,
                previous_normal,
                getattr(self, "dynamic_surface_source", ""),
            )
        identical = bool(
            previous is not None
            and previous[0] == canonical[0]
            and np.allclose(
                previous[1],
                canonical[1],
                rtol=0.0,
                atol=ACTIVE_SURFACE_DEDUPE_POINT_TOL_M,
            )
            and np.allclose(
                previous[2],
                canonical[2],
                rtol=0.0,
                atol=ACTIVE_SURFACE_DEDUPE_NORMAL_TOL,
            )
        )
        if identical:
            # Do not refresh source_time or scene revision.  The periodic ZED
            # work-area replay otherwise invalidates a prescan plan faster than
            # collision-aware IK can finish.
            self._pending_surface_msg = None
            self.get_logger().info(
                f"[SURFACE] identical dynamic plane replay ignored ({source})",
                throttle_duration_sec=2.0,
            )
            return False

        point = np.asarray(point, dtype=float)
        normal = canonical[2]
        self.dynamic_surface_point = point.copy()
        self.dynamic_surface_normal = normal.copy()
        self.dynamic_surface_source = source
        self.dynamic_surface_source_time = time.monotonic()
        self._pending_surface_msg = None
        MoveItExecutor._mark_scene_dirty(self)
        self.get_logger().info(
            f"[SURFACE] dynamic plane 갱신({source}): point=({point[0]:+.3f},"
            f"{point[1]:+.3f},{point[2]:+.3f}) normal=({normal[0]:+.2f},"
            f"{normal[1]:+.2f},{normal[2]:+.2f})",
            throttle_duration_sec=2.0)
        return True

    def on_work_area_corners(self, msg: PoseArray):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "work-area corners"
        ) or self.executing:
            self.get_logger().warn(
                "[SURFACE] 실행 중 work_area corners 갱신 무시 "
                "(현재 plan/collision 기준 고정)",
                throttle_duration_sec=2.0)
            return
        if len(msg.poses) < 4:
            return
        frame = msg.header.frame_id or BASE_FRAME
        pts = np.array([
            [p.position.x, p.position.y, p.position.z]
            for p in msg.poses[:4]
        ], dtype=float)
        frame = self._canonical_world_frame(frame)
        if frame == BASE_FRAME:
            if self._set_dynamic_work_area_corners(pts):
                MoveItExecutor._mark_scene_dirty(self)
            return
        tf = self._lookup_transform_to_base(frame, timeout_s=0.2)
        if tf is None:
            self._pending_corners_msg = msg
            self.get_logger().warn(
                f"[SURFACE] corners TF 실패 ({BASE_FRAME}<-{frame})",
                throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        geometry_changed = self._set_dynamic_work_area_corners(
            quat_apply([q.x, q.y, q.z, q.w], pts)
            + np.array([t.x, t.y, t.z])
        )
        self._pending_corners_msg = None
        if geometry_changed:
            MoveItExecutor._mark_scene_dirty(self)

    def _set_dynamic_work_area_corners(self, pts):
        pts = np.asarray(pts, dtype=float)
        signature = tuple(
            tuple(round(float(v), 3) for v in row)
            for row in pts[:4]
        )
        previous_signature = self._work_area_corners_signature
        if previous_signature is not None and signature == previous_signature:
            # wall_projector republishes the locked work-area geometry for
            # visualization.  Reapplying the same ~1 MB PlanningScene on each
            # message starves this node's safety/status callbacks, so preserve
            # both the accepted geometry and the confirmed scene verbatim.
            return False

        changed = previous_signature is not None
        self._work_area_corners_signature = signature
        self.dynamic_work_area_corners = pts.copy()
        if changed and self.dynamic_surface_source == "d405_refined":
            if (
                self._d405_refined_lock_active
                and self._d405_refined_allow_corner_correction
            ):
                self._d405_refined_allow_corner_correction = False
                self._d405_refined_lock_signature = signature
                self.get_logger().info(
                    "[SURFACE] D405 lock 상태에서 보정된 work_area corners 1회 반영")
            else:
                self._reset_d405_refined_lock(
                    "work_area_corners_changed", clear_surface=True)
        return True

    def on_refine_work_area(self, msg: Bool):
        if not msg.data:
            return
        if self._motion_abort_requested:
            self.get_logger().error(
                "[D405 PRESCAN] execution abort latch가 설정됨 -> "
                "명시적 reset 전 refine 이동 거부"
            )
            self._publish_work_area_refine_status("abort_latched")
            return
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "work-area refine request"
        ) or self.executing:
            self.get_logger().warn(
                "[D405 PRESCAN] 이미 로봇 실행 중 -> work area refine 요청 무시")
            self._publish_work_area_refine_status("busy")
            return
        self._reset_d405_refined_lock(
            "work_area_refine_requested", clear_surface=False)
        self._cancel_work_area_refine_wait_timer()
        self._work_area_refine_request_time = time.monotonic()
        self._work_area_refine_previous_corner_signature = (
            self._work_area_corners_signature)
        self._publish_work_area_refine_status("requested")
        self._try_begin_work_area_refine()

    def _try_begin_work_area_refine(self):
        missing = []
        if self.current_joint_state is None:
            missing.append("joint_state")
        if self.dynamic_surface_point is None or self.dynamic_surface_normal is None:
            missing.append("work_area_plane")
        if self.dynamic_work_area_corners is None:
            missing.append("work_area_corners")
        elif (
            self._work_area_refine_previous_corner_signature is not None
            and self._work_area_corners_signature
            == self._work_area_refine_previous_corner_signature
            and time.monotonic() - float(self._work_area_refine_request_time) < 1.0
        ):
            missing.append("new_work_area_corners")

        if not missing:
            self._cancel_work_area_refine_wait_timer()
            self.get_logger().info(
                "[D405 PRESCAN] work area 선택 후 사전 보정 시작 "
                "(fresh refined plane 이 있어도 새로 측정)")
            self._publish_work_area_refine_status("moving")
            self._begin_d405_prescan(mode="work_area")
            return

        elapsed = time.monotonic() - float(self._work_area_refine_request_time)
        if elapsed > 5.0:
            self.get_logger().warn(
                "[D405 PRESCAN] work area refine 보류 timeout: "
                f"missing={missing}")
            self._cancel_work_area_refine_wait_timer()
            self._publish_work_area_refine_status(
                "timeout", missing=missing)
            return

        if self._work_area_refine_wait_timer is None:
            self.get_logger().info(
                "[D405 PRESCAN] work area refine 준비 대기: "
                f"missing={missing}")
            self._publish_work_area_refine_status(
                "waiting", missing=missing)
            self._work_area_refine_wait_timer = self.create_timer(
                0.2, self._try_begin_work_area_refine)

    def _cancel_work_area_refine_wait_timer(self):
        if self._work_area_refine_wait_timer is not None:
            self._work_area_refine_wait_timer.cancel()
            self.destroy_timer(self._work_area_refine_wait_timer)
            self._work_area_refine_wait_timer = None

    def _publish_work_area_refine_status(self, state, **fields):
        payload = {"state": state}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.work_area_refine_status_pub.publish(msg)

    def _retry_pending_surface_tf(self):
        """Surface/corner 메시지를 TF 준비 전에 받았을 때 나중에 다시 변환."""
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "pending surface TF retry"
        ):
            return
        if self._pending_surface_msg is not None:
            source = self._pending_surface_source
            before_time = self.dynamic_surface_source_time
            self._set_active_surface(
                self._pending_surface_msg,
                source,
            )
            if source == "d405_refined" and self._d405_refined_pose_armed:
                self._finish_refined_surface_update(before_time)
        if self._pending_corners_msg is not None:
            self.on_work_area_corners(self._pending_corners_msg)

    def on_perception_planes(self, msg: PoseArray):
        if MoveItExecutor._execution_snapshot_updates_locked(
            self, "perception planes"
        ) or self.executing:
            self.get_logger().warn(
                "[SCENE] 실행 중 perception plane 갱신 무시 "
                "(현재 plan/collision 기준 고정)",
                throttle_duration_sec=2.0)
            return
        frame = self._canonical_world_frame(msg.header.frame_id or BASE_FRAME)
        transform = None
        if frame != BASE_FRAME:
            transform = self._lookup_transform_to_base(frame, timeout_s=0.2)
            if transform is None:
                self.get_logger().warn(
                    f"[SCENE] planes TF 실패 ({BASE_FRAME}<-{frame})",
                    throttle_duration_sec=2.0)
                return

        planes = []
        for idx, pose in enumerate(msg.poses):
            point = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ], dtype=float)
            q_pose = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            normal = quat_apply(q_pose, [0.0, 0.0, 1.0])
            if transform is not None:
                t = transform.transform.translation
                q = transform.transform.rotation
                q_tf = [q.x, q.y, q.z, q.w]
                point = quat_apply(q_tf, point) + np.array([t.x, t.y, t.z])
                normal = quat_apply(q_tf, normal)
            normal = np.asarray(normal, dtype=float)
            normal /= np.linalg.norm(normal) + 1e-12
            planes.append({
                "index": idx,
                "point": point,
                "normal": normal,
            })

        self.perception_planes = planes
        if planes:
            MoveItExecutor._mark_scene_dirty(self)
            self.get_logger().info(
                f"[SCENE] perception planes 갱신: {len(planes)}개",
                throttle_duration_sec=2.0)

    def on_plane_labels(self, msg: String):
        if self.executing:
            return
        try:
            payload = json.loads(msg.data)
            labels = payload.get("planes", [])
            if not isinstance(labels, list):
                raise ValueError("planes is not a list")
        except Exception as e:
            self.get_logger().warn(f"[SCENE] plane_labels parse 실패: {e}")
            return
        self.perception_plane_labels = labels
        if self.perception_planes:
            MoveItExecutor._mark_scene_dirty(self)

    def on_dynamic_obstacles(self, msg: MarkerArray):
        if self.executing:
            self.get_logger().warn(
                "[SCENE] 실행 중 dynamic obstacle 갱신 무시 "
                "(다음 plan부터 적용)",
                throttle_duration_sec=2.0)
            return
        obstacles = []
        new_ids = set()
        count = 0
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.type != Marker.CUBE:
                continue
            if count >= MAX_DYNAMIC_OBSTACLES:
                break
            if marker.scale.x <= 0.0 or marker.scale.y <= 0.0 or marker.scale.z <= 0.0:
                continue
            converted = self._marker_to_base_collision(marker, count)
            if converted is None:
                continue
            if self._is_robot_self_obstacle(
                    converted["position"], converted["size"]):
                continue
            obstacles.append(converted)
            new_ids.add(converted["id"])
            count += 1

        stale = self._dynamic_obstacle_ids - new_ids
        signature = tuple(
            (
                ob["id"],
                *(round(float(v), 3) for v in ob["position"]),
                *(round(float(v), 3) for v in ob["size"]),
            )
            for ob in obstacles
        )
        if (
            stale
            or new_ids != self._dynamic_obstacle_ids
            or signature != self._dynamic_obstacle_signature
        ):
            self.dynamic_obstacles = obstacles
            self._stale_dynamic_obstacle_ids.update(stale)
            self._dynamic_obstacle_ids = new_ids
            self._dynamic_obstacle_signature = signature
            MoveItExecutor._mark_scene_dirty(self)
            self.get_logger().info(
                f"[SCENE] ZED dynamic obstacles 갱신: {len(obstacles)}개")

    def _marker_to_base_collision(self, marker, index):
        frame = marker.header.frame_id or BASE_FRAME
        pos = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ], dtype=float)
        q_marker = np.array([
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ], dtype=float)
        if np.linalg.norm(q_marker) < 1e-9:
            q_marker = np.array([0.0, 0.0, 0.0, 1.0])

        frame = self._canonical_world_frame(frame)
        if frame == BASE_FRAME:
            base_pos = pos
            base_q = q_marker
        else:
            tf = self._lookup_transform_to_base(frame, timeout_s=0.2)
            if tf is None:
                self.get_logger().warn(
                    f"[SCENE] obstacle TF 실패 ({BASE_FRAME}<-{frame})")
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            q_tf = [q.x, q.y, q.z, q.w]
            base_pos = quat_apply(q_tf, pos) + np.array([t.x, t.y, t.z])
            base_q = quat_multiply(q_tf, q_marker)

        return {
            "id": f"{DYNAMIC_OBSTACLE_PREFIX}{index:03d}",
            "position": base_pos,
            "orientation": base_q,
            "size": np.array([
                marker.scale.x,
                marker.scale.y,
                marker.scale.z,
            ], dtype=float),
        }

    @staticmethod
    def _canonical_world_frame(frame):
        # Isaac Sim publishes the physical world as "World". Treat lowercase
        # "world" from legacy UI/perception code as the same physical frame,
        # not as MoveIt's virtual SRDF frame.
        return "World" if frame == "world" else frame

    def _lookup_transform_to_base(self, source_frame, timeout_s=0.2):
        try:
            return self.tf_buffer.lookup_transform(
                BASE_FRAME,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_s),
            )
        except Exception:
            return None

    @staticmethod
    def _transform_pose_msg_to_base(pose, transform):
        if transform is None:
            return copy.deepcopy(pose)
        return MoveItExecutor._transform_xyz_quat_to_pose(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ],
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            transform,
        )

    @staticmethod
    def _transform_xyz_quat_to_pose(position, orientation, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        pos = np.asarray(position, dtype=float)
        base_pos = quat_apply(q_tf, pos) + np.array([t.x, t.y, t.z])
        base_q = quat_multiply(q_tf, np.asarray(orientation, dtype=float))
        out = Pose()
        out.position.x = float(base_pos[0])
        out.position.y = float(base_pos[1])
        out.position.z = float(base_pos[2])
        out.orientation.x = float(base_q[0])
        out.orientation.y = float(base_q[1])
        out.orientation.z = float(base_q[2])
        out.orientation.w = float(base_q[3])
        return out

    def _is_robot_self_obstacle(self, point_base, size=None):
        """로봇 본체 표면을 ZED dynamic obstacle 로 재등록하지 않도록 차단."""
        extra = 0.0
        if size is not None:
            extra = 0.5 * float(np.linalg.norm(np.asarray(size, dtype=float)))

        frames = self._lookup_robot_link_points()
        if len(frames) < 3:
            self.get_logger().warn(
                "[SCENE] robot self-filter TF 부족 -> dynamic obstacle skip",
                throttle_duration_sec=2.0)
            return True

        p = np.asarray(point_base, dtype=float)
        for pair, radius in ROBOT_LINK_CAPSULE_RADIUS.items():
            if pair[0] not in frames or pair[1] not in frames:
                continue
            dist = self._distance_to_segment(p, frames[pair[0]], frames[pair[1]])
            if dist <= radius + ROBOT_SELF_FILTER_PADDING + extra:
                return True
        return False

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

    @staticmethod
    def _distance_to_segment(point, a, b):
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-12:
            return float(np.linalg.norm(point - a))
        t = float(np.clip(np.dot(point - a, ab) / denom, 0.0, 1.0))
        closest = a + t * ab
        return float(np.linalg.norm(point - closest))

    def on_motion_abort(self, msg: Bool):
        if msg.data and hasattr(self, "_spray_session"):
            self._spray_off()
            self._multi_cancel("ABORT")
        if not msg.data:
            return
        already_aborted = self._motion_abort_requested
        if already_aborted:
            self._cancel_active_follow_joint_goal("repeated motion_abort")
            return
        self.get_logger().error(
            f"[ABORT] {MOTION_ABORT_TOPIC} 수신 -> 현재 궤적과 D405 prescan 을 "
            "즉시 중단")
        self._motion_abort_requested = True
        safety_reason = str(self._safety_status.get("reason", "")).strip()
        if not self._execution_abort_reason:
            self._execution_abort_reason = (
                safety_reason
                if safety_reason.upper() not in {"", "NONE"}
                else "motion_abort"
            )
        self._publish_painting_command("ABORT", 0.0, enable=False)
        self._cancel_joint_command_timer()
        self._cancel_active_follow_joint_goal("motion_abort")
        self._cancel_stage1_scene_wait_timer()
        self._invalidate_stage1_orientation_candidates(clear_candidates=True)
        self._stage1_attempt_token = None
        self._stage1_goal_constraints = None
        self._stage1_on_complete = None
        MoveItExecutor._cancel_force_phase_lease(self)
        MoveItExecutor._cancel_segment_cartesian_timeout(self)
        self._cancel_process_timer()
        self._paint_entry_context = None
        self._contact_escape_context = None
        self._cancel_runtime_tare()
        self._cancel_d405_prescan_timer()
        self._invalidate_d405_prescan_callbacks(clear_token=True)
        self._cancel_work_area_refine_wait_timer()
        self._d405_prescan_active = False
        self._d405_prescan_queue = []
        self._d405_prescan_index = 0
        self._pose_goal_context = None
        self._joint_goal_context = None
        self._segment_cartesian_context = None
        self._process_steps = []
        self._process_force_ready = False
        self._active_segment_path = None
        MoveItExecutor._clear_execution_snapshot(self)
        self._contact_search_context = None
        self._contact_search_step_active = False
        self._contact_search_mode_published_at = 0.0
        self._contact_search_command_context = None
        self._free_space_confirmed = False
        self._execution_free_space_confirmed = False
        self._execution_tare_ready = False
        self._publish_free_space_confirmation(False)
        self._set_contact_collision_allowed(False)
        self.executing = False
        self._publish_work_area_refine_status("aborted")
        self._publish_execution_status("ABORT", self._execution_abort_reason)

    def _request_motion_abort(self, reason: str):
        if self._motion_abort_requested:
            self._cancel_active_follow_joint_goal(reason)
            return
        self.get_logger().error(f"[ABORT] requested: {reason}")
        self._execution_abort_reason = str(reason)
        msg = Bool()
        msg.data = True
        self.on_motion_abort(msg)
        self.motion_abort_pub.publish(msg)

    def _fail_workflow_known_safe(
        self, reason, state="PLAN_REJECTED", **status_fields
    ):
        """End a run without turning a known no-command failure into E-stop.

        IK/planning rejection, an unavailable planning service, or a dispatch
        interlock detected *before* an FJT goal is sent means the robot state
        is known.  Those failures still clear force, tare and the immutable
        run snapshot, but they must not latch ros2_control until a relaunch.
        Once a physical command may be active, force is enabled, or command
        state is unknown, the same request is escalated to the hard-abort path.
        """

        reason = str(reason).strip() or "WORKFLOW_REJECTED"
        command_active = bool(
            getattr(self, "_active_trajectory_goal_token", None) is not None
            or getattr(self, "_joint_command_timer", None) is not None
        )
        uncertain = bool(
            getattr(self, "_fjt_motion_state_unknown", False)
            or getattr(self, "_acm_state_unknown", False)
        )
        force_active = bool(getattr(self, "_painting_command_enable", False))
        if command_active or uncertain or force_active:
            self._request_motion_abort(
                f"WORKFLOW_FAILURE_AFTER_PHYSICAL_COMMIT:{reason}"
            )
            return False
        if getattr(self, "_motion_abort_requested", False):
            return False

        self.get_logger().error(
            f"[WORKFLOW REJECTED - NO COMMAND ACTIVE] {reason}"
        )
        self._reset_painting_process()
        self.executing = False
        self._publish_execution_status(str(state), reason, **status_fields)
        return True

    @staticmethod
    def _dispatch_failure_requires_relaunch(reason):
        reason = str(reason or "")
        return reason in {
            "HARDWARE_MOTION_INHIBITED_RELAUNCH_REQUIRED",
            "FJT_MOTION_STATE_UNKNOWN_RELAUNCH_REQUIRED",
            "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED",
        }

    def _handle_known_dispatch_rejection(self, label, state="PLAN_REJECTED"):
        reason = str(
            getattr(self, "_last_dispatch_inhibit_reason", "")
            or "TRAJECTORY_NOT_DISPATCHED"
        )
        detail = f"{label}:{reason}"
        if MoveItExecutor._dispatch_failure_requires_relaunch(reason):
            self._request_motion_abort(detail)
            return False
        return self._fail_workflow_known_safe(detail, state=state)

    def _handle_process_trajectory_failure(self, label):
        phase = str(
            getattr(self, "_last_trajectory_failure_phase", "")
        )
        if phase == "GOAL_REJECTED":
            self._fail_workflow_known_safe(
                f"{label}:FJT_GOAL_REJECTED",
                state="PLAN_REJECTED",
            )
            return
        self._request_motion_abort(
            f"{label}:FJT_{phase or 'EXECUTION_FAILED'}"
        )

    def on_reset_execution_abort(self, _request, response):
        """Clear the executor latch only under an explicit safe reset."""

        now = time.monotonic()
        blockers = []
        dispatch_inhibit = MoveItExecutor._motion_dispatch_inhibited_reason(self)
        if dispatch_inhibit:
            blockers.append(dispatch_inhibit)
        if self.executing:
            blockers.append("TRAJECTORY_ACTIVE")
        if self._active_trajectory_goal_token is not None:
            blockers.append("ACTION_RESULT_PENDING")
        if self._painting_command_enable:
            blockers.append("FORCE_ENABLED")
        if not self._robot_stationary_for_bias():
            blockers.append("ROBOT_NOT_STATIONARY")
        if self.real_painting_enabled and getattr(self, "process_mode", "paint") != "spray":
            if (
                self._safety_status_time <= 0.0
                or now - self._safety_status_time > self.ft_required_timeout_s
            ):
                blockers.append("SAFETY_STATUS_STALE")
            elif self._safety_status.get("abort_latched") is not False:
                blockers.append("SAFETY_ABORT_STILL_LATCHED")
        if blockers:
            response.success = False
            response.message = ",".join(blockers)
            return response

        self._motion_abort_requested = False
        self._execution_abort_reason = ""
        self._free_space_confirmed = False
        self._execution_free_space_confirmed = False
        self._execution_tare_ready = False
        self._cancel_runtime_tare()
        self._publish_free_space_confirmation(False)
        self._publish_painting_command("IDLE", 0.0, enable=False)
        self._publish_execution_status("IDLE", "manual execution abort reset")
        response.success = True
        response.message = "execution abort latch reset"
        return response

    def _matching_segment_path(self):
        if not self.use_eoat_segments or self._segment_path is None:
            return None
        path = self._segment_path
        if self.real_painting_enabled or not self.dry_run:
            blockers = self._current_real_plan_blockers()
            if blockers:
                self.get_logger().error(
                    "[PAINT PATH] fail-closed gate: " + ", ".join(blockers)
                )
                return None
            return path

        # Legacy schemas are retained only for an explicitly configured dry run.
        if path.path_id and self._waypoints_path_id:
            if path.path_id == self._waypoints_path_id:
                return path
            self.get_logger().warn(
                "[PAINT PATH] dry-run path_id mismatch -> segment ignored"
            )
            return None
        if self.dry_run:
            self.get_logger().warn(
                "[PAINT PATH] explicit dry-run legacy segment has no path_id"
            )
            return path
        return None

    def on_execute(self, msg: Bool):
        if not msg.data:
            return
        dispatch_inhibit = MoveItExecutor._motion_dispatch_inhibited_reason(self)
        if dispatch_inhibit:
            self.get_logger().error(
                "[EXECUTE] hardware/cancel state requires full relaunch: "
                + dispatch_inhibit
            )
            self._publish_execution_status("NOT_READY", dispatch_inhibit)
            return
        if getattr(self, "_motion_abort_requested", False):
            self.get_logger().error(
                "[EXECUTE] execution abort latch가 설정됨 -> "
                "/painting_system/reset_execution_abort 필요"
            )
            self._publish_execution_status(
                "NOT_READY", "EXECUTION_ABORT_LATCHED"
            )
            return
        matching_path = self._matching_segment_path()
        strict_segment_required = self.real_painting_enabled or not self.dry_run
        if strict_segment_required and matching_path is None:
            blockers = self._current_real_plan_blockers()
            reason = ",".join(blockers) or "MATCHING_V3_SEGMENT_REQUIRED"
            self.get_logger().error(
                "[EXECUTE] matching v3 segment 없음 -> planning/action goal 생성 금지: "
                + reason
            )
            self._publish_execution_status("PLAN_REJECTED", reason)
            return
        if not self.current_waypoints and matching_path is None:
            self.get_logger().warn("실행할 웨이포인트가 없습니다")
            self._publish_execution_status("PLAN_REJECTED", "EMPTY_PATH")
            return
        if self.current_joint_state is None:
            self.get_logger().warn("joint_state 미수신 -> 실행 보류")
            return
        if self.executing:
            self.get_logger().warn("이미 실행 중")
            return
        if self._active_trajectory_goal_token is not None:
            self.get_logger().warn(
                "이전 FollowJointTrajectory goal 취소/종료 응답 대기 중"
            )
            return

        if getattr(self, "process_mode", "paint") == "spray" and self._spray_motion_blockers():
            self._publish_execution_status("NOT_READY", ",".join(self._spray_motion_blockers()))
            return

        if self.real_painting_enabled:
            # Force validity is intentionally established after the geometric
            # approach, at the verified pre-contact pose.  Startup still
            # requires every motion/planning dependency and the three tare
            # services, but not a stale bias from a previous robot pose.
            runtime_blockers = self._real_runtime_blockers(
                require_force_ready=False
            )
            if runtime_blockers:
                reason = ",".join(runtime_blockers)
                self.get_logger().error(
                    "[EXECUTE] startup readiness fail-closed: " + reason
                )
                self._publish_execution_status("NOT_READY", reason)
                return

        if not self._joint_state_within_limits(
                self.current_joint_state, "SKETCH start"):
            self.get_logger().error(
                "현재 joint_state 가 MoveIt/실로봇 limit 밖입니다. "
                "스케치 실행을 중단합니다.")
            return

        if self.real_painting_enabled:
            if not isinstance(self._execution_snapshot, dict):
                snapshot_error = self._capture_execution_snapshot(matching_path)
                if snapshot_error:
                    self.get_logger().error(
                        "[EXECUTE] link0 execution snapshot rejected: "
                        + snapshot_error
                    )
                    self._publish_execution_status(
                        "PLAN_REJECTED", "EXECUTION_SNAPSHOT:" + snapshot_error
                    )
                    return
            matching_path = self._execution_snapshot["segment_path"]
        else:
            self._active_segment_path = matching_path

        self._execution_free_space_confirmed = bool(
            self._free_space_confirmed
        )
        self._execution_tare_ready = False
        self._runtime_tare_phase = "IDLE"
        # The operator confirmation is latched for this immutable plan.  Clear
        # the live one-shot signal while the arm moves; it is reasserted only
        # after the 10 mm pre-contact goal has stopped and stationarity holds.
        if self.painting_force_enabled:
            self._publish_free_space_confirmation(False)
        self._publish_painting_command("IDLE", 0.0, enable=False)

        # 첫 Submit 안전성: READY_POSE 가 아니면 먼저 collision-aware joint plan 으로
        # READY_POSE 로 복귀한 뒤 같은 sketch execute 를 다시 시작한다.
        if (START_FROM_READY_BEFORE_SKETCH
                and getattr(self, "model_id", DEFAULT_MODEL) == DEFAULT_MODEL
                and not self._is_at_ready_pose()):
            self.get_logger().warn(
                "현재 자세가 READY_POSE 와 다름 -> READY_POSE 먼저 이동 후 "
                "스케치 제어를 시작합니다.")
            self.executing = True
            self._plan_joint_goal(
                "PRE_SKETCH_READY_POSE",
                READY_POSE_JOINTS,
                STAGE5_SPEED_SCALE,
                finalize_cb=self._pre_sketch_ready_done,
            )
            return

        self._sync_active_target_plane_from_waypoints(self.current_waypoints)
        # Real execution consumes an already accepted, hashed D405 generation.
        # Measuring here would mutate the plane after operator confirmation.
        if not self.real_painting_enabled and self._maybe_begin_d405_prescan():
            return
        self.executing = True
        self._publish_execution_status("SAFETY_APPROACH", "execute accepted")
        self._start_sketch_motion_after_surface_ready()

    def _start_sketch_motion_after_surface_ready(self):
        # D405 refined plane 이 준비된 경우, 기존 ZED 기반 waypoint 를 새 평면에
        # 한 번 더 투영한다. sketch_to_waypoints 가 다시 발행되지 않아도 같은
        # sketch path 를 더 정확한 작업면 위에서 실행하기 위함이다.
        if self._active_segment_path is None or self._active_segment_path.version < 3:
            self._apply_refined_surface_to_current_waypoints()
            self._apply_refined_surface_to_active_segment_path()

        if self._active_segment_path is not None:
            prepared = self._prepare_segment_process()
            if prepared is None:
                self._fail_stage1_before_motion(
                    "STAGE1_SEGMENT_PREPARATION_FAILED"
                )
                return
            target, n = prepared
            safety_tcp = self._safety_tcp_pose
            stage1_on_complete = self._start_segment_process
        else:
            # Legacy PoseArray execution remains available for non-segment paths.
            densified_tip, tcp_wps, target, n = self._compute_snapped_tcp_waypoints()
            if not self._validate_contact_waypoints(densified_tip, target):
                self.get_logger().error(
                    "waypoint safety validation 실패 -> 실행 중단")
                self._fail_stage1_before_motion(
                    "STAGE1_WAYPOINT_VALIDATION_FAILED"
                )
                return
            self._stage3_tip_wps = densified_tip
            self._stage3_tcp_wps = tcp_wps

            fixed_q = self._active_ee_quat(target)
            first_tip = densified_tip[0]
            safety_tip = self._offset_along_normal(first_tip, SAFETY_OFFSET)
            safety_tcp = self._brush_tip_to_tcp(safety_tip)
            safety_tcp.orientation.x = float(fixed_q[0])
            safety_tcp.orientation.y = float(fixed_q[1])
            safety_tcp.orientation.z = float(fixed_q[2])
            safety_tcp.orientation.w = float(fixed_q[3])
            self._safety_tcp_pose = safety_tcp

            last_tip = densified_tip[-1]
            retreat_tip = self._offset_along_normal(last_tip, RETREAT_OFFSET)
            retreat_tcp = self._brush_tip_to_tcp(retreat_tip)
            retreat_tcp.orientation.x = float(fixed_q[0])
            retreat_tcp.orientation.y = float(fixed_q[1])
            retreat_tcp.orientation.z = float(fixed_q[2])
            retreat_tcp.orientation.w = float(fixed_q[3])
            self._retreat_tcp_pose = retreat_tcp
            stage1_on_complete = None

        if not self.real_painting_enabled and self.ft_status_time <= 0.0:
            self.get_logger().warn(
                "[FT GUARD] /ft/status 미수신. 이번 실행은 힘 기반 "
                "과압 정지 없이 진행됩니다.")
        elif (
            not self.real_painting_enabled
            and time.monotonic() - self.ft_status_time > FT_AUTO_ZERO_STALE_SEC
        ):
            self.get_logger().warn(
                "[FT GUARD] /ft/status 가 오래됨. 이번 실행은 힘 기반 "
                "과압 정지 없이 진행될 수 있습니다.")
        elif not self.real_painting_enabled and not self.ft_bias_ready:
            self.get_logger().warn(
                "[FT GUARD] FT bias not ready. 스케치 시작 전 자동 zero 를 "
                "시도합니다.")
        self.get_logger().info("=" * 60)
        self.get_logger().info("=== STAGE 1: free-space approach (OMPL) ===")
        self.get_logger().info(
            f"safety_tcp=({safety_tcp.position.x:.3f},"
            f"{safety_tcp.position.y:.3f},{safety_tcp.position.z:.3f})")
        self._begin_stage1_with_optional_ft_zero(on_complete=stage1_on_complete)

    def _d405_refined_surface_fresh(self):
        if self.real_painting_enabled:
            return bool(
                self._d405_plane_accepted
                and self._current_work_area_id
                and self._current_plane_generation_id
                and self._d405_refined_lock_active
                and self.dynamic_surface_source == "d405_refined"
            )
        return bool(
            self._d405_refined_lock_active
            and self.dynamic_surface_source == "d405_refined"
        )

    def _reset_d405_refined_lock(self, reason: str, clear_surface: bool = False):
        was_locked = self._d405_refined_lock_active
        self._d405_refined_lock_active = False
        self._d405_refined_lock_time = 0.0
        self._d405_refined_lock_signature = None
        self._d405_refined_allow_corner_correction = False
        self._d405_refined_pose_armed = False
        self._d405_refined_pose_generation_id = ""
        if clear_surface and self.dynamic_surface_source == "d405_refined":
            self.dynamic_surface_point = None
            self.dynamic_surface_normal = None
            self.dynamic_surface_source = "work_area_changed"
            self.dynamic_surface_source_time = 0.0
            MoveItExecutor._mark_scene_dirty(self)
        if was_locked or clear_surface:
            self.get_logger().info(
                f"[SURFACE] D405 refined plane lock 해제: {reason}")

    def _maybe_begin_d405_prescan(self):
        if not D405_PREFLIGHT_SCAN_ENABLED:
            return False
        if self._d405_refined_surface_fresh():
            self.get_logger().info(
                "[D405 PRESCAN] fresh refined plane 있음 -> 사전 측정 생략")
            return False

        return self._begin_d405_prescan(mode="sketch")

    def _cancel_d405_orientation_timer(self):
        timer = getattr(self, "_d405_orientation_timer", None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self._d405_orientation_timer = None

    def _cancel_d405_arrival_timer(self):
        timer = getattr(self, "_d405_arrival_timer", None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self._d405_arrival_timer = None

    def _cancel_d405_scene_wait_timer(self):
        timer = getattr(self, "_d405_scene_wait_timer", None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self._d405_scene_wait_timer = None
        self._d405_scene_wait_start = None
        self._d405_scene_wait_revision = None

    def _invalidate_d405_prescan_callbacks(self, *, clear_token):
        """Invalidate delayed IK/MoveGroup/Cartesian callbacks from an old scan."""

        self._cancel_d405_scene_wait_timer()
        self._cancel_d405_orientation_timer()
        self._cancel_d405_arrival_timer()
        self._d405_orientation_generation = int(
            getattr(self, "_d405_orientation_generation", 0)
        ) + 1
        self._d405_orientation_context = None
        self._pose_goal_context = None
        self._d405_cartesian_context = None
        if clear_token:
            self._d405_prescan_token = None

    def _d405_prescan_callback_valid(self, token):
        return bool(
            token is not None
            and token is getattr(self, "_d405_prescan_token", None)
            and getattr(self, "_d405_prescan_active", False)
            and not getattr(self, "_motion_abort_requested", False)
        )

    def _start_d405_scene_wait(self, token, revision):
        if not self._d405_prescan_callback_valid(token):
            return
        revision = int(revision)
        self._cancel_d405_scene_wait_timer()
        if self._d405_scene_revision_confirmed(revision):
            self.get_logger().info(
                f"[D405 PRESCAN] PlanningScene revision {revision} confirmed"
            )
            self._plan_next_d405_prescan_pose()
            return

        self._d405_scene_wait_start = time.monotonic()
        self._d405_scene_wait_revision = revision
        timer_ref = {}

        def wait_for_this_scan():
            self._d405_wait_for_scene_confirmed(
                token,
                revision,
                timer_ref.get("timer"),
            )

        timer = self.create_timer(
            SCENE_WAIT_PERIOD_SEC,
            wait_for_this_scan,
        )
        timer_ref["timer"] = timer
        self._d405_scene_wait_timer = timer
        self.get_logger().info(
            "[D405 PRESCAN] PlanningScene confirmation pending for revision "
            f"{revision} (timeout={SCENE_WAIT_TIMEOUT_SEC:.1f}s)"
        )

    def _d405_wait_for_scene_confirmed(
        self, token, revision, timer_identity
    ):
        revision = int(revision)
        if (
            not self._d405_prescan_callback_valid(token)
            or timer_identity is not getattr(
                self, "_d405_scene_wait_timer", None
            )
            or revision != getattr(self, "_d405_scene_wait_revision", None)
        ):
            # A canceled timer callback can already be queued in the executor.
            # It belongs to an older Set Work Area/run and must not touch the
            # current prescan timer or start its IK requests.
            return

        current_revision = int(getattr(self, "_scene_revision", 0))
        if current_revision != revision:
            self.get_logger().error(
                "[D405 PRESCAN] PlanningScene changed while waiting: "
                f"captured={revision}, current={current_revision}"
            )
            self._cancel_d405_scene_wait_timer()
            self._finish_d405_prescan(success=False)
            return
        if self._d405_scene_revision_confirmed(revision):
            self._cancel_d405_scene_wait_timer()
            self.get_logger().info(
                f"[D405 PRESCAN] PlanningScene revision {revision} confirmed"
            )
            self._plan_next_d405_prescan_pose()
            return

        elapsed = time.monotonic() - float(
            self._d405_scene_wait_start or 0.0
        )
        if elapsed < SCENE_WAIT_TIMEOUT_SEC:
            self.get_logger().info(
                "[D405 PRESCAN] PlanningScene confirmation pending...",
                throttle_duration_sec=1.0,
            )
            return

        self.get_logger().error(
            "[D405 PRESCAN] PlanningScene confirmation timed out before IK"
        )
        self._cancel_d405_scene_wait_timer()
        self._finish_d405_prescan(success=False)

    def _begin_d405_prescan(self, mode="sketch"):
        self._invalidate_d405_prescan_callbacks(clear_token=True)
        scan_poses = self._build_d405_prescan_poses()
        if not scan_poses:
            self.get_logger().warn(
                "[D405 PRESCAN] scan pose 생성 실패")
            return False

        self.executing = True
        self._d405_prescan_active = True
        self._d405_prescan_mode = mode
        self._d405_prescan_queue = scan_poses
        self._d405_prescan_index = 0
        self._d405_prescan_token = object()
        self._d405_selected_orientation_branch = ""
        scene_revision = int(getattr(self, "_scene_revision", 0))
        self.publish_scene_periodic()
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            "=== D405 PRESCAN: work-area plane refinement ===")
        self.get_logger().info(
            f"[D405 PRESCAN] mode={mode}, poses={len(scan_poses)}, "
            f"standoff={D405_PREFLIGHT_SCAN_STANDOFF:.2f}m")
        self._start_d405_scene_wait(
            self._d405_prescan_token,
            scene_revision,
        )
        return True

    def _build_d405_prescan_poses(self):
        self._d405_scan_candidates = []
        try:
            mount = self._d405_mount_transform()
            current = self._current_tcp_pose_np()
            if current is None:
                raise ValueError("current TCP TF unavailable")
            if getattr(self,"_multi_current",None) is not None:
                polygon,normal = self._d405_support_in_base(self._multi_current)
            else:
                basis = self._dynamic_work_area_basis()
                if basis is None:
                    raise ValueError("finite work-area support unavailable")
                center,u,v,hu,hv = basis
                _point,normal = self._active_surface_plane(get_target(self.cfg,self.active_target_name))
                polygon = np.array([center+su*hu*u+sv*hv*v for su,sv in [(-1,1),(1,1),(1,-1),(-1,-1)]])
                polygon = np.array([self._project_point_to_active_surface(point,normal) for point in polygon])
            normal = np.asarray(normal,float)
            normal /= np.linalg.norm(normal)
            camera = current[0]+quat_apply(current[1],mount[0])
            points = measurement_samples(camera,polygon,normal,margin=D405_PREFLIGHT_SCAN_INSET_M)
            self._d405_scan_mount = mount
            self._d405_prescan_surface_normal = normal
            self._d405_prescan_surface_points = points
            for index,point in enumerate(points):
                ordered = [point]+[p for i,p in enumerate(points) if i != index]
                for flipped in (False,True):
                    _position,q = camera_view(point,normal,current,mount,D405_PREFLIGHT_SCAN_STANDOFF,flipped)
                    poses = tuple(self._make_d405_scan_tcp_pose(p,normal,q) for p in ordered)
                    self._d405_scan_candidates.append(dict(
                        name=f"d405_sample_{index}_{'roll180' if flipped else 'near_current'}",
                        candidate_index=len(self._d405_scan_candidates),poses=poses,surface_points=ordered))
            self.get_logger().info(f"[D405 VIEW] {len(points)} interior samples, {len(self._d405_scan_candidates)} calibrated optical poses")
            return list(self._d405_scan_candidates[0]['poses'])
        except Exception as exc:
            self.get_logger().error(f"[D405 VIEW] cannot build calibrated measurement poses: {exc}")
            return []

    def _current_tcp_pose_np(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (
            np.array([t.x, t.y, t.z], dtype=float),
            np.array([q.x, q.y, q.z, q.w], dtype=float),
        )

    def _runtime_tare_tcp_pose_sample(self):
        """Read a timestamped TCP sample for the execution-scoped tare gate."""

        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                EE_LINK,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            return None, f"actual TCP TF unavailable: {exc}"
        stamp = transform.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = int(self.get_clock().now().nanoseconds)
        if stamp_ns <= 0 or now_ns <= 0:
            return None, "actual TCP TF has invalid ROS timestamp"
        age_s = float(now_ns - stamp_ns) / 1e9
        if not math.isfinite(age_s):
            return None, "actual TCP TF age is non-finite"
        if age_s > self.runtime_tare_max_tcp_tf_age_s:
            return (
                None,
                "actual TCP TF stale: %.3fs exceeds %.3fs"
                % (age_s, self.runtime_tare_max_tcp_tf_age_s),
            )
        if age_s < -0.05:
            return None, f"actual TCP TF timestamp is {-age_s:.3f}s in the future"
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (
                np.array(
                    [translation.x, translation.y, translation.z], dtype=float
                ),
                np.array(
                    [rotation.x, rotation.y, rotation.z, rotation.w], dtype=float
                ),
                age_s,
            ),
            "",
        )

    def _waypoint_or_normal_tcp_quat(self, waypoint, normal):
        q_msg = waypoint.orientation
        q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w], dtype=float)
        if float(np.linalg.norm(q)) < 1e-6:
            return self._tcp_quat_for_surface_normal(normal)
        q /= np.linalg.norm(q) + 1e-12
        try:
            tool_axis = quat_apply(q, [0.0, -1.0, 0.0])
            align = float(np.dot(tool_axis, -np.asarray(normal, dtype=float)))
        except Exception:
            align = 0.0
        if align < 0.90:
            self.get_logger().warn(
                f"[D405 PRESCAN] waypoint orientation normal 정렬 낮음 "
                f"(align={align:+.2f}) -> normal 기반 자세로 재계산")
            return self._tcp_quat_for_surface_normal(normal)
        return q

    def _tcp_quat_for_surface_normal_near_current(self, normal):
        """Align TCP local -Y to the surface while preserving current roll.

        The surface normal fixes only one tool axis. Rotation around that normal
        should stay as close as possible to the current TCP orientation, otherwise
        MoveIt may pick a wrist/base-flipped IK solution even for a nearby pose.
        """
        y_axis = np.asarray(normal, dtype=float)
        y_axis /= np.linalg.norm(y_axis) + 1e-12

        current = self._current_tcp_pose_np()
        if current is None:
            return self._tcp_quat_for_surface_normal(normal)

        _tcp_pos, current_q = current
        r_current = quat_to_matrix(current_q)
        x_ref = r_current[:, 0]
        z_ref = r_current[:, 2]

        x_axis = x_ref - y_axis * float(np.dot(x_ref, y_axis))
        if float(np.linalg.norm(x_axis)) > 1e-6:
            x_axis /= np.linalg.norm(x_axis) + 1e-12
            z_axis = np.cross(x_axis, y_axis)
            z_axis /= np.linalg.norm(z_axis) + 1e-12
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis) + 1e-12
        else:
            z_axis = z_ref - y_axis * float(np.dot(z_ref, y_axis))
            if float(np.linalg.norm(z_axis)) < 1e-6:
                return self._tcp_quat_for_surface_normal(normal)
            z_axis /= np.linalg.norm(z_axis) + 1e-12
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis) + 1e-12
            z_axis = np.cross(x_axis, y_axis)
            z_axis /= np.linalg.norm(z_axis) + 1e-12

        q = quat_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))
        old_tool_forward = -quat_apply(current_q, [0.0, 1.0, 0.0])
        new_tool_forward = -quat_apply(q, [0.0, 1.0, 0.0])
        align = float(np.dot(new_tool_forward, -y_axis))
        roll_keep = float(np.dot(
            quat_apply(current_q, [1.0, 0.0, 0.0]),
            quat_apply(q, [1.0, 0.0, 0.0]),
        ))
        self.get_logger().info(
            "[D405 PRESCAN] normal-only 자세 선택 "
            f"(tool_align={align:+.3f}, current_forward_dot={float(np.dot(old_tool_forward, -y_axis)):+.3f}, "
            f"roll_keep={roll_keep:+.3f})")
        return q

    @staticmethod
    def _tcp_quat_for_surface_normal(normal):
        forward = -np.asarray(normal, dtype=float)
        forward /= np.linalg.norm(forward) + 1e-12
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(forward, world_up))) > 0.99:
            world_up = np.array([1.0, 0.0, 0.0], dtype=float)
        y_axis = -forward
        z_axis = world_up - y_axis * float(np.dot(world_up, y_axis))
        z_axis /= np.linalg.norm(z_axis) + 1e-12
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis) + 1e-12
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis) + 1e-12
        return quat_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))

    def _d405_prescan_surface_samples(self, normal):
        expected = ROLLER_RADIUS + CONTACT_CLEARANCE
        if self.current_waypoints:
            pts = np.array([
                [p.position.x, p.position.y, p.position.z]
                for p in self.current_waypoints
            ], dtype=float)
            fallback_center = np.mean(pts, axis=0) - normal * expected
        elif self.dynamic_surface_point is not None:
            fallback_center = np.asarray(self.dynamic_surface_point, dtype=float)
        else:
            fallback_center = np.zeros(3, dtype=float)
        samples = [fallback_center]

        basis = self._dynamic_work_area_basis()
        if basis is not None:
            center, u_axis, v_axis, half_u, half_v = basis
            center = self._project_point_to_active_surface(center, normal)
            u_axis = np.asarray(u_axis, dtype=float)
            u_axis /= np.linalg.norm(u_axis) + 1e-12
            v_axis = np.asarray(v_axis, dtype=float)
            v_axis /= np.linalg.norm(v_axis) + 1e-12

            inset = min(D405_PREFLIGHT_SCAN_INSET_M, half_u, half_v)
            u_limit = max(0.0, float(half_u) - inset)
            v_limit = max(0.0, float(half_v) - inset)
            anchor_u = 0.0
            anchor_v = 0.0
            current = self._current_tcp_pose_np()
            if current is not None:
                tcp_pos, tcp_q = current
                current_camera = (
                    tcp_pos + quat_apply(tcp_q, D405_COLLISION_CENTER)
                )
                desired_surface = (
                    current_camera
                    - np.asarray(normal, dtype=float) * D405_PREFLIGHT_SCAN_STANDOFF
                )
                desired_surface = self._project_point_to_active_surface(
                    desired_surface, normal)
                rel = desired_surface - center
                anchor_u = float(np.clip(
                    np.dot(rel, u_axis), -u_limit, u_limit))
                anchor_v = float(np.clip(
                    np.dot(rel, v_axis), -v_limit, v_limit))
                self.get_logger().info(
                    "[D405 PRESCAN] 작업영역 내 최소 이동 측정점 선택 "
                    f"(u={anchor_u:+.3f}m, v={anchor_v:+.3f}m)")

            probe_u = min(D405_PREFLIGHT_PROBE_OFFSET_M, u_limit)
            probe_v = min(D405_PREFLIGHT_PROBE_OFFSET_M, v_limit)
            uv_candidates = [(anchor_u, anchor_v)]
            if probe_u >= 0.03:
                uv_candidates.append((anchor_u + probe_u, anchor_v))
            if probe_v >= 0.03:
                uv_candidates.append((anchor_u, anchor_v + probe_v))
            if probe_u >= 0.03:
                uv_candidates.append((anchor_u - probe_u, anchor_v))
            if probe_v >= 0.03:
                uv_candidates.append((anchor_u, anchor_v - probe_v))
            uv_candidates.append((0.0, 0.0))

            samples = []
            for u, v in uv_candidates:
                u = float(np.clip(u, -u_limit, u_limit))
                v = float(np.clip(v, -v_limit, v_limit))
                p = center + u * u_axis + v * v_axis
                p = self._project_point_to_active_surface(p, normal)
                samples.append(p)

        unique = []
        for p in samples:
            if not any(float(np.linalg.norm(p - q)) < 0.03 for q in unique):
                unique.append(np.asarray(p, dtype=float))
        if D405_PREFLIGHT_SCAN_MAX_POSES > 0 and len(unique) > D405_PREFLIGHT_SCAN_MAX_POSES:
            self.get_logger().warn(
                "[D405 PRESCAN] 안전 모드: 자동 work-area 촬영 pose 를 "
                f"{len(unique)}개 -> {D405_PREFLIGHT_SCAN_MAX_POSES}개로 제한")
            unique = unique[:D405_PREFLIGHT_SCAN_MAX_POSES]
        return unique

    def _make_d405_scan_tcp_pose(self, surface_point, normal, q):
        normal = np.asarray(normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        q = np.asarray(q, dtype=float)
        q /= np.linalg.norm(q) + 1e-12
        desired_camera = (
            np.asarray(surface_point, dtype=float)
            + normal * D405_PREFLIGHT_SCAN_STANDOFF
        )
        mount = getattr(self,"_d405_scan_mount",None)
        camera_offset_world = quat_apply(q, mount[0] if mount is not None else D405_COLLISION_CENTER)
        tcp_pos = desired_camera - camera_offset_world
        pose = Pose()
        pose.position.x = float(tcp_pos[0])
        pose.position.y = float(tcp_pos[1])
        pose.position.z = float(tcp_pos[2])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

    def _d405_scan_pose_branch(self, *, flipped):
        """Rebuild a complete scan branch at the identical camera centers.

        D405_COLLISION_CENTER has non-zero TCP-local components.  Merely flipping
        the TCP quaternion at a fixed TCP translation would move the physical
        camera and violate the requested standoff.  Both task-equivalent
        branches are therefore rebuilt from the same surface samples and normal.
        """

        points = list(getattr(self, "_d405_prescan_surface_points", ()))
        normal = getattr(self, "_d405_prescan_surface_normal", None)
        queue = list(getattr(self, "_d405_prescan_queue", ()))
        if not points or normal is None or len(points) != len(queue):
            return None
        first = queue[0]
        if flipped:
            first = MoveItExecutor._flip_pose_about_tcp_y(first)
        quaternion = np.array(
            [
                first.orientation.x,
                first.orientation.y,
                first.orientation.z,
                first.orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-9:
            return None
        quaternion /= np.linalg.norm(quaternion)
        return tuple(
            self._make_d405_scan_tcp_pose(point, normal, quaternion)
            for point in points
        )

    def _d405_scene_revision_confirmed(self, expected_revision=None):
        current = int(getattr(self, "_scene_revision", 0))
        if expected_revision is not None and current != int(expected_revision):
            return False
        return bool(
            getattr(self, "scene_confirmed", False)
            and int(getattr(self, "_scene_confirmed_revision", -1)) == current
        )

    def _request_d405_orientation_candidate_iks(
        self, label, pose, finalize_cb
    ):
        """Collision-check all calibrated sample/roll candidates from one seed."""

        del pose  # Branch poses are rebuilt from the captured camera centers.
        token = getattr(self, "_d405_prescan_token", None)
        if not self._d405_prescan_callback_valid(token):
            return
        scene_revision = int(getattr(self, "_scene_revision", 0))
        if not self._d405_scene_revision_confirmed(scene_revision):
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] planning scene revision is not "
                "confirmed before collision-aware IK"
            )
            finalize_cb(False)
            return
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] /compute_ik unavailable"
            )
            finalize_cb(False)
            return
        if self.current_joint_state is None:
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] measured joint seed unavailable"
            )
            finalize_cb(False)
            return
        seed_age = time.monotonic() - float(
            getattr(self, "current_joint_state_time", 0.0)
        )
        if (
            not math.isfinite(seed_age)
            or seed_age < 0.0
            or seed_age > D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S
            or not self._joint_state_within_limits(
                self.current_joint_state, "D405 prescan IK seed"
            )
        ):
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] measured joint seed is stale/invalid "
                f"(age={seed_age:.3f}s)"
            )
            finalize_cb(False)
            return

        candidates = list(getattr(self,"_d405_scan_candidates",()))
        compare_views = bool(candidates)
        if not candidates:
            # Legacy callers without a view pool retain the existing two poses.
            branches = (self._d405_scan_pose_branch(flipped=False),self._d405_scan_pose_branch(flipped=True))
            if any(not branch for branch in branches):
                finalize_cb(False)
                return
            candidates = [dict(name=name,poses=branch,candidate_index=index)
                          for index,(name,branch) in enumerate(zip(
                              ("d405_near_current","d405_flipped_local_y_180"),branches))]

        self._cancel_d405_orientation_timer()
        self._d405_orientation_generation += 1
        generation = self._d405_orientation_generation
        seed_state = copy.deepcopy(self.current_joint_state)
        context = {
            "token": token,
            "generation": generation,
            "scene_revision": scene_revision,
            "label": str(label),
            "finalize_cb": finalize_cb,
            "seed_state": seed_state,
            "candidates": tuple(candidates),
            "compare_views": compare_views,
            "planned_views": [],
            "results": {},
            "ranked": [],
            "rank_index": -1,
            "finalized": False,
            "fjt_dispatched": False,
        }
        self._d405_orientation_context = context

        for index, candidate in enumerate(context["candidates"]):
            request = GetPositionIK.Request()
            request.ik_request.group_name = PLANNING_GROUP
            # Both requests use the same immutable measured seed.  Callback
            # order therefore cannot change the selected orientation.
            request.ik_request.robot_state.joint_state = copy.deepcopy(seed_state)
            request.ik_request.robot_state.is_diff = False
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = EE_LINK
            request.ik_request.pose_stamped.header.frame_id = BASE_FRAME
            request.ik_request.pose_stamped.header.stamp = (
                self.get_clock().now().to_msg()
            )
            request.ik_request.pose_stamped.pose = copy.deepcopy(
                candidate["poses"][0]
            )
            request.ik_request.timeout = Duration(
                seconds=STAGE1_IK_TIMEOUT_S
            ).to_msg()
            try:
                future = self.ik_client.call_async(request)
            except Exception as exc:
                context["results"][index] = {
                    "valid": False,
                    "reason": f"IK_REQUEST_FAILED:{exc}",
                    "candidate_index": index,
                    "candidate": candidate,
                }
                continue
            future.add_done_callback(
                lambda done, c=context, i=index:
                self._d405_orientation_candidate_ik_done(done, c, i)
            )

        if len(context["results"]) == len(context["candidates"]):
            self._finish_d405_orientation_candidate_iks(context)
            return

        def timeout_candidates():
            if (
                context is not self._d405_orientation_context
                or not self._d405_prescan_callback_valid(token)
                or generation != self._d405_orientation_generation
            ):
                return
            for index, candidate in enumerate(context["candidates"]):
                context["results"].setdefault(
                    index,
                    {
                        "valid": False,
                        "reason": "IK_RESPONSE_TIMEOUT",
                        "candidate_index": index,
                        "candidate": candidate,
                    },
                )
            self._finish_d405_orientation_candidate_iks(context)

        self._d405_orientation_timer = self.create_timer(
            max(STAGE1_DUAL_IK_RESPONSE_TIMEOUT_S, len(candidates)*STAGE1_IK_TIMEOUT_S+1.0),
            timeout_candidates,
        )
        self.get_logger().info(
            "[D405 PRESCAN SYMMETRY] collision-aware IK requested for "
            "calibrated measurement candidates"
        )

    def _d405_orientation_candidate_ik_done(self, future, context, index):
        if (
            context is not getattr(self, "_d405_orientation_context", None)
            or context["generation"] != self._d405_orientation_generation
            or not self._d405_prescan_callback_valid(context["token"])
            or index in context["results"]
        ):
            return
        candidate = context["candidates"][index]
        record = {
            "valid": False,
            "reason": "IK_UNKNOWN",
            "candidate_index": index,
            "candidate": candidate,
        }
        try:
            response = future.result()
            error_code = int(response.error_code.val)
            if error_code != 1:
                if error_code == -31:
                    record["reason"] = "IK_NO_COLLISION_FREE_SOLUTION:-31"
                else:
                    record["reason"] = (
                        f"IK_COLLISION_CHECK_FAILED:{error_code}"
                    )
            else:
                metrics, reason = self._stage1_candidate_joint_metrics(
                    response.solution.joint_state,
                    seed_state=context["seed_state"],
                )
                if metrics is None:
                    record["reason"] = reason
                else:
                    record.update(metrics)
                    record["valid"] = True
                    record["reason"] = ""
        except Exception as exc:
            record["reason"] = f"IK_RESPONSE_FAILED:{exc}"

        context["results"][index] = record
        if record["valid"]:
            if (
                record["max_delta"]
                > D405_PREFLIGHT_LARGE_START_GOAL_WARN_RAD
            ):
                self.get_logger().warn(
                    "[D405 PRESCAN SYMMETRY] %s requires a large smooth "
                    "rotation: %s %.1fdeg; retaining because the endpoint "
                    "and complete URDF path are collision checked"
                    % (
                        candidate["name"],
                        record["max_name"],
                        math.degrees(record["max_delta"]),
                    )
                )
            self.get_logger().info(
                "[D405 PRESCAN SYMMETRY] %s collision-aware IK OK: "
                "max=%s %.1fdeg, l2=%.2frad"
                % (
                    candidate["name"],
                    record["max_name"],
                    math.degrees(record["max_delta"]),
                    record["l2_delta"],
                )
            )
        else:
            self.get_logger().warn(
                f"[D405 PRESCAN SYMMETRY] {candidate['name']} rejected: "
                f"{record['reason']}"
            )
        if len(context["results"]) == len(context["candidates"]):
            self._finish_d405_orientation_candidate_iks(context)

    def _finish_d405_orientation_candidate_iks(self, context):
        if (
            context is not getattr(self, "_d405_orientation_context", None)
            or context["generation"] != self._d405_orientation_generation
            or not self._d405_prescan_callback_valid(context["token"])
            or context["finalized"]
            or len(context["results"]) != len(context["candidates"])
        ):
            return
        context["finalized"] = True
        self._cancel_d405_orientation_timer()
        if not self._d405_scene_revision_confirmed(context["scene_revision"]):
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] planning scene changed during view IK"
            )
            context["finalize_cb"](False)
            return

        incomplete_prefixes = (
            "IK_RESPONSE_TIMEOUT",
            "IK_REQUEST_FAILED",
            "IK_RESPONSE_FAILED",
            "IK_COLLISION_CHECK_FAILED",
        )
        incomplete = [
            str(record.get("reason", ""))
            for record in context["results"].values()
            if str(record.get("reason", "")).startswith(incomplete_prefixes)
        ]
        if incomplete:
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] all endpoint collision checks did "
                "not complete: " + ";".join(incomplete)
            )
            context["finalize_cb"](False)
            return

        ranked = [
            record
            for _, record in sorted(context["results"].items())
            if record.get("valid") is True
        ]
        ranked.sort(
            key=lambda record: (
                float(record["max_delta"]),
                float(record["l2_delta"]),
                float(record["sum_delta"]),
                int(record["candidate_index"]),
            )
        )
        context["ranked"] = ranked
        if not ranked:
            reasons = ";".join(
                str(context["results"][index].get("reason", ""))
                for index in range(len(context["candidates"]))
            )
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] no collision-free orientation: "
                + reasons
            )
            context["finalize_cb"](False)
            return
        self._activate_d405_orientation_rank(context, 0)

    def _activate_d405_orientation_rank(self, context, rank_index, trajectory=None):
        if (
            context is not getattr(self, "_d405_orientation_context", None)
            or not self._d405_prescan_callback_valid(context["token"])
            or context.get("fjt_dispatched")
            or not (0 <= int(rank_index) < len(context["ranked"]))
        ):
            return False
        if not self._d405_scene_revision_confirmed(context["scene_revision"]):
            context["finalize_cb"](False)
            return False
        record = context["ranked"][int(rank_index)]
        context["rank_index"] = int(rank_index)
        branch_poses = tuple(record["candidate"]["poses"])
        self._d405_prescan_queue = [copy.deepcopy(pose) for pose in branch_poses]
        self._d405_selected_orientation_branch = record["candidate"]["name"]
        if "surface_points" in record["candidate"]:
            self._d405_prescan_surface_points = list(record["candidate"]["surface_points"])
        self.get_logger().info(
            "[D405 PRESCAN SYMMETRY] planning exact IK branch %s (%d/%d)"
            % (
                self._d405_selected_orientation_branch,
                int(rank_index) + 1,
                len(context["ranked"]),
            )
        )
        if trajectory is None:
            self._send_d405_orientation_plan(context, record)
        else:
            self._dispatch_d405_orientation_trajectory(context, record, trajectory)
        return True

    def _try_next_d405_orientation_rank(self, context, reason):
        if context.get("fjt_dispatched"):
            self._request_motion_abort(
                "D405_PRESCAN_FAILURE_AFTER_DISPATCH:" + str(reason)
            )
            return False
        if context.get("compare_views"):
            return self._advance_d405_view_plans(context)
        next_index = int(context.get("rank_index", -1)) + 1
        if next_index < len(context.get("ranked", ())):
            self.get_logger().warn(
                "[D405 PRESCAN SYMMETRY] selected branch plan rejected "
                f"({reason}); trying the other collision-checked branch"
            )
            return self._activate_d405_orientation_rank(context, next_index)
        self.get_logger().error(
            "[D405 PRESCAN SYMMETRY] all collision-checked branch plans "
            f"rejected: {reason}"
        )
        context["finalize_cb"](False)
        return False

    def _send_d405_orientation_plan(self, context, record):
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        start_state = RobotState()
        start_state.joint_state = copy.deepcopy(context["seed_state"])
        start_state.is_diff = False
        goal.request.start_state = start_state
        # This is the exact normalized six-joint solution returned by the
        # collision-aware IK request, not a second unconstrained pose solve.
        goal.request.goal_constraints = [copy.deepcopy(record["constraints"])]
        goal.request.planner_id = PLANNER_ID
        goal.request.allowed_planning_time = ALLOWED_PLANNING_TIME
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.max_velocity_scaling_factor = D405_PREFLIGHT_SPEED_SCALE
        goal.request.max_acceleration_scaling_factor = D405_PREFLIGHT_SPEED_SCALE
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        rank_index = int(context["rank_index"])
        try:
            future = self.move_action_client.send_goal_async(goal)
        except Exception as exc:
            self._try_next_d405_orientation_rank(
                context, f"SEND_GOAL_REQUEST_FAILED:{exc}"
            )
            return
        future.add_done_callback(
            lambda done, c=context, i=rank_index:
            self._d405_orientation_plan_goal_response(done, c, i)
        )

    def _d405_orientation_plan_goal_response(
        self, future, context, rank_index
    ):
        if (
            context is not getattr(self, "_d405_orientation_context", None)
            or not self._d405_prescan_callback_valid(context["token"])
            or int(rank_index) != int(context.get("rank_index", -1))
        ):
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._try_next_d405_orientation_rank(
                context, f"GOAL_RESPONSE_FAILED:{exc}"
            )
            return
        if not handle.accepted:
            self._try_next_d405_orientation_rank(context, "GOAL_REJECTED")
            return
        self.get_logger().info(
            f"{context['label']} exact-joint goal accepted, planning..."
        )
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            self._try_next_d405_orientation_rank(
                context, f"RESULT_REQUEST_FAILED:{exc}"
            )
            return
        result_future.add_done_callback(
            lambda done, c=context, i=rank_index:
            self._d405_orientation_plan_result(done, c, i)
        )

    def _d405_plan_endpoint_matches_ik(self, trajectory, record):
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            return False, "EMPTY_TRAJECTORY"
        names = list(joint_trajectory.joint_names)
        positions = list(joint_trajectory.points[-1].positions)
        if len(names) != len(positions) or len(set(names)) != len(names):
            return False, "ENDPOINT_JOINT_ARRAY_INVALID"
        endpoint = dict(zip(names, positions))
        goal_state = record["joint_state"]
        if len(goal_state.name) != len(goal_state.position):
            return False, "IK_GOAL_JOINT_ARRAY_INVALID"
        tolerance = STAGE1_JOINT_GOAL_TOL + 1e-3
        for name, goal in zip(goal_state.name, goal_state.position):
            if name not in endpoint:
                return False, f"ENDPOINT_MISSING:{name}"
            actual = float(endpoint[name])
            goal = float(goal)
            if not math.isfinite(actual) or abs(actual - goal) > tolerance:
                return False, f"ENDPOINT_MISMATCH:{name}"
        return True, ""

    def _d405_plan_start_matches_measured(
        self,
        trajectory,
        seed_state,
        *,
        max_age_s=D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S,
        tolerance_rad=D405_PREFLIGHT_START_STATE_TOL_RAD,
    ):
        """Reject a plan if the robot moved after its captured start state."""

        measured_state = getattr(self, "current_joint_state", None)
        measured_time = float(getattr(self, "current_joint_state_time", 0.0))
        age = time.monotonic() - measured_time
        if (
            measured_state is None
            or measured_time <= 0.0
            or not math.isfinite(age)
            or age < 0.0
            or age > float(max_age_s)
        ):
            return False, "START_MEASURED_STATE_STALE"
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            return False, "START_TRAJECTORY_EMPTY"
        names = list(joint_trajectory.joint_names)
        first = list(joint_trajectory.points[0].positions)
        if (
            len(names) != len(first)
            or len(set(names)) != len(names)
            or not set(READY_POSE_JOINTS).issubset(set(names))
        ):
            return False, "START_TRAJECTORY_JOINT_ARRAY_INVALID"

        def state_map(state):
            if (
                state is None
                or len(state.name) != len(state.position)
                or len(set(state.name)) != len(state.name)
            ):
                return None
            values = {}
            for name, value in zip(state.name, state.position):
                value = float(value)
                if not math.isfinite(value):
                    return None
                values[name] = value
            return values

        measured = state_map(measured_state)
        seed = state_map(seed_state)
        if measured is None or seed is None:
            return False, "START_STATE_JOINT_ARRAY_INVALID"
        first_by_name = {}
        for name, value in zip(names, first):
            value = float(value)
            if not math.isfinite(value):
                return False, f"START_TRAJECTORY_NONFINITE:{name}"
            first_by_name[name] = value
        for name in READY_POSE_JOINTS:
            if name not in measured or name not in seed or name not in first_by_name:
                return False, f"START_STATE_MISSING:{name}"
            if (
                abs(measured[name] - seed[name])
                > float(tolerance_rad)
            ):
                return False, f"MEASURED_SEED_MISMATCH:{name}"
            if (
                abs(measured[name] - first_by_name[name])
                > float(tolerance_rad)
            ):
                return False, f"MEASURED_PLAN_START_MISMATCH:{name}"
        return True, ""

    def _d405_post_fjt_arrival_reason(self, trajectory):
        """Return an empty string only for fresh, stationary measured arrival."""

        state = getattr(self, "current_joint_state", None)
        now = time.monotonic()
        state_time = float(getattr(self, "current_joint_state_time", 0.0))
        age = now - state_time
        if (
            state is None
            or state_time <= 0.0
            or not math.isfinite(age)
            or age < 0.0
            or age > D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S
        ):
            return "JOINT_STATE_STALE"
        if len(state.name) != len(state.position) or len(set(state.name)) != len(
            state.name
        ):
            return "MEASURED_JOINT_ARRAY_INVALID"
        measured = dict(zip(state.name, state.position))
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            return "COMMAND_TRAJECTORY_EMPTY"
        names = list(joint_trajectory.joint_names)
        final_positions = list(joint_trajectory.points[-1].positions)
        if (
            len(names) != len(final_positions)
            or len(set(names)) != len(names)
            or any(name not in measured for name in names)
        ):
            return "COMMAND_ENDPOINT_INCOMPLETE"
        errors = []
        for name, command in zip(names, final_positions):
            command = float(command)
            actual = float(measured[name])
            if not math.isfinite(command) or not math.isfinite(actual):
                return f"NONFINITE_FINAL_JOINT:{name}"
            errors.append(abs(actual - command))
        if max(errors, default=float("inf")) > D405_PREFLIGHT_FINAL_JOINT_TOL_RAD:
            return "FINAL_JOINT_ERROR"
        if not self._robot_stationary_for_bias():
            return "ROBOT_NOT_STATIONARY"
        return ""

    def _start_d405_post_fjt_verification(self, context, trajectory):
        """Verify measured RB10 arrival before arming a D405 capture."""

        self._cancel_d405_arrival_timer()
        started = time.monotonic()
        timer_ref = {}
        trajectory = copy.deepcopy(trajectory)

        def verify_arrival():
            timer = timer_ref.get("timer")
            if timer is not getattr(self, "_d405_arrival_timer", None):
                return
            if not self._d405_prescan_callback_valid(context["token"]):
                return
            orientation_context = context is getattr(
                self, "_d405_orientation_context", None
            )
            cartesian = getattr(self, "_d405_cartesian_context", None)
            cartesian_context = bool(
                isinstance(cartesian, dict)
                and cartesian.get("root") is context
            )
            if not orientation_context and not cartesian_context:
                self._cancel_d405_arrival_timer()
                self._request_motion_abort(
                    "D405_PRESCAN_ARRIVAL_CONTEXT_LOST"
                )
                return
            if not self._d405_scene_revision_confirmed(
                context.get("scene_revision")
            ):
                self._cancel_d405_arrival_timer()
                self._request_motion_abort(
                    "D405_PRESCAN_SCENE_CHANGED_AFTER_DISPATCH"
                )
                return
            reason = self._d405_post_fjt_arrival_reason(trajectory)
            if not reason:
                self._cancel_d405_arrival_timer()
                if orientation_context:
                    self._d405_orientation_context = None
                self.get_logger().info(
                    "[D405 PRESCAN ARRIVAL] measured joints reached the "
                    "commanded endpoint and robot is stationary"
                )
                if cartesian_context:
                    self._finish_d405_probe_context(context, True)
                else:
                    context["finalize_cb"](True)
                return
            if time.monotonic() - started <= D405_PREFLIGHT_ARRIVAL_VERIFY_TIMEOUT_S:
                return
            self._cancel_d405_arrival_timer()
            self.get_logger().error(
                "[D405 PRESCAN ARRIVAL] FJT reported success but measured "
                f"arrival was not verified: {reason}"
            )
            self._request_motion_abort(
                "D405_PRESCAN_ARRIVAL_UNVERIFIED:" + reason
            )

        timer = self.create_timer(0.05, verify_arrival)
        timer_ref["timer"] = timer
        self._d405_arrival_timer = timer
        verify_arrival()

    def _d405_orientation_plan_result(self, future, context, rank_index):
        if (
            context is not getattr(self, "_d405_orientation_context", None)
            or not self._d405_prescan_callback_valid(context["token"])
            or int(rank_index) != int(context.get("rank_index", -1))
        ):
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._try_next_d405_orientation_rank(
                context, f"PLANNING_RESULT_FAILED:{exc}"
            )
            return
        if int(result.error_code.val) != 1:
            self._try_next_d405_orientation_rank(
                context, f"MOVEIT_ERROR_{int(result.error_code.val)}"
            )
            return
        if not self._d405_scene_revision_confirmed(context["scene_revision"]):
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] scene revision changed before plan dispatch"
            )
            context["finalize_cb"](False)
            return

        record = context["ranked"][int(rank_index)]
        trajectory = result.planned_trajectory
        endpoint_ok, endpoint_reason = self._d405_plan_endpoint_matches_ik(
            trajectory, record
        )
        if not endpoint_ok:
            self._try_next_d405_orientation_rank(context, endpoint_reason)
            return
        if not self._d405_prescan_trajectory_is_safe(
            trajectory, context["label"]
        ):
            self._try_next_d405_orientation_rank(context, "UNSAFE_TRAJECTORY")
            return
        if not self._d405_scene_revision_confirmed(context["scene_revision"]):
            context["finalize_cb"](False)
            return
        start_ok, start_reason = self._d405_plan_start_matches_measured(
            trajectory,
            context["seed_state"],
            max_age_s=1.0,
        )
        if not start_ok:
            self.get_logger().error(
                "[D405 PRESCAN SYMMETRY] plan dispatch rejected: "
                + start_reason
            )
            context["finalize_cb"](False)
            return

        if context.get("compare_views"):
            metrics = self._trajectory_joint_metrics(trajectory.joint_trajectory)
            positions = np.array([point.positions for point in trajectory.joint_trajectory.points],float)
            travel = np.abs(np.diff(positions,axis=0)).sum(axis=0)
            cost = (float(travel.sum()),float(travel.max()),float(metrics["joint_path"]),int(rank_index))
            context["planned_views"].append((cost,int(rank_index),copy.deepcopy(trajectory)))
            self._advance_d405_view_plans(context)
            return
        self._dispatch_d405_orientation_trajectory(context,record,trajectory)

    def _advance_d405_view_plans(self, context):
        next_index = int(context.get("rank_index",-1))+1
        if next_index < len(context["ranked"]) and len(context["planned_views"]) < 3:
            return self._activate_d405_orientation_rank(context,next_index)
        if not context["planned_views"]:
            context["finalize_cb"](False)
            return False
        cost,index,trajectory = min(context["planned_views"],key=lambda item:item[0])
        self.get_logger().info(f"[D405 VIEW] selected sample path: summed joint travel={cost[0]:.3f}rad, compared={len(context['planned_views'])}")
        return self._activate_d405_orientation_rank(context,index,trajectory=trajectory)

    def _dispatch_d405_orientation_trajectory(self, context, record, trajectory):
        # Recheck at dispatch: comparing plans must never reuse a moved seed or scene.
        inhibit = MoveItExecutor._motion_dispatch_inhibited_reason(self)
        if inhibit:
            self.get_logger().error(f"[D405 VIEW] dispatch/capture inhibited: {inhibit}")
            context["finalize_cb"](False)
            return
        if not self._d405_prescan_callback_valid(context["token"]) or not self._d405_scene_revision_confirmed(context["scene_revision"]):
            context["finalize_cb"](False)
            return
        start_ok,reason = self._d405_plan_start_matches_measured(trajectory,context["seed_state"])
        if not start_ok:
            self.get_logger().error(f"[D405 VIEW] dispatch rejected: {reason}")
            context["finalize_cb"](False)
            return
        current = self._current_tcp_pose_np()
        goal = record["candidate"]["poses"][0]
        target_p = np.array([goal.position.x,goal.position.y,goal.position.z])
        target_q = np.array([goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w])
        if current is not None and np.linalg.norm(current[0]-target_p) <= .002 and abs(np.dot(current[1],target_q)) >= math.cos(math.radians(1.)/2) and not self._d405_post_fjt_arrival_reason(trajectory):
            self.get_logger().info("[D405 VIEW] already stationary at valid measurement pose; no motion dispatched")
            self._start_d405_post_fjt_verification(context,trajectory)
            return
        trajectory = self._rescale_trajectory(trajectory, scale=1.0)
        context["fjt_dispatched"] = True

        def execution_succeeded():
            if not self._d405_prescan_callback_valid(context["token"]):
                return
            if (
                context is not getattr(self, "_d405_orientation_context", None)
                or not self._d405_scene_revision_confirmed(
                    context["scene_revision"]
                )
            ):
                self._request_motion_abort(
                    "D405_PRESCAN_CONTEXT_OR_SCENE_CHANGED_AFTER_FJT"
                )
                return
            self._start_d405_post_fjt_verification(context, trajectory)

        def execution_failed():
            if not self._d405_prescan_callback_valid(context["token"]):
                return
            self._request_motion_abort(
                "D405_PRESCAN_FJT_EXECUTION_FAILED:" + context["label"]
            )

        dispatched = self.execute_trajectory_direct(
            trajectory,
            on_complete=execution_succeeded,
            on_failure=execution_failed,
            on_rejected=lambda: (
                context.__setitem__("fjt_dispatched", False),
                context["finalize_cb"](False),
            ),
            label=(
                context["label"] + " "
                + self._d405_selected_orientation_branch
            ),
        )
        if not dispatched:
            if (
                getattr(self, "_active_trajectory_goal_token", None) is not None
                or getattr(self, "_fjt_motion_state_unknown", False)
            ):
                self._request_motion_abort(
                    "D405_PRESCAN_FJT_DISPATCH_UNKNOWN:" + context["label"]
                )
            else:
                context["fjt_dispatched"] = False
                context["finalize_cb"](False)

    def _plan_next_d405_prescan_pose(self):
        self._cancel_d405_prescan_timer()
        if self._d405_prescan_index >= len(self._d405_prescan_queue):
            self._finish_d405_prescan(success=False)
            return
        pose = self._d405_prescan_queue[self._d405_prescan_index]
        label = f"D405_PRESCAN_{self._d405_prescan_index + 1}"
        self.get_logger().info(
            f"[D405 PRESCAN] {label}: tcp=("
            f"{pose.position.x:.3f},{pose.position.y:.3f},"
            f"{pose.position.z:.3f})")
        if self._d405_prescan_index == 0:
            self._request_d405_orientation_candidate_iks(
                label,
                pose,
                finalize_cb=self._d405_prescan_pose_done,
            )
            return
        self._plan_d405_probe_cartesian(
            label,
            pose,
            finalize_cb=self._d405_prescan_pose_done,
            token=self._d405_prescan_token,
        )

    def _d405_prescan_pose_done(self, success):
        if not success:
            if self._d405_prescan_index == 0:
                self.get_logger().warn(
                    "[D405 PRESCAN] collision-checked anchor pose planning/"
                    "execution failed -> prescan stopped before probe motion"
                )
                self._finish_d405_prescan(success=False)
                return
            self.get_logger().warn(
                f"[D405 PRESCAN] pose#{self._d405_prescan_index + 1} "
                "planning/execution 실패 -> 다음 측정 pose 시도")
            self._d405_prescan_index += 1
            self._plan_next_d405_prescan_pose()
            return
        self._d405_prescan_wait_start = time.monotonic()
        self._d405_prescan_arrived_time = self._d405_prescan_wait_start
        self._d405_prescan_capture_sent = False
        self._d405_prescan_capture_time = 0.0
        self._d405_prescan_timer = self.create_timer(
            0.1, self._wait_d405_prescan_refined)
        self.get_logger().info(
            f"[D405 PRESCAN] 정지 후 refined plane 대기 "
            f"(settle={D405_PREFLIGHT_SCAN_SETTLE_SEC:.1f}s, "
            f"timeout={D405_PREFLIGHT_SCAN_TIMEOUT_SEC:.1f}s)")

    def _wait_d405_prescan_refined(self):
        now = time.monotonic()
        wait_start = float(self._d405_prescan_wait_start or now)
        if now - wait_start < D405_PREFLIGHT_SCAN_SETTLE_SEC:
            return
        if not self._d405_prescan_capture_sent and (
            not self._robot_stationary_for_bias()
            or not 0 <= now-float(self.current_joint_state_time) <= D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S
        ):
            if now-wait_start > D405_PREFLIGHT_SCAN_SETTLE_SEC+D405_PREFLIGHT_SCAN_TIMEOUT_SEC:
                self._finish_d405_prescan(success=False)
            return
        if not self._d405_prescan_capture_sent:
            self._d405_prescan_capture_sent = True
            self._d405_prescan_capture_time = now
            msg = Bool()
            msg.data = True
            if self._d405_prescan_mode == "multi_target":
                self._multi_capture_started = now
                sample = self._d405_capture_sample_in_base()
                if sample is None or not self._multi_request_local_capture(sample):
                    self._finish_d405_prescan(success=False)
                    return
            else:
                self.d405_capture_pub.publish(msg)
            self.get_logger().info(
                "[D405 PRESCAN] D405 pointcloud capture 요청")
            return
        capture_time = float(self._d405_prescan_capture_time or wait_start)
        if self._d405_prescan_mode == "multi_target":
            if self._multi_refined_result is not None:
                self._finish_d405_prescan(success=True)
                return
        if (
            self._d405_prescan_mode != "multi_target"
            and self.dynamic_surface_source == "d405_refined"
            and self.dynamic_surface_source_time >= capture_time
        ):
            self._finish_d405_prescan(success=True)
            return
        if now - capture_time <= D405_PREFLIGHT_SCAN_TIMEOUT_SEC:
            return

        self._cancel_d405_prescan_timer()
        self.get_logger().warn(
            f"[D405 PRESCAN] pose#{self._d405_prescan_index + 1} 에서 "
            "refined plane 미수신 -> 다음 측정 pose 시도")
        self._d405_prescan_index += 1
        self._plan_next_d405_prescan_pose()

    def _finish_d405_prescan(self, success):
        self._cancel_d405_prescan_timer()
        mode = self._d405_prescan_mode
        self._invalidate_d405_prescan_callbacks(clear_token=True)
        self._d405_prescan_active = False
        self._d405_prescan_queue = []
        self._d405_prescan_surface_points = []
        self._d405_prescan_surface_normal = None
        self._d405_prescan_index = 0
        self._d405_selected_orientation_branch = ""
        self._d405_scan_candidates = []
        if mode == "multi_target":
            self.executing = False
            self._multi_scan_done(success)
            return
        if success:
            if mode == "work_area":
                self.get_logger().info(
                    "[D405 PRESCAN] work area refined 완료 -> "
                    "wall_front 가 보정 평면 기준으로 갱신됩니다")
                self._publish_work_area_refine_status("done")
                self.executing = False
                return
            self.get_logger().info(
                "[D405 PRESCAN] refined plane 확보 -> 스케치 경로 보정 후 시작")
            self._start_sketch_motion_after_surface_ready()
            return

        if mode == "work_area":
            self.get_logger().warn(
                "[D405 PRESCAN] work area refined 실패 -> "
                "현재 wall_front 는 ZED 기준입니다")
            self._publish_work_area_refine_status("failed")
            self.executing = False
            return

        if D405_PREFLIGHT_REQUIRE_REFINED:
            self.get_logger().error(
                "[D405 PRESCAN] refined plane 확보 실패 -> 안전을 위해 실행 중단")
            self.executing = False
            return
        self.get_logger().warn(
            "[D405 PRESCAN] refined plane 확보 실패 -> ZED plane 기준으로 계속 진행")
        self._start_sketch_motion_after_surface_ready()

    def _cancel_d405_prescan_timer(self):
        if self._d405_prescan_timer is not None:
            self._d405_prescan_timer.cancel()
            self.destroy_timer(self._d405_prescan_timer)
            self._d405_prescan_timer = None
        self._d405_prescan_wait_start = None
        self._d405_prescan_capture_sent = False
        self._d405_prescan_capture_time = 0.0

    def _apply_refined_surface_to_current_waypoints(self):
        if not self.current_waypoints or not self._d405_refined_surface_fresh():
            return
        normal = np.asarray(self.dynamic_surface_normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        try:
            target = get_target(self.cfg, self.active_target_name)
        except Exception:
            target = get_target(self.cfg, "wall")
        old_normal = self._infer_surface_normal_from_waypoints(
            self.current_waypoints, target)
        old_normal = np.asarray(old_normal, dtype=float)
        old_normal /= np.linalg.norm(old_normal) + 1e-12
        expected = ROLLER_RADIUS + CONTACT_CLEARANCE
        q = self._tcp_quat_for_surface_normal(normal)

        updated = []
        for wp in self.current_waypoints:
            p = np.array([wp.position.x, wp.position.y, wp.position.z], dtype=float)
            old_surface = p - old_normal * expected
            new_surface = self._project_point_to_active_surface(old_surface, normal)
            new_p = new_surface + normal * expected
            out = copy.deepcopy(wp)
            out.position.x = float(new_p[0])
            out.position.y = float(new_p[1])
            out.position.z = float(new_p[2])
            out.orientation.x = float(q[0])
            out.orientation.y = float(q[1])
            out.orientation.z = float(q[2])
            out.orientation.w = float(q[3])
            updated.append(out)
        self.current_waypoints = updated
        self.get_logger().info(
            f"[D405 PRESCAN] current_waypoints {len(updated)}개를 "
            "D405 refined plane 으로 재투영")

    def _apply_refined_surface_to_active_segment_path(self):
        path = self._active_segment_path
        if path is None or not self._d405_refined_surface_fresh():
            return
        plane_point = np.asarray(self.dynamic_surface_point, dtype=float)
        normal = np.asarray(self.dynamic_surface_normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        updated = []
        previous_tangent = None
        for row in path.rows:
            point = np.asarray(row.position, dtype=float)
            projected = point - normal * float(np.dot(point - plane_point, normal))
            tangent = np.asarray(row.tangent, dtype=float)
            tangent = tangent - normal * float(np.dot(tangent, normal))
            if float(np.linalg.norm(tangent)) < 1e-8:
                tangent = previous_tangent
            if tangent is None or float(np.linalg.norm(tangent)) < 1e-8:
                tangent = np.cross(normal, np.array([0.0, 0.0, 1.0]))
            if float(np.linalg.norm(tangent)) < 1e-8:
                tangent = np.cross(normal, np.array([1.0, 0.0, 0.0]))
            tangent /= np.linalg.norm(tangent) + 1e-12
            previous_tangent = tangent.copy()
            updated.append(
                replace(
                    row,
                    position=tuple(float(v) for v in projected),
                    normal=tuple(float(v) for v in normal),
                    tangent=tuple(float(v) for v in tangent),
                )
            )
        self._active_segment_path = replace(path, rows=tuple(updated))
        self.get_logger().info(
            f"[D405 PRESCAN] segment path {len(updated)} rows reprojected "
            "onto refined surface"
        )

    def _segment_tip_pose(self, path: SegmentPath, row, previous_tcp_x=None):
        normal = np.asarray(row.normal, dtype=float)
        rotation = rotation_from_surface_path(
            normal, row.tangent, previous_tcp_x=previous_tcp_x
        )
        q = quat_from_matrix(rotation)
        position = np.asarray(segment_waypoint_position(path, row), dtype=float)
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose, rotation[:, 0].copy()

    def _validate_segment_path_geometry(self, path: SegmentPath):
        if path.contact_geometry_offset_m < ROLLER_RADIUS - 0.002:
            self.get_logger().error(
                "[PAINT PATH] contact geometry is shorter than the roller radius: "
                f"{path.contact_geometry_offset_m:.4f} < {ROLLER_RADIUS:.4f} m"
            )
            return False
        try:
            target = get_target(self.cfg, self.active_target_name)
        except Exception:
            target = get_target(self.cfg, "wall")
        plane_point, active_normal = self._active_surface_plane(target)
        plane_point = np.asarray(plane_point, dtype=float)
        active_normal = np.asarray(active_normal, dtype=float)
        active_normal /= np.linalg.norm(active_normal) + 1e-12
        errors = []
        _snapshot_point, _snapshot_normal, work_area_corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if work_area_corners is None:
            if self.real_painting_enabled:
                errors.append("selected work-area corners unavailable")
        else:
            outside = outside_quad_3d_indices(
                (row.position for row in path.rows),
                work_area_corners,
                boundary_tolerance_m=0.001,
                plane_tolerance_m=0.003,
            )
            if outside:
                errors.append(
                    "surface rows outside selected quadrilateral: "
                    f"count={len(outside)}, first_row={outside[0] + 1}"
                )
        for row in path.rows:
            row_normal = np.asarray(row.normal, dtype=float)
            alignment = float(np.dot(row_normal, active_normal))
            plane_error = abs(
                float(np.dot(np.asarray(row.position) - plane_point, active_normal))
            )
            if alignment < 0.90:
                errors.append(
                    f"row {row.row_number}: normal alignment={alignment:.3f}"
                )
            if plane_error > 0.030:
                errors.append(
                    f"row {row.row_number}: surface plane error={plane_error:.3f}m"
                )
            if (
                row.mode in CLEARANCE_MODES
                and row.offset_m < self.minimum_travel_clearance_m
            ):
                errors.append(
                    f"row {row.row_number}: {row.mode} clearance="
                    f"{row.offset_m:.4f}m"
                )
            if (
                row.mode == "FINAL_RETRACT"
                and row.offset_m + 1e-9 < self.final_retreat_offset_m
            ):
                errors.append(
                    f"row {row.row_number}: FINAL_RETRACT offset="
                    f"{row.offset_m:.4f}m < required "
                    f"{self.final_retreat_offset_m:.4f}m"
                )
        if errors:
            for error in errors[:8]:
                self.get_logger().error(f"[PAINT PATH] {error}")
            return False
        return True

    def _build_segment_orientation_candidate(
        self,
        path: SegmentPath,
        initial_tcp_x,
        *,
        name: str,
    ):
        """Build one complete pose branch for the roller's 180-degree symmetry.

        The cylindrical roller axis is an unoriented line: TCP +X and -X are
        task-equivalent while TCP +Y must keep the accepted surface normal.
        ``rotation_from_surface_path`` already preserves a chosen +X sign
        across rows; seeding it here freezes that sign for the entire Run.
        """

        row_tip_poses = {}
        row_tcp_poses = {}
        previous_tcp_x = np.asarray(initial_tcp_x, dtype=float).copy()
        motion_tip_poses = []
        motion_tcp_poses = []
        for row in path.rows:
            tip_pose, previous_tcp_x = self._segment_tip_pose(
                path, row, previous_tcp_x
            )
            tcp_pose = self._brush_tip_to_tcp(tip_pose)
            row_tip_poses[row.row_number] = tip_pose
            row_tcp_poses[row.row_number] = tcp_pose
            if row.mode in MOTION_MODES:
                motion_tip_poses.append(tip_pose)
                motion_tcp_poses.append(tcp_pose)

        if not motion_tcp_poses:
            raise SegmentPathError("no executable motion rows")
        first_motion = next(row for row in path.rows if row.mode in MOTION_MODES)
        last_motion = next(
            row for row in reversed(path.rows) if row.mode in MOTION_MODES
        )
        safety_row = replace(
            first_motion,
            offset_m=path.safety_approach_offset_m,
        )
        retreat_row = replace(
            last_motion,
            offset_m=path.final_retreat_offset_m,
        )
        first_tip = row_tip_poses[first_motion.row_number]
        last_tip = row_tip_poses[last_motion.row_number]
        safety_tip, _ = self._segment_tip_pose(
            path, safety_row, np.asarray(initial_tcp_x, dtype=float)
        )
        retreat_tip, _ = self._segment_tip_pose(
            path, retreat_row, previous_tcp_x
        )
        safety_tip.orientation = copy.deepcopy(first_tip.orientation)
        retreat_tip.orientation = copy.deepcopy(last_tip.orientation)

        return {
            "name": str(name),
            "initial_tcp_x": tuple(float(v) for v in initial_tcp_x),
            "row_tip_poses": row_tip_poses,
            "row_tcp_poses": row_tcp_poses,
            "motion_tip_poses": motion_tip_poses,
            "motion_tcp_poses": motion_tcp_poses,
            "safety_tcp_pose": self._brush_tip_to_tcp(safety_tip),
            "retreat_tcp_pose": self._brush_tip_to_tcp(retreat_tip),
        }

    def _apply_segment_orientation_candidate(self, candidate):
        """Install one already-validated orientation branch for execution."""

        self._selected_segment_orientation_branch = str(candidate["name"])
        self._safety_tcp_pose = copy.deepcopy(candidate["safety_tcp_pose"])
        self._retreat_tcp_pose = copy.deepcopy(candidate["retreat_tcp_pose"])
        self._stage3_tip_wps = copy.deepcopy(candidate["motion_tip_poses"])
        self._stage3_tcp_wps = copy.deepcopy(candidate["motion_tcp_poses"])
        self._process_row_tcp_poses = copy.deepcopy(
            candidate["row_tcp_poses"]
        )
        self._process_last_tcp_pose = copy.deepcopy(self._safety_tcp_pose)

    @staticmethod
    def _flip_pose_about_tcp_y(pose):
        """Return the physical 180-degree counterpart about TCP local +Y."""

        result = copy.deepcopy(pose)
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
        rotation = quat_to_matrix(quaternion)
        flipped = rotation @ np.diag([-1.0, 1.0, -1.0])
        flipped_quaternion = quat_from_matrix(flipped)
        result.orientation.x = float(flipped_quaternion[0])
        result.orientation.y = float(flipped_quaternion[1])
        result.orientation.z = float(flipped_quaternion[2])
        result.orientation.w = float(flipped_quaternion[3])
        return result

    def _flip_segment_orientation_candidate(self, candidate, *, name):
        """Derive the exact opposite roller branch for every path pose.

        Deriving B from A, instead of recomputing continuity independently,
        preserves the 180-degree relationship even at a 90-degree sketch
        corner where a dot-product sign test is mathematically ambiguous.
        """

        flip = MoveItExecutor._flip_pose_about_tcp_y
        return {
            "name": str(name),
            "initial_tcp_x": tuple(
                -float(value) for value in candidate["initial_tcp_x"]
            ),
            "row_tip_poses": {
                key: flip(pose)
                for key, pose in candidate["row_tip_poses"].items()
            },
            "row_tcp_poses": {
                key: flip(pose)
                for key, pose in candidate["row_tcp_poses"].items()
            },
            "motion_tip_poses": [
                flip(pose) for pose in candidate["motion_tip_poses"]
            ],
            "motion_tcp_poses": [
                flip(pose) for pose in candidate["motion_tcp_poses"]
            ],
            "safety_tcp_pose": flip(candidate["safety_tcp_pose"]),
            "retreat_tcp_pose": flip(candidate["retreat_tcp_pose"]),
        }

    def _current_roller_axis_seed(self, first_row):
        """Return current TCP +X projected onto the accepted surface plane."""

        current = self._current_tcp_pose_np()
        if current is None:
            return None
        quaternion = np.asarray(current[1], dtype=float)
        normal = np.asarray(first_row.normal, dtype=float)
        if quaternion.shape != (4,) or normal.shape != (3,):
            return None
        if not np.all(np.isfinite(quaternion)) or not np.all(np.isfinite(normal)):
            return None
        normal_norm = float(np.linalg.norm(normal))
        quaternion_norm = float(np.linalg.norm(quaternion))
        if normal_norm < 1e-9 or quaternion_norm < 1e-9:
            return None
        normal /= normal_norm
        quaternion /= quaternion_norm
        tcp_x = np.asarray(quat_apply(quaternion, [1.0, 0.0, 0.0]), dtype=float)
        tcp_x -= normal * float(np.dot(tcp_x, normal))
        tcp_x_norm = float(np.linalg.norm(tcp_x))
        if tcp_x_norm < 1e-6:
            return None
        return tcp_x / tcp_x_norm

    def _prepare_segment_process(self):
        path = self._active_segment_path
        if path is None or not self._validate_segment_path_geometry(path):
            return None
        try:
            steps = list(build_execution_steps(path.rows))
            first_motion = next(
                row for row in path.rows if row.mode in MOTION_MODES
            )
            # The two physical solutions are the first row's valid roller axis
            # and its negative.  Current TCP +X only orders those exact
            # solutions; using current +/-X directly can collapse both branches
            # when it is nearly perpendicular to the row's valid roller axis.
            raw_rotation = rotation_from_surface_path(
                first_motion.normal,
                first_motion.tangent,
            )
            raw_tcp_x = np.asarray(raw_rotation[:, 0], dtype=float)
            current_tcp_x = self._current_roller_axis_seed(first_motion)
            preferred_tcp_x = raw_tcp_x
            if (
                current_tcp_x is not None
                and float(np.dot(raw_tcp_x, current_tcp_x)) < 0.0
            ):
                preferred_tcp_x = -raw_tcp_x
            preferred_candidate = self._build_segment_orientation_candidate(
                path,
                preferred_tcp_x,
                name="roller_x_near_current",
            )
            candidates = (
                preferred_candidate,
                self._flip_segment_orientation_candidate(
                    preferred_candidate,
                    name="roller_x_flipped_180",
                ),
            )
        except SegmentPathError as exc:
            self.get_logger().error(f"[PAINT PATH] pose generation failed: {exc}")
            return None
        if not steps:
            self.get_logger().error("[PAINT PATH] no executable motion rows")
            return None

        self._stage1_orientation_candidates = candidates
        self._stage1_orientation_ranked = []
        self._stage1_orientation_rank_index = -1
        # Provisional geometry is needed for status/logging only.  Do not mark a
        # branch selected until *both* endpoint states have completed
        # collision-aware IK validation.
        self._safety_tcp_pose = copy.deepcopy(candidates[0]["safety_tcp_pose"])
        self._retreat_tcp_pose = None
        self._stage3_tip_wps = None
        self._stage3_tcp_wps = None
        self._process_row_tcp_poses = {}
        self._process_last_tcp_pose = None
        self._selected_segment_orientation_branch = ""
        self._stage1_orientation_branch_frozen = False
        self._process_steps = steps
        self._process_step_index = 0
        self._process_force_ready = False
        self._contact_search_confirmed = False
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        self._contact_search_distance_m = 0.0

        target = get_target(self.cfg, self.active_target_name)
        _plane_point, normal = self._active_surface_plane(target)
        self.get_logger().info(
            f"[PAINT PATH] prepared {len(steps)} execution steps from "
            f"{len(path.rows)} rows; force_enabled={self.painting_force_enabled}"
        )
        return target, normal

    def _publish_painting_command(self, mode, force_n, enable=None):
        mode = str(mode).strip().upper() or "IDLE"
        force_n = max(0.0, min(float(force_n), self.max_paint_force_n))
        if mode in ZERO_FORCE_MODES or mode == "RAMP_DOWN":
            force_n = 0.0
        if enable is None:
            enable = self.painting_force_enabled and mode in {
                "RAMP_UP",
                "PAINT",
                "CONTACT",
                "RAMP_DOWN",
            }

        published_at = time.monotonic()
        self._painting_command_generation += 1
        command_context = {
            "generation": int(self._painting_command_generation),
            "mode": mode,
            "force_n": float(force_n),
            "enable": bool(enable),
            "published_at_s": float(published_at),
            "guard_status_sequence": int(self._guard_status_sequence),
            "guard_source_timestamp_s": float(
                self._guard_status_source_time
            ),
        }
        self._painting_command_context = command_context

        force_msg = Float64()
        force_msg.data = force_n
        enable_msg = Bool()
        enable_msg.data = bool(enable)
        mode_msg = String()
        mode_msg.data = mode
        # Publish setpoint and enable before the mode edge. The wrench node also
        # restarts RAMP_UP if a cross-topic delivery reorders these messages.
        self.painting_force_pub.publish(force_msg)
        self.painting_enable_pub.publish(enable_msg)
        self.painting_mode_pub.publish(mode_msg)
        self._painting_command_mode = mode
        self._painting_command_force_n = force_n
        self._painting_command_enable = bool(enable)
        return command_context

    def _reset_painting_process(self):
        """Return the admittance interface and process state to a safe idle."""
        if hasattr(self, "_spray_session"):
            self._spray_off()
        self._cancel_stage1_scene_wait_timer()
        self._invalidate_stage1_orientation_candidates(clear_candidates=True)
        self._stage1_attempt_token = None
        self._stage1_goal_constraints = None
        self._stage1_retried = False
        self._stage1_on_complete = None
        MoveItExecutor._cancel_force_phase_lease(self)
        MoveItExecutor._cancel_segment_cartesian_timeout(self)
        self._cancel_process_timer()
        self._paint_entry_context = None
        self._contact_escape_context = None
        self._publish_painting_command("IDLE", 0.0, enable=False)
        self._active_segment_path = None
        MoveItExecutor._clear_execution_snapshot(self)
        self._process_steps = []
        self._process_step_index = 0
        self._process_row_tcp_poses = {}
        self._process_last_tcp_pose = None
        self._process_force_ready = False
        self._stationary_paint_hold_started_at = 0.0
        self._stationary_paint_hold_samples = []
        self._segment_cartesian_context = None
        self._contact_search_context = None
        self._contact_search_step_active = False
        self._contact_search_confirmed = False
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        self._contact_search_mode_published_at = 0.0
        self._contact_search_command_context = None
        self._free_space_confirmed = False
        self._execution_free_space_confirmed = False
        self._execution_tare_ready = False
        self._cancel_runtime_tare()
        self._publish_free_space_confirmation(False)
        self._execution_abort_reason = ""
        self._set_contact_collision_allowed(False)

    def _cancel_force_phase_lease(self, context=None):
        current = getattr(self, "_force_phase_lease_context", None)
        if context is not None and current is not context:
            return
        timer = getattr(self, "_force_phase_watchdog_timer", None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self._force_phase_watchdog_timer = None
        self._force_phase_lease_context = None

    def _force_phase_lease_tick(self, context):
        if context is not getattr(self, "_force_phase_lease_context", None):
            return
        if getattr(self, "_motion_abort_requested", False):
            MoveItExecutor._cancel_force_phase_lease(self, context)
            return
        if (
            context.get("execution_snapshot")
            is not getattr(self, "_execution_snapshot", None)
        ):
            self._request_motion_abort("FORCE_LEASE_EXECUTION_CONTEXT_CHANGED")
            return
        now = time.monotonic()
        blockers = MoveItExecutor._force_guard_status_blockers(
            self,
            expected_mode=context["mode"],
            active=context["active"],
            command_context=context["command_context"],
            now=now,
        )
        if not blockers:
            context["acknowledged"] = True
            if context["active"] is False:
                context["zero_ack_observed"] = True
            return
        elapsed_s = now - float(context["command_context"]["published_at_s"])
        if (
            context.get("acknowledged", False)
            or MoveItExecutor._guard_ack_fault_is_immediate(blockers)
            or elapsed_s > self.force_guard_mode_ack_timeout_s
        ):
            self._request_motion_abort(
                "FORCE_LEASE_GUARD_LOST:%s:%s"
                % (context["mode"], ",".join(blockers))
            )

    def _set_force_phase_lease_command(
        self, command_context, *, mode, active, step
    ):
        """Start/update the force lease without changing its watchdog owner."""

        if not self.painting_force_enabled:
            return None
        normalized_mode = str(mode).strip().upper()
        context = getattr(self, "_force_phase_lease_context", None)
        if context is None:
            if normalized_mode != "RAMP_UP" or active is not True:
                self._request_motion_abort(
                    f"FORCE_LEASE_MISSING_AT_{normalized_mode or 'UNKNOWN'}"
                )
                return None
            context = {
                "token": object(),
                "execution_snapshot": getattr(self, "_execution_snapshot", None),
                "created_at_s": time.monotonic(),
            }
            self._force_phase_lease_context = context
        elif (
            context.get("execution_snapshot")
            is not getattr(self, "_execution_snapshot", None)
        ):
            self._request_motion_abort("FORCE_LEASE_EXECUTION_CONTEXT_CHANGED")
            return None

        context.update(
            mode=normalized_mode,
            active=bool(active),
            command_context=command_context,
            step=step,
            step_index=int(self._process_step_index),
            acknowledged=False,
            zero_ack_observed=False,
        )
        if getattr(self, "_force_phase_watchdog_timer", None) is None:
            self._force_phase_watchdog_timer = self.create_timer(
                0.02,
                lambda ctx=context: MoveItExecutor._force_phase_lease_tick(
                    self, ctx
                ),
            )
        return context

    def _finish_force_phase_lease_after_zero_ack(self, command_context):
        context = getattr(self, "_force_phase_lease_context", None)
        if (
            context is None
            or context.get("mode") != "RAMP_DOWN"
            or context.get("active") is not False
            or context.get("command_context") is not command_context
        ):
            return False
        blockers = MoveItExecutor._force_guard_status_blockers(
            self,
            expected_mode="RAMP_DOWN",
            active=False,
            command_context=command_context,
        )
        if blockers:
            return False
        context["zero_ack_observed"] = True
        MoveItExecutor._cancel_force_phase_lease(self, context)
        return True

    def _cancel_segment_cartesian_timeout(self, context=None):
        timer = getattr(self, "_segment_cartesian_timeout_timer", None)
        if context is not None:
            owned = context.get("timeout_timer")
            if owned is not None and timer is not owned:
                return
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self._segment_cartesian_timeout_timer = None
        if isinstance(context, dict):
            context["timeout_timer"] = None

    def _arm_segment_cartesian_timeout(self, context):
        MoveItExecutor._cancel_segment_cartesian_timeout(self)
        timeout_s = float(
            self.paint_cartesian_planning_timeout_s
            if context["step"].mode == "PAINT"
            else max(self.paint_cartesian_planning_timeout_s, 5.0)
        )
        timer_ref = {}

        def expired():
            timer = timer_ref.pop("timer", None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            if self._segment_cartesian_timeout_timer is timer:
                self._segment_cartesian_timeout_timer = None
            if context.get("timeout_timer") is timer:
                context["timeout_timer"] = None
            if (
                self._segment_cartesian_context is not context
                or self._motion_abort_requested
            ):
                return
            self._segment_cartesian_context = None
            reason = f"{context['step'].mode}_CARTESIAN_TIMEOUT"
            if (
                context["step"].mode == "PAINT"
                or context.get("contact_escape_context") is not None
            ):
                self._request_motion_abort(reason)
            else:
                self._fail_workflow_known_safe(reason)

        timer = self.create_timer(timeout_s, expired)
        timer_ref["timer"] = timer
        context["timeout_timer"] = timer
        context["planning_deadline_s"] = time.monotonic() + timeout_s
        self._segment_cartesian_timeout_timer = timer

    def _contact_escape_is_current(
        self, context, *, step=None, require_zero_ack=False
    ):
        if not (
            context is not None
            and context is getattr(self, "_contact_escape_context", None)
            and not getattr(self, "_motion_abort_requested", False)
            and context.get("execution_snapshot")
            is getattr(self, "_execution_snapshot", None)
            and context.get("target") == getattr(self, "active_target_name", "")
            and getattr(self, "_contact_collision_allowed", False)
            and getattr(self, "_contact_collision_baseline", None) is not None
        ):
            return False
        if require_zero_ack and context.get("zero_ack_verified") is not True:
            return False
        if require_zero_ack and context.get("phase") not in {
            "ZERO_ACK_VERIFIED",
            "ESCAPE_PLANNING",
            "ESCAPE_DISPATCH",
        }:
            return False
        if step is not None:
            if (
                int(context.get("escape_step_index", -1))
                != int(getattr(self, "_process_step_index", -2))
                or getattr(self, "_process_step_index", -1)
                >= len(getattr(self, "_process_steps", ()))
                or self._process_steps[self._process_step_index] is not step
            ):
                return False
        return True

    def _contact_escape_step_error(self, context, step, poses):
        if not MoveItExecutor._contact_escape_is_current(
            self, context, step=step, require_zero_ack=True
        ):
            return "CONTACT_ESCAPE_CONTEXT_INVALID"
        if step.mode not in {"RETRACT", "FINAL_RETRACT"}:
            return "CONTACT_ESCAPE_MODE_INVALID"
        if len(step.rows) != 1 or len(poses) != 1:
            return "CONTACT_ESCAPE_MUST_BE_ONE_BOUNDED_POSE"
        if self._painting_command_enable or self._process_force_ready:
            return "CONTACT_ESCAPE_FORCE_NOT_OFF"

        row = step.rows[0]
        reference_point = np.asarray(context["surface_point"], dtype=float)
        point = np.asarray(row.position, dtype=float)
        normal = np.asarray(context["normal"], dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        row_normal = np.asarray(row.normal, dtype=float)
        row_normal /= np.linalg.norm(row_normal) + 1e-12
        tangent = np.asarray(context["tangent"], dtype=float)
        tangent /= np.linalg.norm(tangent) + 1e-12
        row_tangent = np.asarray(row.tangent, dtype=float)
        row_tangent /= np.linalg.norm(row_tangent) + 1e-12
        if not all(
            np.all(np.isfinite(values))
            for values in (reference_point, point, normal, row_normal, tangent, row_tangent)
        ):
            return "CONTACT_ESCAPE_GEOMETRY_NONFINITE"
        if float(np.linalg.norm(point - reference_point)) > 0.001:
            return "CONTACT_ESCAPE_SURFACE_POINT_CHANGED"
        if float(np.dot(normal, row_normal)) < 0.999:
            return "CONTACT_ESCAPE_NORMAL_CHANGED"
        if float(np.dot(tangent, row_tangent)) < 0.999:
            return "CONTACT_ESCAPE_TANGENT_CHANGED"

        path = getattr(self, "_active_segment_path", None)
        if path is None:
            return "CONTACT_ESCAPE_PATH_MISSING"
        expected_clearance = float(
            path.final_retreat_offset_m
            if step.mode == "FINAL_RETRACT"
            else path.travel_clearance_m
        )
        if abs(float(row.offset_m) - expected_clearance) > 1e-6:
            return "CONTACT_ESCAPE_CLEARANCE_CHANGED"
        start = np.array(
            [
                context["paint_final_pose"].position.x,
                context["paint_final_pose"].position.y,
                context["paint_final_pose"].position.z,
            ],
            dtype=float,
        )
        target = np.array(
            [poses[0].position.x, poses[0].position.y, poses[0].position.z],
            dtype=float,
        )
        delta = target - start
        outward = float(np.dot(delta, normal))
        tangent_delta = float(np.linalg.norm(delta - normal * outward))
        if (
            not math.isfinite(outward)
            or not math.isfinite(tangent_delta)
            or outward < self.minimum_travel_clearance_m - 0.002
            or abs(outward - expected_clearance) > 0.003
            or tangent_delta > 0.002
        ):
            return "CONTACT_ESCAPE_NOT_BOUNDED_NORMAL_OUTWARD"
        return ""

    def _contact_escape_restore_done(self, context, step, success):
        if (
            context is not getattr(self, "_contact_escape_context", None)
            or self._motion_abort_requested
            or int(context.get("escape_step_index", -1))
            != int(self._process_step_index)
            or self._process_steps[self._process_step_index] is not step
        ):
            return
        if not success:
            self._request_motion_abort("CONTACT_ESCAPE_ACM_RESTORE_FAILED")
            return
        if (
            self._contact_collision_allowed
            or self._acm_update_pending is not None
        ):
            self._request_motion_abort("CONTACT_ESCAPE_ACM_RESTORE_UNVERIFIED")
            return
        self._contact_escape_context = None
        self._complete_process_step()

    def _cancel_process_timer(self):
        if self._process_timer is not None:
            self._process_timer.cancel()
            self.destroy_timer(self._process_timer)
            self._process_timer = None

    def _schedule_process_once(self, delay_s, callback):
        self._cancel_process_timer()
        timer_ref = {}

        def run_once():
            timer = timer_ref.pop("timer", None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            if self._process_timer is timer:
                self._process_timer = None
            callback()

        timer = self.create_timer(max(0.001, float(delay_s)), run_once)
        timer_ref["timer"] = timer
        self._process_timer = timer

    @staticmethod
    def _load_srdf_allowed_collision_pairs(model_id=DEFAULT_MODEL):
        """Load the semantic collision baseline used by the active MoveIt config."""

        candidates = []
        if get_package_share_directory is not None:
            try:
                candidates.append(
                    Path(get_package_share_directory("sketch_control"))
                    / "config"
                    / "rbpodo_named.srdf"
                )
            except Exception:
                pass
        candidates.append(
            Path(__file__).resolve().parents[1]
            / "config"
            / "rbpodo_named.srdf"
        )
        for path in candidates:
            try:
                root = ET.fromstring(model_srdf(model_id, path))
            except (OSError, ET.ParseError):
                continue
            pairs = []
            for element in root.iter("disable_collisions"):
                first = str(element.attrib.get("link1", "")).strip()
                second = str(element.attrib.get("link2", "")).strip()
                if first and second and first != second:
                    pairs.append((first, second))
            if pairs:
                return tuple(pairs)
        return ()

    @staticmethod
    def _acm_pair_value(acm, first, second):
        names = list(acm.entry_names)
        if first not in names or second not in names:
            return False
        i, j = names.index(first), names.index(second)
        if i >= len(acm.entry_values):
            return False
        row = acm.entry_values[i].enabled
        return bool(row[j]) if j < len(row) else False

    @staticmethod
    def _acm_effective_pair_value(acm, first, second):
        names = list(acm.entry_names)
        if first in names and second in names:
            return MoveItExecutor._acm_pair_value(acm, first, second)
        defaults = dict(
            zip(acm.default_entry_names, acm.default_entry_values)
        )
        first_default = defaults.get(first)
        second_default = defaults.get(second)
        if first_default is not None and second_default is not None:
            return bool(first_default) and bool(second_default)
        if first_default is not None:
            return bool(first_default)
        if second_default is not None:
            return bool(second_default)
        return False

    @staticmethod
    def _acm_validation_error(acm, required_allowed_pairs=()):
        if acm is None:
            return "matrix missing"
        names = [str(name) for name in acm.entry_names]
        if not names:
            return "entry_names empty"
        if any(not name for name in names):
            return "entry_names contains an empty name"
        if len(set(names)) != len(names):
            return "entry_names contains duplicates"
        rows = list(acm.entry_values)
        if len(rows) != len(names):
            return f"row count {len(rows)} != name count {len(names)}"
        for index, row in enumerate(rows):
            if len(row.enabled) != len(names):
                return (
                    f"row {index} width {len(row.enabled)} != "
                    f"name count {len(names)}"
                )
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                if bool(rows[i].enabled[j]) != bool(rows[j].enabled[i]):
                    return f"matrix is asymmetric at ({i},{j})"
        default_names = [str(name) for name in acm.default_entry_names]
        if len(default_names) != len(acm.default_entry_values):
            return "default entry names/values size mismatch"
        if len(set(default_names)) != len(default_names):
            return "default_entry_names contains duplicates"
        for first, second in required_allowed_pairs:
            if not MoveItExecutor._acm_pair_value(acm, first, second):
                return f"required SRDF pair missing or disabled: {first}<->{second}"
        return ""

    @staticmethod
    def _acm_clean_baseline_error(acm, required_allowed_pairs=()):
        """Require the pre-motion ACM to match the SRDF allow policy.

        The contact transaction intentionally adds one extra pair later, so
        this stricter predicate is used only for the fetched clean baseline
        and its frozen restore.  Unexpected true pairs or a true default
        could hide a robot/world or self collision before CONTACT_SEARCH.
        """

        error = MoveItExecutor._acm_validation_error(
            acm, required_allowed_pairs
        )
        if error:
            return error
        if any(bool(value) for value in acm.default_entry_values):
            return "clean baseline contains an enabled default entry"
        required = {
            frozenset((str(first), str(second)))
            for first, second in required_allowed_pairs
        }
        names = list(acm.entry_names)
        unexpected = []
        for i, first in enumerate(names):
            for j in range(i + 1, len(names)):
                if not bool(acm.entry_values[i].enabled[j]):
                    continue
                second = names[j]
                if frozenset((first, second)) not in required:
                    unexpected.append(f"{first}<->{second}")
        if unexpected:
            return (
                "clean baseline contains non-SRDF allowed pair(s): "
                + ",".join(sorted(unexpected))
            )
        return ""

    @staticmethod
    def _acm_equal(first, second):
        if first is None or second is None:
            return first is second
        first_error = MoveItExecutor._acm_validation_error(first)
        second_error = MoveItExecutor._acm_validation_error(second)
        if first_error or second_error:
            return False
        first_names = list(first.entry_names)
        second_names = list(second.entry_names)
        if set(first_names) != set(second_names):
            return False
        first_defaults = dict(
            zip(first.default_entry_names, first.default_entry_values)
        )
        second_defaults = dict(
            zip(second.default_entry_names, second.default_entry_values)
        )
        if first_defaults != second_defaults:
            return False
        first_index = {name: index for index, name in enumerate(first_names)}
        second_index = {
            name: index for index, name in enumerate(second_names)
        }
        for first_name in first_names:
            for second_name in first_names:
                first_value = bool(
                    first.entry_values[first_index[first_name]].enabled[
                        first_index[second_name]
                    ]
                )
                second_value = bool(
                    second.entry_values[second_index[first_name]].enabled[
                        second_index[second_name]
                    ]
                )
                if first_value != second_value:
                    return False
        return True

    @staticmethod
    def _acm_with_pair(acm, first, second, allowed):
        validation_error = MoveItExecutor._acm_validation_error(acm)
        if validation_error:
            raise ValueError(f"invalid full ACM: {validation_error}")
        result = copy.deepcopy(acm)
        names = list(result.entry_names)
        rows = list(result.entry_values)

        for name in (first, second):
            if name in names:
                continue
            for row in rows:
                row.enabled.append(False)
            names.append(name)
            new_row = AllowedCollisionEntry()
            new_row.enabled = [False] * len(names)
            rows.append(new_row)

        i, j = names.index(first), names.index(second)
        rows[i].enabled[j] = bool(allowed)
        rows[j].enabled[i] = bool(allowed)
        result.entry_names = names
        result.entry_values = rows
        return result

    def _notify_acm_callbacks(self, callbacks, success):
        for callback in list(callbacks):
            if callback is None:
                continue
            try:
                callback(bool(success))
            except Exception as exc:
                self.get_logger().error(f"[ACM] completion callback failed: {exc}")

    def _cancel_acm_health_query_timer(self, context=None):
        if context is None:
            context = self._acm_health_query_pending
        timer = context.pop("timer", None) if isinstance(context, dict) else None
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)
        if self._acm_health_query_timer is timer:
            self._acm_health_query_timer = None

    def _invalidate_acm_health_query(self):
        context = getattr(self, "_acm_health_query_pending", None)
        if context is None:
            return
        self._cancel_acm_health_query_timer(context)
        self._acm_health_query_pending = None
        self._acm_health_query_seq += 1

    def _request_acm_baseline_health_check(self):
        """Verify the live, clean MoveIt ACM before any real motion.

        A read-only GetPlanningScene timeout has no possible scene side
        effect, so it remains a retryable NOT_READY state.  A structurally
        invalid full response, a missing SRDF disabled-collision pair, or an
        already-allowed roller/target pair proves that the live scene is
        contaminated and therefore latches the relaunch-only blocker.
        """

        if bool(getattr(self, "dry_run", True)):
            return
        target = str(getattr(self, "active_target_name", ""))
        verified_at = float(
            getattr(self, "_acm_baseline_verified_time", 0.0)
        )
        verified_age = time.monotonic() - verified_at
        if (
            getattr(self, "_acm_baseline_verified", False)
            and getattr(self, "_acm_baseline_verified_target_name", "")
            == target
            and verified_at > 0.0
            and 0.0 <= verified_age < ACM_HEALTH_REFRESH_S
        ):
            return
        if (
            getattr(self, "_acm_state_unknown", False)
            or getattr(self, "_acm_health_query_pending", None) is not None
            or getattr(self, "_acm_update_pending", None) is not None
            or getattr(self, "_contact_collision_allowed", False)
        ):
            return
        if not getattr(self, "_required_acm_allowed_pairs", ()):
            self._acm_state_unknown = True
            self._contact_collision_allowed = True
            self.get_logger().error(
                "[ACM] SRDF baseline contract unavailable; physical motion "
                "requires a corrected build and full relaunch"
            )
            return
        client = self.get_planning_scene_client
        if (
            not client.service_is_ready()
            and not client.wait_for_service(timeout_sec=0.0)
        ):
            return

        self._acm_health_query_seq += 1
        context = {
            "seq": self._acm_health_query_seq,
            "target": target,
        }
        self._acm_health_query_pending = context
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        try:
            future = client.call_async(request)
        except Exception as exc:
            self._acm_health_query_pending = None
            self.get_logger().warn(
                f"[ACM] startup baseline query unavailable; will retry: {exc}"
            )
            return

        timer_ref = {}

        def expired():
            timer = timer_ref.get("timer")
            if (
                timer is None
                or context.get("timer") is not timer
                or self._acm_health_query_timer is not timer
            ):
                timer_ref.pop("timer", None)
                return
            timer_ref.pop("timer", None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            if context.get("timer") is timer:
                context.pop("timer", None)
            if self._acm_health_query_timer is timer:
                self._acm_health_query_timer = None
            if self._acm_health_query_pending is not context:
                return
            self._acm_health_query_pending = None
            self.get_logger().warn(
                "[ACM] startup baseline query timed out; physical motion "
                "remains blocked and the query will be retried"
            )

        timer = self.create_timer(ACM_TRANSACTION_TIMEOUT_S, expired)
        timer_ref["timer"] = timer
        context["timer"] = timer
        self._acm_health_query_timer = timer
        future.add_done_callback(
            lambda done, ctx=context: self._acm_health_query_done(done, ctx)
        )

    def _acm_health_query_done(self, future, context):
        if self._acm_health_query_pending is not context:
            return
        self._cancel_acm_health_query_timer(context)
        self._acm_health_query_pending = None
        if (
            getattr(self, "_acm_update_pending", None) is not None
            or getattr(self, "_contact_collision_allowed", False)
            or getattr(self, "_acm_state_unknown", False)
            or str(getattr(self, "active_target_name", ""))
            != context.get("target", "")
        ):
            return
        try:
            baseline = future.result().scene.allowed_collision_matrix
        except Exception as exc:
            self.get_logger().warn(
                f"[ACM] startup baseline response unavailable; will retry: {exc}"
            )
            return

        error = MoveItExecutor._acm_validation_error(
            baseline, self._required_acm_allowed_pairs
        )
        if (
            not error
            and MoveItExecutor._acm_effective_pair_value(
                baseline,
                ROLLER_CONTACT_LINK,
                context["target"],
            )
        ):
            error = (
                "roller/target pair is already allowed before execution"
            )
        elif not error:
            error = MoveItExecutor._acm_clean_baseline_error(
                baseline, self._required_acm_allowed_pairs
            )
        if error:
            self._acm_baseline_verified = False
            self._acm_baseline_verified_target_name = ""
            self._acm_baseline_verified_time = 0.0
            self._acm_state_unknown = True
            # The live matrix may already permit contact. Keep the local state
            # conservative so reset and every unprivileged motion remain
            # blocked until MoveIt and this executor are relaunched together.
            self._contact_collision_allowed = True
            self.get_logger().error(
                f"[ACM] startup baseline unsafe ({error}); full stack "
                "relaunch required"
            )
            if (
                getattr(self, "executing", False)
                and hasattr(self, "_request_motion_abort")
            ):
                self._request_motion_abort(
                    "ACM_BASELINE_UNSAFE_DURING_MOTION"
                )
            return

        self._latest_allowed_collision_matrix = copy.deepcopy(baseline)
        self._acm_baseline_verified = True
        self._acm_baseline_verified_target_name = context["target"]
        self._acm_baseline_verified_time = time.monotonic()
        self.get_logger().info(
            "[ACM] clean SRDF collision baseline verified before motion"
        )

    def _cancel_acm_phase_timeout(self, context=None):
        if context is None:
            context = self._acm_update_pending
        timer = context.pop("timer", None) if isinstance(context, dict) else None
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)
        if self._acm_update_timer is timer:
            self._acm_update_timer = None

    def _arm_acm_phase_timeout(self, context, phase):
        self._cancel_acm_phase_timeout(context)
        timer_ref = {}

        def expired():
            timer = timer_ref.pop("timer", None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            if context.get("timer") is timer:
                context.pop("timer", None)
            if self._acm_update_timer is timer:
                self._acm_update_timer = None
            if (
                self._acm_update_pending is not context
                or context.get("phase") != phase
            ):
                return
            if phase == "FETCH_BASELINE":
                self._contact_collision_allowed = False
                self._clear_acm_transaction(clear_baseline=True)
                self.get_logger().error(
                    "[ACM] baseline fetch timed out; contact motion rejected"
                )
                self._notify_acm_callbacks(
                    context.get("allow_callbacks", ()), False
                )
                self._notify_acm_callbacks(
                    context.get("restore_callbacks", ()), True
                )
                return
            self._mark_acm_state_unknown(
                context, f"{phase.lower()} timed out"
            )

        timer = self.create_timer(ACM_TRANSACTION_TIMEOUT_S, expired)
        timer_ref["timer"] = timer
        context["timer"] = timer
        self._acm_update_timer = timer

    def _clear_acm_transaction(self, *, clear_baseline):
        self._cancel_acm_phase_timeout(self._acm_update_pending)
        self._acm_update_pending = None
        if clear_baseline:
            self._contact_collision_baseline = None
            self._contact_collision_target_name = ""

    def _mark_acm_state_unknown(self, context, reason):
        if self._acm_update_pending is not context:
            return
        self._cancel_acm_phase_timeout(context)
        self._invalidate_acm_health_query()
        self._acm_update_pending = None
        self._acm_state_unknown = True
        self._acm_baseline_verified = False
        self._acm_baseline_verified_target_name = ""
        self._acm_baseline_verified_time = 0.0
        # An ApplyPlanningScene transport/verification failure can mean the
        # pair is still allowed. Treat it as allowed until the whole MoveIt
        # stack is relaunched; never advertise a known-safe local state.
        self._contact_collision_allowed = True
        self.get_logger().error(
            f"[ACM] state unknown ({reason}); full stack relaunch required"
        )
        self._notify_acm_callbacks(context.get("allow_callbacks", ()), False)
        self._notify_acm_callbacks(context.get("restore_callbacks", ()), False)

    def _acm_services_ready(self, *, require_get):
        if (
            not self.apply_scene_client.service_is_ready()
            and not self.apply_scene_client.wait_for_service(timeout_sec=0.0)
        ):
            return False
        if require_get:
            if (
                not self.get_planning_scene_client.service_is_ready()
                and not self.get_planning_scene_client.wait_for_service(
                    timeout_sec=0.0
                )
            ):
                return False
        return True

    def _acm_context_scene_is_current(self, context):
        return bool(
            context.get("scene_revision")
            == int(getattr(self, "_scene_revision", -1))
            and getattr(self, "scene_confirmed", False)
            and int(getattr(self, "_scene_confirmed_revision", -1))
            == int(getattr(self, "_scene_revision", -1))
            and getattr(self, "_scene_apply_inflight_revision", None) is None
        )

    def _request_acm_readback(self, context, expected, phase):
        if self._acm_update_pending is not context:
            return
        if not self._acm_services_ready(require_get=True):
            if phase == "VERIFY_ALLOW":
                context["allow_failed"] = True
                context["restore_requested"] = True
                self._start_acm_restore(context)
            else:
                self._mark_acm_state_unknown(
                    context, "GetPlanningScene unavailable during restore verify"
                )
            return
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        context["phase"] = phase
        try:
            future = self.get_planning_scene_client.call_async(request)
        except Exception as exc:
            if phase == "VERIFY_ALLOW":
                context["allow_failed"] = True
                context["restore_requested"] = True
                context["verify_error"] = str(exc)
                self._start_acm_restore(context)
            else:
                self._mark_acm_state_unknown(
                    context, f"restore readback request failed: {exc}"
                )
            return
        self._arm_acm_phase_timeout(context, phase)
        future.add_done_callback(
            lambda done, ctx=context, wanted=copy.deepcopy(expected), stage=phase:
            self._acm_readback_done(done, ctx, wanted, stage)
        )

    def _acm_readback_done(self, future, context, expected, phase):
        if (
            self._acm_update_pending is not context
            or context.get("phase") != phase
        ):
            return
        self._cancel_acm_phase_timeout(context)
        try:
            observed = future.result().scene.allowed_collision_matrix
        except Exception as exc:
            observed = None
            error = f"readback failed: {exc}"
        else:
            error = MoveItExecutor._acm_validation_error(
                observed, self._required_acm_allowed_pairs
            )
            if not error and not MoveItExecutor._acm_equal(observed, expected):
                error = "readback differs from the full requested matrix"
        if error:
            if phase == "VERIFY_ALLOW":
                context["allow_failed"] = True
                context["restore_requested"] = True
                context["verify_error"] = error
                self.get_logger().error(
                    f"[ACM] roller contact allow verification failed: {error}"
                )
                self._start_acm_restore(context)
            else:
                self._mark_acm_state_unknown(
                    context, f"baseline restore verification failed: {error}"
                )
            return

        self._latest_allowed_collision_matrix = copy.deepcopy(observed)
        if phase == "VERIFY_ALLOW":
            if not self._acm_context_scene_is_current(context):
                context["allow_failed"] = True
                context["restore_requested"] = True
                self.get_logger().error(
                    "[ACM] PlanningScene revision changed during contact allow"
                )
                self._start_acm_restore(context)
                return
            self._contact_collision_allowed = True
            if context.get("restore_requested"):
                self._start_acm_restore(context)
                return
            self._acm_update_pending = None
            self.get_logger().info(
                "[ACM] roller contact pair allowed without changing the "
                "SRDF collision baseline: %s <-> %s"
                % (ROLLER_CONTACT_LINK, context["target"])
            )
            self._notify_acm_callbacks(context.get("allow_callbacks", ()), True)
            return

        self._contact_collision_allowed = False
        self._acm_state_unknown = False
        self._acm_baseline_verified = True
        self._acm_baseline_verified_target_name = str(context["target"])
        self._acm_baseline_verified_time = time.monotonic()
        self._clear_acm_transaction(clear_baseline=True)
        self.get_logger().info(
            "[ACM] exact pre-contact collision matrix restored"
        )
        if context.get("allow_failed") or context.get("allow_cancelled"):
            self._notify_acm_callbacks(context.get("allow_callbacks", ()), False)
        self._notify_acm_callbacks(context.get("restore_callbacks", ()), True)

    def _apply_acm_matrix(self, context, matrix, phase):
        if self._acm_update_pending is not context:
            return
        if (
            phase == "APPLY_ALLOW"
            and not self._acm_context_scene_is_current(context)
        ):
            self._clear_acm_transaction(clear_baseline=True)
            self.get_logger().error(
                "[ACM] PlanningScene changed before contact allow apply"
            )
            self._notify_acm_callbacks(
                context.get("allow_callbacks", ()), False
            )
            return
        if not self._acm_services_ready(require_get=False):
            if phase == "APPLY_ALLOW":
                self._clear_acm_transaction(clear_baseline=True)
                self._notify_acm_callbacks(
                    context.get("allow_callbacks", ()), False
                )
            else:
                self._mark_acm_state_unknown(
                    context, "ApplyPlanningScene unavailable during restore"
                )
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = copy.deepcopy(matrix)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        context["phase"] = phase
        try:
            future = self.apply_scene_client.call_async(request)
        except Exception as exc:
            if phase == "APPLY_ALLOW":
                # call_async throwing before a future is returned is treated
                # as a known non-dispatch; no matrix was accepted for motion.
                self._clear_acm_transaction(clear_baseline=True)
                self._notify_acm_callbacks(
                    context.get("allow_callbacks", ()), False
                )
                self.get_logger().error(f"[ACM] allow request failed: {exc}")
            else:
                self._mark_acm_state_unknown(
                    context, f"baseline restore request failed: {exc}"
                )
            return
        self._arm_acm_phase_timeout(context, phase)
        future.add_done_callback(
            lambda done, ctx=context, sent=copy.deepcopy(matrix), stage=phase:
            self._acm_apply_done(done, ctx, sent, stage)
        )

    def _acm_apply_done(self, future, context, matrix, phase):
        if (
            self._acm_update_pending is not context
            or context.get("phase") != phase
        ):
            return
        self._cancel_acm_phase_timeout(context)
        try:
            success = bool(future.result().success)
        except Exception as exc:
            self._mark_acm_state_unknown(
                context, f"{phase.lower()} response failed: {exc}"
            )
            return
        if not success:
            if phase == "APPLY_ALLOW":
                self._contact_collision_allowed = False
                self._clear_acm_transaction(clear_baseline=True)
                self.get_logger().error(
                    "[ACM] selective roller/surface allow was rejected"
                )
                if context.get("restore_requested"):
                    self._notify_acm_callbacks(
                        context.get("allow_callbacks", ()), False
                    )
                    self._notify_acm_callbacks(
                        context.get("restore_callbacks", ()), True
                    )
                else:
                    self._notify_acm_callbacks(
                        context.get("allow_callbacks", ()), False
                    )
            else:
                self._mark_acm_state_unknown(
                    context, "MoveIt rejected the exact ACM restore"
                )
            return
        self._request_acm_readback(
            context,
            matrix,
            "VERIFY_ALLOW" if phase == "APPLY_ALLOW" else "VERIFY_RESTORE",
        )

    def _start_acm_restore(self, context):
        if self._acm_update_pending is not context:
            return
        baseline = context.get("baseline")
        error = MoveItExecutor._acm_clean_baseline_error(
            baseline, self._required_acm_allowed_pairs
        )
        if error:
            self._mark_acm_state_unknown(
                context, f"frozen baseline invalid during restore: {error}"
            )
            return
        self._apply_acm_matrix(context, baseline, "APPLY_RESTORE")

    def _acm_baseline_done(self, future, context):
        if (
            self._acm_update_pending is not context
            or context.get("phase") != "FETCH_BASELINE"
        ):
            return
        self._cancel_acm_phase_timeout(context)
        if context.get("restore_requested"):
            self._contact_collision_allowed = False
            self._clear_acm_transaction(clear_baseline=True)
            self._notify_acm_callbacks(
                context.get("allow_callbacks", ()), False
            )
            self._notify_acm_callbacks(
                context.get("restore_callbacks", ()), True
            )
            return
        try:
            baseline = future.result().scene.allowed_collision_matrix
        except Exception as exc:
            baseline_received = False
            error = f"GetPlanningScene failed: {exc}"
        else:
            baseline_received = True
            error = MoveItExecutor._acm_validation_error(
                baseline, self._required_acm_allowed_pairs
            )
            if (
                not error
                and MoveItExecutor._acm_effective_pair_value(
                    baseline,
                    ROLLER_CONTACT_LINK,
                    context["target"],
                )
            ):
                error = (
                    "roller/target pair was already allowed in the fetched "
                    "baseline; full MoveIt relaunch required"
                )
            elif not error:
                error = MoveItExecutor._acm_clean_baseline_error(
                    baseline, self._required_acm_allowed_pairs
                )
        if error:
            self._clear_acm_transaction(clear_baseline=True)
            if baseline_received:
                # A full ACM response that is malformed, missing the required
                # SRDF pairs, or already permits roller-wall contact proves
                # that the live MoveIt scene is contaminated. Local state
                # must not pretend that a retry can repair it.
                self._acm_state_unknown = True
                self._acm_baseline_verified = False
                self._acm_baseline_verified_target_name = ""
                self._acm_baseline_verified_time = 0.0
                self._contact_collision_allowed = True
            else:
                self._contact_collision_allowed = False
            self.get_logger().error(
                f"[ACM] refusing roller contact: {error}"
                + (
                    "; full stack relaunch required"
                    if baseline_received
                    else ""
                )
            )
            self._notify_acm_callbacks(
                context.get("allow_callbacks", ()), False
            )
            return
        context["baseline"] = copy.deepcopy(baseline)
        self._acm_baseline_verified = True
        self._acm_baseline_verified_target_name = str(context["target"])
        self._acm_baseline_verified_time = time.monotonic()
        self._contact_collision_baseline = copy.deepcopy(baseline)
        self._contact_collision_target_name = context["target"]
        self._latest_allowed_collision_matrix = copy.deepcopy(baseline)
        try:
            allowed_matrix = MoveItExecutor._acm_with_pair(
                baseline,
                ROLLER_CONTACT_LINK,
                context["target"],
                True,
            )
        except ValueError as exc:
            self._clear_acm_transaction(clear_baseline=True)
            self.get_logger().error(f"[ACM] baseline mutation rejected: {exc}")
            self._notify_acm_callbacks(
                context.get("allow_callbacks", ()), False
            )
            return
        context["allowed_matrix"] = copy.deepcopy(allowed_matrix)
        self._apply_acm_matrix(context, allowed_matrix, "APPLY_ALLOW")

    def _set_contact_collision_allowed(self, allowed, callback=None):
        allowed = bool(allowed)
        if getattr(self, "_acm_state_unknown", False):
            self.get_logger().error(
                "[ACM] collision state is unknown; full stack relaunch required"
            )
            if callback is not None:
                callback(False)
            return
        pending = self._acm_update_pending
        if allowed:
            if (
                not bool(getattr(self, "dry_run", True))
                and (
                    not MoveItExecutor._acm_baseline_is_verified(self)
                )
            ):
                self.get_logger().error(
                    "[ACM] clean startup baseline has not been verified"
                )
                if callback is not None:
                    callback(False)
                return
            # A periodic read-only health GET may have been issued while the
            # clean baseline was still fresh.  Its response is not ordered
            # against the contact Apply/restore sequence, so invalidate it
            # before starting the transaction and trust only the fresh GET
            # owned by the transaction below.
            self._invalidate_acm_health_query()
            if self._contact_collision_allowed and pending is None:
                if callback is not None:
                    callback(True)
                return
            if pending is not None:
                if (
                    not pending.get("restore_requested")
                    and pending.get("phase") in {
                        "FETCH_BASELINE",
                        "APPLY_ALLOW",
                        "VERIFY_ALLOW",
                    }
                ):
                    if callback is not None:
                        pending["allow_callbacks"].append(callback)
                    return
                if callback is not None:
                    callback(False)
                return
            if not self._required_acm_allowed_pairs:
                self.get_logger().error(
                    "[ACM] SRDF baseline contract is unavailable"
                )
                if callback is not None:
                    callback(False)
                return
            if (
                getattr(self, "_scene_apply_inflight_revision", None)
                is not None
                or not getattr(self, "scene_confirmed", False)
                or int(getattr(self, "_scene_confirmed_revision", -1))
                != int(getattr(self, "_scene_revision", 0))
            ):
                self.get_logger().error(
                    "[ACM] current PlanningScene revision is not confirmed"
                )
                if callback is not None:
                    callback(False)
                return
            if not self._acm_services_ready(require_get=True):
                self.get_logger().error(
                    "[ACM] Get/ApplyPlanningScene services unavailable"
                )
                if callback is not None:
                    callback(False)
                return
            self._acm_update_seq += 1
            context = {
                "seq": self._acm_update_seq,
                "phase": "FETCH_BASELINE",
                "target": str(self.active_target_name),
                "scene_revision": int(self._scene_revision),
                "allow_callbacks": [callback] if callback is not None else [],
                "restore_callbacks": [],
                "restore_requested": False,
                "allow_failed": False,
                "allow_cancelled": False,
                "baseline": None,
            }
            self._acm_update_pending = context
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
            try:
                future = self.get_planning_scene_client.call_async(request)
            except Exception as exc:
                self._clear_acm_transaction(clear_baseline=True)
                self.get_logger().error(
                    f"[ACM] baseline request failed: {exc}"
                )
                self._notify_acm_callbacks(
                    context["allow_callbacks"], False
                )
                return
            self._arm_acm_phase_timeout(context, "FETCH_BASELINE")
            future.add_done_callback(
                lambda done, ctx=context: self._acm_baseline_done(done, ctx)
            )
            return

        if pending is not None:
            if callback is not None:
                pending["restore_callbacks"].append(callback)
            pending["restore_requested"] = True
            pending["allow_cancelled"] = True
            cancelled_callbacks = list(pending.get("allow_callbacks", ()))
            pending["allow_callbacks"] = []
            self._notify_acm_callbacks(cancelled_callbacks, False)
            if pending.get("phase") == "FETCH_BASELINE":
                # No Apply request exists yet, so invalidating this fetch is a
                # complete and safe cancellation. Its late response is ignored.
                self._acm_update_seq += 1
                self._contact_collision_allowed = False
                self._clear_acm_transaction(clear_baseline=True)
                self._notify_acm_callbacks(
                    pending.get("restore_callbacks", ()), True
                )
            return
        if self._contact_collision_baseline is None:
            if self._contact_collision_allowed:
                self._acm_state_unknown = True
                self._acm_baseline_verified = False
                self._acm_baseline_verified_target_name = ""
                self._acm_baseline_verified_time = 0.0
                self.get_logger().error(
                    "[ACM] allowed state has no frozen baseline; relaunch required"
                )
                if callback is not None:
                    callback(False)
                return
            if callback is not None:
                callback(True)
            return
        self._acm_update_seq += 1
        context = {
            "seq": self._acm_update_seq,
            "phase": "APPLY_RESTORE",
            "target": self._contact_collision_target_name,
            "scene_revision": int(getattr(self, "_scene_revision", 0)),
            "allow_callbacks": [],
            "restore_callbacks": [callback] if callback is not None else [],
            "restore_requested": True,
            "allow_failed": False,
            "allow_cancelled": False,
            "baseline": copy.deepcopy(self._contact_collision_baseline),
        }
        self._acm_update_pending = context
        self._start_acm_restore(context)

    def _transition_execution_state(self, next_state, reason="", **fields):
        previous = self._execution_state
        status_fields = {
            key: value
            for key, value in fields.items()
            if key not in {
                "state",
                "reason",
                "timestamp_ns",
                "plan_hash",
                "plane_generation_id",
                "work_area_id",
                "execution_snapshot_locked",
                "normal_force_n",
                "contact_confirmed",
            }
        }
        # Transition metadata is authoritative even if a future caller passes
        # a field with the same name.  Build one mapping before expansion so a
        # diagnostic payload can never crash the state transition with
        # duplicate keyword arguments.
        status_fields.update(
            previous_state=previous,
            next_state=str(next_state),
            contact_search_distance_m=float(
                self._contact_search_distance_m
            ),
        )
        self._publish_execution_status(
            next_state,
            reason,
            **status_fields,
        )

    def _contact_search_mode_ack_blockers(self):
        """Require post-command safety/guard acknowledgement before motion."""

        if not self.painting_force_enabled:
            return ()
        blockers = list(self._current_real_plan_blockers())
        blockers.extend(self._real_force_runtime_blockers())
        published_at = float(self._contact_search_mode_published_at)
        if published_at <= 0.0:
            blockers.append("CONTACT_SEARCH_MODE_EDGE_MISSING")
            return tuple(dict.fromkeys(blockers))
        if self._safety_status_time <= published_at:
            blockers.append("SAFETY_CONTACT_SEARCH_ACK_PENDING")
        else:
            safety_mode = str(
                self._safety_status.get("mode", "")
            ).strip().upper()
            if safety_mode != "CONTACT_SEARCH":
                blockers.append(f"SAFETY_MODE_ACK:{safety_mode or 'MISSING'}")
            if self._safety_status.get("ft_valid") is not True:
                blockers.append("SAFETY_ACK_FT_INVALID")
            if self._safety_status.get("tf_valid") is not True:
                blockers.append("SAFETY_ACK_TF_INVALID")
            if self._safety_status.get("bias_ready") is not True:
                blockers.append("SAFETY_ACK_BIAS_NOT_READY")
            if self._safety_status.get("abort_latched") is not False:
                blockers.append("SAFETY_ACK_ABORT_LATCHED")
            if self._safety_status.get("force_enabled") is not False:
                blockers.append("SAFETY_ACK_FORCE_NOT_OFF")
        blockers.extend(
            MoveItExecutor._force_guard_status_blockers(
                self,
                expected_mode="CONTACT_SEARCH",
                active=False,
                command_context=self._contact_search_command_context,
            )
        )
        return tuple(dict.fromkeys(blockers))

    def _wait_for_contact_search_mode_ack(self, step):
        if self._motion_abort_requested:
            return
        blockers = self._contact_search_mode_ack_blockers()
        if not blockers:
            self._publish_execution_status(
                "CONTACT_SEARCH",
                "safety and wrench guard acknowledged CONTACT_SEARCH",
                mode_acknowledged=True,
            )
            self._contact_search_iteration(step)
            return
        elapsed_s = (
            time.monotonic() - self._contact_search_mode_published_at
        )
        if elapsed_s > self.contact_search_mode_ack_timeout_s:
            self._set_contact_collision_allowed(False)
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH mode ACK timeout: " + ",".join(blockers)
            )
            return
        self._schedule_process_once(
            0.02, lambda: self._wait_for_contact_search_mode_ack(step)
        )

    def _start_contact_search(self, step):
        if self.painting_force_enabled:
            blockers = self._real_force_runtime_blockers()
            if blockers:
                self._fail_workflow_known_safe(
                    "CONTACT_SEARCH force gate: " + ",".join(blockers)
                )
                return
        self._contact_search_command_context = self._publish_painting_command(
            "CONTACT_SEARCH", 0.0, enable=False
        )
        self._contact_search_mode_published_at = float(
            self._contact_search_command_context["published_at_s"]
        )
        self._transition_execution_state("CONTACT_SEARCH", "bounded search start")
        self._contact_search_started_at = time.monotonic()
        self._contact_search_distance_m = 0.0
        self._contact_search_step_active = False
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        if not self.real_painting_enabled:
            self._contact_search_confirmed = True
            self._complete_process_step()
            return
        self._set_contact_collision_allowed(
            True,
            lambda success: self._wait_for_contact_search_mode_ack(step)
            if success
            else self._fail_workflow_known_safe(
                "CONTACT_SEARCH ACM update failed"
            ),
        )

    def _contact_search_iteration(self, step):
        if self._motion_abort_requested or self._contact_search_step_active:
            return
        mode_ack_blockers = self._contact_search_mode_ack_blockers()
        if mode_ack_blockers:
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH mode ACK lost: "
                + ",".join(mode_ack_blockers)
            )
            return
        now = time.monotonic()
        safety_fresh = (
            self._safety_status_time > 0.0
            and now - self._safety_status_time <= self.ft_required_timeout_s
        )
        contact = self._contact_search_contact_sensed(now)
        decision = evaluate_contact_search(
            self.contact_search_config,
            elapsed_s=now - self._contact_search_started_at,
            cumulative_distance_m=self._contact_search_distance_m,
            contact_confirmed=contact,
            ft_valid=safety_fresh and self._safety_status.get("ft_valid") is True,
            tf_valid=(
                self._safety_status.get("tf_valid") is True
                and self._current_tcp_pose_np() is not None
            ),
            controller_fault=bool(self._controller_fault),
            abort_latched=bool(
                self._motion_abort_requested
                or self._safety_status.get("abort_latched", True)
            ),
        )
        if decision.action == "CONTACT_CONFIRMED":
            self._contact_search_confirmed = True
            self._transition_execution_state(
                "CONTACT_SEARCH_COMPLETE",
                "contact confirmed",
            )
            self._set_contact_collision_allowed(
                False,
                lambda success: self._complete_process_step()
                if success
                else self._request_motion_abort("CONTACT_SEARCH ACM restore failed"),
            )
            return
        if decision.action == "FAULT":
            self._set_contact_collision_allowed(False)
            self._request_motion_abort(decision.fault_reason)
            return
        self._plan_contact_search_step(step, decision.next_step_m)

    def _contact_search_contact_sensed(self, now=None):
        now = time.monotonic() if now is None else float(now)
        monitor_contact = bool(
            self._painting_contact_confirmed
            and self._painting_contact_feedback_time > 0.0
            and now - self._painting_contact_feedback_time
            <= self.ft_required_timeout_s
        )
        legacy_contact = bool(
            self.ft_contact
            and self.ft_status_time > 0.0
            and now - self.ft_status_time <= FT_FORCE_STALE_SEC
        )
        return monitor_contact or legacy_contact

    def _complete_contact_search_after_early_contact(self):
        if self._motion_abort_requested:
            return
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        self._contact_search_step_active = False
        # This branch bypasses the normal per-step completion, so the step the
        # robot was executing when contact was confirmed has to be accounted
        # for here.  It ran to its natural end (contact-search goals are no
        # longer cancelled), and _apply_contact_depth_correction derives the
        # PAINT depth correction from this total - dropping the final step
        # would bias every stroke one step further off the wall.
        context = self._contact_search_context
        if context is not None:
            try:
                self._contact_search_distance_m += float(context["distance_m"])
                self._process_last_tcp_pose = copy.deepcopy(context["target"])
            except (KeyError, TypeError, ValueError):
                pass
        self._contact_search_context = None
        self._contact_search_confirmed = True
        self._transition_execution_state(
            "CONTACT_SEARCH_COMPLETE", "early contact canceled active trajectory"
        )
        self._set_contact_collision_allowed(
            False,
            lambda success: self._complete_process_step()
            if success
            else self._request_motion_abort("CONTACT_SEARCH ACM restore failed"),
        )

    @staticmethod
    def _cartesian_response_error(
        response, required_fraction=CARTESIAN_COMPLETE_FRACTION
    ):
        """Validate a Cartesian result before any physical command.

        Process states update their logical TCP to the requested final pose,
        so a partial solution cannot be reported as completion.  Collision
        checking stays enabled in MoveIt; this helper only verifies that the
        collision-checked request actually reached its full endpoint.
        """

        error_code = getattr(getattr(response, "error_code", None), "val", None)
        if error_code is None:
            return "CARTESIAN_ERROR_CODE_MISSING"
        if int(error_code) != 1:
            return f"CARTESIAN_ERROR_CODE_{int(error_code)}"
        try:
            fraction = float(response.fraction)
        except (TypeError, ValueError, AttributeError):
            return "CARTESIAN_FRACTION_INVALID"
        if not math.isfinite(fraction) or fraction < 0.0 or fraction > 1.0 + 1e-6:
            return f"CARTESIAN_FRACTION_INVALID:{fraction!r}"
        if fraction + 1e-9 < float(required_fraction):
            return (
                f"CARTESIAN_PATH_INCOMPLETE:{fraction:.6f}<"
                f"{float(required_fraction):.6f}"
            )
        solution = getattr(response, "solution", None)
        trajectory = getattr(solution, "joint_trajectory", None)
        if (
            trajectory is None
            or not trajectory.joint_names
            or not trajectory.points
        ):
            return "CARTESIAN_TRAJECTORY_EMPTY"
        return ""

    def _plan_contact_search_step(self, step, distance_m):
        if not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH planning service unavailable"
            )
            return
        if self._process_last_tcp_pose is None:
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH current pose unavailable"
            )
            return
        normal = np.asarray(step.rows[-1].normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        target = copy.deepcopy(self._process_last_tcp_pose)
        target.position.x -= float(distance_m * normal[0])
        target.position.y -= float(distance_m * normal[1])
        target.position.z -= float(distance_m * normal[2])

        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = PLANNING_GROUP
        request.link_name = EE_LINK
        state = RobotState()
        seed_state = copy.deepcopy(self.current_joint_state)
        state.joint_state = copy.deepcopy(seed_state)
        state.is_diff = False
        request.start_state = state
        request.waypoints = [target]
        request.max_step = min(0.0005, float(distance_m))
        request.jump_threshold = 2.0
        request.avoid_collisions = True
        velocity_scale = max(
            0.001, min(0.05, self.contact_search_speed_mps / 0.10)
        )
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = velocity_scale
        token = object()
        self._contact_search_step_active = True
        self._contact_search_context = {
            "token": token,
            "target": target,
            "distance_m": float(distance_m),
            "seed_state": seed_state,
            "scene_revision": int(getattr(self, "_scene_revision", 0)),
            "execution_snapshot": getattr(self, "_execution_snapshot", None),
        }
        future = self.cartesian_client.call_async(request)
        future.add_done_callback(
            lambda done, expected=token: self._contact_search_plan_done(
                done, expected
            )
        )

    def _contact_search_plan_done(self, future, expected_token):
        context = self._contact_search_context
        if (
            context is None
            or context["token"] is not expected_token
            or self._motion_abort_requested
        ):
            return
        try:
            response = future.result()
        except Exception as exc:
            self._contact_search_step_active = False
            self._fail_workflow_known_safe(
                f"CONTACT_SEARCH planning failed: {exc}"
            )
            return
        response_error = MoveItExecutor._cartesian_response_error(
            response, CARTESIAN_COMPLETE_FRACTION
        )
        if response_error:
            self._contact_search_step_active = False
            self._fail_workflow_known_safe(
                f"CONTACT_SEARCH planning rejected:{response_error}"
            )
            return
        trajectory = self._limit_trajectory_to_cartesian_speed(
            response.solution,
            self._process_last_tcp_pose,
            [context["target"]],
            self.contact_search_speed_mps,
        )

        def complete():
            self._process_last_tcp_pose = copy.deepcopy(context["target"])
            self._contact_search_distance_m += context["distance_m"]
            self._contact_search_step_active = False
            self._contact_search_context = None
            self._publish_execution_status(
                "CONTACT_SEARCH",
                "step complete",
                contact_search_distance_m=float(
                    self._contact_search_distance_m
                ),
            )
            # Contact confirmation lags the physical touch by the monitor
            # filter (40 ms) plus the confirm-hold (40 ms).  Dispatching the
            # next step 1 ms after the previous one meant the confirmation
            # always landed 20-100 ms AFTER a fresh goal was sent, so the
            # early-contact path cancelled a goal still inside its
            # accept/activate window - a known rclcpp_action race that can
            # orphan the terminal result (three FJT_RESULT/CANCEL_TIMEOUT
            # hard stops on 2026-08-14).  A 0.15 s inter-step dwell lets the
            # confirmation arrive BETWEEN steps, where the iteration exits
            # through the clean CONTACT_CONFIRMED decision with no cancel at
            # all.  Cost: +4.5 s over a full 30-step search, still inside
            # contact_search_timeout_s.
            self._schedule_process_once(
                0.15,
                lambda: self._contact_search_iteration(
                    self._process_steps[self._process_step_index]
                ),
            )

        mode_ack_blockers = self._contact_search_mode_ack_blockers()
        if mode_ack_blockers:
            self._contact_search_step_active = False
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH dispatch gate: "
                + ",".join(mode_ack_blockers)
            )
            return
        if self.real_painting_enabled:
            plan_blockers = self._current_real_plan_blockers()
            if plan_blockers:
                self._contact_search_step_active = False
                self._fail_workflow_known_safe(
                    "CONTACT_SEARCH plan identity:"
                    + ",".join(plan_blockers)
                )
                return
        start_ok, start_reason = self._d405_plan_start_matches_measured(
            trajectory, context["seed_state"]
        )
        if not start_ok:
            self._contact_search_step_active = False
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH stale plan start:" + start_reason
            )
            return
        if (
            int(getattr(self, "_scene_revision", 0))
            != int(context["scene_revision"])
            or not getattr(self, "scene_confirmed", False)
            or int(getattr(self, "_scene_confirmed_revision", -1))
            != int(context["scene_revision"])
            or getattr(self, "_execution_snapshot", None)
            is not context["execution_snapshot"]
        ):
            self._contact_search_step_active = False
            self._fail_workflow_known_safe(
                "CONTACT_SEARCH planning context changed before dispatch"
            )
            return
        if not self.execute_trajectory_direct(
            trajectory,
            on_complete=complete,
            on_failure=lambda: self._handle_process_trajectory_failure(
                "CONTACT_SEARCH trajectory"
            ),
            on_rejected=lambda: self._fail_workflow_known_safe(
                "CONTACT_SEARCH trajectory goal rejected"
            ),
            force_guard=True,
            label="CONTACT_SEARCH step",
            requires_contact_acm=True,
        ):
            self._contact_search_step_active = False
            self._handle_known_dispatch_rejection(
                "CONTACT_SEARCH trajectory"
            )

    def _start_segment_process(self):
        if self._motion_abort_requested:
            return
        if self._active_segment_path is None:
            self._fail_workflow_known_safe("segment process path missing")
            return
        self.get_logger().info("=== SEGMENT PAINTING PROCESS START ===")
        self._process_step_index = 0
        self._process_force_ready = False
        self._execute_next_process_step()

    def _complete_process_step(self):
        if self._motion_abort_requested:
            return
        self._process_step_index += 1
        self._schedule_process_once(0.001, self._execute_next_process_step)

    def _execute_next_process_step(self):
        if self._motion_abort_requested:
            return
        if self._process_step_index >= len(self._process_steps):
            self.get_logger().info(
                "[PAINT PROCESS] all explicit segments complete"
            )
            if (
                self._active_segment_path is not None
                and self._active_segment_path.version >= 3
            ):
                self._transition_execution_state("COMPLETE", "final retract complete")
                self._reset_painting_process()
                self.executing = False
                return
            self._publish_painting_command(
                "FINISH_RETRACT", 0.0, enable=False
            )
            self.stage4_retreat()
            return

        step = self._process_steps[self._process_step_index]
        mode = step.mode
        if self.real_painting_enabled:
            plan_blockers = self._current_real_plan_blockers()
            if plan_blockers:
                self._fail_workflow_known_safe(
                    "PROCESS plan identity: " + ",".join(plan_blockers)
                )
                return
        self.get_logger().info(
            f"[PAINT PROCESS] step {self._process_step_index + 1}/"
            f"{len(self._process_steps)} mode={mode} rows="
            f"{step.rows[0].row_number}-{step.rows[-1].row_number} "
            f"force={step.force_n:.3f}N speed={step.speed_mps:.3f}m/s"
        )

        if getattr(self, "process_mode", "paint") == "spray":
            self._spray_execute_step(step)
            return

        if mode == "ABORT":
            self._request_motion_abort("ABORT row in painting process")
            return
        if mode == "CONTACT_SEARCH":
            self._start_contact_search(step)
            return
        if mode in {"RAMP_UP", "RAMP_DOWN"}:
            if mode == "RAMP_DOWN" and self._contact_collision_allowed:
                escape = getattr(self, "_contact_escape_context", None)
                if (
                    escape is None
                    or escape.get("ramp_step_index")
                    != self._process_step_index
                    or escape.get("phase") != "PAINT_COMPLETE"
                ):
                    self._request_motion_abort(
                        "CONTACT_ESCAPE_RAMP_SEQUENCE_INVALID"
                    )
                    return
                escape["phase"] = "RAMP_DOWN"
            self._start_process_ramp(step)
            return
        if mode in {"IDLE", "DWELL"}:
            self._publish_painting_command(mode, step.force_n)
            self._complete_process_step()
            return
        if mode not in MOTION_MODES:
            self._fail_workflow_known_safe(
                f"unsupported execution mode {mode}"
            )
            return
        if mode == "APPROACH_PRECONTACT":
            self._transition_execution_state(
                "APPROACH_PRECONTACT", "precontact motion"
            )
            if self._painting_contact_confirmed or self.ft_contact:
                if self.painting_force_enabled:
                    self._request_motion_abort(
                        "unexpected contact before execution-scoped tare"
                    )
                    return
                self.get_logger().warn(
                    "[APPROACH_PRECONTACT] 조기 접촉 확인 -> 추가 접근 생략"
                )
                self._contact_search_confirmed = True
                self._complete_process_step()
                return
        if mode in CONTACT_MOTION_MODES and not self._process_force_ready:
            self._fail_workflow_known_safe(
                f"{mode} entered before RAMP_UP completion"
            )
            return
        if mode in ZERO_FORCE_MOTION_MODES and self._process_force_ready:
            self._fail_workflow_known_safe(
                f"{mode} entered before RAMP_DOWN completion"
            )
            return

        if mode == "PAINT":
            self._start_paint_entry_barrier(step)
            return
        if self._contact_collision_allowed:
            escape = getattr(self, "_contact_escape_context", None)
            if (
                mode not in {"RETRACT", "FINAL_RETRACT"}
                or not MoveItExecutor._contact_escape_is_current(
                    self, escape, step=step, require_zero_ack=True
                )
            ):
                self._request_motion_abort(
                    f"CONTACT_ACM_ACTIVE_BEFORE_UNAUTHORIZED_{mode}"
                )
                return
            escape["phase"] = "ESCAPE_PLANNING"
        self._publish_painting_command(mode, step.force_n)
        self._transition_execution_state(mode, "segment motion")
        self._plan_process_motion_step(step)

    @staticmethod
    def _guard_ack_fault_is_immediate(blockers):
        """Separate a normal mode-edge transition from an actual guard fault."""

        fatal = {
            "WRENCH_GUARD_STALE",
            "GUARD_FT_INVALID",
            "GUARD_TF_INVALID",
            "GUARD_ABORT_LATCHED",
            "GUARD_CONTROLLER_FAULT",
            "GUARD_COMMAND_EDGE_INVALID",
        }
        return any(str(blocker) in fatal for blocker in blockers)

    def _paint_entry_is_current(self, context):
        return bool(
            context is not None
            and self._paint_entry_context is context
            and not self._motion_abort_requested
            and context.get("step_index") == self._process_step_index
            and self._process_step_index < len(self._process_steps)
            and self._process_steps[self._process_step_index]
            is context.get("step")
            and self._painting_command_context
            is context.get("command_context")
            and getattr(self, "_execution_snapshot", None)
            is context.get("execution_snapshot")
        )

    def _paint_entry_tick(self, context):
        if not self._paint_entry_is_current(context):
            if self._paint_entry_context is context:
                self._cancel_process_timer()
                self._paint_entry_context = None
            return
        blockers = MoveItExecutor._force_guard_status_blockers(
            self,
            expected_mode="PAINT",
            active=True,
            command_context=context["command_context"],
        )
        if blockers:
            elapsed_s = time.monotonic() - float(
                context["command_context"]["published_at_s"]
            )
            if (
                context.get("guard_acknowledged", False)
                or MoveItExecutor._guard_ack_fault_is_immediate(blockers)
                or elapsed_s > self.force_guard_mode_ack_timeout_s
            ):
                self._cancel_process_timer()
                self._request_motion_abort(
                    "PAINT_GUARD_ACTIVE_ACK_FAILED:" + ",".join(blockers)
                )
            return
        context["guard_acknowledged"] = True
        lease = getattr(self, "_force_phase_lease_context", None)
        if (
            self.painting_force_enabled
            and (
                lease is None
                or lease.get("command_context")
                is not context["command_context"]
                or lease.get("mode") != "PAINT"
                or lease.get("active") is not True
            )
        ):
            self._request_motion_abort("PAINT_FORCE_LEASE_CONTEXT_INVALID")
            return
        if lease is not None:
            lease["acknowledged"] = True
        if context.get("acm_ready") and not context.get("planning_started"):
            context["planning_started"] = True
            # Keep this timer alive during the asynchronous Cartesian request.
            # `_process_cartesian_done` transfers supervision to the FJT guard
            # only after one final PAINT-mode ACK check.
            self._plan_process_motion_step(context["step"])

    def _paint_entry_acm_done(self, context, success):
        if not self._paint_entry_is_current(context):
            return
        if not success:
            self._cancel_process_timer()
            self._request_motion_abort("PAINT_ACM_UPDATE_FAILED_AFTER_FORCE_ENABLE")
            return
        context["acm_ready"] = True
        self._paint_entry_tick(context)

    def _start_paint_entry_barrier(self, step):
        if self.painting_force_enabled:
            blockers = MoveItExecutor._force_guard_status_blockers(
                self,
                expected_mode="RAMP_UP",
                active=True,
                command_context=self._painting_command_context,
            )
            if blockers:
                self._request_motion_abort(
                    "PAINT_ENTRY_GUARD_NOT_ACTIVE:" + ",".join(blockers)
                )
                return
        command_context = self._publish_painting_command(
            "PAINT", step.force_n
        )
        if self.painting_force_enabled:
            lease = MoveItExecutor._set_force_phase_lease_command(
                self,
                command_context,
                mode="PAINT",
                active=True,
                step=step,
            )
            if lease is None or self._motion_abort_requested:
                return
        self._transition_execution_state("PAINT", "segment motion")
        context = {
            "token": object(),
            "step": step,
            "step_index": self._process_step_index,
            "command_context": command_context,
            "execution_snapshot": getattr(self, "_execution_snapshot", None),
            "guard_acknowledged": not self.painting_force_enabled,
            "acm_ready": False,
            "planning_started": False,
        }
        self._paint_entry_context = context
        self._cancel_process_timer()
        self._process_timer = self.create_timer(
            0.02, lambda ctx=context: self._paint_entry_tick(ctx)
        )
        self._set_contact_collision_allowed(
            True,
            lambda success, ctx=context: self._paint_entry_acm_done(
                ctx, success
            ),
        )
        self._paint_entry_tick(context)

    def _start_process_ramp(self, step):
        mode = step.mode
        if mode == "RAMP_UP" and not self._contact_search_confirmed:
            self._fail_workflow_known_safe(
                "RAMP_UP before CONTACT_SEARCH success"
            )
            return
        if self.painting_force_enabled:
            if mode == "RAMP_UP":
                blockers = MoveItExecutor._force_guard_status_blockers(
                    self,
                    expected_mode="CONTACT_SEARCH",
                    active=False,
                    command_context=self._contact_search_command_context,
                )
            else:
                blockers = MoveItExecutor._force_guard_status_blockers(
                    self,
                    expected_mode=self._painting_command_mode,
                    active=True,
                    command_context=self._painting_command_context,
                )
            if blockers:
                if self._painting_command_enable:
                    self._request_motion_abort(
                        f"{mode}_ENTRY_GUARD_INVALID:" + ",".join(blockers)
                    )
                else:
                    self._fail_workflow_known_safe(
                        f"{mode} entry guard: " + ",".join(blockers)
                    )
                return
        self._paint_entry_context = None
        self._cancel_process_timer()
        self._transition_execution_state(mode, "force ramp")
        started_at = time.monotonic()
        ramp_status_edge = {
            "published_at_s": started_at,
            "status_sequence": int(self._painting_ramp_status_sequence),
            "source_timestamp_s": float(
                self._painting_ramp_status_source_time
            ),
        }
        ramp_command_context = self._publish_painting_command(
            mode, step.force_n
        )

        if not self.painting_force_enabled or (
            mode == "RAMP_UP" and step.force_n <= 1e-9
        ):
            self._process_force_ready = mode == "RAMP_UP"
            self._schedule_process_once(
                self.painting_ramp_settle_s, self._complete_process_step
            )
            return

        lease_context = MoveItExecutor._set_force_phase_lease_command(
            self,
            ramp_command_context,
            mode=mode,
            active=True,
            step=step,
        )
        if lease_context is None or self._motion_abort_requested:
            return

        self._cancel_process_timer()
        ramp_down_state = {
            "disable_requested": False,
            "disable_requested_at": 0.0,
            "disable_command_context": None,
        }
        ramp_state = {"guard_acknowledged": False}

        def fail_active_guard(reason, blockers):
            self._cancel_process_timer()
            self._request_motion_abort(
                f"{reason}:" + ",".join(blockers)
            )

        def wait_for_ramp():
            if self._motion_abort_requested:
                self._cancel_process_timer()
                return
            if (
                self._process_step_index >= len(self._process_steps)
                or self._process_steps[self._process_step_index] is not step
            ):
                self._cancel_process_timer()
                return
            now = time.monotonic()
            ramp_status = self._painting_ramp_status
            feedback_is_fresh = bool(
                isinstance(ramp_status, dict)
                and self._painting_ramp_status_sequence
                > ramp_status_edge["status_sequence"]
                and self._painting_ramp_status_source_time
                > max(
                    ramp_status_edge["published_at_s"],
                    ramp_status_edge["source_timestamp_s"],
                )
                and self._painting_ramp_status_time > started_at
                and 0.0
                <= now - self._painting_ramp_status_source_time
                <= self.force_guard_status_timeout_s
                and str(ramp_status.get("mode", "")).strip().upper()
                == mode
                and ramp_status.get("force_enable") is True
            )
            ramp_done = bool(
                feedback_is_fresh
                and ramp_status.get("ramp_complete") is True
            )
            if mode == "RAMP_DOWN":
                if not ramp_down_state["disable_requested"]:
                    guard_blockers = MoveItExecutor._force_guard_status_blockers(
                        self,
                        expected_mode="RAMP_DOWN",
                        active=True,
                        command_context=ramp_command_context,
                        now=now,
                    )
                    if guard_blockers:
                        if (
                            ramp_state["guard_acknowledged"]
                            or MoveItExecutor._guard_ack_fault_is_immediate(
                                guard_blockers
                            )
                            or now - started_at
                            > self.force_guard_mode_ack_timeout_s
                        ):
                            fail_active_guard(
                                "RAMP_DOWN_GUARD_ACTIVE_ACK_FAILED",
                                guard_blockers,
                            )
                        return
                    ramp_state["guard_acknowledged"] = True
                    lease_context["acknowledged"] = True
                disable_context = ramp_down_state["disable_command_context"]
                action = ramp_down_handshake_action(
                    ramp_complete=ramp_done,
                    disable_requested=ramp_down_state["disable_requested"],
                    now_s=now,
                    disable_requested_at_s=ramp_down_state[
                        "disable_requested_at"
                    ],
                    guard_status_received_at_s=self._guard_status_time,
                    guard_forwarding=self._guard_status.get("forwarding"),
                    compliance_enabled=self._guard_status.get(
                        "compliance_enabled"
                    ),
                    guard_compliance_active=self._guard_status.get(
                        "compliance_active"
                    ),
                    guard_mode=str(
                        self._guard_status.get("mode", "")
                    ).strip().upper(),
                    guard_force_enable=self._guard_status.get(
                        "force_enable"
                    ),
                    guard_status_sequence=self._guard_status_sequence,
                    disable_requested_guard_sequence=(
                        disable_context["guard_status_sequence"]
                        if disable_context is not None
                        else None
                    ),
                    guard_source_timestamp_s=self._guard_status_source_time,
                    disable_requested_guard_source_timestamp_s=(
                        max(
                            float(disable_context["published_at_s"]),
                            float(
                                disable_context[
                                    "guard_source_timestamp_s"
                                ]
                            ),
                        )
                        if disable_context is not None
                        else None
                    ),
                    guard_timeout_s=self.force_guard_status_timeout_s,
                )
                if action == "DISABLE":
                    # Phase 2: after the slew reaches exactly zero, remove
                    # force/compliance enable and require a subsequent guard
                    # status that confirms zero is actually being published.
                    disable_context = self._publish_painting_command(
                        "RAMP_DOWN", 0.0, enable=False
                    )
                    ramp_down_state["disable_requested"] = True
                    ramp_down_state["disable_requested_at"] = float(
                        disable_context["published_at_s"]
                    )
                    ramp_down_state["disable_command_context"] = (
                        disable_context
                    )
                    MoveItExecutor._set_force_phase_lease_command(
                        self,
                        disable_context,
                        mode="RAMP_DOWN",
                        active=False,
                        step=step,
                    )
                    return
                ramp_done = action == "COMPLETE"
                if (
                    action == "WAIT_GUARD_ZERO_ACK"
                    and now
                    - float(ramp_down_state["disable_requested_at"])
                    > self.force_guard_mode_ack_timeout_s
                ):
                    self._cancel_process_timer()
                    self._request_motion_abort(
                        "RAMP_DOWN_GUARD_ZERO_ACK_TIMEOUT"
                    )
                    return
                if action == "WAIT_GUARD_ZERO_ACK":
                    return
            else:
                guard_blockers = MoveItExecutor._force_guard_status_blockers(
                    self,
                    expected_mode="RAMP_UP",
                    active=True,
                    command_context=ramp_command_context,
                    now=now,
                )
                if guard_blockers:
                    if (
                        ramp_state["guard_acknowledged"]
                        or MoveItExecutor._guard_ack_fault_is_immediate(
                            guard_blockers
                        )
                        or now - started_at
                        > self.force_guard_mode_ack_timeout_s
                    ):
                        fail_active_guard(
                            "RAMP_UP_GUARD_ACTIVE_ACK_FAILED",
                            guard_blockers,
                        )
                    return
                ramp_state["guard_acknowledged"] = True
                lease_context["acknowledged"] = True
            contact_ok = True
            if (
                mode == "RAMP_UP"
                and self.require_contact_before_paint
                and step.force_n > 0.0
            ):
                contact_ok = (
                    self._painting_contact_confirmed
                    and self._painting_contact_feedback_time >= started_at - 0.2
                )
            if ramp_done and contact_ok:
                if mode == "RAMP_DOWN":
                    disable_context = ramp_down_state[
                        "disable_command_context"
                    ]
                    if not MoveItExecutor._finish_force_phase_lease_after_zero_ack(
                        self, disable_context
                    ):
                        self._request_motion_abort(
                            "RAMP_DOWN_FORCE_LEASE_ZERO_ACK_INVALID"
                        )
                        return
                    escape = getattr(self, "_contact_escape_context", None)
                    if escape is not None:
                        if (
                            escape.get("ramp_step_index")
                            != self._process_step_index
                            or escape.get("execution_snapshot")
                            is not getattr(self, "_execution_snapshot", None)
                        ):
                            self._request_motion_abort(
                                "CONTACT_ESCAPE_RAMP_CONTEXT_INVALID"
                            )
                            return
                        escape["zero_ack_verified"] = True
                        escape["phase"] = "ZERO_ACK_VERIFIED"
                self._cancel_process_timer()
                self._process_force_ready = mode == "RAMP_UP"
                self._schedule_process_once(
                    self.painting_ramp_settle_s, self._complete_process_step
                )
                return
            if now - started_at > self.painting_ramp_feedback_timeout_s:
                detail = (
                    "contact not confirmed"
                    if ramp_done and not contact_ok
                    else "force slew completion feedback missing"
                )
                self._cancel_process_timer()
                self._fail_workflow_known_safe(
                    f"{mode} timeout: {detail}"
                )

        self._process_timer = self.create_timer(0.02, wait_for_ramp)

    def _apply_contact_depth_correction(self, step, poses):
        """Move PAINT waypoints onto the wall CONTACT_SEARCH actually found.

        The row poses come from the D405 plane fit.  CONTACT_SEARCH then
        measures where the wall really is, but that measurement used to be
        discarded: PAINT was still commanded to the plane.  When the real
        surface sits in front of the fitted plane the commanded path is
        *behind* the wall, so the trajectory drives the roller into it and
        the contact force climbs as the stroke advances (2026-08-14: plane
        at x=0.524, contact found at x=0.521, i.e. 3 mm of commanded
        over-penetration, force ramping from 3 N to over 40 N within one
        second of the traverse starting).

        The search begins precontact_clearance_m off the fitted surface, so
        the signed error is (clearance - travelled).  Positive means the wall
        is nearer than the plane and the waypoints must back off along the
        outward normal; negative means it is further and they must advance.
        """

        correction_m = float(self.precontact_clearance_m) - float(
            self._contact_search_distance_m
        )
        if not math.isfinite(correction_m):
            return
        # Never let a bad search silently reshape the path: the correction can
        # only ever be as large as the clearance the search was allowed to
        # consume.
        bound = float(self.precontact_clearance_m)
        correction_m = max(-bound, min(bound, correction_m))
        if abs(correction_m) < 1e-5:
            return
        try:
            normal = np.asarray(step.rows[-1].normal, dtype=float)
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        norm = float(np.linalg.norm(normal))
        if not math.isfinite(norm) or norm < 1e-9:
            return
        normal = normal / norm
        # _plan_contact_search_step approaches the wall by SUBTRACTING the
        # normal, so the normal points away from the surface: adding a
        # positive correction backs the waypoints off the wall.
        for pose in poses:
            pose.position.x += float(correction_m * normal[0])
            pose.position.y += float(correction_m * normal[1])
            pose.position.z += float(correction_m * normal[2])
        self.get_logger().info(
            "[PAINT DEPTH] contact search travelled %.4f m of %.4f m clearance "
            "-> shifting %d PAINT waypoint(s) %+.4f m along the surface normal"
            % (
                self._contact_search_distance_m,
                self.precontact_clearance_m,
                len(poses),
                correction_m,
            )
        )

    def _plan_process_motion_step(self, step):
        force_committed = bool(
            step.mode == "PAINT"
            and getattr(self, "painting_force_enabled", False)
            and getattr(self, "_painting_command_enable", False)
        )
        try:
            service_ready = bool(self.cartesian_client.service_is_ready())
        except (AttributeError, RuntimeError):
            service_ready = False
        if not service_ready:
            service_ready = bool(
                self.cartesian_client.wait_for_service(timeout_sec=0.0)
            )
        if not service_ready:
            if force_committed or getattr(self, "_contact_escape_context", None):
                self._request_motion_abort(
                    f"{step.mode}_CARTESIAN_SERVICE_UNAVAILABLE"
                )
            else:
                self._fail_workflow_known_safe(
                    "/compute_cartesian_path unavailable"
                )
            return
        poses = [
            copy.deepcopy(self._process_row_tcp_poses[row.row_number])
            for row in step.rows
        ]
        if not poses:
            self._complete_process_step()
            return
        # The escape clearance is measured from the PAINT end pose, so the
        # retract target has to ride the same correction; otherwise shifting
        # only PAINT shortens the escape by exactly the correction and trips
        # the bounded-outward check (observed 2026-08-14: a +3 mm correction
        # left 7 mm of escape against an 8 mm floor).  Clearance for these
        # moves should be referenced to the real wall anyway, not the plane.
        if step.mode in {"PAINT", "RETRACT", "FINAL_RETRACT"}:
            MoveItExecutor._apply_contact_depth_correction(self, step, poses)
        contact_escape_context = None
        if step.mode in {"RETRACT", "FINAL_RETRACT"}:
            candidate = getattr(self, "_contact_escape_context", None)
            if candidate is not None:
                error = MoveItExecutor._contact_escape_step_error(
                    self, candidate, step, poses
                )
                if error:
                    self._request_motion_abort(error)
                    return
                contact_escape_context = candidate
        # A commissioning path represents a stationary contact hold with one
        # PAINT pose (or repeated identical poses). CONTACT_SEARCH can finish up
        # to one search step away from that nominal pose, so classify the path
        # geometry here and hold after any required final sub-millimetre motion.
        first_paint_position = np.array(
            [
                poses[0].position.x,
                poses[0].position.y,
                poses[0].position.z,
            ],
            dtype=float,
        )
        stationary_paint_hold = bool(
            step.mode == "PAINT"
            and self.stationary_paint_hold_s > 0.0
            and all(
                float(
                    np.linalg.norm(
                        np.array(
                            [pose.position.x, pose.position.y, pose.position.z],
                            dtype=float,
                        )
                        - first_paint_position
                    )
                )
                < 1e-6
                for pose in poses
            )
        )
        if self._process_last_tcp_pose is not None:
            cursor = np.array(
                [
                    self._process_last_tcp_pose.position.x,
                    self._process_last_tcp_pose.position.y,
                    self._process_last_tcp_pose.position.z,
                ],
                dtype=float,
            )
            path_distance = 0.0
            for pose in poses:
                target = np.array(
                    [pose.position.x, pose.position.y, pose.position.z], dtype=float
                )
                path_distance += float(np.linalg.norm(target - cursor))
                cursor = target
            if path_distance < 1e-6:
                if stationary_paint_hold:
                    self._start_stationary_paint_hold(step, poses[-1])
                    return
                self._process_motion_done(poses[-1])
                return

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        rs = RobotState()
        seed_state = copy.deepcopy(self.current_joint_state)
        rs.joint_state = copy.deepcopy(seed_state)
        rs.is_diff = False
        req.start_state = rs
        req.waypoints = poses
        req.max_step = 0.003 if step.mode in CONTACT_MOTION_MODES else 0.005
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        requested_speed = step.speed_mps if step.speed_mps > 0.0 else 0.01
        velocity_scale = max(0.01, min(1.0, requested_speed / 0.10))
        req.max_velocity_scaling_factor = velocity_scale
        req.max_acceleration_scaling_factor = velocity_scale

        token = object()
        self._segment_cartesian_context = {
            "token": token,
            "step_index": self._process_step_index,
            "step": step,
            "poses": poses,
            "stationary_paint_hold": stationary_paint_hold,
            "seed_state": seed_state,
            "scene_revision": int(getattr(self, "_scene_revision", 0)),
            "execution_snapshot": getattr(self, "_execution_snapshot", None),
            "active_path": getattr(self, "_active_segment_path", None),
            "paint_entry_context": (
                self._paint_entry_context if step.mode == "PAINT" else None
            ),
            "paint_command_context": (
                self._painting_command_context
                if step.mode == "PAINT"
                else None
            ),
            "contact_escape_context": contact_escape_context,
            "planning_started_at_s": time.monotonic(),
            "timeout_timer": None,
        }
        context = self._segment_cartesian_context
        try:
            future = self.cartesian_client.call_async(req)
        except Exception as exc:
            self._segment_cartesian_context = None
            if force_committed or contact_escape_context is not None:
                self._request_motion_abort(
                    f"{step.mode}_CARTESIAN_REQUEST_FAILED:{exc}"
                )
            else:
                self._fail_workflow_known_safe(
                    f"{step.mode} cartesian request failed: {exc}"
                )
            return
        MoveItExecutor._arm_segment_cartesian_timeout(self, context)
        future.add_done_callback(
            lambda done, expected_token=token: self._process_cartesian_done(
                done, expected_token
            )
        )

    def _process_cartesian_done(self, future, expected_token):
        context = self._segment_cartesian_context
        if context is None or self._motion_abort_requested:
            return
        if context["token"] is not expected_token:
            return
        step = context["step"]
        if context["step_index"] != self._process_step_index:
            return
        now = time.monotonic()
        try:
            planning_deadline_s = float(context["planning_deadline_s"])
        except (KeyError, TypeError, ValueError, OverflowError):
            planning_deadline_s = math.nan
        if (
            not math.isfinite(planning_deadline_s)
            or now > planning_deadline_s
        ):
            MoveItExecutor._cancel_segment_cartesian_timeout(self, context)
            self._segment_cartesian_context = None
            reason = f"{step.mode}_CARTESIAN_DEADLINE_EXCEEDED"
            if (
                step.mode == "PAINT"
                or context.get("contact_escape_context") is not None
            ):
                self._request_motion_abort(reason)
            else:
                self._fail_workflow_known_safe(reason)
            return
        MoveItExecutor._cancel_segment_cartesian_timeout(self, context)

        def reject_before_dispatch(reason):
            if (
                step.mode == "PAINT"
                or context.get("contact_escape_context") is not None
            ):
                self._request_motion_abort(str(reason))
            else:
                self._fail_workflow_known_safe(str(reason))

        try:
            response = future.result()
        except Exception as exc:
            reject_before_dispatch(
                f"{step.mode} cartesian planning service failed: {exc}"
            )
            return
        response_error = MoveItExecutor._cartesian_response_error(
            response, CARTESIAN_COMPLETE_FRACTION
        )
        if response_error:
            reject_before_dispatch(
                f"{step.mode} cartesian planning rejected:{response_error}"
            )
            return

        trajectory = self._limit_trajectory_to_cartesian_speed(
            response.solution,
            self._process_last_tcp_pose,
            context["poses"],
            step.speed_mps,
        )
        label = f"PAINT PROCESS {self._process_step_index + 1} {step.mode}"
        start_ok, start_reason = self._d405_plan_start_matches_measured(
            trajectory,
            context["seed_state"],
            max_age_s=1.0,
        )
        if not start_ok:
            reject_before_dispatch(
                f"{label}:stale plan start:{start_reason}"
            )
            return
        if (
            self._segment_cartesian_context is not context
            or context["step_index"] != self._process_step_index
            or getattr(self, "_execution_snapshot", None)
            is not context["execution_snapshot"]
            or getattr(self, "_active_segment_path", None)
            is not context["active_path"]
            or int(getattr(self, "_scene_revision", 0))
            != int(context["scene_revision"])
            or not getattr(self, "scene_confirmed", False)
            or int(getattr(self, "_scene_confirmed_revision", -1))
            != int(context["scene_revision"])
        ):
            reject_before_dispatch(
                f"{label}:planning context changed before dispatch"
            )
            return
        if self.real_painting_enabled:
            plan_blockers = self._current_real_plan_blockers()
            if plan_blockers:
                reject_before_dispatch(
                    f"{label}:plan identity:" + ",".join(plan_blockers)
                )
                return
        contact_escape_context = context.get("contact_escape_context")
        if contact_escape_context is not None:
            escape_error = MoveItExecutor._contact_escape_step_error(
                self, contact_escape_context, step, context["poses"]
            )
            if escape_error:
                self._request_motion_abort(escape_error)
                return
            contact_escape_context["phase"] = "ESCAPE_DISPATCH"
        if step.mode == "PAINT" and self.painting_force_enabled:
            paint_entry_context = context.get("paint_entry_context")
            if not self._paint_entry_is_current(paint_entry_context):
                self._request_motion_abort(
                    f"{label}:PAINT entry context changed before dispatch"
                )
                return
            guard_blockers = MoveItExecutor._force_guard_status_blockers(
                self,
                expected_mode="PAINT",
                active=True,
                command_context=context.get("paint_command_context"),
            )
            if guard_blockers:
                self._request_motion_abort(
                    f"{label}:PAINT guard lost before dispatch:"
                    + ",".join(guard_blockers)
                )
                return
            self._cancel_process_timer()
            self._paint_entry_context = None
        self._segment_cartesian_context = None
        if getattr(self, "process_mode", "paint") == "spray":
            self._spray_dispatch = step.mode == "SPRAY"
        if not self.execute_trajectory_direct(
            trajectory,
            on_complete=lambda: self._process_motion_target_reached(
                step,
                context["poses"][-1],
                context["stationary_paint_hold"],
            ),
            on_failure=lambda: self._handle_process_trajectory_failure(
                label
            ),
            on_rejected=lambda: self._request_motion_abort(
                f"{label}:CONTACT_ESCAPE_FJT_GOAL_REJECTED"
            )
            if contact_escape_context is not None
            else self._fail_workflow_known_safe(
                f"{label}:FJT goal rejected"
            ),
            force_guard=True,
            label=label,
            requires_contact_acm=(
                step.mode == "PAINT" or contact_escape_context is not None
            ),
            contact_acm_context=contact_escape_context,
        ):
            if contact_escape_context is not None:
                self._request_motion_abort(
                    f"{label}:CONTACT_ESCAPE_NOT_DISPATCHED"
                )
            else:
                self._handle_known_dispatch_rejection(label)

    def _process_motion_target_reached(
        self, step, final_pose, stationary_paint_hold=False
    ):
        if self._motion_abort_requested:
            return
        if stationary_paint_hold and step.mode == "PAINT":
            self._process_last_tcp_pose = copy.deepcopy(final_pose)
            self._start_stationary_paint_hold(step, final_pose)
            return
        self._process_motion_done(final_pose)

    def _start_stationary_paint_hold(self, step, final_pose):
        """Hold a zero-distance PAINT row while checking every force dependency."""

        duration_s = float(self.stationary_paint_hold_s)
        self._stationary_paint_hold_started_at = time.monotonic()
        self._stationary_paint_hold_samples = []
        self._transition_execution_state(
            "PAINT_HOLD",
            "zero-distance commissioning hold",
        )

        def poll_hold():
            if self._motion_abort_requested:
                return
            now = time.monotonic()
            decision = evaluate_stationary_paint_hold(
                elapsed_s=now - self._stationary_paint_hold_started_at,
                duration_s=duration_s,
                now_s=now,
                safety_status_received_at_s=self._safety_status_time,
                guard_status_received_at_s=self._guard_status_time,
                contact_received_at_s=self._painting_contact_feedback_time,
                safety_timeout_s=self.ft_required_timeout_s,
                guard_timeout_s=0.5,
                contact_timeout_s=self.ft_required_timeout_s,
                abort_requested=self._motion_abort_requested,
                safety_abort_latched=self._safety_status.get(
                    "abort_latched"
                ),
                ft_valid=self._safety_status.get("ft_valid"),
                tf_valid=self._safety_status.get("tf_valid"),
                guard_forwarding=self._guard_status.get("forwarding"),
                guard_compliance_enabled=self._guard_status.get(
                    "compliance_enabled"
                ),
                guard_compliance_active=self._guard_status.get(
                    "compliance_active"
                ),
                guard_controller_fault=self._guard_status.get(
                    "controller_fault"
                ),
                contact_confirmed=self._painting_contact_confirmed,
            )
            if decision.action == "FAULT":
                self._request_motion_abort(decision.fault_reason)
                return

            filtered = self._safety_status.get("filtered_wrench")
            try:
                normal_force_n = float(filtered[1])
            except (IndexError, TypeError, ValueError):
                normal_force_n = math.nan
            if not math.isfinite(normal_force_n):
                self._request_motion_abort("PAINT_HOLD_FORCE_INVALID")
                return
            self._stationary_paint_hold_samples.append(normal_force_n)

            if decision.action == "COMPLETE":
                samples = tuple(self._stationary_paint_hold_samples)
                self._publish_execution_status(
                    "PAINT_HOLD_COMPLETE",
                    "stationary force hold verified",
                    hold_duration_s=duration_s,
                    target_force_n=float(step.force_n),
                    normal_force_min_n=min(samples),
                    normal_force_max_n=max(samples),
                    normal_force_mean_n=sum(samples) / len(samples),
                    normal_force_sample_count=len(samples),
                )
                self._stationary_paint_hold_started_at = 0.0
                self._stationary_paint_hold_samples = []
                self._process_motion_done(final_pose)
                return
            self._schedule_process_once(0.05, poll_hold)

        self._schedule_process_once(0.05, poll_hold)

    def _publish_free_space_confirmation(self, confirmed):
        msg = Bool()
        msg.data = bool(confirmed)
        self._free_space_confirmed = msg.data
        self._free_space_confirmed_time = time.monotonic()
        self.free_space_confirmed_pub.publish(msg)

    def _cancel_runtime_tare_timer(self):
        if self._runtime_tare_timer is not None:
            self._runtime_tare_timer.cancel()
            self.destroy_timer(self._runtime_tare_timer)
            self._runtime_tare_timer = None

    def _cancel_runtime_tare(self, reset_phase=True):
        self._cancel_runtime_tare_timer()
        self._runtime_tare_token = None
        self._runtime_tare_quiet_started_at = 0.0
        self._runtime_tare_verify_started_at = 0.0
        self._runtime_tare_context = None
        self._runtime_tare_actual_metrics = {}
        if reset_phase:
            self._runtime_tare_phase = "IDLE"

    def _runtime_tare_fail(self, reason):
        if self._motion_abort_requested:
            return
        self._runtime_tare_phase = "FAILED"
        self._cancel_runtime_tare_timer()
        self._runtime_tare_token = None
        self._execution_tare_ready = False
        self._publish_free_space_confirmation(False)
        self._fail_workflow_known_safe(
            f"PRECONTACT_TARE:{reason}",
            state="PRECONTACT_TARE_FAILED",
            tare_failure_reason=str(reason),
            tare_phase="FAILED",
        )

    def _begin_runtime_tare_before_contact(self, step, planned_tcp_pose):
        """Tare at a stopped, execution-scoped pre-contact pose.

        The hardware service owns raw-sample stationarity and corrected
        residual checks.  This executor additionally enforces the workflow
        interlocks around it and waits for the independent force monitor and
        wrench guard to become ready before CONTACT_SEARCH is allowed to move.
        """
        if not self.painting_force_enabled:
            self._complete_process_step()
            return
        if not self.runtime_tare_enabled:
            self._runtime_tare_fail("runtime tare is disabled")
            return
        if not self._execution_free_space_confirmed:
            self._runtime_tare_fail(
                "execution did not latch operator free-space confirmation"
            )
            return
        try:
            clearance_m = min(float(row.offset_m) for row in step.rows)
        except (AttributeError, TypeError, ValueError):
            clearance_m = math.nan
        if (
            not math.isfinite(clearance_m)
            or clearance_m + 1e-9 < self.runtime_tare_min_clearance_m
        ):
            self._runtime_tare_fail(
                "APPROACH_PRECONTACT clearance %.4fm is below runtime tare "
                "minimum %.4fm"
                % (clearance_m, self.runtime_tare_min_clearance_m)
            )
            return
        if not self._runtime_tare_services_ready():
            self._runtime_tare_fail("required runtime tare/reset service unavailable")
            return

        self._cancel_runtime_tare()
        path = self._active_segment_path
        if path is None or getattr(path, "version", 0) < 3:
            self._runtime_tare_fail("active immutable v3 plan unavailable")
            return
        try:
            plane_point, plane_normal = self._active_surface_plane()
            plane_point = np.asarray(plane_point, dtype=float)
            plane_normal = np.asarray(plane_normal, dtype=float)
            normal_norm = float(np.linalg.norm(plane_normal))
            if (
                plane_point.shape != (3,)
                or plane_normal.shape != (3,)
                or not np.all(np.isfinite(plane_point))
                or not np.all(np.isfinite(plane_normal))
                or not math.isfinite(normal_norm)
                or normal_norm < 1e-9
            ):
                raise ValueError("accepted active plane is non-finite")
            plane_normal = plane_normal / normal_norm
        except (TypeError, ValueError) as exc:
            self._runtime_tare_fail(f"accepted active plane invalid: {exc}")
            return
        self._runtime_tare_context = {
            "planned_tcp_pose": copy.deepcopy(planned_tcp_pose),
            "plane_point": plane_point.copy(),
            "plane_normal": plane_normal.copy(),
            "contact_geometry_offset_m": float(
                path.contact_geometry_offset_m
            ),
            "path_id": str(path.path_id),
            "plan_hash": str(path.plan_hash),
            "work_area_id": str(path.work_area_id),
            "plane_generation_id": str(path.plane_generation_id),
        }
        self._publish_painting_command("IDLE", 0.0, enable=False)
        self._publish_free_space_confirmation(False)
        now = time.monotonic()
        self._runtime_tare_token = object()
        self._runtime_tare_started_at = now
        self._runtime_tare_phase_started_at = now
        self._runtime_tare_phase = "SETTLING"
        self._execution_tare_ready = False
        pose_ok, pose_error, metrics = self._runtime_tare_actual_pose_guard()
        if not pose_ok:
            self._runtime_tare_fail(pose_error)
            return
        self._runtime_tare_actual_metrics = metrics
        self._transition_execution_state(
            "PRECONTACT_TARE",
            "10 mm free-space pose reached; waiting for stationary interlocks",
        )
        self._runtime_tare_timer = self.create_timer(
            0.05, self._runtime_tare_tick
        )

    def _runtime_tare_call(self, client, pending_phase, done_callback):
        token = self._runtime_tare_token
        if token is None or self._motion_abort_requested:
            return
        if not client.service_is_ready():
            self._runtime_tare_fail(f"{pending_phase} service unavailable")
            return
        self._runtime_tare_phase = pending_phase
        self._runtime_tare_phase_started_at = time.monotonic()
        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            self._runtime_tare_fail(f"{pending_phase} request failed: {exc}")
            return
        future.add_done_callback(
            lambda completed, expected=token: done_callback(completed, expected)
        )

    def _runtime_tare_trigger_result(self, future, expected_token, operation):
        if (
            expected_token is not self._runtime_tare_token
            or self._motion_abort_requested
        ):
            return None
        try:
            response = future.result()
        except Exception as exc:
            self._runtime_tare_fail(f"{operation} service failed: {exc}")
            return None
        if response is None or not bool(response.success):
            detail = "no response" if response is None else str(response.message)
            self._runtime_tare_fail(f"{operation} rejected: {detail}")
            return None
        return response

    def _runtime_tare_identity_error(self):
        """Return why the immutable accepted plan/plane identity changed."""

        context = self._runtime_tare_context
        active = self._active_segment_path
        snapshot = getattr(self, "_execution_snapshot", None)
        source = (
            snapshot.get("segment_path")
            if isinstance(snapshot, dict)
            else self._segment_path
        )
        if not isinstance(context, dict):
            return "runtime tare geometry context unavailable"
        if active is None or source is None:
            return "active/source segment plan unavailable"
        expected = {
            "path_id": str(context.get("path_id", "")),
            "plan_hash": str(context.get("plan_hash", "")),
            "work_area_id": str(context.get("work_area_id", "")),
            "plane_generation_id": str(
                context.get("plane_generation_id", "")
            ),
        }
        if not all(expected.values()):
            return "runtime tare plan identity contains an empty field"
        for label, path in (("active", active), ("snapshot", source)):
            if getattr(path, "version", 0) < 3:
                return f"{label} segment is not immutable v3"
            for field, value in expected.items():
                if str(getattr(path, field, "")) != value:
                    return f"{label} segment {field} changed"
        # Once Run is accepted, live D405/work-area/plan topics describe only
        # a future candidate.  They are intentionally excluded from the
        # current execution identity.  The frozen path and tare context above
        # remain authoritative while actual TCP/force/controller checks keep
        # running fail-closed.
        if isinstance(snapshot, dict):
            for field, value in expected.items():
                if str(snapshot.get(field, "")) != value:
                    return f"execution snapshot {field} changed"
            return ""

        live = {
            "path_id": str(self._waypoints_path_id),
            "accepted_path_id": str(self._accepted_plan_path_id),
            "plan_hash": str(self._accepted_plan_hash),
            "work_area_id": str(self._current_work_area_id),
            "plane_generation_id": str(self._current_plane_generation_id),
        }
        comparisons = (
            ("waypoint path_id", live["path_id"], expected["path_id"]),
            (
                "accepted path_id",
                live["accepted_path_id"],
                expected["path_id"],
            ),
            ("accepted plan_hash", live["plan_hash"], expected["plan_hash"]),
            (
                "work_area_id",
                live["work_area_id"],
                expected["work_area_id"],
            ),
            (
                "plane_generation_id",
                live["plane_generation_id"],
                expected["plane_generation_id"],
            ),
        )
        for label, actual, expected_value in comparisons:
            if actual != expected_value:
                return f"{label} changed"
        if self._d405_plane_accepted is not True:
            return "accepted D405 plane invalidated"
        return ""

    def _runtime_tare_actual_pose_guard(self):
        """Verify actual roller clearance and final-goal tracking before tare."""

        identity_error = self._runtime_tare_identity_error()
        if identity_error:
            return False, f"plan/plane identity: {identity_error}", {}
        context = self._runtime_tare_context
        actual, tf_error = self._runtime_tare_tcp_pose_sample()
        if actual is None:
            return False, tf_error or "actual TCP TF unavailable", {}
        try:
            tcp_position = np.asarray(actual[0], dtype=float)
            tcp_quaternion = np.asarray(actual[1], dtype=float)
            tcp_tf_age_s = float(actual[2])
            plane_point = np.asarray(context["plane_point"], dtype=float)
            plane_normal = np.asarray(context["plane_normal"], dtype=float)
            planned = context["planned_tcp_pose"]
            planned_position = np.array(
                [
                    planned.position.x,
                    planned.position.y,
                    planned.position.z,
                ],
                dtype=float,
            )
            planned_quaternion = np.array(
                [
                    planned.orientation.x,
                    planned.orientation.y,
                    planned.orientation.z,
                    planned.orientation.w,
                ],
                dtype=float,
            )
            contact_geometry_m = float(context["contact_geometry_offset_m"])
        except (AttributeError, KeyError, TypeError, ValueError):
            return False, "runtime tare pose context malformed", {}
        arrays = (
            tcp_position,
            tcp_quaternion,
            plane_point,
            plane_normal,
            planned_position,
            planned_quaternion,
        )
        if (
            tcp_position.shape != (3,)
            or tcp_quaternion.shape != (4,)
            or plane_point.shape != (3,)
            or plane_normal.shape != (3,)
            or planned_position.shape != (3,)
            or planned_quaternion.shape != (4,)
            or not all(np.all(np.isfinite(values)) for values in arrays)
            or not math.isfinite(contact_geometry_m)
            or contact_geometry_m < 0.0
            or not math.isfinite(tcp_tf_age_s)
        ):
            return False, "actual/planned tare geometry is non-finite", {}
        actual_q_norm = float(np.linalg.norm(tcp_quaternion))
        planned_q_norm = float(np.linalg.norm(planned_quaternion))
        normal_norm = float(np.linalg.norm(plane_normal))
        if min(actual_q_norm, planned_q_norm, normal_norm) < 1e-9:
            return False, "tare quaternion/plane normal has zero norm", {}
        tcp_quaternion /= actual_q_norm
        planned_quaternion /= planned_q_norm
        plane_normal /= normal_norm

        tool_axis = np.asarray(
            quat_apply(tcp_quaternion, [0.0, -1.0, 0.0]), dtype=float
        )
        if tool_axis.shape != (3,) or not np.all(np.isfinite(tool_axis)):
            return False, "actual TCP tool axis is non-finite", {}
        roller_center = tcp_position + tool_axis * EOAT_TIP_OFFSET
        actual_clearance_m = float(
            np.dot(roller_center - plane_point, plane_normal)
            - contact_geometry_m
        )
        position_error_m = float(
            np.linalg.norm(tcp_position - planned_position)
        )
        quaternion_dot = float(
            np.clip(abs(np.dot(tcp_quaternion, planned_quaternion)), 0.0, 1.0)
        )
        orientation_error_deg = math.degrees(
            2.0 * math.acos(quaternion_dot)
        )
        tool_alignment = float(
            np.clip(np.dot(tool_axis, -plane_normal), -1.0, 1.0)
        )
        tool_axis_error_deg = math.degrees(math.acos(tool_alignment))
        metrics = {
            "actual_clearance_m": actual_clearance_m,
            "tcp_position_error_m": position_error_m,
            "tcp_orientation_error_deg": orientation_error_deg,
            "tool_axis_normal_error_deg": tool_axis_error_deg,
            "tcp_tf_age_s": tcp_tf_age_s,
        }
        if not all(math.isfinite(value) for value in metrics.values()):
            return False, "computed runtime tare geometry is non-finite", metrics
        if actual_clearance_m < self.runtime_tare_min_actual_clearance_m:
            return (
                False,
                "actual roller-wall clearance %.4fm is below %.4fm"
                % (
                    actual_clearance_m,
                    self.runtime_tare_min_actual_clearance_m,
                ),
                metrics,
            )
        if position_error_m > self.runtime_tare_max_tcp_position_error_m:
            return (
                False,
                "actual TCP position error %.4fm exceeds %.4fm"
                % (
                    position_error_m,
                    self.runtime_tare_max_tcp_position_error_m,
                ),
                metrics,
            )
        if (
            orientation_error_deg
            > self.runtime_tare_max_tcp_orientation_error_deg
        ):
            return (
                False,
                "actual TCP orientation error %.2fdeg exceeds %.2fdeg"
                % (
                    orientation_error_deg,
                    self.runtime_tare_max_tcp_orientation_error_deg,
                ),
                metrics,
            )
        if (
            tool_axis_error_deg
            > self.runtime_tare_max_tcp_orientation_error_deg
        ):
            return (
                False,
                "actual tool-axis/plane-normal error %.2fdeg exceeds %.2fdeg"
                % (
                    tool_axis_error_deg,
                    self.runtime_tare_max_tcp_orientation_error_deg,
                ),
                metrics,
            )
        return True, "", metrics

    def _runtime_hardware_tare_done(self, future, expected_token):
        response = self._runtime_tare_trigger_result(
            future, expected_token, "hardware runtime free-space tare"
        )
        if response is None:
            return
        pose_ok, pose_error, metrics = self._runtime_tare_actual_pose_guard()
        if not pose_ok:
            self._runtime_tare_fail(pose_error)
            return
        self._runtime_tare_actual_metrics = metrics
        self._runtime_tare_phase = "WAIT_FINITE_FT"
        self._runtime_tare_phase_started_at = time.monotonic()
        self._publish_execution_status(
            "PRECONTACT_TARE",
            "hardware tare accepted; waiting for fresh finite F/T",
            tare_phase=self._runtime_tare_phase,
            tare_message=str(response.message),
        )

    def _runtime_safety_reset_done(self, future, expected_token):
        response = self._runtime_tare_trigger_result(
            future, expected_token, "force safety reset"
        )
        if response is None:
            return
        # reset_safety intentionally invalidates the adaptive monitor bias.
        # Reassert free space and wait for a completely new bias window.
        self._runtime_tare_phase = "WAIT_MONITOR_BIAS"
        self._runtime_tare_phase_started_at = time.monotonic()
        self._runtime_tare_verify_started_at = 0.0
        self._publish_free_space_confirmation(True)
        self._publish_execution_status(
            "PRECONTACT_TARE",
            "safety latch reset; collecting fresh monitor bias",
            tare_phase=self._runtime_tare_phase,
        )

    def _runtime_guard_reset_done(self, future, expected_token):
        response = self._runtime_tare_trigger_result(
            future, expected_token, "wrench guard reset"
        )
        if response is None:
            return
        self._runtime_tare_phase = "WAIT_GUARD_STATUS"
        self._runtime_tare_phase_started_at = time.monotonic()
        self._publish_execution_status(
            "PRECONTACT_TARE",
            "guard reset accepted; waiting for fresh zero-output status",
            tare_phase=self._runtime_tare_phase,
        )

    def _runtime_tare_residual_ok(self, status):
        try:
            filtered = tuple(float(value) for value in status["filtered_wrench"])
            raw = tuple(float(value) for value in status["raw_wrench"])
        except (KeyError, TypeError, ValueError):
            return False
        if len(filtered) != 6 or len(raw) != 6:
            return False
        if not all(math.isfinite(value) for value in filtered + raw):
            return False
        for wrench in (filtered, raw):
            force_norm = math.sqrt(sum(value * value for value in wrench[:3]))
            torque_norm = math.sqrt(sum(value * value for value in wrench[3:]))
            if force_norm > self.runtime_tare_max_force_norm_n:
                return False
            if torque_norm > self.runtime_tare_max_torque_norm_nm:
                return False
        return True

    def _runtime_tare_tick(self):
        if self._runtime_tare_token is None or self._motion_abort_requested:
            self._cancel_runtime_tare_timer()
            return
        now = time.monotonic()
        if now - self._runtime_tare_started_at > self.runtime_tare_timeout_s:
            self._runtime_tare_fail(
                f"timeout in phase {self._runtime_tare_phase}"
            )
            return
        phase = self._runtime_tare_phase
        # The arm must remain at the same measured free-space pose throughout
        # hardware tare, monitor rebias, residual verification and both reset
        # handshakes.  A valid pose only at service dispatch is insufficient:
        # drift/contact while any async response is pending must fail closed.
        pose_ok, pose_error, metrics = self._runtime_tare_actual_pose_guard()
        if not pose_ok:
            self._runtime_tare_fail(pose_error)
            return
        self._runtime_tare_actual_metrics = metrics

        # The confirmation is execution-scoped and may only be asserted while
        # the arm is stopped with compliance and requested force both off.
        self._publish_painting_command("IDLE", 0.0, enable=False)
        moving = self._trajectory_command_active()
        stationary = self._robot_stationary_for_bias()
        guard_fresh = (
            self._guard_status_time > 0.0
            and now - self._guard_status_time <= 0.5
        )
        compliance_off = bool(
            guard_fresh
            and self._guard_status.get("compliance_active") is False
            and self._guard_status.get("compliance_enabled") is False
            and self._guard_status.get("forwarding") is False
        )
        if moving or not stationary or not compliance_off:
            self._runtime_tare_quiet_started_at = 0.0
            self._publish_free_space_confirmation(False)
            return
        self._publish_free_space_confirmation(True)

        if phase == "SETTLING":
            if self._runtime_tare_quiet_started_at <= 0.0:
                self._runtime_tare_quiet_started_at = now
                return
            if now - self._runtime_tare_quiet_started_at < self.runtime_tare_quiet_s:
                return
            self._runtime_tare_call(
                self.runtime_ft_tare_client,
                "HARDWARE_TARE_PENDING",
                self._runtime_hardware_tare_done,
            )
            return

        if phase == "WAIT_FINITE_FT":
            safety_fresh = (
                self._safety_status_time >= self._runtime_tare_phase_started_at
                and now - self._safety_status_time <= self.ft_required_timeout_s
            )
            if not safety_fresh:
                return
            if (
                self._safety_status.get("ft_valid") is True
                and self._safety_status.get("tf_valid") is True
            ):
                self._runtime_tare_call(
                    self.force_safety_reset_client,
                    "SAFETY_RESET_PENDING",
                    self._runtime_safety_reset_done,
                )
            return

        if phase == "WAIT_MONITOR_BIAS":
            safety_fresh = (
                self._safety_status_time >= self._runtime_tare_phase_started_at
                and now - self._safety_status_time <= self.ft_required_timeout_s
            )
            monitor_ready = bool(
                safety_fresh
                and self._safety_status.get("ft_valid") is True
                and self._safety_status.get("tf_valid") is True
                and self._safety_status.get("abort_latched") is False
                and self._safety_status.get("bias_ready") is True
            )
            if monitor_ready:
                self._runtime_tare_phase = "VERIFY_RESIDUAL"
                self._runtime_tare_phase_started_at = now
                self._runtime_tare_verify_started_at = 0.0
            return

        if phase == "VERIFY_RESIDUAL":
            safety_fresh = (
                self._safety_status_time >= self._runtime_tare_phase_started_at
                and now - self._safety_status_time <= self.ft_required_timeout_s
            )
            residual_ok = bool(
                safety_fresh
                and self._safety_status.get("ft_valid") is True
                and self._safety_status.get("tf_valid") is True
                and self._safety_status.get("abort_latched") is False
                and self._safety_status.get("bias_ready") is True
                and self._runtime_tare_residual_ok(self._safety_status)
            )
            if not residual_ok:
                self._runtime_tare_verify_started_at = 0.0
                return
            if self._runtime_tare_verify_started_at <= 0.0:
                self._runtime_tare_verify_started_at = now
                return
            if (
                now - self._runtime_tare_verify_started_at
                < self.runtime_tare_verify_duration_s
            ):
                return
            self._runtime_tare_call(
                self.wrench_guard_reset_client,
                "GUARD_RESET_PENDING",
                self._runtime_guard_reset_done,
            )
            return

        if phase == "WAIT_GUARD_STATUS":
            guard_ready = bool(
                self._guard_status_time >= self._runtime_tare_phase_started_at
                and now - self._guard_status_time <= 0.5
                and self._guard_status.get("controller_fault") is False
                and self._guard_status.get("compliance_active") is False
                and self._guard_status.get("compliance_enabled") is False
                and self._guard_status.get("forwarding") is False
            )
            if not guard_ready:
                return
            self._execution_tare_ready = True
            blockers = self._real_force_runtime_blockers()
            if blockers:
                self._execution_tare_ready = False
                self._runtime_tare_fail(
                    "post-tare force gate: " + ",".join(blockers)
                )
                return
            self._cancel_runtime_tare_timer()
            self._runtime_tare_token = None
            self._runtime_tare_phase = "COMPLETE"
            self._transition_execution_state(
                "PRECONTACT_TARE_COMPLETE",
                "fresh hardware/monitor bias and guard verified",
                **self._runtime_tare_actual_metrics,
            )
            self._complete_process_step()

    def _process_motion_done(self, final_pose):
        if self._motion_abort_requested:
            return
        self._process_last_tcp_pose = copy.deepcopy(final_pose)
        step = self._process_steps[self._process_step_index]
        if step.mode == "APPROACH_PRECONTACT" and self.painting_force_enabled:
            self._begin_runtime_tare_before_contact(step, final_pose)
            return
        if step.mode == "PAINT":
            ramp_index = self._process_step_index + 1
            escape_index = self._process_step_index + 2
            force_lease = getattr(self, "_force_phase_lease_context", None)
            if (
                not self._contact_collision_allowed
                or self._contact_collision_baseline is None
                or getattr(self, "_contact_escape_context", None) is not None
                or (
                    self.painting_force_enabled
                    and (
                        force_lease is None
                        or force_lease.get("mode") != "PAINT"
                        or force_lease.get("active") is not True
                        or force_lease.get("acknowledged") is not True
                    )
                )
                or escape_index >= len(self._process_steps)
                or self._process_steps[ramp_index].mode != "RAMP_DOWN"
                or self._process_steps[escape_index].mode
                not in {"RETRACT", "FINAL_RETRACT"}
            ):
                self._request_motion_abort(
                    "PAINT_CONTACT_ESCAPE_SEQUENCE_INVALID"
                )
                return
            final_row = step.rows[-1]
            self._contact_escape_context = {
                "token": object(),
                "phase": "PAINT_COMPLETE",
                "execution_snapshot": getattr(self, "_execution_snapshot", None),
                "target": str(self.active_target_name),
                "paint_step_index": int(self._process_step_index),
                "ramp_step_index": int(ramp_index),
                "escape_step_index": int(escape_index),
                "paint_final_pose": copy.deepcopy(final_pose),
                "surface_point": tuple(float(v) for v in final_row.position),
                "normal": tuple(float(v) for v in final_row.normal),
                "tangent": tuple(float(v) for v in final_row.tangent),
                "zero_ack_verified": False,
            }
            # Keep the selective roller/target allowance while force slews to
            # zero and while the roller moves strictly outward.  Restoring it
            # at the contact pose makes MoveIt reject RETRACT at fraction 0.
            self._complete_process_step()
            return
        escape = getattr(self, "_contact_escape_context", None)
        if step.mode in {"RETRACT", "FINAL_RETRACT"} and escape is not None:
            if not MoveItExecutor._contact_escape_is_current(
                self, escape, step=step, require_zero_ack=True
            ):
                self._request_motion_abort("CONTACT_ESCAPE_COMPLETION_INVALID")
                return
            escape["phase"] = "RESTORE_PENDING"
            self._set_contact_collision_allowed(
                False,
                lambda success, ctx=escape, expected_step=step:
                MoveItExecutor._contact_escape_restore_done(
                    self, ctx, expected_step, success
                ),
            )
            return
        self._complete_process_step()

    def _limit_trajectory_to_cartesian_speed(
        self, trajectory, start_pose, poses, speed_mps
    ):
        if speed_mps <= 0.0 or not trajectory.joint_trajectory.points:
            return trajectory
        points = []
        if start_pose is not None:
            points.append(
                np.array(
                    [
                        start_pose.position.x,
                        start_pose.position.y,
                        start_pose.position.z,
                    ],
                    dtype=float,
                )
            )
        points.extend(
            np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            for pose in poses
        )
        distance = sum(
            float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:])
        )
        if distance <= 1e-6:
            return trajectory
        desired_duration = distance / speed_mps
        actual_duration = self._point_time_sec(
            trajectory.joint_trajectory.points[-1]
        )
        if actual_duration <= 0.0 or actual_duration >= desired_duration:
            return trajectory
        scale = max(0.001, actual_duration / desired_duration)
        self.get_logger().info(
            f"[PAINT SPEED] distance={distance:.4f}m target={speed_mps:.3f}m/s "
            f"duration {actual_duration:.2f}s -> {desired_duration:.2f}s"
        )
        return self._rescale_trajectory(trajectory, scale=scale)

    def _cancel_ft_auto_zero_timer(self):
        if self._ft_auto_zero_timer is not None:
            self._ft_auto_zero_timer.cancel()
            self.destroy_timer(self._ft_auto_zero_timer)
            self._ft_auto_zero_timer = None

    def _begin_stage1_with_optional_ft_zero(self, on_complete=None):
        """Sketch 실행 직전 F/T bias 를 자동 갱신한다.

        센서가 아예 없거나 /ft/status 가 안 들어오는 Isaac-only 테스트에서는
        기존처럼 진행한다. 센서가 살아 있으면 접촉 없는 상태에서 zero 완료를
        기다린 뒤 Stage 1 을 시작한다.
        """
        if self.real_painting_enabled:
            # The execution-scoped hardware tare belongs at the final
            # pre-contact pose, not at an arbitrary launch/start pose.  Stage
            # 1 and APPROACH_PRECONTACT remain geometry-controlled; the
            # process state machine refuses CONTACT_SEARCH until the tare,
            # monitor reset/bias, guard reset, and residual window all pass.
            self.stage1_approach_free(on_complete=on_complete)
            return

        if not FT_AUTO_ZERO_BEFORE_SKETCH:
            self.stage1_approach_free(on_complete=on_complete)
            return

        now = time.monotonic()
        if self.ft_status_time <= 0.0:
            self.get_logger().warn(
                "[FT ZERO] /ft/status 미수신 -> 자동 zero 생략")
            self.stage1_approach_free(on_complete=on_complete)
            return
        if now - self.ft_status_time > FT_AUTO_ZERO_STALE_SEC:
            self.get_logger().warn(
                "[FT ZERO] /ft/status stale -> 자동 zero 생략")
            self.stage1_approach_free(on_complete=on_complete)
            return
        if self.ft_contact:
            self.get_logger().error(
                "[FT ZERO] 이미 접촉 상태로 판단됨 -> zero 금지, 실행 중단. "
                "EOAT 를 작업면에서 떼고 다시 실행하세요.")
            self.executing = False
            return

        self._cancel_ft_auto_zero_timer()
        request_time = time.monotonic()
        msg = Bool()
        msg.data = True
        self.ft_zero_pub.publish(msg)
        self.ft_bias_ready = False
        self.ft_normal_force_n = None
        self.ft_contact = False
        self.ft_state = "zero_requested"
        self.get_logger().info(
            f"[FT ZERO] 스케치 시작 전 자동 zero 요청 "
            f"(timeout={FT_AUTO_ZERO_TIMEOUT_SEC:.1f}s)")

        def _wait_zero_done():
            now_inner = time.monotonic()
            if (
                self.ft_bias_ready
                and self.ft_status_time >= request_time
                and now_inner - self.ft_status_time <= FT_AUTO_ZERO_STALE_SEC
            ):
                self._cancel_ft_auto_zero_timer()
                self.get_logger().info("[FT ZERO] 완료 -> STAGE 1 시작")
                self.stage1_approach_free(on_complete=on_complete)
                return

            if now_inner - request_time > FT_AUTO_ZERO_TIMEOUT_SEC:
                self._cancel_ft_auto_zero_timer()
                self.executing = False
                self.get_logger().error(
                    "[FT ZERO] timeout -> 실행 중단. "
                    "AFT200 wrench 토픽과 /ft/status 를 확인하세요.")

        self._ft_auto_zero_timer = self.create_timer(
            FT_AUTO_ZERO_CHECK_PERIOD, _wait_zero_done)

    def _pre_sketch_ready_done(self, success):
        self._joint_goal_context = None
        self.executing = False
        if not success:
            MoveItExecutor._clear_execution_snapshot(self)
            self.get_logger().error(
                "PRE_SKETCH_READY_POSE 이동 실패 -> 스케치 실행 중단. "
                "RViz 현재 자세가 joint limit/충돌/도달성 조건을 만족하는지 확인 필요.")
            return
        self.get_logger().info(
            "PRE_SKETCH_READY_POSE 완료 -> 같은 스케치 경로 실행을 시작합니다.")
        timer_ref = {}

        def _restart_after_joint_state_update():
            timer = timer_ref.pop("timer", None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            msg = Bool()
            msg.data = True
            self.on_execute(msg)

        timer_ref["timer"] = self.create_timer(
            0.25, _restart_after_joint_state_update)

    def on_debug_trigger_stage1(self, msg: Bool):
        """디버그용 — 실로봇 검증 시 Stage 1 (자유공간 approach) 단독 호출용.

        current_waypoints[0] 을 surface waypoint 로 보고, surface normal 방향으로
        DEBUG_STAGE1_OFFSET 만큼 떨어진 safe_pose 계산.
        현재 joint state → safe_pose 를 Stage 1 planner 로 plan, 성공 시 실행.
        Stage 2~5 는 트리거하지 않는다.

        publish_test_waypoint 로 /sketch_waypoints 먼저 publish 한 후 실행할 것.
        """
        if not msg.data:
            return
        if self.executing:
            self.get_logger().warn(
                "이미 실행 중 -> /debug_trigger_stage1 무시")
            return
        if not self.current_waypoints:
            self.get_logger().error(
                "/sketch_waypoints 가 비어있음. "
                "publish_test_waypoint 먼저 실행 필요.")
            return
        if self.current_joint_state is None:
            self.get_logger().error("joint_state 미수신 -> 실행 보류")
            return

        self.get_logger().info(
            "[DEBUG] /debug_trigger_stage1 수신 -> Stage 1 단독 실행")

        waypoint = self.current_waypoints[0]

        nx, ny, nz = DEBUG_STAGE1_SURFACE_NORMAL
        safe_pose = Pose()
        safe_pose.position.x = waypoint.position.x + DEBUG_STAGE1_OFFSET * nx
        safe_pose.position.y = waypoint.position.y + DEBUG_STAGE1_OFFSET * ny
        safe_pose.position.z = waypoint.position.z + DEBUG_STAGE1_OFFSET * nz
        safe_pose.orientation = copy.deepcopy(waypoint.orientation)

        self.get_logger().info(
            f"[DEBUG] Stage 1 target safe_pose: "
            f"({safe_pose.position.x:.3f}, {safe_pose.position.y:.3f}, "
            f"{safe_pose.position.z:.3f})"
        )
        self.get_logger().info(
            f"[DEBUG] safe_pose orientation (xyzw): "
            f"({safe_pose.orientation.x:.4f}, {safe_pose.orientation.y:.4f}, "
            f"{safe_pose.orientation.z:.4f}, {safe_pose.orientation.w:.4f})"
        )

        self._safety_tcp_pose = safe_pose
        self.executing = True
        self.stage1_approach_free(on_complete=self._debug_stage1_done)

    def _debug_stage1_done(self):
        self.get_logger().info("[DEBUG] Stage 1 단독 실행 완료")
        self.executing = False

    def on_debug_trigger_jog(self, msg: Bool):
        """매우 작은 joint-space 동작으로 motion pipeline 검증.

        OMPL / IK / Cartesian goal / planning scene 의존성 모두 없음.
        한 joint (wrist3) 에 JOG_DELTA_RAD 를 JOG_DURATION_SEC 동안 적용.
        Trajectory 직접 빌드 → execute_trajectory_direct 호출.
        """
        if not msg.data:
            return
        if self.executing:
            self.get_logger().warn(
                "이미 실행 중 -> /debug_trigger_jog 무시")
            return

        self.get_logger().info(
            "[DEBUG] /debug_trigger_jog 수신 -> joint-space jog 단독 실행")

        if self.current_joint_state is None or \
           len(self.current_joint_state.position) == 0:
            self.get_logger().error(
                "current_joint_state 미수신. /joint_states 흐름 확인.")
            return

        n = len(self.current_joint_state.position)
        if JOG_JOINT_INDEX >= n:
            self.get_logger().error(
                f"JOG_JOINT_INDEX={JOG_JOINT_INDEX} 가 joint 수({n}) 초과")
            return

        current = list(self.current_joint_state.position)
        target = list(current)
        target[JOG_JOINT_INDEX] = current[JOG_JOINT_INDEX] + JOG_DELTA_RAD

        target_joint_name = self.current_joint_state.name[JOG_JOINT_INDEX]
        self.get_logger().info(
            f"[DEBUG] jog target joint: {target_joint_name} "
            f"({current[JOG_JOINT_INDEX]:.4f} -> {target[JOG_JOINT_INDEX]:.4f}, "
            f"delta={JOG_DELTA_RAD:+.4f} rad)")
        self.get_logger().info(
            f"[DEBUG] jog duration: {JOG_DURATION_SEC}s, "
            f"points: {JOG_NUM_POINTS}, "
            f"평균 각속도: {JOG_DELTA_RAD / JOG_DURATION_SEC:.4f} rad/s "
            f"(≈ {(JOG_DELTA_RAD / JOG_DURATION_SEC) * 57.3:.2f}°/s)")

        traj = JointTrajectory()
        traj.joint_names = list(self.current_joint_state.name)

        avg_v = JOG_DELTA_RAD / JOG_DURATION_SEC
        for i in range(JOG_NUM_POINTS):
            alpha = i / (JOG_NUM_POINTS - 1)  # 0.0 ~ 1.0
            point = JointTrajectoryPoint()
            point.positions = [
                current[j] + alpha * (target[j] - current[j])
                for j in range(n)
            ]
            if i == 0 or i == JOG_NUM_POINTS - 1:
                point.velocities = [0.0] * n
            else:
                point.velocities = [0.0] * n
                point.velocities[JOG_JOINT_INDEX] = avg_v
            t = alpha * JOG_DURATION_SEC
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(point)

        rt = RobotTrajectory()
        rt.joint_trajectory = traj

        self.executing = True
        try:
            if self.execute_trajectory_direct(
                rt,
                on_complete=self._jog_done,
                on_failure=lambda: self._request_motion_abort(
                    "DEBUG_JOG_EXECUTION_FAILED"
                ),
                on_rejected=self._jog_done,
                label="DEBUG joint jog",
            ):
                self.get_logger().info("[DEBUG] jog trajectory sent.")
            else:
                self.executing = False
        except Exception as e:
            self.get_logger().error(f"jog trajectory 전송 실패: {e}")
            self.executing = False

    def _jog_done(self):
        """Jog 완료 콜백 (Stage chain 진입 없음, executing 플래그만 해제)."""
        self.get_logger().info("[DEBUG] jog 단독 실행 완료")
        self.executing = False

    def on_debug_trigger_stage5(self, msg: Bool):
        """디버그용 — 실로봇 검증 시 Stage 5 (READY_POSE 복귀) 단독 호출용.
        Stage 1~4 거치지 않고 바로 Stage 5 만 trigger."""
        if not msg.data:
            return
        if self.executing:
            self.get_logger().warn(
                "이미 실행 중 -> /debug_trigger_stage5 무시")
            return
        self.get_logger().info(
            "[DEBUG] /debug_trigger_stage5 수신 -> Stage 5 단독 실행")
        self.executing = True
        self.stage5_return_to_ready()

    def on_go_ready_pose(self, msg: Bool):
        if msg.data:
            self._start_preset_motion("ready")

    def on_go_calibration_pose(self, msg: Bool):
        if msg.data:
            self._start_preset_motion("calib")

    def on_robot_pose_preset(self, msg: String):
        name = (msg.data or "").strip().lower()
        if name.endswith("_pose"):
            name = name[:-5]
        self._start_preset_motion(name)

    def _start_preset_motion(self, name):
        if getattr(self, "model_id", DEFAULT_MODEL) != DEFAULT_MODEL:
            self.get_logger().warn("This arm has no commissioned joint presets; use a planned sketch target")
            return
        if name not in PRESET_POSES:
            self.get_logger().warn(
                f"알 수 없는 pose preset '{name}'. "
                f"사용 가능: {sorted(PRESET_POSES.keys())}")
            return
        if self.executing:
            self.get_logger().warn(
                f"이미 실행 중 -> preset '{name}' 무시")
            return
        if self.current_joint_state is None:
            self.get_logger().warn(
                f"joint_state 미수신 -> preset '{name}' 실행 보류")
            return

        label, joints, speed_scale = PRESET_POSES[name]
        self.get_logger().info(
            f"[PRESET] '{name}' 요청 -> {label} "
            f"(speed_scale={speed_scale:.2f})")
        self.executing = True
        self._plan_joint_goal(
            label,
            joints,
            speed_scale,
            finalize_cb=lambda success: self._preset_motion_finalize(
                label, success),
        )

    def _preset_motion_finalize(self, label, success):
        self._joint_goal_context = None
        if success:
            self.get_logger().info(f"[PRESET] {label} 이동 완료")
        else:
            self.get_logger().warn(f"[PRESET] {label} 이동 실패")
        self.executing = False

    def on_scene_update(self, msg):
        """모니터링 용. scene_confirmed 는 ApplyPlanningScene 결과로 설정."""
        # Monitored planning-scene traffic also carries component/diff
        # messages whose ACM is empty or partial. Those messages are not an
        # authoritative baseline: caching one previously caused the next
        # roller-wall update to replace all SRDF collision allowances.
        acm = msg.allowed_collision_matrix
        acm_error = MoveItExecutor._acm_validation_error(
            acm, self._required_acm_allowed_pairs
        )
        if not acm_error:
            self._latest_allowed_collision_matrix = copy.deepcopy(acm)
        elif acm.entry_names or acm.entry_values:
            signature = (
                len(acm.entry_names),
                len(acm.entry_values),
                acm_error,
            )
            if signature != getattr(
                self, "_last_invalid_monitored_acm_signature", None
            ):
                self._last_invalid_monitored_acm_signature = signature
                self.get_logger().warn(
                    "[ACM] ignored non-authoritative monitored-scene matrix: "
                    + acm_error
                )
        scene_ids = set(obj.id for obj in msg.world.collision_objects)
        objects_ok = self._enabled_ids.issubset(scene_ids)
        eoat_ok = (
            not PUBLISH_EOAT_ATTACHED_OBJECT
            or any(
                ao.object.id == "eoat"
                for ao in msg.robot_state.attached_collision_objects)
        )
        if objects_ok and eoat_ok and not getattr(self, "_monitor_logged", False):
            self._monitor_logged = True
            self.get_logger().info(
                f"[INFO] /monitored_planning_scene 에 {sorted(self._enabled_ids)} 보임 "
                "(EOAT 는 robot_description 고정 링크) "
                "(apply 결과로 confirmed 됨)")

    # ---- PlanningScene (물체들 + EoAT AttachedCollisionObject) ----------------
    def publish_scene_periodic(self):
        if (
            getattr(self, "_acm_update_pending", None) is not None
            or getattr(self, "_contact_collision_allowed", False)
        ):
            # Do not race a world-geometry ApplyPlanningScene request against
            # the full-ACM contact transaction. Execution geometry is frozen
            # while contact is active.
            return
        if self._joint_command_timer is not None:
            self.get_logger().warn(
                f"[SCENE] {self.joint_command_topic} playback 중 PlanningScene apply 보류",
                throttle_duration_sec=2.0)
            return
        revision = int(getattr(self, "_scene_revision", 0))
        if (
            self.scene_confirmed
            and int(getattr(self, "_scene_confirmed_revision", -1)) == revision
        ):
            return
        if self.scene_confirmed:
            # A confirmation without the matching immutable geometry revision
            # is never sufficient for collision-aware IK or planning.
            self.scene_confirmed = False
        if getattr(self, "_scene_apply_inflight_revision", None) is not None:
            return

        ps = PlanningScene()
        ps.is_diff = True
        ps.world = PlanningSceneWorld()

        # 현재 executor 가 이전에 등록했던 obstacle 중 사라진 것만 제거한다.
        # 없는 object 를 매번 REMOVE 하면 /apply_planning_scene 이 success=False 를
        # 반환해서 scene 검증이 계속 실패한다.
        for stale_id in sorted(self._stale_dynamic_obstacle_ids):
            co = CollisionObject()
            co.id = stale_id
            co.header.frame_id = BASE_FRAME
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        self._stale_dynamic_obstacle_ids.clear()

        # --- 활성 물체 전부 ---
        world_to_base = self._lookup_transform_to_base("World", timeout_s=0.05)
        if world_to_base is None:
            self.get_logger().warn(
                f"[SCENE] {BASE_FRAME}<-World TF 미수신 -> scene publish 보류",
                throttle_duration_sec=2.0)
            return

        for obj in self.cfg["objects"]:
            if not obj.get("enabled", True):
                continue
            if obj["name"] == self.active_target_name:
                dynamic_target = self._dynamic_target_collision_object(obj)
                if dynamic_target is not None:
                    ps.world.collision_objects.append(dynamic_target)
                else:
                    self.get_logger().warn(
                        "[SCENE] active target perception plane 미수신 -> "
                        "static wall fallback 등록 안 함",
                        throttle_duration_sec=2.0)
                continue

            co = CollisionObject()
            co.id = obj["name"]
            co.header.frame_id = BASE_FRAME
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            padding = 0.0 if obj["name"] == self.active_target_name \
                else WORLD_COLLISION_PADDING
            prim.dimensions = [
                float(v) + 2.0 * padding for v in obj["size"]
            ]
            pose = self._transform_xyz_quat_to_pose(
                obj["position"], [0.0, 0.0, 0.0, 1.0], world_to_base)
            co.primitives.append(prim)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
            ps.world.collision_objects.append(co)

        # --- ZED 인식 잔여 장애물 voxel ---
        for obstacle in self.dynamic_obstacles:
            co = CollisionObject()
            co.id = obstacle["id"]
            co.header.frame_id = BASE_FRAME
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [
                float(v) + 2.0 * WORLD_COLLISION_PADDING
                for v in obstacle["size"]
            ]
            pose = Pose()
            pose.position.x = float(obstacle["position"][0])
            pose.position.y = float(obstacle["position"][1])
            pose.position.z = float(obstacle["position"][2])
            pose.orientation.x = float(obstacle["orientation"][0])
            pose.orientation.y = float(obstacle["orientation"][1])
            pose.orientation.z = float(obstacle["orientation"][2])
            pose.orientation.w = float(obstacle["orientation"][3])
            co.primitives.append(prim)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
            ps.world.collision_objects.append(co)

        if hasattr(self, "_multi_catalog"):
            objects = self._multi_collision_objects()
            if objects is None:
                return
            ps.world.collision_objects.extend(objects)

        # --- EoAT (tcp -> AFT200 -> EOAT no-camera mesh -> D405, tcp 에 attached) ---
        eoat_aco = AttachedCollisionObject()
        eoat_aco.link_name = EE_LINK  # "tcp"
        eoat_aco.object.id = "eoat"
        eoat_aco.object.header.frame_id = EE_LINK

        # mesh[0] — AFT200 F/T sensor. Isaac Sim 과 같은 실제 collision STL.
        try:
            aft_mesh = _load_aft200_mesh()
            aft_mesh_pose = Pose()
            aft_mesh_pose.orientation.w = 1.0
            eoat_aco.object.meshes.append(aft_mesh)
            eoat_aco.object.mesh_poses.append(aft_mesh_pose)
        except Exception as e:
            self.get_logger().warn(
                f"AFT200 mesh 로드 실패 -> bbox primitive fallback 사용: {e}")
            aft_prim = SolidPrimitive()
            aft_prim.type = SolidPrimitive.BOX
            aft_prim.dimensions = list(AFT200_SIZE)
            aft_pose = Pose()
            aft_pose.position.x = AFT200_CENTER[0]
            aft_pose.position.y = AFT200_CENTER[1]
            aft_pose.position.z = AFT200_CENTER[2]
            aft_pose.orientation.w = 1.0
            eoat_aco.object.primitives.append(aft_prim)
            eoat_aco.object.primitive_poses.append(aft_pose)

        # mesh[1] — RR-00A_B EOAT no-camera collision mesh.
        # CAD +Z 를 TCP -Y 로 회전하고, AFT200 뒤에 붙는 위치까지 변환해 둔 mesh.
        try:
            eoat_mesh = _load_eoat_no_camera_mesh()
            eoat_mesh_pose = Pose()
            eoat_mesh_pose.orientation.w = 1.0
            eoat_aco.object.meshes.append(eoat_mesh)
            eoat_aco.object.mesh_poses.append(eoat_mesh_pose)
        except Exception as e:
            self.get_logger().warn(
                f"EOAT no-camera mesh 로드 실패 -> roller primitive fallback 사용: {e}")

            support_prim = SolidPrimitive()
            support_prim.type = SolidPrimitive.CYLINDER
            support_prim.dimensions = [ROLLER_FORWARD_REACH, ROLLER_SUPPORT_RADIUS]
            support_pose = Pose()
            sx, sy, sz = _axis_offset(
                TOOL_AXIS, AFT200_LENGTH + ROLLER_FORWARD_REACH / 2.0)
            support_pose.position.x = sx
            support_pose.position.y = sy
            support_pose.position.z = sz
            sqx, sqy, sqz, sqw = _cylinder_axis_quat(TOOL_AXIS)
            support_pose.orientation.x = sqx
            support_pose.orientation.y = sqy
            support_pose.orientation.z = sqz
            support_pose.orientation.w = sqw
            eoat_aco.object.primitives.append(support_prim)
            eoat_aco.object.primitive_poses.append(support_pose)

            roller_prim = SolidPrimitive()
            roller_prim.type = SolidPrimitive.CYLINDER
            roller_prim.dimensions = [ROLLER_LENGTH, ROLLER_RADIUS]
            roller_pose = Pose()
            cx, cy, cz = _axis_offset(TOOL_AXIS, EOAT_TIP_OFFSET)
            roller_pose.position.x = cx
            roller_pose.position.y = cy
            roller_pose.position.z = cz
            lqx, lqy, lqz, lqw = _cylinder_axis_quat(ROLLER_LONG_AXIS)
            roller_pose.orientation.x = lqx
            roller_pose.orientation.y = lqy
            roller_pose.orientation.z = lqz
            roller_pose.orientation.w = lqw
            eoat_aco.object.primitives.append(roller_prim)
            eoat_aco.object.primitive_poses.append(roller_pose)

        # primitive[1] — Intel RealSense D405. It is physically attached to the
        # EOAT, so MoveIt must treat it as robot geometry.
        d405_prim = SolidPrimitive()
        d405_prim.type = SolidPrimitive.BOX
        d405_prim.dimensions = list(D405_SIZE)
        d405_pose = Pose()
        d405_pose.position.x = D405_COLLISION_CENTER[0]
        d405_pose.position.y = D405_COLLISION_CENTER[1]
        d405_pose.position.z = D405_COLLISION_CENTER[2]
        d405_pose.orientation.w = 1.0
        eoat_aco.object.primitives.append(d405_prim)
        eoat_aco.object.primitive_poses.append(d405_pose)

        eoat_aco.object.operation = CollisionObject.ADD
        # 장착 플랜지 쪽 접촉만 허용. link5 는 손목 충돌을 잡기 위해 제외.
        eoat_aco.touch_links = list(EOAT_TOUCH_LINKS)
        if PUBLISH_EOAT_ATTACHED_OBJECT:
            ps.robot_state.attached_collision_objects.append(eoat_aco)
        ps.robot_state.is_diff = True

        # publish 도 유지 (RViz 시각화 용)
        self.scene_pub.publish(ps)

        # ApplyPlanningScene service 로 진짜 등록 (MoveIt 의 collision detection 에 반영)
        if self.apply_scene_client.wait_for_service(timeout_sec=1.0):
            req = ApplyPlanningScene.Request()
            req.scene = ps
            self._scene_apply_inflight_revision = revision
            try:
                future = self.apply_scene_client.call_async(req)
            except Exception as exc:
                self._scene_apply_inflight_revision = None
                self.get_logger().warn(
                    f"ApplyPlanningScene request 실패: {exc}"
                )
                return
            future.add_done_callback(
                lambda done, requested_revision=revision:
                self._apply_scene_done(done, requested_revision)
            )
        else:
            self.get_logger().warn("/apply_planning_scene service 없음")

        if not self.scene_initialized:
            self.get_logger().info(
                "PlanningScene: 물체 publish + apply 시도 "
                "(EOAT collision 은 robot_description 고정 링크)")
            self.scene_initialized = True

    def _dynamic_target_collision_object(self, obj):
        """Perception 기반 활성 target collision slab 생성.

        objects.yaml 의 target box 는 nominal fallback 이고, 실제 작업에서는
        ZED 가 발행한 work_area_plane/work_area_corners 를 우선 사용한다.
        local +Z face 가 작업 표면이 되도록 center 를 normal 반대쪽으로 둔다.
        """
        surface_point, surface_normal, _corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if surface_point is None or surface_normal is None:
            return None

        normal = np.asarray(surface_normal, dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        size = np.asarray(obj.get("size", [1.0, 0.02, 1.0]), dtype=float)
        thickness = float(np.min(size)) if size.size else TARGET_COLLISION_MIN_THICKNESS
        thickness = float(np.clip(
            thickness,
            TARGET_COLLISION_MIN_THICKNESS,
            TARGET_COLLISION_MAX_THICKNESS,
        ))
        matched_extent = self._matched_perception_plane_extent(
            normal, surface_point)

        area_basis = self._dynamic_work_area_basis()
        if area_basis is not None:
            surface_center, u_axis, v_axis, half_u, half_v = area_basis
            surface_center = self._project_point_to_active_surface(
                surface_center, normal)
            width = 2.0 * (half_u + TARGET_COLLISION_MARGIN)
            height = 2.0 * (half_v + TARGET_COLLISION_MARGIN)
            source = "work_area_corners"
            if (
                TARGET_COLLISION_USE_NOMINAL_EXTENT_AFTER_D405
                and (
                    getattr(self, "_execution_snapshot", {}).get(
                        "surface_source"
                    )
                    if isinstance(
                        getattr(self, "_execution_snapshot", None), dict
                    )
                    else self.dynamic_surface_source
                ) == "d405_refined"
            ):
                tangent_sizes = sorted([float(v) for v in size], reverse=True)
                if len(tangent_sizes) >= 2:
                    width = max(width, tangent_sizes[0])
                    height = max(height, tangent_sizes[1])
                    source += "+d405_nominal_target_extent"
            if matched_extent is not None:
                width = max(width, matched_extent[0])
                height = max(height, matched_extent[1])
                source += f"+{matched_extent[2]}_plane_extent"
        else:
            surface_center = np.asarray(surface_point, dtype=float)
            u_axis, v_axis = self._plane_basis_from_normal(normal)
            if matched_extent is not None:
                width, height = matched_extent[:2]
                source = f"{matched_extent[2]}_plane_extent"
            else:
                tangent_sizes = sorted([float(v) for v in size], reverse=True)
                width = tangent_sizes[0] if tangent_sizes else WORK_AREA_W
                height = tangent_sizes[1] if len(tangent_sizes) > 1 else WORK_AREA_H
                source = "target_surface"

        u_axis = np.asarray(u_axis, dtype=float)
        u_axis /= np.linalg.norm(u_axis) + 1e-12
        v_axis = np.asarray(v_axis, dtype=float)
        v_axis /= np.linalg.norm(v_axis) + 1e-12
        if float(np.dot(np.cross(u_axis, v_axis), normal)) < 0.0:
            v_axis = -v_axis

        center = np.asarray(surface_center, dtype=float) - normal * (thickness / 2.0)
        q = quat_from_matrix(np.column_stack([u_axis, v_axis, normal]))

        co = CollisionObject()
        co.id = obj["name"]
        co.header.frame_id = BASE_FRAME
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(width), float(height), float(thickness)]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self.get_logger().info(
            f"[SCENE] active target collision 동적 갱신({source}): "
            f"center=({center[0]:+.3f},{center[1]:+.3f},{center[2]:+.3f}) "
            f"size=({width:.3f},{height:.3f},{thickness:.3f}) "
            f"normal=({normal[0]:+.2f},{normal[1]:+.2f},{normal[2]:+.2f})",
            throttle_duration_sec=2.0,
        )
        return co

    def _matched_perception_plane_extent(self, normal, surface_point):
        """선택된 surface 와 같은 평면으로 보이는 scanner plane 크기 반환."""
        if not self.perception_planes:
            return None
        n = np.asarray(normal, dtype=float)
        n /= np.linalg.norm(n) + 1e-12
        p = np.asarray(surface_point, dtype=float)

        best = None
        for plane in self.perception_planes:
            pn = np.asarray(plane["normal"], dtype=float)
            pn /= np.linalg.norm(pn) + 1e-12
            align = abs(float(np.dot(pn, n)))
            if align < 0.90:
                continue
            plane_dist = abs(float(np.dot(p - plane["point"], pn)))
            if plane_dist > 0.10:
                continue

            label = self._plane_label_for_index(plane["index"])
            if label is None:
                continue
            kind = str(label.get("type", "plane"))
            if kind == "floor":
                continue
            try:
                dims = [
                    float(v)
                    for v in label.get("size", [])[:2]
                    if float(v) > 0.02
                ]
            except (TypeError, ValueError):
                continue
            if len(dims) < 2:
                continue
            dims = sorted(dims, reverse=True)
            # 같은 무한 평면 위에서는 centroid 가 멀 수 있으므로 plane distance
            # 와 normal 정렬을 우선한다. wall label 은 동률일 때만 약간 선호.
            score = plane_dist + (1.0 - align) * 0.25
            if kind == "wall":
                score -= 0.01
            if best is None or score < best[0]:
                best = (score, dims[0], dims[1], kind)

        if best is None:
            return None
        return best[1], best[2], best[3]

    def _plane_label_for_index(self, index):
        if index < 0 or index >= len(self.perception_plane_labels):
            return None
        label = self.perception_plane_labels[index]
        return label if isinstance(label, dict) else None

    @staticmethod
    def _plane_basis_from_normal(normal):
        n = np.asarray(normal, dtype=float)
        n /= np.linalg.norm(n) + 1e-12
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(ref, n))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
        u_axis = np.cross(n, ref)
        u_axis /= np.linalg.norm(u_axis) + 1e-12
        v_axis = np.cross(n, u_axis)
        v_axis /= np.linalg.norm(v_axis) + 1e-12
        return u_axis, v_axis

    def _apply_scene_done(self, future, requested_revision=None):
        """ApplyPlanningScene service 응답 처리. 성공 시 scene_confirmed."""
        if requested_revision is None:
            requested_revision = int(getattr(self, "_scene_revision", 0))
        requested_revision = int(requested_revision)
        if (
            getattr(self, "_scene_apply_inflight_revision", None)
            == requested_revision
        ):
            self._scene_apply_inflight_revision = None
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f"ApplyPlanningScene 실패: {e}")
            return
        current_revision = int(getattr(self, "_scene_revision", 0))
        pending = getattr(self, "_multi_scene_pending_ids", None)
        if resp.success and pending is not None and pending[0] == requested_revision:
            self._multi_scene_ids = set(pending[1])
        if resp.success and requested_revision != current_revision:
            self.get_logger().warn(
                "[SCENE] stale apply response ignored: requested revision "
                f"{requested_revision}, current {current_revision}"
            )
            return
        if resp.success and not self.scene_confirmed:
            self.scene_confirmed = True
            self._scene_confirmed_revision = requested_revision
            self.get_logger().info(
                "[OK] PlanningScene apply 성공 "
                "(wall/obstacles 등록, EOAT 는 robot_description 고정 링크)")
        elif not resp.success:
            self.get_logger().warn("ApplyPlanningScene 실패 (success=False)")

    # ---- joint limit / trajectory safety ------------------------------------
    def _joint_state_within_limits(self, joint_state, label):
        if joint_state is None or not joint_state.name:
            self.get_logger().error(f"[JOINT LIMIT] {label}: joint_state 없음")
            return False
        positions = list(joint_state.position)
        if len(joint_state.name) != len(positions):
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: name/position size mismatch"
            )
            return False
        if len(set(joint_state.name)) != len(joint_state.name):
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: duplicate joint name"
            )
            return False
        if not set(READY_POSE_JOINTS).issubset(set(joint_state.name)):
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: required arm joint missing"
            )
            return False
        ok = True
        for name, pos in zip(joint_state.name, positions):
            if name not in getattr(self, "_robot_joint_limits", JOINT_LIMITS):
                continue
            lower, upper = getattr(self, "_robot_joint_limits", JOINT_LIMITS)[name]
            value = float(pos)
            if (
                not math.isfinite(value)
                or value < lower - JOINT_LIMIT_MARGIN
                or value > upper + JOINT_LIMIT_MARGIN
            ):
                self.get_logger().error(
                    f"[JOINT LIMIT] {label}: {name}={value:+.4f} rad "
                    f"outside [{lower:+.2f}, {upper:+.2f}]")
                ok = False
        return ok

    def _trajectory_within_joint_limits(self, jt, label):
        if not jt.joint_names:
            self.get_logger().error(f"[JOINT LIMIT] {label}: joint_names 없음")
            return False
        if len(set(jt.joint_names)) != len(jt.joint_names):
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: duplicate joint name in trajectory"
            )
            return False
        if not jt.points:
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: trajectory points empty"
            )
            return False
        if not set(READY_POSE_JOINTS).issubset(set(jt.joint_names)):
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: required arm joint missing"
            )
            return False
        expected_size = len(jt.joint_names)
        previous_time = -math.inf
        previous_positions = None
        for point_idx, point in enumerate(jt.points):
            if len(point.positions) != expected_size:
                self.get_logger().error(
                    f"[JOINT LIMIT] {label}: point#{point_idx} position size "
                    f"{len(point.positions)} != {expected_size}"
                )
                return False
            if not all(math.isfinite(float(value)) for value in point.positions):
                self.get_logger().error(
                    f"[JOINT LIMIT] {label}: point#{point_idx} has non-finite "
                    "joint position"
                )
                return False
            point_time = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )
            if (
                not math.isfinite(point_time)
                or point_time < 0.0
                or (point_idx > 0 and point_time <= previous_time + 1e-9)
            ):
                self.get_logger().error(
                    f"[TRAJECTORY] {label}: point#{point_idx} has invalid/"
                    f"non-monotonic time_from_start={point_time!r}"
                )
                return False
            positions = [float(value) for value in point.positions]
            if previous_positions is not None:
                max_step = self._max_joint_delta(
                    previous_positions, positions
                )
                if max_step > MAX_COMMAND_SEGMENT_JOINT_DELTA_RAD:
                    self.get_logger().error(
                        f"[TRAJECTORY] {label}: point#{point_idx} adjacent "
                        f"joint jump={math.degrees(max_step):.1f}deg > "
                        f"{math.degrees(MAX_COMMAND_SEGMENT_JOINT_DELTA_RAD):.1f}deg"
                    )
                    return False
            previous_time = point_time
            previous_positions = positions

        violations = []
        for point_idx, point in enumerate(jt.points):
            for joint_idx, name in enumerate(jt.joint_names):
                if name not in getattr(self, "_robot_joint_limits", JOINT_LIMITS) or joint_idx >= len(point.positions):
                    continue
                lower, upper = getattr(self, "_robot_joint_limits", JOINT_LIMITS)[name]
                value = float(point.positions[joint_idx])
                if value < lower - JOINT_LIMIT_MARGIN or value > upper + JOINT_LIMIT_MARGIN:
                    violations.append((point_idx, name, value, lower, upper))
                    if len(violations) >= 5:
                        break
            if len(violations) >= 5:
                break

        if not violations:
            return True

        for point_idx, name, value, lower, upper in violations:
            self.get_logger().error(
                f"[JOINT LIMIT] {label}: point#{point_idx} "
                f"{name}={value:+.4f} rad outside [{lower:+.2f}, {upper:+.2f}]")
        self.get_logger().error(
            f"[JOINT LIMIT] {label}: trajectory rejected before robot command. "
            "MoveIt/Cartesian result would exceed real RB10 joint limits.")
        return False

    # ---- 궤적 실행 (FollowJointTrajectory action) ---------------------------
    def execute_trajectory_direct(
        self,
        traj,
        on_complete=None,
        on_failure=None,
        on_rejected=None,
        force_guard=False,
        label="trajectory",
        requires_contact_acm=False,
        contact_acm_context=None,
    ):
        """RB10 driver 의 FollowJointTrajectory action 으로 trajectory 전송.
        on_complete: action 성공 후 호출할 callback (다음 stage 트리거용).
        함수명은 호출처 호환을 위해 유지."""
        # Central physical-command interlock.  Planning-only MoveIt and
        # GetCartesianPath requests may still run so dry-run validates the
        # generated plan, but neither FollowJointTrajectory nor the direct
        # joint-command backend is allowed to receive a command.
        self._last_dispatch_inhibit_reason = ""
        self._last_trajectory_failure_phase = ""
        dispatch_inhibit = MoveItExecutor._motion_dispatch_inhibited_reason(
            self,
            requires_contact_acm=bool(requires_contact_acm),
            contact_acm_context=contact_acm_context,
        )
        if dispatch_inhibit:
            self._last_dispatch_inhibit_reason = str(dispatch_inhibit)
            self.get_logger().error(
                f"[DISPATCH BLOCKED] {label}: {dispatch_inhibit}"
            )
            return False
        guard_blockers = MoveItExecutor._trajectory_force_guard_blockers(
            self, label, force_guard
        )
        if guard_blockers:
            self._last_dispatch_inhibit_reason = (
                "WRENCH_GUARD_INVALID:" + ",".join(guard_blockers)
            )
            self.get_logger().error(
                f"[DISPATCH BLOCKED] {label}: "
                f"{self._last_dispatch_inhibit_reason}"
            )
            return False
        if self.dry_run:
            if self._motion_abort_requested:
                self.get_logger().warn(
                    f"[DRY RUN] {label} completion not simulated after abort"
                )
                return False
            jt = traj.joint_trajectory
            if not jt.points or not jt.joint_names:
                self.get_logger().error(
                    f"[DRY RUN] {label} rejected: empty trajectory"
                )
                return False
            if not self._trajectory_within_joint_limits(jt, f"dry_run:{label}"):
                return False
            self.get_logger().info(
                f"[DRY RUN] physical dispatch blocked: {label}, "
                f"points={len(jt.points)}"
            )
            if hasattr(self, "_publish_execution_status"):
                self._publish_execution_status(
                    "DRY_RUN",
                    "physical trajectory dispatch blocked",
                    trajectory_label=str(label),
                    trajectory_points=len(jt.points),
                )
            if on_complete is not None:
                on_complete()
            else:
                self.executing = False
            return True
        if self.execution_backend == "joint_command":
            return self._execute_trajectory_joint_command(
                traj, on_complete=on_complete,
                on_failure=on_failure, force_guard=force_guard, label=label)
        return self._execute_trajectory_follow_joint(
            traj,
            on_complete=on_complete,
            on_failure=on_failure,
            on_rejected=on_rejected,
            force_guard=force_guard,
            label=label,
        )

    def _point_time_sec(self, point):
        return (
            float(point.time_from_start.sec) +
            float(point.time_from_start.nanosec) * 1e-9
        )

    def _joint_command_max_speed(self, label):
        text = (label or "").lower()
        if "stage 3" in text or "contact" in text:
            return JOINT_COMMAND_CONTACT_MAX_SPEED_RAD_S
        if "stage 1" in text or "stage 2" in text or "stage 4" in text:
            return JOINT_COMMAND_APPROACH_MAX_SPEED_RAD_S
        return JOINT_COMMAND_DEFAULT_MAX_SPEED_RAD_S

    def _current_positions_for_joints(self, joint_names):
        if self.current_joint_state is None or not self.current_joint_state.name:
            return None
        current = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ))
        if any(name not in current for name in joint_names):
            return None
        return [float(current[name]) for name in joint_names]

    @staticmethod
    def _max_joint_delta(a, b):
        if len(a) != len(b):
            return 0.0
        return max(abs(float(x) - float(y)) for x, y in zip(a, b))

    def _trajectory_joint_metrics(self, jt):
        points = list(jt.points)
        current_positions = self._current_positions_for_joints(jt.joint_names)
        if current_positions is not None and points:
            first_positions = [float(v) for v in points[0].positions]
            if self._max_joint_delta(
                current_positions, first_positions) > JOINT_COMMAND_INSERT_START_TOL_RAD:
                start_point = JointTrajectoryPoint()
                start_point.positions = current_positions
                points = [start_point] + points

        total_joint_distance = 0.0
        max_segment_joint_delta = 0.0
        max_segment_distance = 0.0
        for i in range(1, len(points)):
            prev = [float(v) for v in points[i - 1].positions]
            cur = [float(v) for v in points[i].positions]
            delta_vec = np.asarray(cur, dtype=float) - np.asarray(prev, dtype=float)
            segment_distance = float(np.linalg.norm(delta_vec))
            segment_joint_delta = self._max_joint_delta(prev, cur)
            total_joint_distance += segment_distance
            max_segment_distance = max(max_segment_distance, segment_distance)
            max_segment_joint_delta = max(max_segment_joint_delta, segment_joint_delta)

        if len(points) >= 2:
            start_goal_delta = self._max_joint_delta(
                points[0].positions, points[-1].positions)
        else:
            start_goal_delta = 0.0

        return {
            "point_count": len(points),
            "joint_path": total_joint_distance,
            "max_segment_l2": max_segment_distance,
            "max_joint_delta": max_segment_joint_delta,
            "start_goal_delta": start_goal_delta,
        }

    def _d405_prescan_trajectory_is_safe(self, traj, label):
        if not str(label).startswith("D405_PRESCAN"):
            return True
        jt = traj.joint_trajectory
        if not jt.joint_names or not jt.points:
            self.get_logger().error(
                f"[D405 PRESCAN SAFETY] {label} plan rejected: empty trajectory"
            )
            return False
        if not self._trajectory_within_joint_limits(jt, label):
            return False
        metrics = self._trajectory_joint_metrics(jt)
        reasons = []
        metric_names = (
            "joint_path",
            "max_segment_l2",
            "max_joint_delta",
            "start_goal_delta",
        )
        nonfinite = [
            name for name in metric_names
            if not math.isfinite(float(metrics[name]))
        ]
        if nonfinite:
            reasons.append("nonfinite_metrics=" + ",".join(nonfinite))
        if metrics["joint_path"] > D405_PREFLIGHT_MAX_JOINT_PATH_RAD:
            # MoveIt has already collision-checked the complete URDF path.
            # Integrated joint distance depends on topology and planner
            # interpolation, so retain it as a commissioning diagnostic rather
            # than rejecting an otherwise valid camera approach.
            self.get_logger().warn(
                f"[D405 PRESCAN SAFETY] {label}: diagnostic only: "
                f"joint_path={metrics['joint_path']:.2f}rad > "
                f"{D405_PREFLIGHT_MAX_JOINT_PATH_RAD:.2f}rad"
            )
        if (
            metrics["start_goal_delta"]
            > D405_PREFLIGHT_LARGE_START_GOAL_WARN_RAD
        ):
            self.get_logger().warn(
                f"[D405 PRESCAN SAFETY] {label}: diagnostic only: "
                f"start_goal_delta={metrics['start_goal_delta']:.2f}rad > "
                f"{D405_PREFLIGHT_LARGE_START_GOAL_WARN_RAD:.2f}rad; "
                "collision-checked smooth rotation retained"
            )
        if metrics["point_count"] > D405_PREFLIGHT_MAX_PLAN_POINTS:
            # OMPL waypoint count depends strongly on discretization.  Keep it
            # visible, but do not reject a collision-checked path solely for
            # having a dense representation.
            self.get_logger().warn(
                f"[D405 PRESCAN SAFETY] {label}: diagnostic only: "
                f"points={metrics['point_count']} > "
                f"{D405_PREFLIGHT_MAX_PLAN_POINTS}"
            )

        self.get_logger().info(
            f"[D405 PRESCAN SAFETY] {label}: "
            f"points={metrics['point_count']}, "
            f"joint_path={metrics['joint_path']:.2f}rad, "
            "start_goal_delta="
            f"{math.degrees(metrics['start_goal_delta']):.1f}deg, "
            f"max_segment_l2={math.degrees(metrics['max_segment_l2']):.1f}deg")
        if not reasons:
            return True
        self.get_logger().error(
            f"[D405 PRESCAN SAFETY] {label} plan rejected: "
            + ", ".join(reasons))
        return False

    def _playback_points_and_times(self, jt, label):
        """Normalize MoveIt timing and cap Isaac joint command playback velocity.

        Some MoveIt results start at t>0 or have sparse/zero timestamps. Isaac
        then appears to wait and finally jump. For direct Isaac joint command
        playback, build a local time axis from joint-space path length instead
        of preserving every MoveIt segment duration. This avoids repeated
        slow-fast-slow pulses at dense Cartesian waypoints.
        """
        points = list(jt.points)
        raw_times = [self._point_time_sec(p) for p in points]
        raw_start = raw_times[0] if raw_times else 0.0

        inserted_start = False
        current_positions = self._current_positions_for_joints(jt.joint_names)
        if current_positions is not None and points:
            first_positions = [float(v) for v in points[0].positions]
            start_delta = self._max_joint_delta(current_positions, first_positions)
            if start_delta > JOINT_COMMAND_INSERT_START_TOL_RAD:
                start_point = JointTrajectoryPoint()
                start_point.positions = current_positions
                start_point.velocities = [0.0] * len(current_positions)
                start_point.accelerations = [0.0] * len(current_positions)
                points = [start_point] + points
                inserted_start = True

        max_speed = max(self._joint_command_max_speed(label), 1e-6)
        playback_times = [0.0]
        max_segment_joint_delta = 0.0
        max_segment_distance = 0.0
        total_joint_distance = 0.0
        for i in range(1, len(points)):
            prev = [float(v) for v in points[i - 1].positions]
            cur = [float(v) for v in points[i].positions]
            delta_vec = np.asarray(cur, dtype=float) - np.asarray(prev, dtype=float)
            segment_distance = float(np.linalg.norm(delta_vec))
            segment_joint_delta = self._max_joint_delta(prev, cur)
            max_segment_joint_delta = max(max_segment_joint_delta, segment_joint_delta)
            max_segment_distance = max(max_segment_distance, segment_distance)
            total_joint_distance += segment_distance
            playback_times.append(total_joint_distance / max_speed)

        if len(playback_times) == 1:
            playback_times[0] = 0.0

        self.get_logger().info(
            f"[TIMING] {label}: raw_start={raw_start:.2f}s, "
            f"raw_end={(raw_times[-1] if raw_times else 0.0):.2f}s, "
            f"playback={playback_times[-1]:.2f}s, "
            f"max_speed_cap={max_speed:.2f}rad/s, "
            f"insert_start={'yes' if inserted_start else 'no'}, "
            f"joint_path={total_joint_distance:.2f}rad, "
            f"max_segment_l2={math.degrees(max_segment_distance):.1f}deg, "
            f"max_joint_delta={math.degrees(max_segment_joint_delta):.1f}deg")
        return points, playback_times

    def _cancel_joint_command_timer(self):
        if self._joint_command_timer is not None:
            self._joint_command_timer.cancel()
            self.destroy_timer(self._joint_command_timer)
            self._joint_command_timer = None

    def _ft_guard_triggered(self, label):
        if getattr(self, "process_mode", "paint") == "spray":
            return False
        if self.ft_normal_force_n is None:
            return False
        if time.monotonic() - self.ft_status_time > FT_FORCE_STALE_SEC:
            return False
        if not self.ft_bias_ready:
            self.get_logger().warn(
                f"[FT GUARD] {label}: FT bias not ready; guard inactive",
                throttle_duration_sec=2.0)
            return False
        if self.ft_normal_force_n >= self.ft_abort_force_n:
            self.get_logger().error(
                f"[FT GUARD] {label}: normal force "
                f"{self.ft_normal_force_n:.1f}N >= "
                f"{self.ft_abort_force_n:.1f}N -> trajectory stop")
            return True
        return False

    def _joint_command_from_positions(self, joint_names, positions):
        """Isaac Sim joint command JointState 생성.

        trajectory joint_names 는 MoveIt 순서이고, Isaac JointGraph 는 현재
        /joint_states 순서도 받을 수 있으므로 이름 기준으로 재정렬한다.
        """
        if len(joint_names) != len(positions):
            raise ValueError(
                f"trajectory joint_names({len(joint_names)})와 "
                f"positions({len(positions)}) 길이가 다름")

        cmd_by_name = dict(zip(joint_names, positions))
        if self.current_joint_state is not None and self.current_joint_state.name:
            names = list(self.current_joint_state.name)
            current_by_name = dict(zip(
                self.current_joint_state.name,
                self.current_joint_state.position,
            ))
            positions = [
                float(cmd_by_name.get(name, current_by_name.get(name, 0.0)))
                for name in names
            ]
        else:
            names = list(joint_names)
            positions = [float(v) for v in positions]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        return msg

    def _joint_command_from_point(self, joint_names, point):
        """Trajectory point 를 Isaac Sim joint command JointState 로 변환."""
        return self._joint_command_from_positions(joint_names, point.positions)

    def _sample_trajectory_positions(self, points, times, idx, elapsed):
        """시간 elapsed 에서 trajectory position 을 보간한다.

        Isaac Sim position target 재생에서는 MoveIt velocity 를 그대로 쓰는
        cubic Hermite 보간이 작은 overshoot/진동을 만들 수 있다. 직접 재생은
        joint-space retiming 된 점들을 선형 보간해서 waypoint마다 가감속이
        반복되지 않게 한다.
        """
        if elapsed <= times[0]:
            return [float(v) for v in points[0].positions]
        if elapsed >= times[-1]:
            return [float(v) for v in points[-1].positions]

        while idx < len(points) - 2 and times[idx + 1] <= elapsed:
            idx += 1

        p0 = points[idx]
        p1 = points[idx + 1]
        t0 = times[idx]
        t1 = times[idx + 1]
        dt = t1 - t0
        if dt <= 1e-9:
            return [float(v) for v in p1.positions], idx

        a = max(0.0, min(1.0, (elapsed - t0) / dt))
        p0_pos = [float(v) for v in p0.positions]
        p1_pos = [float(v) for v in p1.positions]

        positions = [
            p0_pos[i] + a * (p1_pos[i] - p0_pos[i])
            for i in range(len(p0_pos))
        ]
        return positions, idx

    @staticmethod
    def _slew_limit_positions(current, desired, max_step):
        if current is None or len(current) != len(desired):
            return [float(v) for v in desired]
        out = []
        for c, d in zip(current, desired):
            c = float(c)
            d = float(d)
            delta = d - c
            if abs(delta) <= max_step:
                out.append(d)
            else:
                out.append(c + math.copysign(max_step, delta))
        return out

    def _execute_trajectory_joint_command(self, traj, on_complete=None,
                                          on_failure=None, force_guard=False,
                                          label="trajectory"):
        """Isaac Sim JointGraph 가 구독하는 전용 topic 으로 trajectory 재생."""
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} 실행 안 함 — motion abort 상태")
            return False
        jt = traj.joint_trajectory
        if not jt.points:
            self.get_logger().warn("빈 궤적")
            if on_complete is None:
                self.executing = False
            return False
        if not jt.joint_names:
            self.get_logger().error("trajectory joint_names 비어 있음")
            self.executing = False
            return False
        if not self._trajectory_within_joint_limits(jt, "joint_command"):
            self.executing = False
            return False

        self._cancel_joint_command_timer()

        start_time = time.monotonic()
        playback_points, point_times = self._playback_points_and_times(jt, label)
        end_time = max(point_times)
        max_speed = self._joint_command_max_speed(label)
        state = {
            "idx": 0,
            "done": False,
            "last_cmd": self._current_positions_for_joints(jt.joint_names),
            "last_tick_time": start_time,
        }

        self.get_logger().info(
            f"{self.joint_command_topic} 재생: {len(playback_points)} 포인트, "
            f"duration={end_time:.2f}s, "
            f"publish_rate={1.0 / JOINT_COMMAND_TIMER_PERIOD:.0f}Hz, "
            f"interpolation=on, ft_guard={'on' if force_guard else 'off'}")

        def _tick():
            if state["done"]:
                return

            now_tick = time.monotonic()
            elapsed = now_tick - start_time
            if force_guard and self._ft_guard_triggered(label):
                state["done"] = True
                self._cancel_joint_command_timer()
                self.executing = False
                if on_failure is not None:
                    on_failure()
                return
            try:
                sampled = self._sample_trajectory_positions(
                    playback_points, point_times, state["idx"], elapsed)
                if isinstance(sampled, tuple):
                    positions, state["idx"] = sampled
                else:
                    positions = sampled
                tick_dt = max(1e-3, now_tick - state["last_tick_time"])
                state["last_tick_time"] = now_tick
                max_step = (
                    max_speed * tick_dt * JOINT_COMMAND_SLEW_MULTIPLIER
                )
                positions = self._slew_limit_positions(
                    state["last_cmd"], positions, max_step)
                state["last_cmd"] = list(positions)
                self.joint_cmd_pub.publish(
                    self._joint_command_from_positions(jt.joint_names, positions))
            except Exception as e:
                self.get_logger().error(
                    f"{self.joint_command_topic} publish 실패: {e}")
                state["done"] = True
                self._cancel_joint_command_timer()
                self.executing = False
                if on_failure is not None:
                    on_failure()
                return

            final_positions = [float(v) for v in playback_points[-1].positions]
            final_error = self._max_joint_delta(
                state["last_cmd"] or final_positions, final_positions)
            if elapsed >= end_time and final_error <= JOINT_COMMAND_FINAL_TOL_RAD:
                state["done"] = True
                try:
                    self.joint_cmd_pub.publish(
                        self._joint_command_from_point(
                            jt.joint_names, playback_points[-1]))
                except Exception:
                    pass
                self._cancel_joint_command_timer()
                self.get_logger().info(
                    f">>> 궤적 실행 완료 ({self.joint_command_topic} playback)")
                if on_complete is not None:
                    on_complete()
                else:
                    self.executing = False

        self._joint_command_timer = self.create_timer(
            JOINT_COMMAND_TIMER_PERIOD, _tick)
        _tick()
        return True

    def _cancel_fjt_guard_timer(self):
        if self._fjt_guard_timer is not None:
            self._fjt_guard_timer.cancel()
            self.destroy_timer(self._fjt_guard_timer)
            self._fjt_guard_timer = None

    def _trajectory_force_guard_blockers(self, label, force_guard, now=None):
        """Return phase-exact guard blockers for a physical trajectory."""

        if (
            not force_guard
            or not getattr(self, "painting_force_enabled", False)
            or not getattr(self, "_execution_tare_ready", False)
        ):
            return ()
        label = str(label)
        if label.startswith("CONTACT_SEARCH"):
            return MoveItExecutor._force_guard_status_blockers(
                self,
                expected_mode="CONTACT_SEARCH",
                active=False,
                command_context=getattr(
                    self, "_contact_search_command_context", None
                ),
                now=now,
            )
        if getattr(self, "_painting_command_enable", False):
            return MoveItExecutor._force_guard_status_blockers(
                self,
                expected_mode=getattr(self, "_painting_command_mode", ""),
                active=True,
                command_context=getattr(
                    self, "_painting_command_context", None
                ),
                now=now,
            )
        return ()

    def _post_tare_fjt_safety_blockers(self, now):
        if (
            not getattr(self, "real_painting_enabled", False)
            or not getattr(self, "_execution_tare_ready", False)
        ):
            return ()
        blockers = []
        status_time = float(getattr(self, "_safety_status_time", 0.0))
        timeout_s = float(getattr(self, "ft_required_timeout_s", 0.0))
        status = getattr(self, "_safety_status", {})
        if (
            status_time <= 0.0
            or timeout_s <= 0.0
            or now - status_time < 0.0
            or now - status_time > timeout_s
        ):
            blockers.append("SAFETY_STATUS_STALE")
        if status.get("ft_valid") is not True:
            blockers.append("FT_INVALID")
        if status.get("tf_valid") is not True:
            blockers.append("FT_TF_INVALID")
        if status.get("abort_latched") is not False:
            blockers.append(
                "SAFETY_ABORT_LATCHED:"
                + str(status.get("reason", "UNKNOWN"))
            )
        return tuple(blockers)

    def _latch_fjt_motion_unknown(self, token, label, reason):
        if token is not self._active_trajectory_goal_token:
            return
        first_latch = not self._fjt_motion_state_unknown
        self._fjt_motion_state_unknown = True
        self._execution_abort_reason = f"{reason}:{label}"
        if first_latch:
            self.get_logger().error(
                f"[FJT UNKNOWN] {reason} ({label}); active token retained"
            )
        if not self._motion_abort_requested:
            self._request_motion_abort(f"{reason}:{label}")
        else:
            self._publish_motion_abort_latch()
        # The high-level abort path marks executing false, but without a
        # definitive action result physical motion is still possible.
        self.executing = True
        if first_latch:
            self._publish_execution_status(
                "FJT_MOTION_UNKNOWN",
                "action state unknown; hardware inhibit and full stack "
                "relaunch required",
                trajectory_label=str(label),
                fault_reason=str(reason),
            )
        self._cancel_fjt_guard_timer()

    def _start_fjt_guard_timer(self, token, label, force_guard):
        self._cancel_fjt_guard_timer()
        timer_ref = {"timer": None}

        def cancel_this_timer():
            timer = timer_ref["timer"]
            if timer is None:
                return
            if timer is self._fjt_guard_timer:
                self._cancel_fjt_guard_timer()
                return
            # A queued callback from an old goal must never cancel the newer
            # goal's guard timer. Destroy only the timer captured here.
            try:
                timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(timer)
            except Exception:
                pass

        def check_guard():
            if token is not self._active_trajectory_goal_token:
                cancel_this_timer()
                return
            if (
                getattr(self, "_contact_search_cancel_token", None) is token
                and MoveItExecutor._consume_ready_active_fjt_result(
                    self, token
                )
            ):
                return
            now = time.monotonic()
            watchdog_reason = follow_joint_watchdog_reason(
                now_s=now,
                result_deadline_s=self._active_trajectory_result_deadline,
                cancel_requested=self._active_trajectory_cancel_requested,
                cancel_deadline_s=self._active_trajectory_cancel_deadline,
                abort_latched=self._motion_abort_requested,
            )
            if watchdog_reason == "FJT_CANCEL_TIMEOUT":
                self._latch_fjt_motion_unknown(
                    token, label, "FJT_CANCEL_TIMEOUT"
                )
                return
            if watchdog_reason:
                if watchdog_reason == "FJT_WATCHDOG_TIME_INVALID":
                    self._request_motion_abort(watchdog_reason)
                    return
                self._request_motion_abort(f"FJT_RESULT_TIMEOUT:{label}")
                return
            post_tare_blockers = self._post_tare_fjt_safety_blockers(now)
            if post_tare_blockers:
                if not self._motion_abort_requested:
                    self._request_motion_abort(
                        "POST_TARE_FJT_SAFETY:%s:%s"
                        % (label, ",".join(post_tare_blockers))
                    )
                return
            guard_blockers = MoveItExecutor._trajectory_force_guard_blockers(
                self, label, force_guard, now=now
            )
            if guard_blockers:
                if not self._motion_abort_requested:
                    self._request_motion_abort(
                        "FJT_WRENCH_GUARD:%s:%s"
                        % (label, ",".join(guard_blockers))
                    )
                return
            if (
                force_guard
                and str(label).startswith("CONTACT_SEARCH")
                and self._contact_search_contact_sensed()
            ):
                self._contact_search_cancel_on_contact = True
                self._contact_search_cancel_token = token
                # Short contact-search goals can become SUCCESS while the
                # result callback is merely queued behind this timer. Consume
                # that result before sending a late Cancel request.
                if MoveItExecutor._consume_ready_active_fjt_result(
                    self, token
                ):
                    return
                if MoveItExecutor._finalize_contact_from_terminal_status(
                    self, token, label
                ):
                    return
                # Do not cancel a contact-search step at all.  Each step is
                # bounded to contact_search_step_m and completes in ~0.4 s,
                # while contact confirmation lags the physical touch by the
                # monitor filter plus hold (~100 ms) and therefore lands in
                # the middle of a step - on 2026-08-14 it landed 40 ms before
                # natural completion, so the cancel raced the SUCCESS result
                # and the terminal result was orphaned (five FJT_CANCEL /
                # FJT_RESULT timeout hard stops, each killing a whole job).
                # Letting the step finish costs at most one step of extra
                # penetration and removes the race entirely: the flag set
                # above makes the terminal handler finish the search through
                # its early-contact branch.
                return
            if force_guard and self._ft_guard_triggered(label):
                self._request_motion_abort(f"legacy FT guard triggered in {label}")

        timer = self.create_timer(0.02, check_guard)
        timer_ref["timer"] = timer
        self._fjt_guard_timer = timer

    @staticmethod
    def _active_fjt_result_is_ready(executor, token):
        if token is not getattr(executor, "_active_trajectory_goal_token", None):
            return False
        future = getattr(executor, "_active_trajectory_result_future", None)
        if future is None:
            return False
        done = getattr(future, "done", None)
        if not callable(done):
            return False
        try:
            return bool(done())
        except Exception:
            return False

    @staticmethod
    def _consume_ready_active_fjt_result(executor, token):
        if not MoveItExecutor._active_fjt_result_is_ready(executor, token):
            return False
        consumer = getattr(
            executor, "_active_trajectory_result_consumer", None
        )
        future = getattr(executor, "_active_trajectory_result_future", None)
        if not callable(consumer) or future is None:
            return False
        consumer(future)
        # The consumer may itself discover a transport/result exception and
        # latch motion-unknown while retaining the token. Report consumed only
        # after it actually committed/cleared this goal; otherwise callers
        # must still attempt cancellation of the possibly moving controller.
        return token is not getattr(
            executor, "_active_trajectory_goal_token", None
        )

    @staticmethod
    def _finalize_contact_from_terminal_status(executor, token, label):
        """Reconcile terminal action status before issuing a late cancel."""

        if (
            token is not getattr(executor, "_active_trajectory_goal_token", None)
            or getattr(executor, "_active_trajectory_terminal_committed", False)
            or getattr(executor, "_motion_abort_requested", False)
            or getattr(executor, "_contact_search_cancel_token", None) is not token
        ):
            return False
        handle = getattr(executor, "_active_trajectory_goal_handle", None)
        try:
            status = int(getattr(handle, "status", GoalStatus.STATUS_UNKNOWN))
        except (TypeError, ValueError):
            status = GoalStatus.STATUS_UNKNOWN
        if status not in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
        }:
            return False
        executor._active_trajectory_terminal_committed = True
        executor.get_logger().info(
            f">>> CONTACT_SEARCH terminal status reconciled before cancel "
            f"(status={status}, {label})"
        )
        executor._clear_active_follow_joint_goal(token)
        executor._complete_contact_search_after_early_contact()
        return True

    def _cancel_active_follow_joint_goal(self, reason):
        token = self._active_trajectory_goal_token
        handle = self._active_trajectory_goal_handle
        if token is None:
            return
        if MoveItExecutor._consume_ready_active_fjt_result(self, token):
            return
        if MoveItExecutor._finalize_contact_from_terminal_status(
            self,
            token,
            getattr(self, "_active_trajectory_label", "") or "trajectory",
        ):
            return
        try:
            handle_status = int(
                getattr(handle, "status", GoalStatus.STATUS_UNKNOWN)
            )
        except (TypeError, ValueError):
            handle_status = GoalStatus.STATUS_UNKNOWN
        if handle_status in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }:
            # Terminal action status is authoritative evidence that no cancel
            # should be sent. The already-requested result is allowed to
            # settle; its normal result deadline still bounds transport loss.
            return
        # Repeated /motion_abort samples must not keep pushing the watchdog
        # deadline into the future while the action goal response is pending.
        # A single immutable deadline guarantees escalation to
        # FJT_CANCEL_TIMEOUT if the controller never establishes a known state.
        if self._active_trajectory_cancel_requested:
            return
        if handle is None:
            self.get_logger().warn(
                f"[FJT CANCEL] goal response pending; cancel queued ({reason})"
            )
            self._active_trajectory_cancel_requested = True
            self._active_trajectory_cancel_reason = str(reason)
            self._active_trajectory_cancel_deadline = (
                time.monotonic() + self.fjt_cancel_timeout_s
            )
            return
        self._active_trajectory_cancel_requested = True
        self._active_trajectory_cancel_reason = str(reason)
        self._active_trajectory_cancel_deadline = (
            time.monotonic() + self.fjt_cancel_timeout_s
        )
        self.get_logger().error(
            f"[FJT CANCEL] {self._active_trajectory_label or 'trajectory'}: {reason}"
        )
        try:
            future = handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"[FJT CANCEL] request failed: {exc}")
            return

        def cancel_done(done_future):
            if token is not self._active_trajectory_goal_token:
                return
            try:
                response = done_future.result()
                accepted = bool(response.goals_canceling)
            except Exception as exc:
                self.get_logger().error(f"[FJT CANCEL] response failed: {exc}")
                return
            if accepted:
                self.get_logger().warn("[FJT CANCEL] controller accepted cancellation")
            else:
                # An empty cancel response often means the goal crossed to a
                # terminal state just before cancellation. Reconcile the
                # result/status, but never treat a cancel ACK as terminal.
                if MoveItExecutor._consume_ready_active_fjt_result(self, token):
                    return
                if MoveItExecutor._finalize_contact_from_terminal_status(
                    self, token, self._active_trajectory_label or "trajectory"
                ):
                    return
                self.get_logger().warn(
                    "[FJT CANCEL] controller did not accept cancellation; "
                    "waiting for terminal result"
                )

        future.add_done_callback(cancel_done)

    def _clear_active_follow_joint_goal(self, token):
        if token is not self._active_trajectory_goal_token:
            return
        self._cancel_fjt_guard_timer()
        self._active_trajectory_goal_handle = None
        self._active_trajectory_goal_token = None
        self._active_trajectory_label = ""
        self._active_trajectory_result_future = None
        self._active_trajectory_result_consumer = None
        self._active_trajectory_terminal_committed = False
        self._active_trajectory_cancel_requested = False
        self._active_trajectory_cancel_reason = ""
        self._active_trajectory_result_deadline = 0.0
        self._active_trajectory_cancel_deadline = 0.0
        if getattr(self, "_contact_search_cancel_token", None) is token:
            self._contact_search_cancel_on_contact = False
            self._contact_search_cancel_token = None
        # A late terminal result must never clear a relaunch-only unknown
        # motion latch. Only constructing a fresh executor resets it.

    def _execute_trajectory_follow_joint(
        self,
        traj,
        on_complete=None,
        on_failure=None,
        on_rejected=None,
        force_guard=False,
        label="trajectory",
    ):
        """FollowJointTrajectory action 으로 trajectory 전송."""
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} 실행 안 함 - motion abort 상태")
            return False
        if not traj.joint_trajectory.points:
            self.get_logger().warn("빈 궤적")
            if on_complete is None:
                self.executing = False
            return False
        if self._active_trajectory_goal_token is not None:
            self.get_logger().error(
                f"{label} 실행 안 함 - another FollowJointTrajectory goal is active"
            )
            return False
        if not self._trajectory_within_joint_limits(
                traj.joint_trajectory, "follow_joint_trajectory"):
            self.executing = False
            return False

        try:
            action_ready = bool(self.traj_action_client.server_is_ready())
        except (AttributeError, RuntimeError):
            action_ready = False
        if not action_ready:
            action_ready = bool(
                self.traj_action_client.wait_for_server(timeout_sec=0.0)
            )
        if not action_ready:
            self.get_logger().error("FollowJointTrajectory action server 없음")
            self.executing = False
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj.joint_trajectory  # trajectory_msgs/JointTrajectory 그대로
        # path/goal tolerances 는 비워둠 (controller 디폴트 사용).
        # 필요시 차후에 GoalTolerance 추가.

        self.get_logger().info(
            f"FollowJointTrajectory 전송: {len(traj.joint_trajectory.points)} 포인트 "
            f"({label}, cancel_guard={'on' if force_guard else 'off'})")

        token = object()
        self._active_trajectory_goal_token = token
        self._active_trajectory_goal_sent_at = time.monotonic()
        self._active_trajectory_goal_handle = None
        self._active_trajectory_label = label
        self._active_trajectory_result_future = None
        self._active_trajectory_result_consumer = None
        self._active_trajectory_terminal_committed = False
        self._active_trajectory_cancel_requested = False
        self._active_trajectory_cancel_reason = ""
        self._contact_search_cancel_on_contact = False
        self._contact_search_cancel_token = None
        expected_duration_s = max(
            self._point_time_sec(traj.joint_trajectory.points[-1]), 0.0
        )
        self._active_trajectory_result_deadline = (
            time.monotonic()
            + expected_duration_s
            + self.fjt_result_timeout_margin_s
        )
        self._active_trajectory_cancel_deadline = 0.0
        try:
            send_future = self.traj_action_client.send_goal_async(goal)
        except Exception as exc:
            self.get_logger().error(f"send_goal request failed: {exc}")
            self._latch_fjt_motion_unknown(
                token, label, "FJT_GOAL_REQUEST_UNKNOWN"
            )
            return False
        self._start_fjt_guard_timer(token, label, force_guard)

        def _goal_response(fut):
            try:
                handle = fut.result()
            except Exception as e:
                self.get_logger().error(f"send_goal 실패: {e}")
                self._latch_fjt_motion_unknown(
                    token, label, "FJT_GOAL_RESPONSE_UNKNOWN"
                )
                return
            if not handle.accepted:
                self.get_logger().error("Trajectory goal rejected")
                self._last_trajectory_failure_phase = "GOAL_REJECTED"
                self._clear_active_follow_joint_goal(token)
                self.executing = False
                if on_rejected is not None:
                    on_rejected()
                elif on_failure is not None:
                    on_failure()
                return
            if token is not self._active_trajectory_goal_token:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._active_trajectory_goal_handle = handle
            pending_cancel = self._active_trajectory_cancel_requested
            pending_cancel_deadline = self._active_trajectory_cancel_deadline
            pending_cancel_reason = str(
                getattr(self, "_active_trajectory_cancel_reason", "")
                or "abort arrived before acceptance"
            )
            try:
                result_future = handle.get_result_async()
            except Exception as exc:
                self.get_logger().error(f"get_result request failed: {exc}")
                # The goal is accepted and may be moving even though result
                # transport setup failed. Convert any queued pre-acceptance
                # cancel to a real controller request before latching UNKNOWN.
                if self._motion_abort_requested or pending_cancel:
                    if pending_cancel:
                        self._active_trajectory_cancel_requested = False
                    self._cancel_active_follow_joint_goal(
                        pending_cancel_reason
                    )
                    if (
                        pending_cancel
                        and pending_cancel_deadline > 0.0
                        and token is self._active_trajectory_goal_token
                    ):
                        self._active_trajectory_cancel_deadline = (
                            pending_cancel_deadline
                        )
                self._latch_fjt_motion_unknown(
                    token, label, "FJT_RESULT_REQUEST_UNKNOWN"
                )
                return
            self._active_trajectory_result_future = result_future
            self._active_trajectory_result_consumer = _result_done
            result_future.add_done_callback(_result_done)
            if token is not self._active_trajectory_goal_token:
                return
            if pending_cancel:
                self._active_trajectory_cancel_requested = False
            if self._motion_abort_requested or pending_cancel:
                if MoveItExecutor._consume_ready_active_fjt_result(
                    self, token
                ):
                    return
                if MoveItExecutor._finalize_contact_from_terminal_status(
                    self, token, label
                ):
                    return
                self._cancel_active_follow_joint_goal(pending_cancel_reason)
                if (
                    pending_cancel
                    and pending_cancel_deadline > 0.0
                    and token is self._active_trajectory_goal_token
                ):
                    # Late action acceptance must not buy another full cancel
                    # timeout. Preserve the original immutable deadline.
                    self._active_trajectory_cancel_deadline = (
                        pending_cancel_deadline
                    )

        def _result_done(fut):
            if token is not self._active_trajectory_goal_token:
                self.get_logger().warn(
                    f"stale trajectory result ignored ({label})"
                )
                return
            if self._active_trajectory_terminal_committed:
                return
            try:
                wrapped = fut.result()
                status = int(wrapped.status)
                res = wrapped.result
            except Exception as e:
                self.get_logger().error(f"trajectory result 실패: {e}")
                self._latch_fjt_motion_unknown(
                    token, label, "FJT_RESULT_UNKNOWN"
                )
                return
            self._active_trajectory_terminal_committed = True
            unknown_before_result = bool(self._fjt_motion_state_unknown)
            aborted = self._motion_abort_requested
            early_contact_cancel = bool(
                self._contact_search_cancel_on_contact
                and str(label).startswith("CONTACT_SEARCH")
                and getattr(self, "_contact_search_cancel_token", None) is token
            )
            result_success = bool(
                status == GoalStatus.STATUS_SUCCEEDED
                and int(getattr(res, "error_code", -1)) == 0
            )
            self._clear_active_follow_joint_goal(token)
            if unknown_before_result:
                self.get_logger().error(
                    f"late terminal result observed after FJT motion became "
                    f"unknown ({label}); relaunch latch remains active"
                )
                self.executing = True
                return
            if (
                early_contact_cancel
                and not aborted
                and (
                    status == GoalStatus.STATUS_CANCELED
                    or result_success
                )
            ):
                self.get_logger().info(
                    f">>> CONTACT_SEARCH completed from terminal action "
                    f"status={status} ({label})"
                )
                self._complete_contact_search_after_early_contact()
                return
            if status == GoalStatus.STATUS_CANCELED or aborted:
                self._last_trajectory_failure_phase = "CANCELED"
                self.get_logger().warn(
                    f">>> FollowJointTrajectory canceled ({label})"
                )
                self._contact_search_cancel_on_contact = False
                self._contact_search_cancel_token = None
                self.executing = False
                if on_failure is not None and not aborted:
                    on_failure()
                return
            self._contact_search_cancel_on_contact = False
            self._contact_search_cancel_token = None
            # FollowJointTrajectory.Result.error_code: 0 = SUCCESSFUL
            if status != GoalStatus.STATUS_SUCCEEDED or res.error_code != 0:
                self._last_trajectory_failure_phase = "EXECUTION_FAILED"
                self.get_logger().error(
                    f"Trajectory 실행 에러 status={status} code={res.error_code}: "
                    f"{res.error_string}")
                self.executing = False
                if on_failure is not None:
                    on_failure()
                return
            self.get_logger().info(">>> 궤적 실행 완료 (action SUCCESS)")
            if on_complete is not None:
                on_complete()
            else:
                self.executing = False

        send_future.add_done_callback(_goal_response)
        return True

    # ---- helpers for 4-stage approach ---------------------------------------
    def _active_surface_plane(self, target=None):
        surface_point, surface_normal, _corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if surface_point is not None and surface_normal is not None:
            return surface_point, surface_normal
        if target is None:
            target = get_target(self.cfg, self.active_target_name)
        return get_surface_plane(target)

    def _active_ee_quat(self, target):
        if self._stage3_tcp_wps:
            q = self._stage3_tcp_wps[0].orientation
            return np.array([q.x, q.y, q.z, q.w], dtype=float)
        return ee_quat_for_target(target)

    def _offset_along_normal(self, pose, distance):
        """pose 를 active target 의 표면 normal 방향으로 distance 만큼 후퇴."""
        _, n = self._active_surface_plane()
        out = copy.deepcopy(pose)
        out.position.x += distance * float(n[0])
        out.position.y += distance * float(n[1])
        out.position.z += distance * float(n[2])
        return out

    def _make_pose_constraints(self, pose, link_name=EE_LINK, frame=BASE_FRAME):
        """Pose 를 MoveGroup goal 의 Constraints 로 변환."""
        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link_name
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.001]  # 1mm tolerance (tight)
        pc.constraint_region.primitives.append(sp)
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link_name
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.02
        oc.absolute_y_axis_tolerance = 0.02
        oc.absolute_z_axis_tolerance = 0.02
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        return c

    @staticmethod
    def _nearest_joint_equivalent(name, value, current):
        """Choose the equivalent revolute angle closest to current state."""
        if name not in JOINT_LIMITS:
            return float(value)
        lower, upper = JOINT_LIMITS[name]
        candidates = []
        for k in range(-2, 3):
            candidate = float(value) + 2.0 * math.pi * k
            if lower - JOINT_LIMIT_MARGIN <= candidate <= upper + JOINT_LIMIT_MARGIN:
                candidates.append(candidate)
        if not candidates:
            return float(value)
        return min(candidates, key=lambda x: abs(x - float(current)))

    def _make_joint_goal_constraints(
            self, joint_state, tolerance=0.02, joint_names=None):
        """IK 결과 joint_state 를 MoveGroup joint goal constraints 로 변환."""
        c = Constraints()
        goal_map = dict(zip(joint_state.name, joint_state.position))
        current_map = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        )) if self.current_joint_state is not None else {}
        current_names = set(self.current_joint_state.name) \
            if self.current_joint_state is not None else set(goal_map.keys())
        names = joint_names or joint_state.name
        for name in names:
            if name not in goal_map or name not in current_names:
                continue
            position = self._nearest_joint_equivalent(
                name, goal_map[name], current_map.get(name, goal_map[name]))
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = float(tolerance)
            jc.tolerance_below = float(tolerance)
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _stage1_joint_delta_summary(self, goal_joint_state):
        if self.current_joint_state is None:
            return None, 0.0
        current = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ))
        goal = dict(zip(goal_joint_state.name, goal_joint_state.position))
        deltas = []
        for name in READY_POSE_JOINTS.keys():
            if name in current and name in goal:
                goal_pos = self._nearest_joint_equivalent(
                    name, goal[name], current[name])
                d = abs(float(goal_pos) - float(current[name]))
                deltas.append((name, d))
        if not deltas:
            return None, 0.0
        return max(deltas, key=lambda item: item[1])

    def _log_stage1_joint_delta(self, goal_joint_state):
        max_name, max_delta = self._stage1_joint_delta_summary(goal_joint_state)
        if not max_name:
            return max_name, max_delta
        self.get_logger().info(
            f"[STAGE 1 IK] nearest joint goal: max_delta="
            f"{math.degrees(max_delta):.1f}deg ({max_name})")
        if max_delta > STAGE1_LARGE_JOINT_DELTA_WARN_RAD:
            self.get_logger().warn(
                f"[STAGE 1 IK] 큰 관절 이동 감지: {max_name} "
                f"{math.degrees(max_delta):.1f}deg. "
                "MoveIt 이 collision-free plan 은 찾지만 최단/최소 관절 이동을 "
                "수학적으로 보장하지는 않음.")
        return max_name, max_delta

    def _stage1_trajectory_is_safe(self, traj, label):
        jt = traj.joint_trajectory
        if not self._trajectory_within_joint_limits(
            jt, f"STAGE 1 {label}"
        ):
            return False
        metrics = self._trajectory_joint_metrics(jt)
        reasons = []
        if metrics["joint_path"] > STAGE1_MAX_JOINT_PATH_RAD:
            self.get_logger().warn(
                f"[STAGE 1 SAFETY] {label}: diagnostic only: "
                f"joint_path={metrics['joint_path']:.2f}rad > "
                f"{STAGE1_MAX_JOINT_PATH_RAD:.2f}rad"
            )
        if (
            metrics["start_goal_delta"]
            > STAGE1_LARGE_JOINT_DELTA_WARN_RAD
        ):
            self.get_logger().warn(
                f"[STAGE 1 SAFETY] {label}: diagnostic only: "
                f"start_goal_delta={metrics['start_goal_delta']:.2f}rad > "
                f"{STAGE1_LARGE_JOINT_DELTA_WARN_RAD:.2f}rad; "
                "collision-checked smooth rotation retained"
            )
        if metrics["point_count"] > STAGE1_MAX_PLAN_POINTS:
            self.get_logger().warn(
                f"[STAGE 1 SAFETY] {label}: diagnostic only: "
                f"points={metrics['point_count']} > {STAGE1_MAX_PLAN_POINTS}"
            )

        self.get_logger().info(
            f"[STAGE 1 SAFETY] {label}: "
            f"points={metrics['point_count']}, "
            f"joint_path={metrics['joint_path']:.2f}rad, "
            f"start_goal_delta={metrics['start_goal_delta']:.2f}rad, "
            f"max_segment_l2={math.degrees(metrics['max_segment_l2']):.1f}deg")
        if not reasons:
            return True
        self.get_logger().error(
            f"[STAGE 1 SAFETY] {label} plan rejected: "
            + ", ".join(reasons))
        return False

    def _compute_snapped_tcp_waypoints(self):
        """current_waypoints 를 densify + tcp 변환한 결과 반환.
        새 perception 흐름: sketch_to_waypoints_node 가 이미 wall_plane 좌표를
        roller center contact-clearance plane 으로 변환해서 보냄.
        따라서 yaml 기반 snap 불필요. waypoint 그대로 사용.
        target/n 은 caller 호환을 위해 yaml 에서 계속 반환 (offset_along_normal 에서 사용).
        Returns: (snapped_tip_wps, tcp_wps, target, n)
        """
        target = get_target(self.cfg, self.active_target_name)
        _sp, n = self._active_surface_plane(target)
        snapped = [copy.deepcopy(wp) for wp in self.current_waypoints]
        densified = self._densify_waypoints(snapped, spacing_m=0.005)
        tcp_wps = [self._brush_tip_to_tcp(wp) for wp in densified]
        self.get_logger().info(
            f"표면 스냅 SKIP (새 perception 흐름): N={len(snapped)} "
            f"첫점=({snapped[0].position.x:.3f},{snapped[0].position.y:.3f},"
            f"{snapped[0].position.z:.3f})")
        return densified, tcp_wps, target, n

    def _sync_active_target_plane_from_waypoints(self, waypoints):
        """Perception 기반 waypoint plane 에 맞춰 active target collision 위치 보정.

        objects.yaml 은 실험실 기준 nominal wall 이고, 실제 wall plane 은 ZED 가
        매번 인식한다. sketch_to_waypoints 는 roller center 를 발행하므로,
        mean(waypoints) 에서 roller radius+clearance 를 빼 active target 의
        collision surface 를 normal 축 방향으로 갱신한다.
        """
        if not waypoints:
            return
        try:
            target = get_target(self.cfg, self.active_target_name)
        except Exception as e:
            self.get_logger().warn(f"active target lookup 실패: {e}")
            return

        if self.dynamic_surface_point is not None:
            return
        expected = ROLLER_RADIUS + CONTACT_CLEARANCE
        pts = np.array([
            [p.position.x, p.position.y, p.position.z] for p in waypoints
        ], dtype=float)
        normal = self._infer_surface_normal_from_waypoints(waypoints, target)
        surface_point = np.mean(pts, axis=0) - normal * expected

        self.dynamic_surface_point = surface_point
        self.dynamic_surface_normal = normal

        axis = int(np.argmax(np.abs(normal)))
        half_axis = float(target["size"][axis]) / 2.0
        old_center_axis = float(target["position"][axis])
        if abs(float(normal[axis])) > 0.5:
            new_center_axis = (
                float(surface_point[axis]) - float(normal[axis]) * half_axis
            )
            target["position"][axis] = new_center_axis
        else:
            new_center_axis = old_center_axis

        MoveItExecutor._mark_scene_dirty(self)
        self.get_logger().warn(
            f"[SCENE SYNC] {self.active_target_name} plane 을 waypoint 로 복구: "
            f"point=({surface_point[0]:+.3f},{surface_point[1]:+.3f},"
            f"{surface_point[2]:+.3f}) normal=({normal[0]:+.2f},"
            f"{normal[1]:+.2f},{normal[2]:+.2f}), "
            f"axis={['x','y','z'][axis]}, center "
            f"{old_center_axis:+.3f}->{new_center_axis:+.3f} "
            f"(work_area_plane TF miss fallback)")

    def _infer_surface_normal_from_waypoints(self, waypoints, target):
        """sketch_to_waypoints 가 넣은 EE orientation 으로 free-space normal 복구.

        sketch_to_waypoints convention:
          local TOOL_AXIS = forward = -surface_normal
        따라서 surface normal 은 waypoint orientation 의 -TOOL_AXIS 방향이다.
        orientation 이 없거나 이상하면 objects.yaml 의 nominal normal 로 fallback.
        """
        normals = []
        for wp in waypoints:
            try:
                tool_axis = self._local_axis_in_world(wp.orientation, TOOL_AXIS)
            except Exception:
                continue
            n = -np.asarray(tool_axis, dtype=float)
            norm = float(np.linalg.norm(n))
            if norm > 1e-6:
                normals.append(n / norm)
        if normals:
            normal = np.mean(normals, axis=0)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-6:
                return normal / norm
        _plane_point, normal = get_surface_plane(target)
        normal = np.asarray(normal, dtype=float)
        return normal / (np.linalg.norm(normal) + 1e-12)

    def _validate_contact_waypoints(self, waypoints, target):
        """작업영역/벽 clearance 검증.

        waypoints 는 wall surface point 가 아니라 roller center 이다. 모든 점은
        wall surface 에서 ROLLER_RADIUS + CONTACT_CLEARANCE 만큼 free-space 방향에
        있어야 하며, yellow work area 에 해당하는 0.6m x 0.6m 영역 안이어야 한다.
        """
        if not waypoints:
            self.get_logger().error("waypoint 없음")
            return False
        plane_point, normal = self._active_surface_plane(target)
        normal = np.asarray(normal, dtype=float)
        expected = ROLLER_RADIUS + CONTACT_CLEARANCE

        axis = int(np.argmax(np.abs(normal)))
        lateral_axes = [i for i in range(3) if i != axis]
        area_basis = self._dynamic_work_area_basis()
        half_limits = {
            lateral_axes[0]: WORK_AREA_W / 2.0 + WORK_AREA_MARGIN,
            lateral_axes[1]: WORK_AREA_H / 2.0 + WORK_AREA_MARGIN,
        }
        names = ["x", "y", "z"]
        bad = []
        for i, wp in enumerate(waypoints):
            p = np.array([wp.position.x, wp.position.y, wp.position.z])
            clearance = float(np.dot(p - plane_point, normal))
            if abs(clearance - expected) > CONTACT_PLANE_TOL:
                bad.append(
                    f"#{i}: clearance {clearance:.3f}m "
                    f"(expected {expected:.3f}±{CONTACT_PLANE_TOL:.3f})")
                continue
            if area_basis is not None:
                center, u_axis, v_axis, half_u, half_v = area_basis
                du = float(np.dot(p - center, u_axis))
                dv = float(np.dot(p - center, v_axis))
                if abs(du) > half_u or abs(dv) > half_v:
                    bad.append(
                        f"#{i}: outside dynamic work area "
                        f"(u={du:.3f}/{half_u:.3f}, v={dv:.3f}/{half_v:.3f})")
            else:
                for ax in lateral_axes:
                    if abs(float(p[ax] - plane_point[ax])) > half_limits[ax]:
                        bad.append(
                            f"#{i}: {names[ax]}={p[ax]:.3f} outside work area")
                        break
        if bad:
            for msg in bad[:5]:
                self.get_logger().error("[WAYPOINT SAFETY] " + msg)
            if len(bad) > 5:
                self.get_logger().error(
                    f"[WAYPOINT SAFETY] ... and {len(bad)-5} more")
            return False
        area_desc = (
            "dynamic work area"
            if area_basis is not None
            else f"work area={WORK_AREA_W:.2f}x{WORK_AREA_H:.2f}m"
        )
        self.get_logger().info(
            f"[WAYPOINT SAFETY] {len(waypoints)}점 검증 OK: "
            f"roller center clearance={expected*1000:.1f}mm, "
            f"{area_desc}")
        return True

    def _dynamic_work_area_basis(self):
        surface_point, surface_normal, corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if corners is None or len(corners) < 4:
            return None
        tl, tr, br, bl = np.asarray(corners[:4], dtype=float)
        if surface_point is not None and surface_normal is not None:
            pts = self._project_points_to_active_surface(
                np.asarray([tl, tr, br, bl], dtype=float))
            tl, tr, br, bl = pts
            n = np.asarray(surface_normal, dtype=float)
            n /= np.linalg.norm(n) + 1e-12
        else:
            n = None
        center = (tl + tr + br + bl) / 4.0
        u_vec = ((tr - tl) + (br - bl)) / 2.0
        v_vec = ((bl - tl) + (br - tr)) / 2.0
        width = float(np.linalg.norm(u_vec))
        height = float(np.linalg.norm(v_vec))
        if width < 1e-6 or height < 1e-6:
            return None
        if n is not None:
            u_vec = u_vec - n * float(np.dot(u_vec, n))
            v_vec = v_vec - n * float(np.dot(v_vec, n))
            width = float(np.linalg.norm(u_vec))
            height = float(np.linalg.norm(v_vec))
            if width < 1e-6 or height < 1e-6:
                return None
            u_axis = u_vec / width
            v_axis = v_vec / height
            # Make an orthonormal basis on the refined plane while preserving
            # the sign closest to the image-derived vertical axis.
            v_ortho = np.cross(n, u_axis)
            v_ortho /= np.linalg.norm(v_ortho) + 1e-12
            if float(np.dot(v_ortho, v_axis)) < 0.0:
                v_ortho = -v_ortho
            v_axis = v_ortho
        else:
            u_axis = u_vec / width
            v_axis = v_vec / height
        return (
            center,
            u_axis,
            v_axis,
            width / 2.0 + WORK_AREA_MARGIN,
            height / 2.0 + WORK_AREA_MARGIN,
        )

    def _project_point_to_active_surface(self, point, normal=None):
        """Keep ZED-derived work-area lateral center, but use active/D405 depth."""
        p = np.asarray(point, dtype=float)
        surface_point, surface_normal, _corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if surface_point is None:
            return p
        n = (
            np.asarray(normal, dtype=float)
            if normal is not None
            else np.asarray(surface_normal, dtype=float)
        )
        n /= np.linalg.norm(n) + 1e-12
        plane_p = np.asarray(surface_point, dtype=float)
        return p - float(np.dot(p - plane_p, n)) * n

    def _project_points_to_active_surface(self, points, normal=None):
        pts = np.asarray(points, dtype=float)
        surface_point, surface_normal, _corners = (
            MoveItExecutor._execution_surface_geometry(self)
        )
        if surface_point is None:
            return pts
        n = (
            np.asarray(normal, dtype=float)
            if normal is not None
            else np.asarray(surface_normal, dtype=float)
        )
        n /= np.linalg.norm(n) + 1e-12
        plane_p = np.asarray(surface_point, dtype=float)
        signed = (pts - plane_p) @ n
        return pts - signed[:, None] * n

    def _check_normal_motion(self, start_pose, end_pose, normal, expected_align,
                             label):
        start = np.array([
            start_pose.position.x,
            start_pose.position.y,
            start_pose.position.z,
        ], dtype=float)
        end = np.array([
            end_pose.position.x,
            end_pose.position.y,
            end_pose.position.z,
        ], dtype=float)
        delta = end - start
        dist = float(np.linalg.norm(delta))
        if dist < 1e-6:
            self.get_logger().error(f"[{label}] 이동 거리 0 -> 중단")
            return False
        direction = delta / dist
        align = float(np.dot(direction, np.asarray(normal, dtype=float)))
        if align < expected_align:
            self.get_logger().error(
                f"[{label}] normal 방향 정렬 실패: align={align:+.3f}, "
                f"required>={expected_align:.3f}")
            return False
        self.get_logger().info(
            f"[{label}] normal motion OK: dist={dist*100:.1f}cm, "
            f"align={align:+.3f}")
        return True

    # ---- Stage 1: free-space approach via MoveGroup action -------------------
    def _cancel_stage1_ik_candidate_timer(self):
        timer = getattr(self, "_stage1_ik_candidate_timer", None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self._stage1_ik_candidate_timer = None

    def _invalidate_stage1_orientation_candidates(self, clear_candidates=False):
        """Invalidate every outstanding dual-orientation callback."""

        self._cancel_stage1_ik_candidate_timer()
        self._stage1_ik_candidate_generation = int(
            getattr(self, "_stage1_ik_candidate_generation", 0)
        ) + 1
        self._stage1_ik_candidate_results = {}
        self._stage1_ik_candidates_finalized_generation = -1
        self._stage1_ik_seed_state = None
        self._stage1_orientation_ranked = []
        self._stage1_orientation_rank_index = -1
        self._stage1_orientation_branch_frozen = False
        self._selected_segment_orientation_branch = ""
        if clear_candidates:
            self._stage1_orientation_candidates = ()

    def _fail_stage1_before_motion(self, reason):
        """Cleanly reject Stage 1 while no physical trajectory is active.

        A planning/IK rejection must not leave an immutable execution snapshot
        or a UI ``SAFETY_APPROACH`` state behind, but it also must not create a
        hardware emergency latch when no command was dispatched.
        """

        reason = str(reason).strip() or "STAGE1_PLAN_REJECTED"
        if (
            getattr(self, "_active_trajectory_goal_token", None) is not None
            or getattr(self, "_joint_command_timer", None) is not None
            or getattr(self, "_fjt_motion_state_unknown", False)
        ):
            self._request_motion_abort(f"STAGE1_FAILURE_AFTER_DISPATCH:{reason}")
            return
        self.get_logger().error(f"[STAGE 1] rejected before motion: {reason}")
        self._cancel_stage1_scene_wait_timer()
        self._stage1_retried = False
        self._stage1_goal_constraints = None
        self._stage1_on_complete = None
        self._reset_painting_process()
        self.executing = False
        self._publish_execution_status("PLAN_REJECTED", reason)

    def _cancel_stage1_scene_wait_timer(self):
        if self._stage1_scene_wait_timer is not None:
            self._stage1_scene_wait_timer.cancel()
            self.destroy_timer(self._stage1_scene_wait_timer)
            self._stage1_scene_wait_timer = None
        self._stage1_scene_wait_start = None

    def stage1_approach_free(self, on_complete=None):
        # on_complete: 실행 완료 후 호출할 콜백. None 이면 production 기본 (Stage 2 chain).
        self._stage1_on_complete = on_complete or self.stage2_approach_linear
        self._cancel_stage1_scene_wait_timer()
        self._cancel_stage1_ik_candidate_timer()
        self._stage1_attempt_token = object()
        self._stage1_retried = False
        self._stage1_goal_constraints = None

        # PlanningScene 검증 대기 (attached EOAT + wall 등록 확인).
        # rclpy callback 안에서 sleep 으로 기다리면 ApplyPlanningScene 응답도 같이
        # 막힐 수 있으므로, 비동기 타이머로 confirmed 된 뒤에만 Stage 1 을 시작한다.
        if not self.scene_confirmed:
            self.get_logger().info(
                "PlanningScene 검증 대기 시작 "
                f"(timeout={SCENE_WAIT_TIMEOUT_SEC:.1f}s)")
            self.publish_scene_periodic()
            self._stage1_scene_wait_start = time.monotonic()
            token = self._stage1_attempt_token
            timer_ref = {}

            def wait_for_this_run():
                self._stage1_wait_for_scene_confirmed(
                    token,
                    timer_ref.get("timer"),
                )

            timer = self.create_timer(
                SCENE_WAIT_PERIOD_SEC,
                wait_for_this_run,
            )
            timer_ref["timer"] = timer
            self._stage1_scene_wait_timer = timer
            return

        self.get_logger().info("PlanningScene 검증 OK")
        self._start_stage1_after_scene_confirmed()

    def _stage1_wait_for_scene_confirmed(
        self, stage1_token=None, timer_identity=None
    ):
        if stage1_token is not None and (
            stage1_token is not self._stage1_attempt_token
            or timer_identity is not self._stage1_scene_wait_timer
            or not self.executing
            or self._motion_abort_requested
        ):
            # A destroyed timer callback can already be queued in the executor.
            # It belongs to the old Run and must not cancel or start the new one.
            return
        if self.scene_confirmed:
            self._cancel_stage1_scene_wait_timer()
            self.get_logger().info("PlanningScene 검증 OK")
            self._start_stage1_after_scene_confirmed()
            return

        elapsed = time.monotonic() - float(self._stage1_scene_wait_start or 0.0)
        if elapsed < SCENE_WAIT_TIMEOUT_SEC:
            self.get_logger().info(
                "PlanningScene 검증 대기 중...",
                throttle_duration_sec=1.0,
            )
            return

        self._cancel_stage1_scene_wait_timer()
        self.get_logger().error(
            "PlanningScene 미검증 -> 안전을 위해 Stage 1 계획/실행 중단 "
            "(wall/EoAT collision 이 MoveIt 에 확정되지 않음)")
        self._fail_stage1_before_motion("PLANNING_SCENE_NOT_CONFIRMED")

    def _start_stage1_after_scene_confirmed(self):
        blockers = self._stage1_pre_motion_blockers()
        if blockers:
            self._fail_stage1_before_motion(
                "STAGE1_ASYNC_READINESS:" + ",".join(blockers)
            )
            return
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server 없음 (/move_action)")
            self._fail_stage1_before_motion("MOVE_GROUP_UNAVAILABLE")
            return
        if not self._joint_state_within_limits(
                self.current_joint_state, "STAGE 1 start"):
            self.get_logger().error(
                "STAGE 1 시작 joint_state 가 limit 밖 -> 실행 중단")
            self._fail_stage1_before_motion("STAGE1_START_JOINT_INVALID")
            return

        self._request_stage1_nearest_ik()

    def _request_stage1_nearest_ik(self):
        """현재 joint state 를 seed 로 쓰는 IK 를 먼저 풀어 Stage 1 wrist flip 을 줄인다."""
        if len(getattr(self, "_stage1_orientation_candidates", ())) == 2:
            self._request_stage1_orientation_candidate_iks()
            return
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "/compute_ik 서비스 없음 -> Stage 1 pose goal fallback")
            self._send_stage1_plan(
                [self._make_pose_constraints(self._safety_tcp_pose)],
                "pose goal fallback",
            )
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.robot_state.is_diff = False
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.pose_stamped.header.frame_id = BASE_FRAME
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = self._safety_tcp_pose
        req.ik_request.timeout = Duration(seconds=STAGE1_IK_TIMEOUT_S).to_msg()

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._stage1_ik_done)

    def _stage1_candidate_joint_metrics(
        self, solution_joint_state, seed_state=None
    ):
        """Validate/canonicalize one IK result and return deterministic costs."""

        if seed_state is None:
            seed_state = self._stage1_ik_seed_state
        required = tuple(READY_POSE_JOINTS.keys())
        if seed_state is None:
            return None, "IK_SEED_UNAVAILABLE"
        if (
            len(solution_joint_state.name) != len(solution_joint_state.position)
            or len(seed_state.name) != len(seed_state.position)
        ):
            return None, "IK_JOINT_ARRAY_SIZE_MISMATCH"
        if len(set(solution_joint_state.name)) != len(solution_joint_state.name):
            return None, "IK_DUPLICATE_JOINT_NAME"

        goal_map = dict(zip(
            solution_joint_state.name,
            solution_joint_state.position,
        ))
        seed_map = dict(zip(seed_state.name, seed_state.position))
        if any(name not in goal_map or name not in seed_map for name in required):
            return None, "IK_REQUIRED_JOINT_MISSING"

        normalized = JointState()
        normalized.header = copy.deepcopy(solution_joint_state.header)
        normalized.name = list(required)
        deltas = []
        for name in required:
            raw_goal = float(goal_map[name])
            current = float(seed_map[name])
            if not math.isfinite(raw_goal) or not math.isfinite(current):
                return None, f"IK_NONFINITE:{name}"
            goal = self._nearest_joint_equivalent(name, raw_goal, current)
            lower, upper = getattr(self, "_robot_joint_limits", JOINT_LIMITS)[name]
            if (
                not math.isfinite(goal)
                or goal < lower - JOINT_LIMIT_MARGIN
                or goal > upper + JOINT_LIMIT_MARGIN
            ):
                return None, f"IK_JOINT_LIMIT:{name}"
            normalized.position.append(float(goal))
            deltas.append((name, abs(float(goal) - current)))

        max_name, max_delta = max(deltas, key=lambda item: item[1])
        delta_values = np.asarray([item[1] for item in deltas], dtype=float)
        constraints = Constraints()
        for name, goal in zip(normalized.name, normalized.position):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = str(name)
            joint_constraint.position = float(goal)
            joint_constraint.tolerance_above = STAGE1_JOINT_GOAL_TOL
            joint_constraint.tolerance_below = STAGE1_JOINT_GOAL_TOL
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return {
            "joint_state": normalized,
            "constraints": constraints,
            "max_name": str(max_name),
            "max_delta": float(max_delta),
            "l2_delta": float(np.linalg.norm(delta_values)),
            "sum_delta": float(np.sum(delta_values)),
        }, ""

    def _request_stage1_orientation_candidate_iks(self):
        """Collision-check both task-equivalent 180-degree EOAT poses."""

        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self._fail_stage1_before_motion("DUAL_IK_SERVICE_UNAVAILABLE")
            return
        if self.current_joint_state is None:
            self._fail_stage1_before_motion("DUAL_IK_SEED_UNAVAILABLE")
            return

        self._cancel_stage1_ik_candidate_timer()
        self._stage1_ik_candidate_generation += 1
        generation = self._stage1_ik_candidate_generation
        token = self._stage1_attempt_token
        self._stage1_ik_candidate_results = {}
        self._stage1_ik_candidates_finalized_generation = -1
        self._stage1_ik_seed_state = copy.deepcopy(self.current_joint_state)

        for index, candidate in enumerate(self._stage1_orientation_candidates):
            req = GetPositionIK.Request()
            req.ik_request.group_name = PLANNING_GROUP
            # Both candidates use the exact same captured measured seed so
            # callback timing cannot bias the comparison.
            req.ik_request.robot_state.joint_state = copy.deepcopy(
                self._stage1_ik_seed_state
            )
            req.ik_request.robot_state.is_diff = False
            # This evaluates the complete MoveIt robot state, including the
            # D405, AFT cable guard and EOAT collision geometry in the URDF.
            req.ik_request.avoid_collisions = True
            req.ik_request.ik_link_name = EE_LINK
            req.ik_request.pose_stamped.header.frame_id = BASE_FRAME
            req.ik_request.pose_stamped.header.stamp = (
                self.get_clock().now().to_msg()
            )
            req.ik_request.pose_stamped.pose = copy.deepcopy(
                candidate["safety_tcp_pose"]
            )
            req.ik_request.timeout = Duration(
                seconds=STAGE1_IK_TIMEOUT_S
            ).to_msg()
            try:
                future = self.ik_client.call_async(req)
            except Exception as exc:
                self._stage1_ik_candidate_results[index] = {
                    "valid": False,
                    "reason": f"IK_REQUEST_FAILED:{exc}",
                    "candidate_index": index,
                }
                continue
            future.add_done_callback(
                lambda done, g=generation, i=index, t=token:
                self._stage1_orientation_candidate_ik_done(done, g, i, t)
            )

        if len(self._stage1_ik_candidate_results) == 2:
            self._finish_stage1_orientation_candidate_iks(generation, token)
            return

        def timeout_candidates():
            if (
                generation != self._stage1_ik_candidate_generation
                or token is not self._stage1_attempt_token
            ):
                # This callback may belong to an already-cancelled timer from a
                # previous Run.  Never cancel the shared timer slot here: it
                # could now be the watchdog for the new generation.
                return
            for index in range(2):
                self._stage1_ik_candidate_results.setdefault(
                    index,
                    {
                        "valid": False,
                        "reason": "IK_RESPONSE_TIMEOUT",
                        "candidate_index": index,
                    },
                )
            self._finish_stage1_orientation_candidate_iks(generation, token)

        self._stage1_ik_candidate_timer = self.create_timer(
            STAGE1_DUAL_IK_RESPONSE_TIMEOUT_S,
            timeout_candidates,
        )
        self.get_logger().info(
            "[STAGE 1 SYMMETRY] collision-aware IK requested for both "
            "D405/cable/EOAT orientation branches"
        )

    def _stage1_orientation_candidate_ik_done(
        self, future, generation, index, token
    ):
        if (
            generation != self._stage1_ik_candidate_generation
            or token is not self._stage1_attempt_token
            or self._motion_abort_requested
            or index in self._stage1_ik_candidate_results
        ):
            return
        candidate = self._stage1_orientation_candidates[index]
        record = {
            "valid": False,
            "reason": "IK_UNKNOWN",
            "candidate_index": index,
            "candidate": candidate,
        }
        try:
            response = future.result()
            if response.error_code.val != 1:
                error_code = int(response.error_code.val)
                if error_code == -31:  # MoveItErrorCodes.NO_IK_SOLUTION
                    record["reason"] = "IK_NO_COLLISION_FREE_SOLUTION:-31"
                else:
                    record["reason"] = (
                        f"IK_COLLISION_CHECK_FAILED:{error_code}"
                    )
            else:
                metrics, reason = self._stage1_candidate_joint_metrics(
                    response.solution.joint_state
                )
                if metrics is None:
                    record["reason"] = reason
                else:
                    record.update(metrics)
                    record["valid"] = True
                    record["reason"] = ""
        except Exception as exc:
            record["reason"] = f"IK_RESPONSE_FAILED:{exc}"
        self._stage1_ik_candidate_results[index] = record
        if record["valid"]:
            if record["max_delta"] > STAGE1_LARGE_JOINT_DELTA_WARN_RAD:
                self.get_logger().warn(
                    "[STAGE 1 SYMMETRY] %s requires a large smooth "
                    "rotation: %s %.1fdeg; retaining because endpoint and "
                    "complete URDF path are collision checked"
                    % (
                        candidate["name"],
                        record["max_name"],
                        math.degrees(record["max_delta"]),
                    )
                )
            self.get_logger().info(
                "[STAGE 1 SYMMETRY] %s collision-aware IK OK: "
                "max=%s %.1fdeg, l2=%.2frad"
                % (
                    candidate["name"],
                    record["max_name"],
                    math.degrees(record["max_delta"]),
                    record["l2_delta"],
                )
            )
        else:
            self.get_logger().warn(
                f"[STAGE 1 SYMMETRY] {candidate['name']} rejected: "
                f"{record['reason']}"
            )
        if len(self._stage1_ik_candidate_results) == 2:
            self._finish_stage1_orientation_candidate_iks(generation, token)

    def _finish_stage1_orientation_candidate_iks(self, generation, token):
        if (
            generation != self._stage1_ik_candidate_generation
            or token is not self._stage1_attempt_token
            or self._motion_abort_requested
        ):
            return
        if len(self._stage1_ik_candidate_results) != 2:
            return
        if self._stage1_ik_candidates_finalized_generation == generation:
            return
        self._stage1_ik_candidates_finalized_generation = generation
        self._cancel_stage1_ik_candidate_timer()
        incomplete_prefixes = (
            "IK_RESPONSE_TIMEOUT",
            "IK_REQUEST_FAILED",
            "IK_RESPONSE_FAILED",
            "IK_COLLISION_CHECK_FAILED",
        )
        incomplete = [
            str(record.get("reason", ""))
            for record in self._stage1_ik_candidate_results.values()
            if str(record.get("reason", "")).startswith(incomplete_prefixes)
        ]
        if incomplete:
            # The operator explicitly requires both D405/cable/EOAT variants
            # to be collision checked.  A transport timeout is not a negative
            # collision result and may not be silently treated as one.
            self._fail_stage1_before_motion(
                "ROLLER_ORIENTATION_COLLISION_CHECK_INCOMPLETE:"
                + ";".join(incomplete)
            )
            return
        valid = [
            record
            for _, record in sorted(self._stage1_ik_candidate_results.items())
            if record.get("valid") is True
        ]
        valid.sort(
            key=lambda record: (
                float(record["max_delta"]),
                float(record["l2_delta"]),
                float(record["sum_delta"]),
                int(record["candidate_index"]),
            )
        )
        self._stage1_orientation_ranked = valid
        if not valid:
            reasons = ";".join(
                str(self._stage1_ik_candidate_results[index].get("reason", ""))
                for index in range(2)
            )
            self._fail_stage1_before_motion(
                "NO_COLLISION_FREE_ROLLER_ORIENTATION:" + reasons
            )
            return
        self._activate_stage1_orientation_rank(0)

    def _activate_stage1_orientation_rank(self, rank_index):
        if self._stage1_orientation_branch_frozen:
            return False
        if not (0 <= int(rank_index) < len(self._stage1_orientation_ranked)):
            return False
        record = self._stage1_orientation_ranked[int(rank_index)]
        self._stage1_orientation_rank_index = int(rank_index)
        self._stage1_retried = False
        self._apply_segment_orientation_candidate(record["candidate"])
        self.get_logger().info(
            "[STAGE 1 SYMMETRY] planning branch %s (%d/%d); endpoint "
            "collision checks for both candidates completed; selected endpoint passed"
            % (
                record["candidate"]["name"],
                int(rank_index) + 1,
                len(self._stage1_orientation_ranked),
            )
        )
        self._send_stage1_plan(
            [copy.deepcopy(record["constraints"])],
            "collision-checked roller branch " + record["candidate"]["name"],
            stage1_token=self._stage1_attempt_token,
            candidate_rank_index=int(rank_index),
        )
        return True

    def _try_next_stage1_orientation_branch(self, reason):
        if self._stage1_orientation_branch_frozen:
            return False
        next_index = int(self._stage1_orientation_rank_index) + 1
        if next_index < len(self._stage1_orientation_ranked):
            self.get_logger().warn(
                f"[STAGE 1 SYMMETRY] current branch plan rejected ({reason}); "
                "trying the other collision-checked orientation"
            )
            return self._activate_stage1_orientation_rank(next_index)
        self._fail_stage1_before_motion(
            "ALL_ROLLER_ORIENTATION_PLANS_REJECTED:" + str(reason)
        )
        return False

    def _stage1_ik_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(
                f"STAGE 1 IK service 실패: {e} -> pose goal fallback")
            self._send_stage1_plan(
                [self._make_pose_constraints(self._safety_tcp_pose)],
                "pose goal fallback",
            )
            return

        if resp.error_code.val != 1:
            self.get_logger().warn(
                f"STAGE 1 IK 실패 error_code={resp.error_code.val} "
                "-> pose goal fallback")
            self._send_stage1_plan(
                [self._make_pose_constraints(self._safety_tcp_pose)],
                "pose goal fallback",
            )
            return

        max_name, max_delta = self._log_stage1_joint_delta(
            resp.solution.joint_state)
        joint_goal = self._make_joint_goal_constraints(
            resp.solution.joint_state,
            tolerance=STAGE1_JOINT_GOAL_TOL,
            joint_names=list(READY_POSE_JOINTS.keys()),
        )
        if not joint_goal.joint_constraints:
            self.get_logger().warn(
                "STAGE 1 IK 결과에 사용 가능한 joint 없음 -> pose goal fallback")
            self._send_stage1_plan(
                [self._make_pose_constraints(self._safety_tcp_pose)],
                "pose goal fallback",
            )
            return

        self._send_stage1_plan([joint_goal], "nearest IK joint goal")

    def _try_stage1_cartesian_approach(self, reason):
        """Use a short Cartesian move to the safety pose when IK flips joints."""
        if not self.cartesian_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "/compute_cartesian_path 서비스 없음 -> Stage 1 중단")
            self._fail_stage1_before_motion("STAGE1_CARTESIAN_SERVICE_UNAVAILABLE")
            return
        if self.current_joint_state is None:
            self.get_logger().error("joint_state 미수신 -> Stage 1 중단")
            self._fail_stage1_before_motion("STAGE1_CARTESIAN_SEED_UNAVAILABLE")
            return

        self.get_logger().warn(
            f"[STAGE 1] {reason} -> local cartesian approach 시도 "
            f"(fraction>={STAGE1_CARTESIAN_FRACTION*100:.1f}% 필요)")

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        req.start_state = rs
        req.waypoints = [self._safety_tcp_pose]
        req.max_step = 0.01
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = STAGE1_SPEED_SCALE
        req.max_acceleration_scaling_factor = STAGE1_SPEED_SCALE

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self._stage1_cartesian_result)

    def _send_stage1_plan(
        self,
        goal_constraints,
        label,
        planner_id=PLANNER_ID,
        *,
        stage1_token=None,
        candidate_rank_index=None,
    ):
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        goal.request.start_state = rs
        goal.request.goal_constraints = goal_constraints
        goal.request.planner_id = planner_id
        goal.request.allowed_planning_time = ALLOWED_PLANNING_TIME
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.max_velocity_scaling_factor = STAGE1_SPEED_SCALE
        goal.request.max_acceleration_scaling_factor = STAGE1_SPEED_SCALE

        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        self._stage1_goal_constraints = goal_constraints
        self.get_logger().info(
            f"STAGE 1 planning request: {label}, "
            f"speed_scale={STAGE1_SPEED_SCALE:.2f}")
        token = stage1_token or self._stage1_attempt_token
        try:
            future = self.move_action_client.send_goal_async(goal)
        except Exception as exc:
            self.get_logger().error(f"STAGE 1 send_goal request 실패: {exc}")
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("SEND_GOAL_REQUEST_FAILED")
            else:
                self._fail_stage1_before_motion("STAGE1_SEND_GOAL_REQUEST_FAILED")
            return
        future.add_done_callback(
            lambda done, t=token, i=candidate_rank_index:
            self._stage1_goal_response(done, t, i)
        )

    def _retry_stage1_with_default_planner(
        self, stage1_token=None, candidate_rank_index=None
    ):
        """planner_id 를 비워서 MoveGroup 의 기본 planner 로 재시도."""
        constraints = self._stage1_goal_constraints or [
            self._make_pose_constraints(self._safety_tcp_pose)
        ]
        self._send_stage1_plan(
            constraints,
            "same goal with default planner",
            planner_id="",
            stage1_token=stage1_token,
            candidate_rank_index=candidate_rank_index,
        )

    def _stage1_goal_response(
        self, future, stage1_token=None, candidate_rank_index=None
    ):
        if (
            stage1_token is not None
            and stage1_token is not self._stage1_attempt_token
        ):
            self.get_logger().warn("stale STAGE 1 goal response ignored")
            return
        if (
            candidate_rank_index is not None
            and int(candidate_rank_index) != self._stage1_orientation_rank_index
        ):
            self.get_logger().warn("stale STAGE 1 branch response ignored")
            return
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 1 planning response ignored after abort")
            return
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"STAGE 1 send_goal 실패: {e}")
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("GOAL_RESPONSE_FAILED")
            else:
                self._fail_stage1_before_motion("STAGE1_GOAL_RESPONSE_FAILED")
            return
        if not handle.accepted:
            self.get_logger().error("STAGE 1 goal rejected")
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("GOAL_REJECTED")
            else:
                self._fail_stage1_before_motion("STAGE1_GOAL_REJECTED")
            return
        self.get_logger().info("STAGE 1 goal accepted, planning...")
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            self.get_logger().error(f"STAGE 1 get_result 실패: {exc}")
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("RESULT_REQUEST_FAILED")
            else:
                self._fail_stage1_before_motion("STAGE1_RESULT_REQUEST_FAILED")
            return
        result_future.add_done_callback(
            lambda done, t=stage1_token, i=candidate_rank_index:
            self._stage1_result(done, t, i)
        )

    def _stage1_result(
        self, future, stage1_token=None, candidate_rank_index=None
    ):
        if (
            stage1_token is not None
            and stage1_token is not self._stage1_attempt_token
        ):
            self.get_logger().warn("stale STAGE 1 planning result ignored")
            return
        if (
            candidate_rank_index is not None
            and int(candidate_rank_index) != self._stage1_orientation_rank_index
        ):
            self.get_logger().warn("stale STAGE 1 branch result ignored")
            return
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 1 planning result ignored after abort")
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"STAGE 1 result 실패: {e}")
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("PLANNING_RESULT_FAILED")
            else:
                self._fail_stage1_before_motion("STAGE1_PLANNING_RESULT_FAILED")
            return
        if result.error_code.val != 1:  # MoveItErrorCodes.SUCCESS = 1
            self.get_logger().error(
                f"STAGE 1 planning 실패 error_code={result.error_code.val}")
            # planner_id mismatch 가능성 — 빈 planner_id 로 한 번 재시도
            if not getattr(self, "_stage1_retried", False):
                self._stage1_retried = True
                self.get_logger().warn(
                    f"planner_id='{PLANNER_ID}' 실패 — 서버 기본 planner 로 재시도")
                self._retry_stage1_with_default_planner(
                    stage1_token,
                    candidate_rank_index,
                )
                return
            self._stage1_retried = False
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch(
                    f"MOVEIT_ERROR_{result.error_code.val}"
                )
            else:
                self._fail_stage1_before_motion(
                    f"STAGE1_MOVEIT_ERROR_{result.error_code.val}"
                )
            return

        # 성공 시 retry flag 리셋
        self._stage1_retried = False

        traj = result.planned_trajectory
        n_points = len(traj.joint_trajectory.points)
        self.get_logger().info(f"STAGE 1 planning OK: {n_points} 포인트")
        if candidate_rank_index is not None:
            try:
                selected_record = self._stage1_orientation_ranked[
                    int(candidate_rank_index)
                ]
            except (IndexError, TypeError, ValueError):
                self._fail_stage1_before_motion(
                    "STAGE1_SELECTED_BRANCH_CONTEXT_INVALID"
                )
                return
            endpoint_ok, endpoint_reason = self._d405_plan_endpoint_matches_ik(
                traj, selected_record
            )
            if not endpoint_ok:
                self._try_next_stage1_orientation_branch(
                    "PLANNED_ENDPOINT_" + endpoint_reason
                )
                return
            start_ok, start_reason = self._d405_plan_start_matches_measured(
                traj,
                self._stage1_ik_seed_state,
                max_age_s=1.0,
            )
            if not start_ok:
                self._fail_stage1_before_motion(
                    "STAGE1_PLAN_START_" + start_reason
                )
                return
        if not self._stage1_trajectory_is_safe(traj, "OMPL"):
            if candidate_rank_index is not None:
                self._try_next_stage1_orientation_branch("UNSAFE_TRAJECTORY")
            else:
                self._fail_stage1_before_motion("STAGE1_UNSAFE_TRAJECTORY")
            return

        # MoveGroup request 의 max_velocity_scaling_factor 가 이미 timing 에
        # 반영되어 있다. 여기서 다시 같은 scale 을 적용하면 0.1 x 0.1 이 되어
        # 접근 동작이 100초 이상 걸린다.
        traj = self._rescale_trajectory(traj, scale=1.0)
        blockers = self._stage1_pre_motion_blockers()
        if blockers:
            self._fail_stage1_before_motion(
                "STAGE1_DISPATCH_READINESS:" + ",".join(blockers)
            )
            return
        if candidate_rank_index is not None:
            self._stage1_orientation_branch_frozen = True
            self._publish_execution_status(
                "SAFETY_APPROACH",
                "collision-checked roller orientation selected",
                orientation_branch=self._selected_segment_orientation_branch,
                collision_checked_orientation_candidates=2,
            )
        on_complete = getattr(self, "_stage1_on_complete", None) \
            or self.stage2_approach_linear
        if not self.execute_trajectory_direct(
            traj,
            on_complete=on_complete,
            on_failure=lambda: self._request_motion_abort(
                "STAGE1_TRAJECTORY_EXECUTION_FAILED"
            ),
            on_rejected=lambda: self._fail_stage1_before_motion(
                "STAGE1_FJT_GOAL_REJECTED"
            ),
            label=(
                "STAGE 1 " + self._selected_segment_orientation_branch
                if candidate_rank_index is not None
                else "STAGE 1"
            ),
        ):
            if (
                not self._motion_abort_requested
                and self._active_trajectory_goal_token is None
                and not self._fjt_motion_state_unknown
            ):
                self._fail_stage1_before_motion("STAGE1_DISPATCH_REJECTED")

    def _stage1_cartesian_result(self, future):
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 1 cartesian result ignored after abort")
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"STAGE 1 cartesian 실패: {e}")
            self._fail_stage1_before_motion("STAGE1_CARTESIAN_RESPONSE_FAILED")
            return

        try:
            fraction = float(resp.fraction)
        except (TypeError, ValueError, AttributeError):
            fraction = math.nan
        self.get_logger().info(
            f"STAGE 1 cartesian: {fraction*100:.1f}%")
        response_error = MoveItExecutor._cartesian_response_error(
            resp, CARTESIAN_COMPLETE_FRACTION
        )
        if response_error:
            self.get_logger().error(
                "STAGE 1 cartesian rejected: " + response_error
            )
            self._fail_stage1_before_motion(
                "STAGE1_CARTESIAN_INCOMPLETE:" + response_error
            )
            return

        traj = resp.solution
        if not self._stage1_trajectory_is_safe(traj, "cartesian"):
            self._fail_stage1_before_motion("STAGE1_CARTESIAN_UNSAFE")
            return
        traj = self._rescale_trajectory(traj, scale=1.0)
        on_complete = getattr(self, "_stage1_on_complete", None) \
            or self.stage2_approach_linear
        if not self.execute_trajectory_direct(
                traj, on_complete=on_complete,
                on_failure=lambda: self._request_motion_abort(
                    "STAGE1_CARTESIAN_EXECUTION_FAILED"
                ),
                on_rejected=lambda: self._fail_stage1_before_motion(
                    "STAGE1_CARTESIAN_FJT_GOAL_REJECTED"
                ),
                label="STAGE 1 cartesian"):
            if (
                not self._motion_abort_requested
                and self._active_trajectory_goal_token is None
                and not self._fjt_motion_state_unknown
            ):
                self._fail_stage1_before_motion(
                    "STAGE1_CARTESIAN_DISPATCH_REJECTED"
                )

    # ---- Stage 2: linear approach to first surface point (cartesian) --------
    def stage2_approach_linear(self):
        self.get_logger().info("=== STAGE 2: linear approach (cartesian) ===")
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_cartesian_path 서비스 없음 (Stage 2)")
            self._fail_workflow_known_safe(
                "LEGACY_STAGE2_CARTESIAN_SERVICE_UNAVAILABLE"
            )
            return

        target = get_target(self.cfg, self.active_target_name)
        _, n = self._active_surface_plane(target)

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK

        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        req.start_state = rs

        # 목표: 첫 접촉-clearance 롤러 중심점의 tcp 좌표
        req.waypoints = [self._stage3_tcp_wps[0]]
        req.max_step = 0.005
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = STAGE2_SPEED_SCALE
        req.max_acceleration_scaling_factor = STAGE2_SPEED_SCALE

        # Orientation constraint (Stage 3 와 동일)
        ee_q = self._active_ee_quat(target)
        brush_dir_world = -np.asarray(n, dtype=float)
        free_axis = int(np.argmax(np.abs(brush_dir_world)))
        tol = [0.2, 0.2, 0.2]
        tol[free_axis] = 3.14
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation.x = float(ee_q[0])
        oc.orientation.y = float(ee_q[1])
        oc.orientation.z = float(ee_q[2])
        oc.orientation.w = float(ee_q[3])
        oc.absolute_x_axis_tolerance = tol[0]
        oc.absolute_y_axis_tolerance = tol[1]
        oc.absolute_z_axis_tolerance = tol[2]
        oc.weight = 1.0
        req.path_constraints = Constraints()
        req.path_constraints.orientation_constraints.append(oc)

        # 디버그: stage 2 의 시작 → 끝 거리와 방향 검증
        end_pose = self._stage3_tcp_wps[0]
        # 현재 tcp 위치는 정확히 모르지만, 직전 stage 1 의 의도된 도착점 (safety_tcp_pose) 로 근사
        start_pose = self._safety_tcp_pose
        delta = np.array([
            end_pose.position.x - start_pose.position.x,
            end_pose.position.y - start_pose.position.y,
            end_pose.position.z - start_pose.position.z,
        ])
        dist = float(np.linalg.norm(delta))
        direction = delta / (dist + 1e-9)
        _, n_target = self._active_surface_plane(target)
        align = float(np.dot(direction, -np.asarray(n_target)))  # +1 이 완벽한 normal 진입
        self.get_logger().info(
            f"[STAGE 2 DEBUG] dist={dist*100:.1f}cm "
            f"direction=({direction[0]:+.2f},{direction[1]:+.2f},{direction[2]:+.2f}) "
            f"normal_align={align:+.3f} (1.0=perfect)")
        if align < MIN_APPROACH_NORMAL_ALIGN:
            self.get_logger().error(
                f"STAGE 2 접근 방향이 surface normal 과 맞지 않음 "
                f"({align:+.3f} < {MIN_APPROACH_NORMAL_ALIGN}) -> 중단")
            self._fail_workflow_known_safe(
                "LEGACY_STAGE2_APPROACH_DIRECTION_INVALID"
            )
            return

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self._stage2_done)

    def _stage2_done(self, future):
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 2 result ignored after abort")
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"STAGE 2 서비스 실패: {e}")
            self._fail_workflow_known_safe(
                f"LEGACY_STAGE2_CARTESIAN_RESPONSE_FAILED:{e}"
            )
            return
        response_error = MoveItExecutor._cartesian_response_error(resp)
        if response_error:
            self._fail_workflow_known_safe(
                "LEGACY_STAGE2_CARTESIAN_REJECTED:" + response_error
            )
            return
        traj = self._rescale_trajectory(resp.solution, scale=1.0)
        if not self.execute_trajectory_direct(
                traj,
                on_complete=self.plan_cartesian,
                on_failure=lambda: self._request_motion_abort(
                    "LEGACY_STAGE2_EXECUTION_FAILED"
                ),
                on_rejected=lambda: self._fail_workflow_known_safe(
                    "LEGACY_STAGE2_FJT_GOAL_REJECTED"
                ),
                force_guard=True,
                label="STAGE 2 approach"):
            self._handle_known_dispatch_rejection("LEGACY_STAGE2")

    # ---- roller center/tool tip → tcp 오프셋 변환 -----------------------------
    @staticmethod
    def _local_axis_in_world(q, axis):
        """쿼터니언 q 기준 로컬 axis ('+x'/'-x'/...) 의 world 방향 단위벡터."""
        x, y, z, w = q.x, q.y, q.z, q.w
        if axis in ("+x", "-x"):
            v = np.array([1 - 2*(y*y + z*z), 2*(x*y + z*w), 2*(x*z - y*w)])
        elif axis in ("+y", "-y"):
            v = np.array([2*(x*y - z*w), 1 - 2*(x*x + z*z), 2*(y*z + x*w)])
        elif axis in ("+z", "-z"):
            v = np.array([2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)])
        else:
            raise ValueError(f"unknown axis: {axis}")
        if axis.startswith("-"):
            v = -v
        return v

    @staticmethod
    def _brush_tip_to_tcp(pose):
        """tool tip (= 롤러 회전축 중심) 기준 좌표를 tcp 기준으로 변환.
        AFT200+roller 는 tcp 의 로컬 TOOL_AXIS 방향으로 EOAT_TIP_OFFSET 만큼 뻗음."""
        axis_world = MoveItExecutor._local_axis_in_world(
            pose.orientation, TOOL_AXIS)
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        new_pos = pos - EOAT_TIP_OFFSET * axis_world
        new_pose = Pose()
        new_pose.position.x = float(new_pos[0])
        new_pose.position.y = float(new_pos[1])
        new_pose.position.z = float(new_pos[2])
        new_pose.orientation = copy.deepcopy(pose.orientation)
        return new_pose

    # ---- 3D 웨이포인트 스플라인 밀집화 ------------------------------------------
    @staticmethod
    def _densify_waypoints(waypoints, spacing_m=0.005):
        """3D poses 를 거리 기준 선형 보간으로 등간격 재샘플."""
        if len(waypoints) < 2:
            return waypoints
        positions = np.array([[p.position.x, p.position.y, p.position.z]
                              for p in waypoints])
        # 중복/매우 가까운 점 제거
        diffs = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        keep = np.concatenate([[True], diffs > 1e-6])
        positions = positions[keep]
        if len(positions) < 2:
            return waypoints
        dists = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        t = np.concatenate([[0], np.cumsum(dists)])
        total = t[-1]
        if total < 1e-6:
            return waypoints
        n = max(len(waypoints), int(total / spacing_m))
        t_new = np.linspace(0, total, n)
        new_wps = []
        ref_ori = waypoints[0].orientation
        xyz = [np.interp(t_new, t, positions[:, i]) for i in range(3)]
        for idx in range(len(t_new)):
            p = Pose()
            p.position.x = float(xyz[0][idx])
            p.position.y = float(xyz[1][idx])
            p.position.z = float(xyz[2][idx])
            p.orientation = copy.deepcopy(ref_ori)
            new_wps.append(p)
        return new_wps

    # ---- Stage 3: Cartesian path 계획 + 실행 --------------------------------
    def plan_cartesian(self):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_cartesian_path 서비스 없음")
            return

        # 새 perception 흐름: sketch_to_waypoints_node 가 이미 wall_plane 좌표를
        # roller center contact-clearance plane 으로 변환해서 보냄.
        # target/n 은 OrientationConstraint 계산을 위해 아래에서 계속 yaml 에서 받음.
        target = get_target(self.cfg, self.active_target_name)
        _sp, n = self._active_surface_plane(target)
        snapped = self._stage3_tip_wps or [
            copy.deepcopy(wp) for wp in self.current_waypoints
        ]
        if not self._validate_contact_waypoints(snapped, target):
            self.executing = False
            return
        self.get_logger().info(
            f"표면 스냅 SKIP (새 perception 흐름): N={len(snapped)} "
            f"첫점=({snapped[0].position.x:.3f},{snapped[0].position.y:.3f},"
            f"{snapped[0].position.z:.3f})")

        # 3D 스플라인 밀집화
        densified = snapped
        self.get_logger().info(
            f"웨이포인트 밀집화: {len(snapped)} -> {len(densified)}")

        # roller center/tool tip → tcp 오프셋 적용
        tcp_wps = [self._brush_tip_to_tcp(wp) for wp in densified]

        # [EOAT_CHECK] 첫 waypoint 변환 검증
        _first_tip = densified[0]
        _first_tcp = tcp_wps[0]
        _dist = np.linalg.norm([
            _first_tip.position.x - _first_tcp.position.x,
            _first_tip.position.y - _first_tcp.position.y,
            _first_tip.position.z - _first_tcp.position.z,
        ])
        self.get_logger().info(
            f"[EOAT_CHECK] roller_center=({_first_tip.position.x:.3f},"
            f"{_first_tip.position.y:.3f},{_first_tip.position.z:.3f}) → "
            f"tcp=({_first_tcp.position.x:.3f},{_first_tcp.position.y:.3f},"
            f"{_first_tcp.position.z:.3f}) 거리={_dist*100:.1f}cm "
            f"(기대={EOAT_TIP_OFFSET*100:.1f}cm)"
        )

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK

        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        req.start_state = rs

        req.waypoints = tcp_wps
        req.max_step = 0.005  # 5mm
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = STAGE3_SPEED_SCALE
        req.max_acceleration_scaling_factor = STAGE3_SPEED_SCALE

        # ---- OrientationConstraint: tcp 의 EE 자세 유지 ----
        # tool 축(tcp 의 TOOL_AXIS) 중심 회전만 자유. 나머지는 tight.
        ee_q = self._active_ee_quat(target)
        brush_dir_world = -np.asarray(n, dtype=float)
        free_axis = int(np.argmax(np.abs(brush_dir_world)))
        tol = [0.2, 0.2, 0.2]
        tol[free_axis] = 3.14
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation.x = float(ee_q[0])
        oc.orientation.y = float(ee_q[1])
        oc.orientation.z = float(ee_q[2])
        oc.orientation.w = float(ee_q[3])
        oc.absolute_x_axis_tolerance = tol[0]
        oc.absolute_y_axis_tolerance = tol[1]
        oc.absolute_z_axis_tolerance = tol[2]
        oc.weight = 1.0
        req.path_constraints = Constraints()
        req.path_constraints.orientation_constraints.append(oc)

        # ---- 디버그: roller center 의 tcp TOOL_AXIS 방향 검증 ----
        first_tip = densified[0]
        tip_p = np.array([first_tip.position.x, first_tip.position.y, first_tip.position.z])
        local_axis_in_world = self._local_axis_in_world(
            first_tip.orientation, TOOL_AXIS)
        self.get_logger().info(
            f"[CHECK] roller_center=({tip_p[0]:.3f},{tip_p[1]:.3f},{tip_p[2]:.3f}) | "
            f"tcp local{TOOL_AXIS} in world=({local_axis_in_world[0]:+.2f},"
            f"{local_axis_in_world[1]:+.2f},{local_axis_in_world[2]:+.2f}) "
            f"[기대: -normal=({brush_dir_world[0]:+.2f},"
            f"{brush_dir_world[1]:+.2f},{brush_dir_world[2]:+.2f})]")
        self.get_logger().info(
            f"Cartesian 요청: {len(tcp_wps)} wp, 첫 tcp=("
            f"{tcp_wps[0].position.x:.3f},{tcp_wps[0].position.y:.3f},"
            f"{tcp_wps[0].position.z:.3f}) | "
            f"ori constraint free_axis={['x','y','z'][free_axis]} tol={tol}")

        fut = self.cartesian_client.call_async(req)
        fut.add_done_callback(self._cartesian_done)

    def _cartesian_done(self, future):
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 3 result ignored after abort")
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Cartesian 서비스 실패: {e}")
            self._fail_workflow_known_safe(
                f"LEGACY_STAGE3_CARTESIAN_RESPONSE_FAILED:{e}"
            )
            return
        response_error = MoveItExecutor._cartesian_response_error(resp)
        if response_error:
            self._fail_workflow_known_safe(
                "LEGACY_STAGE3_CARTESIAN_REJECTED:" + response_error
            )
            return

        traj = self._rescale_trajectory(resp.solution, scale=1.0)
        if not self.execute_trajectory_direct(
                traj,
                on_complete=self.stage4_retreat,
                on_failure=lambda: self._request_motion_abort(
                    "LEGACY_STAGE3_EXECUTION_FAILED"
                ),
                on_rejected=lambda: self._fail_workflow_known_safe(
                    "LEGACY_STAGE3_FJT_GOAL_REJECTED"
                ),
                force_guard=True,
                label="STAGE 3 contact path"):
            self._handle_known_dispatch_rejection("LEGACY_STAGE3")

    # ---- Stage 4: linear retreat (cartesian) --------------------------------
    def stage4_retreat(self):
        self.get_logger().info("=== STAGE 4: linear retreat (cartesian) ===")
        if self._active_segment_path is not None:
            self._publish_painting_command(
                "FINISH_RETRACT", 0.0, enable=False
            )
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_cartesian_path 서비스 없음 (Stage 4)")
            self._fail_workflow_known_safe(
                "FINAL_RETREAT_CARTESIAN_SERVICE_UNAVAILABLE"
            )
            return

        target = get_target(self.cfg, self.active_target_name)
        _, n = self._active_surface_plane(target)

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK

        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        req.start_state = rs

        req.waypoints = [self._retreat_tcp_pose]
        req.max_step = 0.005
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = STAGE4_SPEED_SCALE
        req.max_acceleration_scaling_factor = STAGE4_SPEED_SCALE

        if self._active_segment_path is not None:
            q_retreat = self._retreat_tcp_pose.orientation
            ee_q = [q_retreat.x, q_retreat.y, q_retreat.z, q_retreat.w]
        else:
            ee_q = self._active_ee_quat(target)
        brush_dir_world = -np.asarray(n, dtype=float)
        free_axis = int(np.argmax(np.abs(brush_dir_world)))
        tol = [0.2, 0.2, 0.2]
        tol[free_axis] = 3.14
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation.x = float(ee_q[0])
        oc.orientation.y = float(ee_q[1])
        oc.orientation.z = float(ee_q[2])
        oc.orientation.w = float(ee_q[3])
        oc.absolute_x_axis_tolerance = tol[0]
        oc.absolute_y_axis_tolerance = tol[1]
        oc.absolute_z_axis_tolerance = tol[2]
        oc.weight = 1.0
        req.path_constraints = Constraints()
        req.path_constraints.orientation_constraints.append(oc)

        segment_process = self._active_segment_path is not None
        retreat_start = (
            self._process_last_tcp_pose
            if segment_process
            else self._stage3_tcp_wps[-1]
        )
        retreat_delta = np.array(
            [
                self._retreat_tcp_pose.position.x - retreat_start.position.x,
                self._retreat_tcp_pose.position.y - retreat_start.position.y,
                self._retreat_tcp_pose.position.z - retreat_start.position.z,
            ],
            dtype=float,
        )
        if segment_process and float(np.linalg.norm(retreat_delta)) < 1e-6:
            self.get_logger().info(
                "STAGE 4 생략: 명시적 FINISH_RETRACT가 이미 안전 후퇴 pose에 도달"
            )
            self._all_stages_done()
            return
        if not self._check_normal_motion(
                retreat_start, self._retreat_tcp_pose,
                n, MIN_APPROACH_NORMAL_ALIGN, "STAGE 4"):
            self._fail_workflow_known_safe(
                "FINAL_RETREAT_DIRECTION_INVALID"
            )
            return

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self._stage4_done)

    def _stage4_done(self, future):
        if self._motion_abort_requested:
            self.get_logger().warn("STAGE 4 result ignored after abort")
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"STAGE 4 서비스 실패: {e}")
            self._fail_workflow_known_safe(
                f"FINAL_RETREAT_CARTESIAN_RESPONSE_FAILED:{e}"
            )
            return
        response_error = MoveItExecutor._cartesian_response_error(resp)
        if response_error:
            self._fail_workflow_known_safe(
                "FINAL_RETREAT_CARTESIAN_REJECTED:" + response_error
            )
            return
        traj = self._rescale_trajectory(resp.solution, scale=1.0)
        segment_process = self._active_segment_path is not None
        if not self.execute_trajectory_direct(
                traj,
                on_complete=self._all_stages_done,
                on_failure=lambda: self._request_motion_abort(
                    "FINAL_RETREAT_EXECUTION_FAILED"
                ),
                on_rejected=lambda: self._fail_workflow_known_safe(
                    "FINAL_RETREAT_FJT_GOAL_REJECTED"
                ),
                force_guard=segment_process,
                label="STAGE 4 final retract"):
            self._handle_known_dispatch_rejection("FINAL_RETREAT")

    def _all_stages_done(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info(">>> Stage 1→2→3→4 완료")
        # READY 복귀도 비접촉 free-space motion이다. Stage 4가 끝나는 즉시
        # admittance force를 끄고 segment 실행 상태를 정리한다.
        self._reset_painting_process()
        if RETURN_TO_READY_AFTER_SKETCH:
            self.stage5_return_to_ready()
        else:
            self.get_logger().info(
                ">>> 모든 stage 완료 (1→2→3→4, READY_POSE 복귀 없음)")
            self.get_logger().info("=" * 60)
            self.executing = False

    # ---- Stage 5: return to READY_POSE (joint goal via OMPL) ----------------
    def _is_at_ready_pose(self, tol_rad=0.05):
        """현재 joint state 가 READY_POSE 와 가까운지 (joint 당 tol_rad 이내)."""
        if getattr(self, "model_id", DEFAULT_MODEL) != DEFAULT_MODEL:
            return False
        if self.current_joint_state is None:
            return False
        cs = dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position
        ))
        for jn, target in READY_POSE_JOINTS.items():
            if jn not in cs:
                return False
            if abs(cs[jn] - target) > tol_rad:
                return False
        return True

    def stage5_return_to_ready(self):
        if getattr(self, "model_id", DEFAULT_MODEL) != DEFAULT_MODEL:
            self.get_logger().warn("No commissioned ready pose for this arm; no return motion issued")
            self.executing = False
            return
        self.get_logger().info(
            f"=== STAGE 5: return to READY_POSE (joint goal, {PLANNER_ID}) ===")
        self._plan_joint_goal(
            "READY_POSE",
            READY_POSE_JOINTS,
            STAGE5_SPEED_SCALE,
            finalize_cb=self._stage5_finalize,
        )

    def _plan_joint_goal(self, label, joints, speed_scale, finalize_cb):
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"MoveGroup action server 없음 — {label} 이동 불가")
            finalize_cb(success=False)
            return
        if self.current_joint_state is None:
            self.get_logger().warn(f"joint_state 미수신 — {label} 이동 불가")
            finalize_cb(success=False)
            return
        if not self._joint_state_within_limits(
                self.current_joint_state, f"{label} start"):
            self.get_logger().warn(
                f"{label} 이동 불가 — 현재 joint_state 가 limit 밖")
            finalize_cb(success=False)
            return

        self._joint_goal_context = {
            "label": label,
            "finalize_cb": finalize_cb,
        }

        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        goal.request.start_state = rs

        # Joint goal constraints
        constraints = Constraints()
        for jn, target in joints.items():
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(target)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]

        goal.request.planner_id = PLANNER_ID
        goal.request.allowed_planning_time = ALLOWED_PLANNING_TIME
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.max_velocity_scaling_factor = speed_scale
        goal.request.max_acceleration_scaling_factor = speed_scale

        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(self._joint_goal_response)

    def _plan_pose_goal(self, label, pose, speed_scale, finalize_cb):
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"MoveGroup action server 없음 — {label} 이동 불가")
            finalize_cb(success=False)
            return
        if self.current_joint_state is None:
            self.get_logger().warn(f"joint_state 미수신 — {label} 이동 불가")
            finalize_cb(success=False)
            return
        if not self._joint_state_within_limits(
                self.current_joint_state, f"{label} start"):
            self.get_logger().warn(
                f"{label} 이동 불가 — 현재 joint_state 가 limit 밖")
            finalize_cb(success=False)
            return

        self._pose_goal_context = {
            "label": label,
            "finalize_cb": finalize_cb,
        }

        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        rs = RobotState()
        rs.joint_state = self.current_joint_state
        rs.is_diff = False
        goal.request.start_state = rs
        goal.request.goal_constraints = [
            self._make_pose_constraints(pose)
        ]
        goal.request.planner_id = PLANNER_ID
        goal.request.allowed_planning_time = ALLOWED_PLANNING_TIME
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.max_velocity_scaling_factor = speed_scale
        goal.request.max_acceleration_scaling_factor = speed_scale

        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(self._pose_goal_response)

    @staticmethod
    def _d405_probe_pose_for_scale(context, scale):
        """Return a shorter target without changing the probe orientation."""

        start = np.asarray(context["start_tcp_position"], dtype=float)
        original = context["original_pose"]
        goal = np.array(
            [
                original.position.x,
                original.position.y,
                original.position.z,
            ],
            dtype=float,
        )
        scale = float(scale)
        if (
            start.shape != (3,)
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(goal))
            or not math.isfinite(scale)
            or scale <= 0.0
            or scale > 1.0
        ):
            return None
        displacement = (goal - start) * scale
        if float(np.linalg.norm(displacement)) < D405_PREFLIGHT_ADAPTIVE_MIN_MOVE_M:
            return None
        pose = copy.deepcopy(original)
        target = start + displacement
        pose.position.x = float(target[0])
        pose.position.y = float(target[1])
        pose.position.z = float(target[2])
        return pose

    def _finish_d405_probe_context(self, context, success):
        if context.get("finished"):
            return
        if not self._d405_prescan_callback_valid(context.get("token")):
            return
        context["finished"] = True
        current = getattr(self, "_d405_cartesian_context", None)
        if current is context or (
            isinstance(current, dict) and current.get("root") is context
        ):
            self._d405_cartesian_context = None
        context["finalize_cb"](bool(success))

    def _plan_d405_probe_cartesian(
        self,
        label,
        pose,
        finalize_cb,
        *,
        token=None,
        retry_index=0,
        root_context=None,
    ):
        """Plan a complete D405 probe, retrying only with a shorter target.

        A partial Cartesian solution is never dispatched.  All retries retain
        the same measured joint seed, start TCP position, prescan token, and
        confirmed planning-scene revision.
        """

        token = token if token is not None else getattr(
            self, "_d405_prescan_token", None
        )
        if not self._d405_prescan_callback_valid(token):
            return

        if root_context is None:
            scene_revision = int(getattr(self, "_scene_revision", 0))
            if not self._d405_scene_revision_confirmed(scene_revision):
                self.get_logger().error(
                    f"[D405 PRESCAN] {label}: scene revision is not confirmed"
                )
                finalize_cb(False)
                return
            if self.current_joint_state is None:
                self.get_logger().warn(
                    f"joint_state 미수신 — {label} 이동 불가"
                )
                finalize_cb(False)
                return
            seed_age = time.monotonic() - float(
                getattr(self, "current_joint_state_time", 0.0)
            )
            if (
                not math.isfinite(seed_age)
                or seed_age < 0.0
                or seed_age > D405_PREFLIGHT_JOINT_STATE_MAX_AGE_S
                or not self._joint_state_within_limits(
                    self.current_joint_state, f"{label} Cartesian seed"
                )
            ):
                self.get_logger().warn(
                    f"[D405 PRESCAN] {label}: joint seed stale/invalid "
                    f"(age={seed_age:.3f}s)"
                )
                finalize_cb(False)
                return
            current_tcp = self._current_tcp_pose_np()
            if current_tcp is None:
                self.get_logger().warn(
                    f"[D405 PRESCAN] {label}: current TCP pose unavailable"
                )
                finalize_cb(False)
                return
            root_context = {
                "token": token,
                "scene_revision": scene_revision,
                "label": str(label),
                "finalize_cb": finalize_cb,
                "seed_state": copy.deepcopy(self.current_joint_state),
                "start_tcp_position": np.asarray(current_tcp[0], dtype=float).copy(),
                "original_pose": copy.deepcopy(pose),
                "finished": False,
                "fjt_dispatched": False,
            }
        elif (
            root_context.get("finished")
            or root_context.get("token") is not token
            or not self._d405_scene_revision_confirmed(
                root_context.get("scene_revision")
            )
        ):
            if not root_context.get("finished"):
                self._finish_d405_probe_context(root_context, False)
            return

        scales = (1.0,) + tuple(D405_PREFLIGHT_ADAPTIVE_PROBE_SCALES)
        retry_index = int(retry_index)
        if retry_index < 0 or retry_index >= len(scales):
            self._finish_d405_probe_context(root_context, False)
            return
        scale = float(scales[retry_index])
        target_pose = self._d405_probe_pose_for_scale(root_context, scale)
        if target_pose is None:
            self.get_logger().warn(
                f"[D405 PRESCAN] {label}: adaptive probe target is too short/invalid"
            )
            self._finish_d405_probe_context(root_context, False)
            return
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"/compute_cartesian_path 서비스 없음 — {label} 이동 불가"
            )
            self._finish_d405_probe_context(root_context, False)
            return

        attempt = {
            "root": root_context,
            "token": token,
            "retry_index": retry_index,
            "scale": scale,
            "label": str(label),
        }
        self._d405_cartesian_context = attempt

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = PLANNING_GROUP
        req.link_name = EE_LINK
        rs = RobotState()
        rs.joint_state = copy.deepcopy(root_context["seed_state"])
        rs.is_diff = False
        req.start_state = rs
        req.waypoints = [target_pose]
        req.max_step = 0.01
        req.jump_threshold = 2.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = D405_PREFLIGHT_SPEED_SCALE
        req.max_acceleration_scaling_factor = D405_PREFLIGHT_SPEED_SCALE

        distance = float(np.linalg.norm(
            np.array(
                [
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z,
                ],
                dtype=float,
            )
            - root_context["start_tcp_position"]
        ))
        self.get_logger().info(
            f"[D405 PRESCAN] {label}: Cartesian probe attempt "
            f"{retry_index + 1}/{len(scales)}, target={distance * 1000.0:.1f}mm"
        )
        try:
            future = self.cartesian_client.call_async(req)
        except Exception as exc:
            self.get_logger().warn(f"{label} cartesian request failed: {exc}")
            self._finish_d405_probe_context(root_context, False)
            return
        future.add_done_callback(
            lambda done, ctx=attempt: self._d405_probe_cartesian_done(done, ctx)
        )

    def _d405_probe_cartesian_done(self, future, context=None):
        context = context or getattr(self, "_d405_cartesian_context", None)
        if not isinstance(context, dict):
            return
        root = context.get("root")
        if (
            not isinstance(root, dict)
            or context is not getattr(self, "_d405_cartesian_context", None)
            or root.get("finished")
            or not self._d405_prescan_callback_valid(context.get("token"))
        ):
            return
        label = root["label"]
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{label} cartesian 서비스 실패: {exc}")
            self._finish_d405_probe_context(root, False)
            return

        error_code = int(getattr(getattr(resp, "error_code", None), "val", 0))
        if error_code != 1:
            self.get_logger().warn(
                f"[D405 PRESCAN] {label}: Cartesian service error {error_code}"
            )
            self._finish_d405_probe_context(root, False)
            return
        try:
            fraction = float(resp.fraction)
        except (TypeError, ValueError, AttributeError) as exc:
            self.get_logger().error(
                f"[D405 PRESCAN] {label}: invalid Cartesian fraction: {exc}"
            )
            self._finish_d405_probe_context(root, False)
            return
        self.get_logger().info(
            f"[D405 PRESCAN] {label} cartesian fraction={fraction*100:.1f}%"
        )
        if not math.isfinite(fraction) or fraction < 0.0 or fraction > 1.0 + 1e-6:
            self.get_logger().error(
                f"[D405 PRESCAN] {label}: invalid Cartesian fraction {fraction!r}"
            )
            self._finish_d405_probe_context(root, False)
            return
        if fraction < D405_PREFLIGHT_CARTESIAN_FRACTION:
            retry_index = int(context["retry_index"])
            next_retry = retry_index + 1
            scales = (1.0,) + tuple(D405_PREFLIGHT_ADAPTIVE_PROBE_SCALES)
            if next_retry < len(scales):
                self.get_logger().warn(
                    f"{label} Cartesian fraction {fraction*100:.1f}% < "
                    f"{D405_PREFLIGHT_CARTESIAN_FRACTION*100:.1f}%: "
                    "partial trajectory discarded; retrying a shorter target"
                )
                self._plan_d405_probe_cartesian(
                    label,
                    root["original_pose"],
                    root["finalize_cb"],
                    token=root["token"],
                    retry_index=next_retry,
                    root_context=root,
                )
                return
            self.get_logger().warn(
                f"{label} Cartesian fraction {fraction*100:.1f}% < "
                f"{D405_PREFLIGHT_CARTESIAN_FRACTION*100:.1f}%: "
                "partial trajectory discarded"
            )
            self._finish_d405_probe_context(root, False)
            return

        if not self._d405_scene_revision_confirmed(root["scene_revision"]):
            self.get_logger().error(
                f"[D405 PRESCAN] {label}: scene changed before probe dispatch"
            )
            self._finish_d405_probe_context(root, False)
            return
        traj = resp.solution
        if not self._d405_prescan_trajectory_is_safe(traj, label):
            self._finish_d405_probe_context(root, False)
            return
        if not self._d405_scene_revision_confirmed(root["scene_revision"]):
            self._finish_d405_probe_context(root, False)
            return
        start_ok, start_reason = self._d405_plan_start_matches_measured(
            traj, root["seed_state"]
        )
        if not start_ok:
            self.get_logger().error(
                f"[D405 PRESCAN] {label}: probe dispatch rejected: "
                + start_reason
            )
            self._finish_d405_probe_context(root, False)
            return

        traj = self._rescale_trajectory(traj, scale=1.0)
        root["fjt_dispatched"] = True

        def execution_succeeded():
            if not self._d405_prescan_callback_valid(root["token"]):
                return
            current = getattr(self, "_d405_cartesian_context", None)
            if (
                not isinstance(current, dict)
                or current.get("root") is not root
                or not self._d405_scene_revision_confirmed(
                    root["scene_revision"]
                )
            ):
                self._request_motion_abort(
                    "D405_PRESCAN_CONTEXT_OR_SCENE_CHANGED_AFTER_FJT"
                )
                return
            self._start_d405_post_fjt_verification(root, traj)

        def execution_failed():
            if self._d405_prescan_callback_valid(root["token"]):
                self._request_motion_abort(
                    "D405_PRESCAN_FJT_EXECUTION_FAILED:" + label
                )

        dispatched = self.execute_trajectory_direct(
            traj,
            on_complete=execution_succeeded,
            on_failure=execution_failed,
            on_rejected=lambda: (
                root.__setitem__("fjt_dispatched", False),
                self._finish_d405_probe_context(root, False),
            ),
            label=label,
        )
        if not dispatched:
            if (
                getattr(self, "_active_trajectory_goal_token", None) is not None
                or getattr(self, "_fjt_motion_state_unknown", False)
            ):
                self._request_motion_abort(
                    "D405_PRESCAN_FJT_DISPATCH_UNKNOWN:" + label
                )
            else:
                root["fjt_dispatched"] = False
                self._finish_d405_probe_context(root, False)

    def _pose_goal_response(self, future):
        ctx = self._pose_goal_context or {}
        label = ctx.get("label", "pose goal")
        finalize_cb = ctx.get("finalize_cb", lambda success: None)
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} response 무시 — motion abort 상태")
            finalize_cb(success=False)
            return
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"{label} send_goal 실패: {e}")
            finalize_cb(success=False)
            return
        if not handle.accepted:
            self.get_logger().warn(f"{label} goal rejected")
            finalize_cb(success=False)
            return
        self.get_logger().info(f"{label} goal accepted, planning...")
        handle.get_result_async().add_done_callback(self._pose_goal_result)

    def _pose_goal_result(self, future):
        ctx = self._pose_goal_context or {}
        label = ctx.get("label", "pose goal")
        finalize_cb = ctx.get("finalize_cb", lambda success: None)
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} result 무시 — motion abort 상태")
            finalize_cb(success=False)
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().warn(f"{label} result 실패: {e}")
            finalize_cb(success=False)
            return
        if result.error_code.val != 1:
            self.get_logger().warn(
                f"{label} planning 실패 error_code={result.error_code.val}")
            finalize_cb(success=False)
            return

        traj = result.planned_trajectory
        n_points = len(traj.joint_trajectory.points)
        self.get_logger().info(f"{label} planning OK: {n_points} 포인트")
        if not self._d405_prescan_trajectory_is_safe(traj, label):
            finalize_cb(success=False)
            return
        traj = self._rescale_trajectory(traj, scale=1.0)
        if not self.execute_trajectory_direct(
                traj,
                on_complete=lambda: finalize_cb(success=True),
                on_failure=lambda: self._request_motion_abort(
                    f"{label}:FJT_EXECUTION_FAILED"
                ),
                on_rejected=lambda: finalize_cb(success=False),
                label=label):
            finalize_cb(success=False)

    def _joint_goal_response(self, future):
        ctx = self._joint_goal_context or {}
        label = ctx.get("label", "joint goal")
        finalize_cb = ctx.get("finalize_cb", lambda success: None)
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} response 무시 — motion abort 상태")
            finalize_cb(success=False)
            return
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"{label} send_goal 실패: {e}")
            finalize_cb(success=False)
            return
        if not handle.accepted:
            self.get_logger().warn(f"{label} goal rejected")
            finalize_cb(success=False)
            return
        self.get_logger().info(f"{label} goal accepted, planning...")
        handle.get_result_async().add_done_callback(self._joint_goal_result)

    def _joint_goal_result(self, future):
        ctx = self._joint_goal_context or {}
        label = ctx.get("label", "joint goal")
        finalize_cb = ctx.get("finalize_cb", lambda success: None)
        if self._motion_abort_requested:
            self.get_logger().warn(f"{label} result 무시 — motion abort 상태")
            finalize_cb(success=False)
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().warn(f"{label} result 실패: {e}")
            finalize_cb(success=False)
            return
        if result.error_code.val != 1:
            self.get_logger().warn(
                f"{label} planning 실패 error_code={result.error_code.val}")
            finalize_cb(success=False)
            return

        traj = result.planned_trajectory
        n_points = len(traj.joint_trajectory.points)
        self.get_logger().info(f"{label} planning OK: {n_points} 포인트")
        traj = self._rescale_trajectory(traj, scale=1.0)
        if not self.execute_trajectory_direct(
                traj,
                on_complete=lambda: finalize_cb(success=True),
                on_failure=lambda: self._request_motion_abort(
                    f"{label}:FJT_EXECUTION_FAILED"
                ),
                on_rejected=lambda: finalize_cb(success=False),
                label=label):
            finalize_cb(success=False)

    def _stage5_finalize(self, success):
        self._joint_goal_context = None
        self._reset_painting_process()
        if success:
            self.get_logger().info("Stage 5 완료: READY_POSE 복귀")
            self.get_logger().info(">>> 모든 stage 완료 (1→2→3→4→5)")
        else:
            self.get_logger().warn(
                "Stage 5 (READY 복귀) 실패. 다음 Submit 의 시작 위치 부적합 가능.")
        self.get_logger().info("=" * 60)
        self.executing = False

    def _rescale_trajectory(self, traj, scale=0.3):
        if not traj.joint_trajectory.points:
            return traj
        if scale >= 0.999:
            return traj
        for p in traj.joint_trajectory.points:
            total_ns = p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec
            total_ns = int(total_ns / scale)
            p.time_from_start.sec = total_ns // 1_000_000_000
            p.time_from_start.nanosec = total_ns % 1_000_000_000
            if p.velocities:
                p.velocities = [v * scale for v in p.velocities]
            if p.accelerations:
                p.accelerations = [a * scale * scale for a in p.accelerations]
        return traj


def main(args=None):
    rclpy.init(args=args)
    node = MoveItExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
