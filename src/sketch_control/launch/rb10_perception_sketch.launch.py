import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_ZED_CALIBRATION_FILE = (
    str(Path(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws")) / "zed_d405_apriltag_calibration.json")
)
DEFAULT_D405_CALIBRATION_FILE = (
    str(Path(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws")) / "d405_eyeinhand_charuco_calibration.json")
)


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_inverse(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / n, -y / n, -z / n, w / n)


def _quat_normalize(q):
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _is_truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_zed_optical_pose(context):
    zed_x = float(LaunchConfiguration("zed_x").perform(context))
    zed_y = float(LaunchConfiguration("zed_y").perform(context))
    zed_z = float(LaunchConfiguration("zed_z").perform(context))
    q_world_optical = _quat_normalize((
        float(LaunchConfiguration("zed_qx").perform(context)),
        float(LaunchConfiguration("zed_qy").perform(context)),
        float(LaunchConfiguration("zed_qz").perform(context)),
        float(LaunchConfiguration("zed_qw").perform(context)),
    ))

    use_file = _is_truthy(
        LaunchConfiguration("use_zed_calibration_file").perform(context))
    path = Path(
        LaunchConfiguration("zed_calibration_file").perform(context)
    ).expanduser()
    pose_key = str(
        LaunchConfiguration("zed_calibration_pose_key").perform(context)
    ).strip() or "T_world_zed_optical"
    if not use_file:
        return zed_x, zed_y, zed_z, q_world_optical
    if not path.exists():
        get_logger("rb10_perception_sketch").warning(
            f"ZED calibration file not found, using launch args: {path}")
        return zed_x, zed_y, zed_z, q_world_optical

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if pose_key not in data:
            get_logger("rb10_perception_sketch").warning(
                f"ZED calibration key {pose_key!r} not found in {path}; "
                "using 'T_world_zed_optical'")
            pose_key = "T_world_zed_optical"
        pose = data[pose_key]
        t = pose["translation"]
        q = pose["rotation_xyzw"]
        zed_x, zed_y, zed_z = [float(v) for v in t[:3]]
        q_world_optical = _quat_normalize(tuple(float(v) for v in q[:4]))
    except Exception as exc:
        get_logger("rb10_perception_sketch").warning(
            "Failed to read ZED calibration file, using launch args: "
            f"{path}: {exc}")
        return zed_x, zed_y, zed_z, q_world_optical

    get_logger("rb10_perception_sketch").info(
        "Loaded ZED calibration from "
        f"{path} [{pose_key}]: "
        f"t=({zed_x:+.4f}, {zed_y:+.4f}, {zed_z:+.4f}), "
        f"q=({q_world_optical[0]:+.5f}, {q_world_optical[1]:+.5f}, "
        f"{q_world_optical[2]:+.5f}, {q_world_optical[3]:+.5f})")
    return zed_x, zed_y, zed_z, q_world_optical


def _load_d405_mount_pose(context):
    """Return tcp->D405 base frame for perception TF only.

    The calibration JSON stores tcp->d405_color_optical_frame.  RealSense uses
    a fixed d405_link->color_optical rotation, so convert the optical pose back
    to the camera base frame that the RealSense driver publishes.
    """
    d405_x = float(LaunchConfiguration("d405_x").perform(context))
    d405_y = float(LaunchConfiguration("d405_y").perform(context))
    d405_z = float(LaunchConfiguration("d405_z").perform(context))
    q_tcp_link = _quat_normalize((
        float(LaunchConfiguration("d405_qx").perform(context)),
        float(LaunchConfiguration("d405_qy").perform(context)),
        float(LaunchConfiguration("d405_qz").perform(context)),
        float(LaunchConfiguration("d405_qw").perform(context)),
    ))
    if q_tcp_link[3] < 0.0:
        q_tcp_link = tuple(-v for v in q_tcp_link)

    use_file = _is_truthy(
        LaunchConfiguration("use_d405_calibration_file").perform(context))
    path = Path(
        LaunchConfiguration("d405_calibration_file").perform(context)
    ).expanduser()
    pose_key = str(
        LaunchConfiguration("d405_calibration_pose_key").perform(context)
    ).strip() or "T_d405_optical_to_tcp"
    if not use_file:
        return d405_x, d405_y, d405_z, q_tcp_link
    if not path.exists():
        get_logger("rb10_perception_sketch").warning(
            f"D405 calibration file not found, using launch args: {path}")
        return d405_x, d405_y, d405_z, q_tcp_link

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if pose_key not in data:
            get_logger("rb10_perception_sketch").warning(
                f"D405 calibration key {pose_key!r} not found in {path}; "
                "using launch args")
            return d405_x, d405_y, d405_z, q_tcp_link
        pose = data[pose_key]
        t = pose["translation"]
        q_tcp_optical = _quat_normalize(
            tuple(float(v) for v in pose["rotation_xyzw"][:4]))
        d405_x, d405_y, d405_z = [float(v) for v in t[:3]]

        # RealSense ROS optical convention from d405_link to color optical:
        # rpy=(-pi/2, 0, -pi/2), translation zero in _d405.urdf.xacro.
        q_link_to_optical = (-0.5, 0.5, -0.5, 0.5)
        q_tcp_link = _quat_normalize(
            _quat_multiply(q_tcp_optical, _quat_inverse(q_link_to_optical))
        )
        if q_tcp_link[3] < 0.0:
            q_tcp_link = tuple(-v for v in q_tcp_link)
    except Exception as exc:
        get_logger("rb10_perception_sketch").warning(
            "Failed to read D405 calibration file, using launch args: "
            f"{path}: {exc}")
        return d405_x, d405_y, d405_z, q_tcp_link

    get_logger("rb10_perception_sketch").info(
        "Loaded D405 calibration from "
        f"{path} [{pose_key}] for perception TF: "
        f"tcp->d405_link t=({d405_x:+.4f}, {d405_y:+.4f}, {d405_z:+.4f}), "
        f"q=({q_tcp_link[0]:+.5f}, {q_tcp_link[1]:+.5f}, "
        f"{q_tcp_link[2]:+.5f}, {q_tcp_link[3]:+.5f})")
    return d405_x, d405_y, d405_z, q_tcp_link


def _make_zed_static_tfs(context, *args, **kwargs):
    """Publish World->ZED left camera frame from the calibrated optical pose."""
    zed_x, zed_y, zed_z, q_world_optical = _load_zed_optical_pose(context)

    # ROS optical convention from zed_left_camera_frame to optical frame:
    # rpy=(-pi/2, 0, -pi/2).  Translation is zero in the ZED wrapper URDF.
    q_left_to_optical = (-0.5, 0.5, -0.5, 0.5)
    q_world_left = _quat_normalize(
        _quat_multiply(q_world_optical, _quat_inverse(q_left_to_optical))
    )

    def _fmt(v):
        return f"{float(v):.17g}"

    world_to_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_world_to_left_camera_static_tf",
        output="screen",
        arguments=[
            "--frame-id", "World",
            "--child-frame-id", "zed_left_camera_frame",
            "--x", _fmt(zed_x),
            "--y", _fmt(zed_y),
            "--z", _fmt(zed_z),
            "--qx", _fmt(q_world_left[0]),
            "--qy", _fmt(q_world_left[1]),
            "--qz", _fmt(q_world_left[2]),
            "--qw", _fmt(q_world_left[3]),
        ],
    )
    left_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_left_camera_to_optical_static_tf",
        output="screen",
        arguments=[
            "--frame-id", "zed_left_camera_frame",
            "--child-frame-id", "zed_left_camera_frame_optical",
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", _fmt(q_left_to_optical[0]),
            "--qy", _fmt(q_left_to_optical[1]),
            "--qz", _fmt(q_left_to_optical[2]),
            "--qw", _fmt(q_left_to_optical[3]),
        ],
    )
    return [world_to_left, left_to_optical]


def _make_d405_mount_static_tf(context, *args, **kwargs):
    if not _is_truthy(LaunchConfiguration("use_d405_mount_tf").perform(context)):
        return []

    d405_x, d405_y, d405_z, q_tcp_link = _load_d405_mount_pose(context)
    child_frame = LaunchConfiguration("d405_mount_child_frame").perform(context)

    def _fmt(v):
        return f"{float(v):.17g}"

    return [Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d405_mount_static_tf",
        output="screen",
        arguments=[
            "--frame-id", "tcp",
            "--child-frame-id", child_frame,
            "--x", _fmt(d405_x),
            "--y", _fmt(d405_y),
            "--z", _fmt(d405_z),
            "--qx", _fmt(q_tcp_link[0]),
            "--qy", _fmt(q_tcp_link[1]),
            "--qz", _fmt(q_tcp_link[2]),
            "--qw", _fmt(q_tcp_link[3]),
        ],
    )]


def generate_launch_description():
    painting_config_file = LaunchConfiguration("painting_config_file")
    real_painting_enabled = LaunchConfiguration("real_painting_enabled")
    dry_run = LaunchConfiguration("dry_run")
    use_sim_depth_pointcloud = LaunchConfiguration("use_sim_depth_pointcloud")
    use_sim_d405_depth_pointcloud = LaunchConfiguration(
        "use_sim_d405_depth_pointcloud")
    launch_wall_detector = LaunchConfiguration("launch_wall_detector")
    launch_environment_scanner = LaunchConfiguration("launch_environment_scanner")
    use_d405_refinement = LaunchConfiguration("use_d405_refinement")
    use_d405_mount_tf = LaunchConfiguration("use_d405_mount_tf")
    use_d405_optical_tf = LaunchConfiguration("use_d405_optical_tf")
    d405_mount_child_frame = LaunchConfiguration("d405_mount_child_frame")
    use_ft_normal_controller = LaunchConfiguration("use_ft_normal_controller")
    ft_wrench_topic = LaunchConfiguration("ft_wrench_topic")
    ft_force_sign = LaunchConfiguration("ft_force_sign")
    ft_target_force_n = LaunchConfiguration("ft_target_force_n")
    ft_contact_threshold_n = LaunchConfiguration("ft_contact_threshold_n")
    ft_warn_force_n = LaunchConfiguration("ft_warn_force_n")
    ft_abort_force_n = LaunchConfiguration("ft_abort_force_n")
    ft_torque_warn_nm = LaunchConfiguration("ft_torque_warn_nm")
    ft_torque_abort_nm = LaunchConfiguration("ft_torque_abort_nm")
    ft_torque_balance_gain_rad_per_nm = LaunchConfiguration(
        "ft_torque_balance_gain_rad_per_nm")
    ft_max_orientation_correction_rad = LaunchConfiguration(
        "ft_max_orientation_correction_rad")
    zed_calibration_pose_key = LaunchConfiguration("zed_calibration_pose_key")

    zed_static_tfs = OpaqueFunction(function=_make_zed_static_tfs)
    d405_mount_static_tf = OpaqueFunction(function=_make_d405_mount_static_tf)

    # D405 is rigidly mounted on the TCP/EOAT. The RealSense driver normally
    # publishes the D405 base -> optical frames; this bridges the moving robot
    # TCP to the camera base link so the D405 cloud can be transformed to
    # ZED/world.  realsense2_camera may prefix the base frame, e.g.
    # d405_d405_link when camera_namespace:=d405 and camera_name:=d405.
    # The transform is created by _make_d405_mount_static_tf so real runs can
    # load the wrist-camera calibration JSON without moving the MoveIt mesh.

    static_d405_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d405_optical_static_tf",
        output="screen",
        condition=IfCondition(use_d405_optical_tf),
        arguments=[
            "--frame-id", "d405_link",
            "--child-frame-id", "d405_color_optical_frame",
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", "-0.5",
            "--qy", "0.5",
            "--qz", "-0.5",
            "--qw", "0.5",
        ],
    )

    depth_to_pointcloud = Node(
        package="sketch_control",
        executable="depth_to_pointcloud",
        name="depth_to_pointcloud",
        output="screen",
        condition=IfCondition(use_sim_depth_pointcloud),
    )

    d405_depth_to_pointcloud = Node(
        package="sketch_control",
        executable="depth_to_pointcloud",
        name="d405_depth_to_pointcloud",
        output="screen",
        condition=IfCondition(use_sim_d405_depth_pointcloud),
        parameters=[{
            "depth_topic": "/d405/d405/depth/image_rect_raw",
            "camera_info_topic": "/d405/d405/depth/camera_info",
            "point_cloud_topic": "/d405/d405/depth/color/points",
            "stride": 2,
            "min_depth_m": 0.05,
            "max_depth_m": 1.2,
        }],
    )

    wall_detector = Node(
        package="sketch_control",
        executable="wall_detector",
        name="wall_detector",
        output="screen",
        condition=IfCondition(launch_wall_detector),
    )

    target_selector = Node(
        package="sketch_control",
        executable="target_selector",
        name="target_selector",
        output="screen",
    )

    wall_projector = Node(
        package="sketch_control",
        executable="wall_projector",
        name="wall_projector",
        output="screen",
        parameters=[
            painting_config_file,
            {
                "front_view_source": LaunchConfiguration("front_view_source"),
            },
        ],
    )

    sketch_to_waypoints = Node(
        package="sketch_control",
        executable="sketch_to_waypoints",
        name="sketch_to_waypoints",
        output="screen",
        parameters=[
            painting_config_file,
            {
                "real_painting_enabled": ParameterValue(
                    real_painting_enabled, value_type=bool
                ),
                "dry_run": ParameterValue(dry_run, value_type=bool),
            },
        ],
    )

    environment_scanner = Node(
        package="sketch_control",
        executable="environment_scanner",
        name="environment_scanner",
        output="screen",
        condition=IfCondition(launch_environment_scanner),
        parameters=[painting_config_file],
    )

    d405_surface_refiner = Node(
        package="sketch_control",
        executable="d405_surface_refiner",
        name="d405_surface_refiner",
        output="screen",
        condition=IfCondition(use_d405_refinement),
        parameters=[painting_config_file],
    )

    ft_normal_controller = Node(
        package="sketch_control",
        executable="ft_normal_controller",
        name="ft_normal_controller",
        output="screen",
        condition=IfCondition(use_ft_normal_controller),
        parameters=[{
            "wrench_topic": ft_wrench_topic,
            "base_frame": "link0",
            "sensor_frame": "tcp",
            "lock_refined_surface": True,
            # "5"/"-1" 같은 값이 INTEGER 로 파싱되면 double 선언과 충돌해
            # 노드가 즉사하므로 float 타입을 명시적으로 강제한다.
            "force_sign": ParameterValue(ft_force_sign, value_type=float),
            "target_force_n": ParameterValue(ft_target_force_n, value_type=float),
            "contact_threshold_n": ParameterValue(
                ft_contact_threshold_n, value_type=float),
            "warn_force_n": ParameterValue(ft_warn_force_n, value_type=float),
            "abort_force_n": ParameterValue(ft_abort_force_n, value_type=float),
            "torque_warn_nm": ParameterValue(ft_torque_warn_nm, value_type=float),
            "torque_abort_nm": ParameterValue(ft_torque_abort_nm, value_type=float),
            "torque_balance_gain_rad_per_nm": ParameterValue(
                ft_torque_balance_gain_rad_per_nm, value_type=float),
            "max_orientation_correction_rad": ParameterValue(
                ft_max_orientation_correction_rad, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "painting_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sketch_control"),
                "config",
                "painting_system_real.yaml",
            ]),
            description="Shared painting-system ROS parameter file",
        ),
        DeclareLaunchArgument(
            "real_painting_enabled",
            default_value="false",
            description="Enable strict real-painting path gates",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Keep robot/force execution disabled while true",
        ),
        DeclareLaunchArgument(
            "use_sim_depth_pointcloud",
            default_value="true",
            description="Isaac Sim depth image -> ZED-compatible PointCloud2",
        ),
        DeclareLaunchArgument(
            "use_sim_d405_depth_pointcloud",
            default_value="true",
            description="Isaac Sim D405 depth image -> D405 PointCloud2",
        ),
        DeclareLaunchArgument(
            "launch_wall_detector",
            default_value="true",
            description=(
                "Run the continuous legacy ZED whole-cloud wall RANSAC. "
                "The D405 authoritative front-view workflow does not need it."
            ),
        ),
        DeclareLaunchArgument(
            "launch_environment_scanner",
            default_value="true",
            description="Run the optional ZED whole-cloud environment scanner",
        ),
        DeclareLaunchArgument(
            "use_d405_refinement",
            default_value="true",
            description="Use wrist D405 point cloud to refine work-area distance",
        ),
        DeclareLaunchArgument(
            "use_d405_mount_tf",
            default_value="true",
            description="Publish tcp->d405_link static mount transform",
        ),
        DeclareLaunchArgument(
            "d405_mount_child_frame",
            default_value="d405_link",
            description=(
                "D405 base frame attached to tcp. Use d405_d405_link with "
                "the real realsense2_camera namespace/name launch."
            ),
        ),
        DeclareLaunchArgument(
            "use_d405_calibration_file",
            default_value="false",
            description=(
                "Load tcp->D405 optical hand-eye calibration JSON and convert "
                "it to tcp->d405_link for perception TF only."
            ),
        ),
        DeclareLaunchArgument(
            "d405_calibration_file",
            default_value=DEFAULT_D405_CALIBRATION_FILE,
            description="JSON saved by d405_eyeinhand_charuco_calibrator",
        ),
        DeclareLaunchArgument(
            "d405_calibration_pose_key",
            default_value="T_d405_optical_to_tcp",
            description=(
                "Pose entry containing tcp->d405_color_optical_frame in the "
                "D405 hand-eye calibration JSON."
            ),
        ),
        DeclareLaunchArgument(
            "use_d405_optical_tf",
            default_value="true",
            description=(
                "Publish manual d405_link->d405_color_optical_frame TF. "
                "Set false when realsense2_camera publishes D405 camera TFs."
            ),
        ),
        DeclareLaunchArgument(
            "use_ft_normal_controller",
            default_value="true",
            description="Compute wall-normal contact force from AFT200 wrench",
        ),
        DeclareLaunchArgument(
            "ft_wrench_topic",
            default_value="/aft200/ft",
            description="AFT200 WrenchStamped topic",
        ),
        DeclareLaunchArgument(
            "ft_force_sign",
            default_value="1.0",
            description="Use -1.0 if pushing the wall reports negative normal force",
        ),
        DeclareLaunchArgument(
            "ft_target_force_n",
            default_value="1.6",
            description="Desired roller normal force during painting",
        ),
        DeclareLaunchArgument(
            "ft_contact_threshold_n",
            default_value="0.6",
            description="Contact detection threshold for wall-normal force",
        ),
        DeclareLaunchArgument(
            "ft_warn_force_n",
            default_value="2.4",
            description="Warning threshold for wall-normal force",
        ),
        DeclareLaunchArgument(
            "ft_abort_force_n",
            default_value="5.0",
            description="Emergency stop threshold for contact stages",
        ),
        DeclareLaunchArgument(
            "ft_torque_warn_nm",
            default_value="0.12",
            description="Warning threshold for roller balance torque",
        ),
        DeclareLaunchArgument(
            "ft_torque_abort_nm",
            default_value="0.25",
            description="Emergency stop threshold for roller balance torque",
        ),
        DeclareLaunchArgument(
            "ft_torque_balance_gain_rad_per_nm",
            default_value="0.08",
            description="Orientation correction gain from balance torque",
        ),
        DeclareLaunchArgument(
            "ft_max_orientation_correction_rad",
            default_value="0.03",
            description="Max per-chunk orientation correction from balance torque",
        ),
        DeclareLaunchArgument(
            "use_zed_calibration_file",
            default_value="true",
            description=(
                "Load World->ZED optical calibration from JSON when present. "
                "Falls back to zed_x/y/z/q launch args if missing."
            ),
        ),
        DeclareLaunchArgument(
            "zed_calibration_file",
            default_value=DEFAULT_ZED_CALIBRATION_FILE,
            description=(
                "JSON saved by apriltag_dual_camera_calibrator containing "
                "T_world_zed_optical."
            ),
        ),
        DeclareLaunchArgument(
            "zed_calibration_pose_key",
            default_value="T_world_zed_optical",
            description=(
                "Pose entry to load from the ZED calibration JSON. Use "
                "T_base_zed_optical when World and link0 are the same frame."
            ),
        ),
        DeclareLaunchArgument(
            "zed_x",
            default_value="-0.4715750877078567",
            description="World->zed_left_camera_frame_optical calibrated x",
        ),
        DeclareLaunchArgument(
            "zed_y",
            default_value="0.2562866180459926",
            description="World->zed_left_camera_frame_optical calibrated y",
        ),
        DeclareLaunchArgument(
            "zed_z",
            default_value="1.0085930379522832",
            description="World->zed_left_camera_frame_optical calibrated z",
        ),
        DeclareLaunchArgument(
            "zed_qx",
            default_value="-0.5362126233398715",
            description="World->zed_left_camera_frame_optical calibrated qx",
        ),
        DeclareLaunchArgument(
            "zed_qy",
            default_value="0.6265018657327951",
            description="World->zed_left_camera_frame_optical calibrated qy",
        ),
        DeclareLaunchArgument(
            "zed_qz",
            default_value="-0.4297485208739163",
            description="World->zed_left_camera_frame_optical calibrated qz",
        ),
        DeclareLaunchArgument(
            "zed_qw",
            default_value="0.36781468650800403",
            description="World->zed_left_camera_frame_optical calibrated qw",
        ),
        DeclareLaunchArgument(
            "d405_x",
            default_value="0.00905",
            description="Fallback tcp->d405_link x for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_y",
            default_value="-0.07640",
            description="Fallback tcp->d405_link y for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_z",
            default_value="0.04375",
            description="Fallback tcp->d405_link z for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qx",
            default_value="0.0",
            description="Fallback tcp->d405_link qx for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qy",
            default_value="0.0",
            description="Fallback tcp->d405_link qy for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qz",
            default_value="-0.7071067811865475",
            description="Fallback tcp->d405_link qz for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qw",
            default_value="0.7071067811865476",
            description="Fallback tcp->d405_link qw for perception TF",
        ),
        DeclareLaunchArgument(
            "front_view_source",
            default_value="d405",
            description=(
                "wall_front 캔버스 소스: 'd405'(작업영역/경로를 D405 정면 뷰에서) "
                "또는 'zed'(기존 ZED warp, 회귀용)"
            ),
        ),
        zed_static_tfs,
        d405_mount_static_tf,
        static_d405_optical_tf,
        depth_to_pointcloud,
        d405_depth_to_pointcloud,
        wall_detector,
        target_selector,
        wall_projector,
        sketch_to_waypoints,
        environment_scanner,
        d405_surface_refiner,
        ft_normal_controller,
    ])
