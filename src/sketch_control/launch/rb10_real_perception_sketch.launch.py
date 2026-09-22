import json
import os
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from sketch_control.outpost_camera import camera_status


def _validate_camera_backend(context):
    backend = LaunchConfiguration('camera_backend').perform(context)
    if backend not in ('native', 'outpost'):
        raise ValueError('camera_backend must be native or outpost')
    if backend == 'outpost':
        for name, kind in (('zed', 'zed'), ('d405', 'realsense')):
            if LaunchConfiguration(f'launch_{name}_driver').perform(context).lower() == 'true':
                raise ValueError('Outpost must be the sole camera owner; disable native drivers')
            camera_status(LaunchConfiguration('outpost_http').perform(context),
                          LaunchConfiguration(f'outpost_{name}_hw_id').perform(context),
                          LaunchConfiguration(f'outpost_{name}_serial').perform(context), kind)
    return []


DEFAULT_FT_CONFIG_PATH = str(Path(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws")) / "aft200_force_threshold.json")
DEFAULT_D405_CALIBRATION_FILE = (
    str(Path(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws")) / "d405_eyeinhand_charuco_calibration.json")
)


def _fmt_float(value, fallback):
    try:
        v = float(value)
        if not math.isfinite(v):
            return str(fallback)
    except (TypeError, ValueError):
        return str(fallback)
    text = f"{v:.6g}"
    # 소수점/지수 표기가 없으면 launch 가 INTEGER 로 파싱해
    # double 파라미터 선언과 충돌하므로 ".0" 을 붙여 DOUBLE 을 강제한다.
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return text


def _load_ft_defaults():
    defaults = {
        "force_sign": "1.0",
        "target_force_n": "1.6",
        "contact_threshold_n": "0.6",
        "warn_force_n": "2.4",
        "abort_force_n": "5.0",
        "torque_warn_nm": "0.12",
        "torque_abort_nm": "0.25",
        "torque_balance_gain_rad_per_nm": "0.08",
        "max_orientation_correction_rad": "0.03",
    }
    path = Path(DEFAULT_FT_CONFIG_PATH).expanduser()
    if not path.exists():
        return defaults
    try:
        data = json.loads(path.read_text())
    except Exception:
        return defaults
    if not isinstance(data, dict):
        return defaults
    key_map = {
        "force_sign": "force_sign",
        "target_force_n": "target_force_n",
        "contact_threshold_n": "contact_threshold_n",
        "warn_force_n": "warn_force_n",
        "abort_force_n": "abort_force_n",
        "torque_warn_nm": "torque_warn_nm",
        "torque_abort_nm": "torque_abort_nm",
        "torque_balance_gain_rad_per_nm": "torque_balance_gain_rad_per_nm",
        "max_orientation_correction_rad": "max_orientation_correction_rad",
    }
    for out_key, json_key in key_map.items():
        if json_key in data:
            defaults[out_key] = _fmt_float(data[json_key], defaults[out_key])
    target = float(defaults["target_force_n"])
    abort = float(defaults["abort_force_n"])
    if "contact_threshold_n" not in data:
        defaults["contact_threshold_n"] = _fmt_float(
            max(0.3, round(0.4 * target, 1)), "0.6")
    if "warn_force_n" not in data:
        warn = max(target + 2.0, abort * 0.8)
        if warn >= abort:
            warn = abort * 0.9
        defaults["warn_force_n"] = _fmt_float(warn, "20.0")
    return defaults


def generate_launch_description():
    ft_defaults = _load_ft_defaults()
    is_outpost = PythonExpression(["'", LaunchConfiguration('camera_backend'), "' == 'outpost'"])
    outpost_bridge = Node(package='sketch_control', executable='outpost_bridge',
        name='sketch_outpost_bridge', output='screen', condition=IfCondition(is_outpost),
        parameters=[{key: ParameterValue(LaunchConfiguration(key), value_type=str) for key in
            ('outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial')}])

    painting_config_file = LaunchConfiguration("painting_config_file")
    real_painting_enabled = LaunchConfiguration("real_painting_enabled")
    dry_run = LaunchConfiguration("dry_run")
    launch_zed_driver = LaunchConfiguration("launch_zed_driver")
    launch_d405_driver = LaunchConfiguration("launch_d405_driver")
    launch_aft_ethernet_driver = LaunchConfiguration("launch_aft_ethernet_driver")
    launch_rbpodo_eft_bridge = LaunchConfiguration("launch_rbpodo_eft_bridge")
    use_ft_normal_controller = LaunchConfiguration("use_ft_normal_controller")
    launch_wall_detector = LaunchConfiguration("launch_wall_detector")
    launch_environment_scanner = LaunchConfiguration("launch_environment_scanner")

    zed_camera_model = LaunchConfiguration("zed_camera_model")
    zed_serial_number = LaunchConfiguration("zed_serial_number")
    zed_camera_id = LaunchConfiguration("zed_camera_id")
    zed_param_overrides = LaunchConfiguration("zed_param_overrides")

    d405_serial_no = LaunchConfiguration("d405_serial_no")
    d405_usb_port_id = LaunchConfiguration("d405_usb_port_id")
    d405_initial_reset = LaunchConfiguration("d405_initial_reset")
    d405_depth_profile = LaunchConfiguration("d405_depth_profile")
    d405_color_profile = LaunchConfiguration("d405_color_profile")

    ft_wrench_topic = LaunchConfiguration("ft_wrench_topic")
    rbpodo_system_state_topic = LaunchConfiguration("rbpodo_system_state_topic")
    aft_sensor_ip = LaunchConfiguration("aft_sensor_ip")
    aft_sensor_port = LaunchConfiguration("aft_sensor_port")
    aft_transport = LaunchConfiguration("aft_transport")
    aft_frame_id = LaunchConfiguration("aft_frame_id")
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

    zed_x = LaunchConfiguration("zed_x")
    zed_y = LaunchConfiguration("zed_y")
    zed_z = LaunchConfiguration("zed_z")
    zed_qx = LaunchConfiguration("zed_qx")
    zed_qy = LaunchConfiguration("zed_qy")
    zed_qz = LaunchConfiguration("zed_qz")
    zed_qw = LaunchConfiguration("zed_qw")
    use_zed_calibration_file = LaunchConfiguration("use_zed_calibration_file")
    zed_calibration_file = LaunchConfiguration("zed_calibration_file")
    zed_calibration_pose_key = LaunchConfiguration("zed_calibration_pose_key")
    use_d405_calibration_file = LaunchConfiguration("use_d405_calibration_file")
    d405_calibration_file = LaunchConfiguration("d405_calibration_file")
    d405_calibration_pose_key = LaunchConfiguration("d405_calibration_pose_key")
    d405_x = LaunchConfiguration("d405_x")
    d405_y = LaunchConfiguration("d405_y")
    d405_z = LaunchConfiguration("d405_z")
    d405_qx = LaunchConfiguration("d405_qx")
    d405_qy = LaunchConfiguration("d405_qy")
    d405_qz = LaunchConfiguration("d405_qz")
    d405_qw = LaunchConfiguration("d405_qw")

    zed_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            ])
        ),
        condition=IfCondition(launch_zed_driver),
        launch_arguments={
            "camera_name": "zed",
            "camera_model": zed_camera_model,
            "node_name": "zed_node",
            "serial_number": zed_serial_number,
            "camera_id": zed_camera_id,
            "use_sim_time": "false",
            "sim_mode": "false",
            "publish_urdf": "false",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "false",
            "enable_ipc": "false",
            "param_overrides": zed_param_overrides,
        }.items(),
    )

    d405_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        condition=IfCondition(launch_d405_driver),
        launch_arguments={
            "camera_namespace": "d405",
            "camera_name": "d405",
            "device_type": "d405",
            "serial_no": d405_serial_no,
            "usb_port_id": d405_usb_port_id,
            "initial_reset": d405_initial_reset,
            "enable_color": "true",
            "enable_depth": "true",
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "false",
            "pointcloud.allow_no_texture_points": "false",
            "depth_module.depth_profile": d405_depth_profile,
            "depth_module.color_profile": d405_color_profile,
            "publish_tf": "true",
            "base_frame_id": "d405_link",
            "tf_prefix": "",
            "log_level": "info",
        }.items(),
    )

    aft_driver = Node(
        package="sketch_control",
        executable="aft200_ethernet_driver",
        name="aft200_ethernet_driver",
        output="screen",
        condition=IfCondition(launch_aft_ethernet_driver),
        parameters=[{
            "sensor_ip": aft_sensor_ip,
            "sensor_port": aft_sensor_port,
            "transport": aft_transport,
            "frame_id": aft_frame_id,
            "wrench_topic": ft_wrench_topic,
        }],
    )

    rbpodo_eft_bridge = Node(
        package="sketch_control",
        executable="rbpodo_eft_bridge",
        name="rbpodo_eft_bridge",
        output="screen",
        condition=IfCondition(launch_rbpodo_eft_bridge),
        parameters=[{
            "system_state_topic": rbpodo_system_state_topic,
            "frame_id": aft_frame_id,
            "wrench_topic": ft_wrench_topic,
        }],
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("sketch_control"),
                "launch",
                "rb10_perception_sketch.launch.py",
            ])
        ),
        launch_arguments={
            "painting_config_file": painting_config_file,
            "real_painting_enabled": real_painting_enabled,
            "dry_run": dry_run,
            "use_sim_depth_pointcloud": "false",
            "use_sim_d405_depth_pointcloud": "false",
            "launch_wall_detector": launch_wall_detector,
            "launch_environment_scanner": launch_environment_scanner,
            "front_view_source": LaunchConfiguration("front_view_source"),
            "use_d405_refinement": "true",
            "use_ft_normal_controller": use_ft_normal_controller,
            "use_d405_mount_tf": "true",
            "use_d405_optical_tf": PythonExpression(["'true' if '", LaunchConfiguration('camera_backend'), "' == 'outpost' else 'false'"]),
            "d405_mount_child_frame": PythonExpression(["'d405_link' if '", LaunchConfiguration('camera_backend'), "' == 'outpost' else 'd405_d405_link'"]),
            "ft_wrench_topic": ft_wrench_topic,
            "ft_force_sign": ft_force_sign,
            "ft_target_force_n": ft_target_force_n,
            "ft_contact_threshold_n": ft_contact_threshold_n,
            "ft_warn_force_n": ft_warn_force_n,
            "ft_abort_force_n": ft_abort_force_n,
            "ft_torque_warn_nm": ft_torque_warn_nm,
            "ft_torque_abort_nm": ft_torque_abort_nm,
            "ft_torque_balance_gain_rad_per_nm": ft_torque_balance_gain_rad_per_nm,
            "ft_max_orientation_correction_rad": ft_max_orientation_correction_rad,
            "use_zed_calibration_file": use_zed_calibration_file,
            "zed_calibration_file": zed_calibration_file,
            "zed_calibration_pose_key": zed_calibration_pose_key,
            "use_d405_calibration_file": use_d405_calibration_file,
            "d405_calibration_file": d405_calibration_file,
            "d405_calibration_pose_key": d405_calibration_pose_key,
            "d405_x": d405_x,
            "d405_y": d405_y,
            "d405_z": d405_z,
            "d405_qx": d405_qx,
            "d405_qy": d405_qy,
            "d405_qz": d405_qz,
            "d405_qw": d405_qw,
            "zed_x": zed_x,
            "zed_y": zed_y,
            "zed_z": zed_z,
            "zed_qx": zed_qx,
            "zed_qy": zed_qy,
            "zed_qz": zed_qz,
            "zed_qw": zed_qw,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_backend', default_value='outpost'),
        DeclareLaunchArgument('outpost_http', default_value='http://127.0.0.1:8100'),
        *[DeclareLaunchArgument(key, default_value='') for key in
          ('outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial')],
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
            description="Keep motion/force execution in dry-run mode",
        ),
        DeclareLaunchArgument(
            "front_view_source",
            default_value="d405",
            description=(
                "wall_front 캔버스 소스: 'd405'(작업영역/경로를 D405 정면 뷰에서) "
                "또는 'zed'(기존 ZED warp, 회귀용)"
            ),
        ),
        DeclareLaunchArgument(
            "launch_zed_driver",
            default_value="false",
            description="Launch Stereolabs zed_wrapper for the real ZED camera",
        ),
        DeclareLaunchArgument(
            "launch_d405_driver",
            default_value="false",
            description="Launch realsense2_camera for the wrist D405",
        ),
        DeclareLaunchArgument(
            "launch_wall_detector",
            default_value="false",
            description=(
                "Run legacy continuous whole-ZED-cloud RANSAC. Disabled for "
                "the authoritative D405 front-view painting workflow."
            ),
        ),
        DeclareLaunchArgument(
            "launch_environment_scanner",
            default_value="false",
            description=(
                "Run optional ZED point-cloud environment scanning. Static "
                "MoveIt scene geometry remains available when disabled."
            ),
        ),
        DeclareLaunchArgument(
            "launch_rbpodo_eft_bridge",
            default_value="true",
            description="Bridge RB controller SystemState.eft to /aft200/ft",
        ),
        DeclareLaunchArgument(
            "use_ft_normal_controller",
            default_value="false",
            description="Launch ft_normal_controller inside this perception launch",
        ),
        DeclareLaunchArgument(
            "launch_aft_ethernet_driver",
            default_value="false",
            description="Launch direct Ethernet driver for standalone AFT200-D80-EN",
        ),
        DeclareLaunchArgument(
            "zed_camera_model",
            default_value="zed2i",
            description="ZED model passed to zed_wrapper",
        ),
        DeclareLaunchArgument(
            "zed_serial_number",
            default_value="0",
            description="ZED serial number; 0 lets the wrapper choose the camera",
        ),
        DeclareLaunchArgument(
            "zed_camera_id",
            default_value="-1",
            description="ZED camera ID; -1 lets the wrapper choose the camera",
        ),
        DeclareLaunchArgument(
            "zed_param_overrides",
            default_value=(
                "general.grab_compute_capping_fps:=15.0;"
                "general.pub_frame_rate:=10.0;"
                "depth.publish_point_cloud:=false;"
                "depth.max_depth:=2.5;"
                "depth.depth_stabilization:=0;"
                "pos_tracking.pos_tracking_enabled:=false"
            ),
            description="Semicolon-separated zed_wrapper parameter overrides",
        ),
        DeclareLaunchArgument(
            "d405_serial_no",
            default_value="''",
            description="D405 serial number; empty quotes let librealsense choose",
        ),
        DeclareLaunchArgument(
            "d405_usb_port_id",
            default_value="''",
            description="D405 USB port ID; empty quotes let librealsense choose",
        ),
        DeclareLaunchArgument(
            "d405_initial_reset",
            default_value="false",
            description="Reset the D405 at startup if device enumeration is flaky",
        ),
        DeclareLaunchArgument(
            "d405_depth_profile",
            default_value="640x480x15",
            description="D405 depth stream profile used by point-cloud refinement",
        ),
        DeclareLaunchArgument(
            "d405_color_profile",
            default_value="640x480x15",
            description="D405 color stream profile used by the wall-front UI",
        ),
        DeclareLaunchArgument(
            "ft_wrench_topic",
            default_value="/aft200/ft",
            description="AFT200 WrenchStamped topic",
        ),
        DeclareLaunchArgument(
            "rbpodo_system_state_topic",
            default_value="/rbpodo_hardware/system_state",
            description="RB controller SystemState topic containing the eft field",
        ),
        DeclareLaunchArgument(
            "aft_sensor_ip",
            default_value="192.168.1.199",
            description="AFT200 sensor IP address",
        ),
        DeclareLaunchArgument(
            "aft_sensor_port",
            default_value="0",
            description="AFT200 port. 0 auto-probes 50000 then legacy 80/8890",
        ),
        DeclareLaunchArgument(
            "aft_transport",
            default_value="tcp",
            description="AFT200 transport: tcp or udp",
        ),
        DeclareLaunchArgument(
            "aft_frame_id",
            default_value="tcp",
            description="Frame id used in /aft200/ft messages",
        ),
        DeclareLaunchArgument(
            "ft_force_sign",
            default_value=ft_defaults["force_sign"],
            description="Use -1.0 if pushing the work surface reports negative force",
        ),
        DeclareLaunchArgument(
            "ft_target_force_n",
            default_value=ft_defaults["target_force_n"],
            description="Desired roller normal force during painting",
        ),
        DeclareLaunchArgument(
            "ft_contact_threshold_n",
            default_value=ft_defaults["contact_threshold_n"],
            description="Contact detection threshold for wall-normal force",
        ),
        DeclareLaunchArgument(
            "ft_warn_force_n",
            default_value=ft_defaults["warn_force_n"],
            description="Warning threshold for wall-normal force",
        ),
        DeclareLaunchArgument(
            "ft_abort_force_n",
            default_value=ft_defaults["abort_force_n"],
            description="Emergency stop threshold for contact stages",
        ),
        DeclareLaunchArgument(
            "ft_torque_warn_nm",
            default_value=ft_defaults["torque_warn_nm"],
            description="Warning threshold for roller balance torque",
        ),
        DeclareLaunchArgument(
            "ft_torque_abort_nm",
            default_value=ft_defaults["torque_abort_nm"],
            description="Emergency stop threshold for roller balance torque",
        ),
        DeclareLaunchArgument(
            "ft_torque_balance_gain_rad_per_nm",
            default_value=ft_defaults["torque_balance_gain_rad_per_nm"],
            description="Orientation correction gain from balance torque",
        ),
        DeclareLaunchArgument(
            "ft_max_orientation_correction_rad",
            default_value=ft_defaults["max_orientation_correction_rad"],
            description="Max per-chunk orientation correction from balance torque",
        ),
        DeclareLaunchArgument(
            "use_zed_calibration_file",
            default_value="true",
            description=(
                "Load ZED calibration JSON saved by "
                "apriltag_dual_camera_calibrator when present."
            ),
        ),
        DeclareLaunchArgument(
            "zed_calibration_file",
            default_value=str(Path(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws")) / "zed_d405_apriltag_calibration.json"),
            description="Path to ZED calibration JSON file",
        ),
        DeclareLaunchArgument(
            "zed_calibration_pose_key",
            default_value="T_world_zed_optical",
            description=(
                "Pose entry to load from the ZED calibration JSON. The "
                "current measured real-cell camera pose uses T_world_zed_optical."
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
            "use_d405_calibration_file",
            default_value="true",
            description=(
                "Load D405 eye-in-hand calibration for perception TF only. "
                "MoveIt/URDF mesh remains nominal."
            ),
        ),
        DeclareLaunchArgument(
            "d405_calibration_file",
            default_value=DEFAULT_D405_CALIBRATION_FILE,
            description="Path to D405 eye-in-hand calibration JSON file",
        ),
        DeclareLaunchArgument(
            "d405_calibration_pose_key",
            default_value="T_d405_optical_to_tcp",
            description="Pose entry in the D405 calibration JSON",
        ),
        DeclareLaunchArgument(
            "d405_x",
            default_value="0.009647071938425011",
            description="Fallback calibrated tcp->d405_link x for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_y",
            default_value="-0.07061966473868395",
            description="Fallback calibrated tcp->d405_link y for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_z",
            default_value="0.06153146466141274",
            description="Fallback calibrated tcp->d405_link z for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qx",
            default_value="-0.023971150931625784",
            description="Fallback calibrated tcp->d405_link qx for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qy",
            default_value="-0.0048795469934848",
            description="Fallback calibrated tcp->d405_link qy for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qz",
            default_value="-0.6992492523656593",
            description="Fallback calibrated tcp->d405_link qz for perception TF",
        ),
        DeclareLaunchArgument(
            "d405_qw",
            default_value="0.7144592759634508",
            description="Fallback calibrated tcp->d405_link qw for perception TF",
        ),
        OpaqueFunction(function=_validate_camera_backend),
        RegisterEventHandler(OnProcessExit(target_action=outpost_bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Outpost bridge stopped; perception invalid'))])),
        outpost_bridge,
        zed_driver,
        d405_driver,
        aft_driver,
        rbpodo_eft_bridge,
        perception,
    ])
