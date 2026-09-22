"""Top-level, fail-closed launch for the RB10/RB20 sketch system."""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from sketch_control.robot_models import (DEFAULT_MODEL, validate_model,
                                         model_calibration_files, validate_calibration_files)
from sketch_control.outpost_camera import camera_status


def _validate_interlock_values(
    *,
    use_fake_hardware,
    use_isaac_sim,
    real_painting_enabled,
    dry_run,
    painting_force_enabled,
    spray_motion_test=False,
):
    """Reject launch combinations that could bypass the real-motion gate."""

    real_hardware = not bool(use_fake_hardware) and not bool(use_isaac_sim)
    if bool(spray_motion_test) and bool(painting_force_enabled):
        raise RuntimeError("spray_motion_test requires painting_force_enabled=false")
    if real_hardware and not bool(dry_run) and not bool(real_painting_enabled):
        raise RuntimeError(
            "real hardware with dry_run=false requires "
            "real_painting_enabled=true; refusing unvalidated geometry motion"
        )
    if bool(painting_force_enabled) and (
        not bool(real_painting_enabled) or bool(dry_run)
    ):
        raise RuntimeError(
            "painting_force_enabled=true requires "
            "real_painting_enabled=true and dry_run=false"
        )


def _validate_launch_interlocks(context, *args, **kwargs):
    del args, kwargs
    model = validate_model(LaunchConfiguration("model_id").perform(context))

    def enabled(name):
        return LaunchConfiguration(name).perform(context).strip().lower() in {
            "true",
            "1",
            "yes",
            "on",
        }

    _validate_interlock_values(
        use_fake_hardware=enabled("use_fake_hardware"),
        use_isaac_sim=enabled("use_isaac_sim"),
        real_painting_enabled=enabled("real_painting_enabled"),
        dry_run=enabled("dry_run"),
        painting_force_enabled=enabled("painting_force_enabled"),
        spray_motion_test=enabled("spray_motion_test"),
    )
    paths = model_calibration_files(os.environ.get("SKETCH_WORKSPACE", "~/sketch_robot_ws"), model)
    for key in paths:
        paths[key] = LaunchConfiguration(key).perform(context).strip() or paths[key]
    if (model != DEFAULT_MODEL and enabled("launch_perception")
            and not enabled("use_fake_hardware") and not enabled("use_isaac_sim")):
        validate_calibration_files(paths)
    backend = LaunchConfiguration('camera_backend').perform(context)
    if backend not in ('outpost', 'native'):
        raise ValueError('camera_backend must be outpost or native')
    if enabled('use_fake_hardware') or enabled('use_isaac_sim'):
        paths['camera_backend'] = 'native'
    elif enabled('launch_perception') and backend == 'outpost':
        for camera, kind in (('zed', 'zed'), ('d405', 'realsense')):
            if enabled(f'launch_{camera}_driver'):
                raise ValueError('Outpost requires native camera drivers disabled')
            camera_status(LaunchConfiguration('outpost_http').perform(context),
                LaunchConfiguration(f'outpost_{camera}_hw_id').perform(context),
                LaunchConfiguration(f'outpost_{camera}_serial').perform(context), kind)
    return [SetLaunchConfiguration(key, value) for key, value in paths.items()]


def generate_launch_description():
    config_file = LaunchConfiguration("painting_config_file")
    real_painting_enabled = LaunchConfiguration("real_painting_enabled")
    dry_run = LaunchConfiguration("dry_run")
    force_enabled = LaunchConfiguration("painting_force_enabled")
    fake_sensor_commands = PythonExpression([
        "'true' if '",
        LaunchConfiguration("fake_sensor_commands"),
        "'.lower() in ('true', '1', 'yes', 'on') or (('",
        LaunchConfiguration("fake_sensor_commands"),
        "'.lower() == 'auto') and ('",
        LaunchConfiguration("use_fake_hardware"),
        "'.lower() in ('true', '1', 'yes', 'on'))) else 'false'",
    ])

    arguments = [
        DeclareLaunchArgument("zed_calibration_file", default_value=""),
        DeclareLaunchArgument("d405_calibration_file", default_value=""),
        DeclareLaunchArgument(
            "spray_motion_test", default_value="false",
            description="Current EOAT without gun: spray path motion only; gun output always OFF",
        ),
        DeclareLaunchArgument(
            "painting_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sketch_control"),
                "config",
                "painting_system_real.yaml",
            ]),
            description="Single ROS parameter file shared by the painting nodes",
        ),
        DeclareLaunchArgument(
            "real_painting_enabled",
            default_value="false",
            description="First explicit interlock for real robot painting",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Second interlock; must be false for real motion/force",
        ),
        DeclareLaunchArgument(
            "painting_force_enabled",
            default_value="false",
            description="Third interlock; non-zero wrench remains off by default",
        ),
        DeclareLaunchArgument(
            "desired_contact_force_n",
            default_value="20.0",
            description=(
                "Force-pipeline activation/ramp magnitude; the real profile "
                "uses it as the lower edge of the 20..30 N hold band"
            ),
        ),
        DeclareLaunchArgument(
            "stationary_paint_hold_s",
            default_value="0.0",
            description=(
                "Optional hold for a zero-distance PAINT row; zero preserves "
                "normal path execution"
            ),
        ),
        DeclareLaunchArgument(
            "enable_interlock_flight_recorder",
            default_value="true",
            description=(
                "Keep an in-memory pre-trigger motion/force history and dump it "
                "when a robot interlock or motion abort occurs"
            ),
        ),
        DeclareLaunchArgument("robot_ip", default_value="10.0.2.7"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="auto",
            description=(
                "Use injectable fake F/T data. 'auto' enables it only when "
                "use_fake_hardware is true, avoiding a real robot connection."
            ),
        ),
        DeclareLaunchArgument("use_isaac_sim", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        # The HTTP supervisor starts these existing groups independently.
        # Defaults retain the original integrated launch behavior.
        DeclareLaunchArgument("launch_robot_control", default_value="true"),
        DeclareLaunchArgument("launch_perception", default_value="true"),
        DeclareLaunchArgument("launch_force_pipeline", default_value="true"),
        DeclareLaunchArgument("launch_executor", default_value="true"),
        DeclareLaunchArgument(
            "launch_rosbridge",
            default_value="false",
            description="Start rosbridge_websocket for the browser UI",
        ),
        DeclareLaunchArgument(
            "rosbridge_max_message_size",
            default_value="10000000",
            description=(
                "Maximum rosbridge JSON message size. Raw ZED/D405 Image "
                "messages exceed the rosbridge node's 1 MB built-in default."
            ),
        ),
        DeclareLaunchArgument("model_id", default_value="rb10_1300e_u"),
        DeclareLaunchArgument("launch_zed_driver", default_value="false"),
        DeclareLaunchArgument("launch_d405_driver", default_value="false"),
        DeclareLaunchArgument('camera_backend', default_value='outpost'),
        DeclareLaunchArgument('outpost_http', default_value='http://127.0.0.1:8100'),
        *[DeclareLaunchArgument(key, default_value='') for key in
          ('outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial')],
        DeclareLaunchArgument(
            "launch_wall_detector",
            default_value="false",
            description=(
                "Legacy continuous ZED wall RANSAC; unnecessary when the "
                "authoritative front view is D405."
            ),
        ),
        DeclareLaunchArgument(
            "launch_environment_scanner",
            default_value="false",
            description="Optional ZED whole-cloud environment scan pipeline",
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
        ),
        DeclareLaunchArgument("d405_depth_profile", default_value="640x480x15"),
        DeclareLaunchArgument("d405_color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("launch_rbpodo_eft_bridge", default_value="true"),
        DeclareLaunchArgument("launch_aft_ethernet_driver", default_value="false"),
        DeclareLaunchArgument("front_view_source", default_value="d405"),
        DeclareLaunchArgument("d405_serial_no", default_value="''"),
        DeclareLaunchArgument("d405_usb_port_id", default_value="''"),
    ]

    moveit = IncludeLaunchDescription(
        condition=IfCondition(LaunchConfiguration("launch_robot_control")),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("sketch_control"),
                "launch",
                "rb10_moveit_full.launch.py",
            ])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "fake_sensor_commands": fake_sensor_commands,
            "use_isaac_sim": LaunchConfiguration("use_isaac_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "model_id": LaunchConfiguration("model_id"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "use_admittance": "true",
            # The shared painting_config_file is appended last by the child
            # launch and contains the complete controller profile.
            "admittance_profile": "",
            "admittance_use_case": "painting",
            "painting_config_file": config_file,
            "start_admittance_reset": "false",
            "start_variable_impedance": "false",
        }.items(),
    )

    perception = IncludeLaunchDescription(
        condition=IfCondition(LaunchConfiguration("launch_perception")),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("sketch_control"),
                "launch",
                "rb10_real_perception_sketch.launch.py",
            ])
        ),
        launch_arguments={
            "painting_config_file": config_file,
            **{key: LaunchConfiguration(key) for key in
               ('camera_backend', 'outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial',
                'outpost_d405_hw_id', 'outpost_d405_serial')},
            "real_painting_enabled": real_painting_enabled,
            "dry_run": dry_run,
            "launch_zed_driver": LaunchConfiguration("launch_zed_driver"),
            "launch_d405_driver": LaunchConfiguration("launch_d405_driver"),
            "launch_wall_detector": LaunchConfiguration("launch_wall_detector"),
            "launch_environment_scanner": LaunchConfiguration(
                "launch_environment_scanner"
            ),
            "zed_param_overrides": LaunchConfiguration("zed_param_overrides"),
            "d405_depth_profile": LaunchConfiguration("d405_depth_profile"),
            "d405_color_profile": LaunchConfiguration("d405_color_profile"),
            "zed_calibration_file": LaunchConfiguration("zed_calibration_file"),
            "d405_calibration_file": LaunchConfiguration("d405_calibration_file"),
            "launch_rbpodo_eft_bridge": LaunchConfiguration(
                "launch_rbpodo_eft_bridge"
            ),
            "launch_aft_ethernet_driver": LaunchConfiguration(
                "launch_aft_ethernet_driver"
            ),
            "front_view_source": LaunchConfiguration("front_view_source"),
            "d405_serial_no": LaunchConfiguration("d405_serial_no"),
            "d405_usb_port_id": LaunchConfiguration("d405_usb_port_id"),
            # The guarded force pipeline below supersedes the legacy helper.
            "use_ft_normal_controller": "false",
        }.items(),
    )

    force_pipeline = IncludeLaunchDescription(
        condition=IfCondition(LaunchConfiguration("launch_force_pipeline")),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rbpodo_painting_control"),
                "launch",
                "painting_admittance_control.launch.py",
            ])
        ),
        launch_arguments={
            "config_file": config_file,
            "real_painting_enabled": real_painting_enabled,
            "dry_run": dry_run,
            "enable_force": force_enabled,
            "desired_contact_force_n": LaunchConfiguration(
                "desired_contact_force_n"
            ),
            "start_segment_mode_node": "false",
        }.items(),
    )

    executor = Node(
        condition=IfCondition(LaunchConfiguration("launch_executor")),
        package="sketch_control",
        executable="moveit_executor",
        name="moveit_executor",
        output="screen",
        parameters=[
            config_file,
            {
                "real_painting_enabled": ParameterValue(
                    real_painting_enabled, value_type=bool
                ),
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "model_id": LaunchConfiguration("model_id"),
                "spray_motion_test": ParameterValue(
                    LaunchConfiguration("spray_motion_test"), value_type=bool
                ),
                "painting_force_enabled": ParameterValue(
                    force_enabled, value_type=bool
                ),
                "stationary_paint_hold_s": ParameterValue(
                    LaunchConfiguration("stationary_paint_hold_s"),
                    value_type=float,
                ),
            },
        ],
    )

    interlock_flight_recorder = Node(
        package="sketch_control",
        executable="interlock_flight_recorder",
        name="interlock_flight_recorder",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("enable_interlock_flight_recorder")
        ),
        parameters=[config_file],
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="painting_rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
        parameters=[{
            "max_message_size": ParameterValue(
                LaunchConfiguration("rosbridge_max_message_size"),
                value_type=int,
            ),
        }],
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=_validate_launch_interlocks),
            moveit,
            perception,
            force_pipeline,
            executor,
            interlock_flight_recorder,
            rosbridge,
        ]
    )
