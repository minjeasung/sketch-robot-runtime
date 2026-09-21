from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


RUNTIME_OVERRIDE_PARAMETERS = frozenset({
    "desired_contact_force_n",
    "dry_run",
    "enable_force",
    "real_painting_enabled",
})


def _parameter_layers(config_path, parameter_overrides):
    """Use the shared YAML for tuning, with only explicit run interlocks on top."""
    if not config_path:
        return [parameter_overrides]
    runtime_overrides = {
        key: value
        for key, value in parameter_overrides.items()
        if key in RUNTIME_OVERRIDE_PARAMETERS
    }
    return [config_path, runtime_overrides] if runtime_overrides else [config_path]


def _node_with_optional_config(context, *, node_kwargs, parameter_overrides):
    config_path = LaunchConfiguration("config_file").perform(context).strip()
    parameters = _parameter_layers(config_path, parameter_overrides)
    return [Node(parameters=parameters, **node_kwargs)]


def generate_launch_description():
    default_schedule_csv = PathJoinSubstitution(
        [
            FindPackageShare("rbpodo_painting_control"),
            "config",
            "example_admittance_segment_modes.csv",
        ]
    )

    def float_param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    def bool_param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    def str_param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=str)

    args = [
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("real_painting_enabled", default_value="false"),
        DeclareLaunchArgument("enable_force", default_value="false"),
        DeclareLaunchArgument("desired_contact_force_n", default_value="0.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("force_ramp_up_duration_s", default_value="2.0"),
        DeclareLaunchArgument("force_ramp_down_duration_s", default_value="1.0"),
        DeclareLaunchArgument("max_command_force_n", default_value="15.0"),
        DeclareLaunchArgument("max_command_torque_nm", default_value="0.0"),
        DeclareLaunchArgument("max_force_slew_rate_nps", default_value="5.0"),
        DeclareLaunchArgument("target_wrench_sign", default_value="-1.0"),
        DeclareLaunchArgument("contact_force_sign", default_value="1.0"),
        DeclareLaunchArgument("absolute_normal_force", default_value="true"),
        DeclareLaunchArgument("target_wrench_axis", default_value="force_y"),
        DeclareLaunchArgument(
            "requested_wrench_topic",
            default_value="/painting_admittance/requested_wrench_reference",
        ),
        DeclareLaunchArgument("wrench_reference_topic", default_value=""),
        DeclareLaunchArgument(
            "controller_wrench_reference_topic",
            default_value="/admittance_controller/wrench_reference",
        ),
        DeclareLaunchArgument(
            "compliance_enable_topic",
            default_value="/admittance_controller/compliance_enable",
        ),
        DeclareLaunchArgument(
            "compliance_active_topic",
            default_value="/admittance_controller/compliance_active",
        ),
        DeclareLaunchArgument(
            "normal_limit_reached_topic",
            default_value="/admittance_controller/normal_limit_reached",
        ),
        DeclareLaunchArgument(
            "controller_fault_topic",
            default_value="/painting_admittance/controller_fault",
        ),
        DeclareLaunchArgument("tcp_frame", default_value="tcp"),
        DeclareLaunchArgument("ft_frame", default_value="ft_link"),
        DeclareLaunchArgument("input_wrench_topic", default_value="/force_torque_sensor_broadcaster/wrench"),
        DeclareLaunchArgument("raw_wrench_topic", default_value="/force_torque_sensor_broadcaster_raw/wrench"),
        DeclareLaunchArgument("force_filter_tau_s", default_value="0.10"),
        DeclareLaunchArgument("force_deadband_n", default_value="0.5"),
        DeclareLaunchArgument("bias_sample_duration_s", default_value="1.0"),
        DeclareLaunchArgument("update_bias_in_noncontact", default_value="true"),
        DeclareLaunchArgument("contact_detect_threshold_n", default_value="1.5"),
        DeclareLaunchArgument("contact_release_threshold_n", default_value="0.8"),
        DeclareLaunchArgument("contact_confirm_duration_s", default_value="0.10"),
        DeclareLaunchArgument("travel_collision_threshold_n", default_value="3.0"),
        DeclareLaunchArgument("over_force_warn_n", default_value="10.0"),
        DeclareLaunchArgument("over_force_abort_n", default_value="15.0"),
        DeclareLaunchArgument("stale_timeout_s", default_value="0.2"),
        DeclareLaunchArgument("raw_stale_timeout_s", default_value="0.2"),
        DeclareLaunchArgument("max_message_age_s", default_value="0.2"),
        DeclareLaunchArgument("tf_valid_timeout_s", default_value="0.2"),
        DeclareLaunchArgument("sensor_force_saturation_n", default_value="190.0"),
        DeclareLaunchArgument("sensor_torque_saturation_nm", default_value="9.5"),
        DeclareLaunchArgument("requested_wrench_timeout_s", default_value="0.10"),
        DeclareLaunchArgument("mode_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("enable_timeout_s", default_value="0.20"),
        DeclareLaunchArgument(
            "executor_heartbeat_timeout_s", default_value="0.20"
        ),
        DeclareLaunchArgument("ft_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("tf_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("safety_status_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("controller_status_timeout_s", default_value="0.20"),
        DeclareLaunchArgument(
            "compliance_activation_timeout_s", default_value="0.50"
        ),
        DeclareLaunchArgument("normal_limit_abort_duration_s", default_value="0.25"),
        DeclareLaunchArgument("start_segment_mode_node", default_value="false"),
        DeclareLaunchArgument("schedule_csv", default_value=default_schedule_csv),
        DeclareLaunchArgument("start_enable_force", default_value="false"),
        DeclareLaunchArgument("loop_schedule", default_value="false"),
    ]

    wrench_reference = OpaqueFunction(
        function=_node_with_optional_config,
        kwargs={
            "node_kwargs": {
                "package": "rbpodo_painting_control",
                "executable": "painting_wrench_reference_node",
                "name": "painting_wrench_reference",
                "output": "screen",
            },
            "parameter_overrides": {
                "requested_wrench_topic": LaunchConfiguration("requested_wrench_topic"),
                "wrench_reference_topic": LaunchConfiguration("wrench_reference_topic"),
                "tcp_frame": LaunchConfiguration("tcp_frame"),
                "ft_frame": LaunchConfiguration("ft_frame"),
                "surface_normal_tcp_axis": ParameterValue("+y", value_type=str),
                "target_wrench_axis": str_param("target_wrench_axis"),
                "target_wrench_sign": float_param("target_wrench_sign"),
                "desired_contact_force_n": float_param("desired_contact_force_n"),
                "enable_force": bool_param("enable_force"),
                "dry_run": bool_param("dry_run"),
                "publish_rate_hz": float_param("publish_rate_hz"),
                "force_ramp_up_duration_s": float_param("force_ramp_up_duration_s"),
                "force_ramp_down_duration_s": float_param("force_ramp_down_duration_s"),
                "max_command_force_n": float_param("max_command_force_n"),
                "max_force_slew_rate_nps": float_param("max_force_slew_rate_nps"),
            },
        },
    )

    wrench_guard = OpaqueFunction(
        function=_node_with_optional_config,
        kwargs={
            "node_kwargs": {
                "package": "rbpodo_painting_control",
                "executable": "painting_wrench_guard_node",
                "name": "painting_wrench_guard",
                "output": "screen",
            },
            "parameter_overrides": {
                "requested_wrench_topic": LaunchConfiguration(
                    "requested_wrench_topic"
                ),
                "controller_wrench_reference_topic": LaunchConfiguration(
                    "controller_wrench_reference_topic"
                ),
                "compliance_enable_topic": LaunchConfiguration(
                    "compliance_enable_topic"
                ),
                "compliance_active_topic": LaunchConfiguration(
                    "compliance_active_topic"
                ),
                "normal_limit_reached_topic": LaunchConfiguration(
                    "normal_limit_reached_topic"
                ),
                "controller_fault_topic": LaunchConfiguration(
                    "controller_fault_topic"
                ),
                "ft_wrench_topic": LaunchConfiguration("input_wrench_topic"),
                "ft_frame": LaunchConfiguration("ft_frame"),
                "tcp_frame": LaunchConfiguration("tcp_frame"),
                "real_painting_enabled": bool_param("real_painting_enabled"),
                "publish_rate_hz": float_param("publish_rate_hz"),
                "requested_wrench_timeout_s": float_param(
                    "requested_wrench_timeout_s"
                ),
                "mode_timeout_s": float_param("mode_timeout_s"),
                "enable_timeout_s": float_param("enable_timeout_s"),
                "executor_heartbeat_timeout_s": float_param(
                    "executor_heartbeat_timeout_s"
                ),
                "ft_timeout_s": float_param("ft_timeout_s"),
                "tf_timeout_s": float_param("tf_timeout_s"),
                "safety_status_timeout_s": float_param(
                    "safety_status_timeout_s"
                ),
                "controller_status_timeout_s": float_param(
                    "controller_status_timeout_s"
                ),
                "compliance_activation_timeout_s": float_param(
                    "compliance_activation_timeout_s"
                ),
                "normal_limit_abort_duration_s": float_param(
                    "normal_limit_abort_duration_s"
                ),
                "max_command_force_n": float_param("max_command_force_n"),
                "max_command_torque_nm": float_param("max_command_torque_nm"),
            },
        },
    )

    force_monitor = OpaqueFunction(
        function=_node_with_optional_config,
        kwargs={
            "node_kwargs": {
                "package": "rbpodo_painting_control",
                "executable": "painting_force_monitor_node",
                "name": "painting_force_monitor",
                "output": "screen",
            },
            "parameter_overrides": {
                "input_wrench_topic": LaunchConfiguration("input_wrench_topic"),
                "raw_wrench_topic": LaunchConfiguration("raw_wrench_topic"),
                "ft_frame": LaunchConfiguration("ft_frame"),
                "tcp_frame": LaunchConfiguration("tcp_frame"),
                "controller_fault_topic": LaunchConfiguration(
                    "controller_fault_topic"
                ),
                "normal_limit_reached_topic": LaunchConfiguration(
                    "normal_limit_reached_topic"
                ),
                "normal_limit_abort_duration_s": float_param(
                    "normal_limit_abort_duration_s"
                ),
                "force_filter_tau_s": float_param("force_filter_tau_s"),
                "force_deadband_n": float_param("force_deadband_n"),
                "bias_sample_duration_s": float_param("bias_sample_duration_s"),
                "update_bias_in_noncontact": bool_param("update_bias_in_noncontact"),
                # The commanded wrench points into the wall (-TCP Y), while
                # the measured wall reaction points out of it (+TCP Y).
                "contact_force_sign": float_param("contact_force_sign"),
                "absolute_normal_force": bool_param("absolute_normal_force"),
                "contact_detect_threshold_n": float_param("contact_detect_threshold_n"),
                "contact_release_threshold_n": float_param("contact_release_threshold_n"),
                "contact_confirm_duration_s": float_param("contact_confirm_duration_s"),
                "travel_collision_threshold_n": float_param("travel_collision_threshold_n"),
                "over_force_warn_n": float_param("over_force_warn_n"),
                "over_force_abort_n": float_param("over_force_abort_n"),
                "stale_timeout_s": float_param("stale_timeout_s"),
                "raw_stale_timeout_s": float_param("raw_stale_timeout_s"),
                "max_message_age_s": float_param("max_message_age_s"),
                "tf_valid_timeout_s": float_param("tf_valid_timeout_s"),
                "sensor_force_saturation_n": float_param(
                    "sensor_force_saturation_n"
                ),
                "sensor_torque_saturation_nm": float_param(
                    "sensor_torque_saturation_nm"
                ),
            },
        },
    )

    segment_mode = Node(
        package="rbpodo_painting_control",
        executable="painting_segment_mode_node",
        name="painting_segment_mode",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_segment_mode_node")),
        parameters=[
            {
                "schedule_csv": LaunchConfiguration("schedule_csv"),
                "publish_rate_hz": 20.0,
                "loop": bool_param("loop_schedule"),
                "start_enable_force": bool_param("start_enable_force"),
            }
        ],
    )

    return LaunchDescription(
        args + [wrench_reference, force_monitor, wrench_guard, segment_mode]
    )
