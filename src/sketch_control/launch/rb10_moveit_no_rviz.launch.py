import os
from sketch_control.robot_models import DEFAULT_MODEL, model_srdf, validate_model

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


robot_ip = LaunchConfiguration("robot_ip")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
use_isaac_sim = LaunchConfiguration("use_isaac_sim")
fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
human_collab = LaunchConfiguration("human_collab")
model_id = LaunchConfiguration("model_id")
cb_simulation = LaunchConfiguration("cb_simulation")
use_sim_time_cfg = LaunchConfiguration("use_sim_time")
use_admittance_cfg = LaunchConfiguration("use_admittance")
admittance_profile_cfg = LaunchConfiguration("admittance_profile")


def _named_srdf(selected_model=DEFAULT_MODEL):
    return model_srdf(selected_model)


def _robot_description_with_eoat():
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("sketch_control"),
            "urdf",
            "rbpodo_with_eoat.urdf.xacro",
        ]),
        " robot_ip:=", robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " use_isaac_sim:=", use_isaac_sim,
        " fake_sensor_commands:=", fake_sensor_commands,
        " human_collab:=", human_collab,
        " cb_simulation:=", cb_simulation,
        " model_id:=", model_id,
    ])
    return {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="True if there is no RB Cobot Control Box",
        ),
        DeclareLaunchArgument(
            "use_isaac_sim",
            default_value="true",
            description="Use Isaac Sim hardware bridge",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from simulation",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="True when using fake sensor commands",
        ),
        DeclareLaunchArgument(
            "human_collab",
            default_value="false",
            description="Clamp the hardware F/T signal for collaborative admittance",
        ),
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="false",
            description="RB Control Box simulation mode flag",
        ),
        DeclareLaunchArgument(
            "model_id",
            default_value="rb10_1300e_u",
            description="RB model id",
        ),
        DeclareLaunchArgument(
            "use_admittance",
            default_value="false",
            description="Route JTC commands through admittance_controller",
        ),
        DeclareLaunchArgument(
            "admittance_profile",
            default_value="default",
            description="Comma-separated rbpodo_bringup admittance profile names",
        ),
        DeclareLaunchArgument("admittance_use_case", default_value="free_push"),
        DeclareLaunchArgument("start_admittance_reset", default_value="auto"),
        DeclareLaunchArgument("start_variable_impedance", default_value="auto"),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    selected_model = validate_model(model_id.perform(context))
    use_sim_time = {"use_sim_time": use_sim_time_cfg}
    is_isaac_sim = context.perform_substitution(use_isaac_sim).lower() in (
        "true",
        "1",
        "yes",
    )
    use_admittance = context.perform_substitution(use_admittance_cfg).lower() in (
        "true",
        "1",
        "yes",
    )
    admittance_profiles = [
        p.strip()
        for p in context.perform_substitution(admittance_profile_cfg).split(",")
        if p.strip()
    ]
    admittance_use_case = LaunchConfiguration(
        "admittance_use_case"
    ).perform(context).strip().lower()
    if admittance_use_case not in ("free_push", "painting"):
        raise RuntimeError("admittance_use_case must be 'free_push' or 'painting'")
    if admittance_use_case == "painting" and "painting_normal_y" not in admittance_profiles:
        admittance_profiles = (
            ["painting_normal_y"]
            if admittance_profiles == ["default"]
            else [*admittance_profiles, "painting_normal_y"]
        )
    painting_contact_mode = (
        admittance_use_case == "painting"
        or "painting_normal_y" in admittance_profiles
    )

    def helper_enabled(argument_name):
        value = LaunchConfiguration(argument_name).perform(context).strip().lower()
        if value == "auto":
            return not painting_contact_mode
        return value in ("true", "1", "yes", "on")
    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "use_isaac_sim": use_isaac_sim,
        "fake_sensor_commands": fake_sensor_commands,
        "human_collab": human_collab,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder("rbpodo", package_name="rbpodo_moveit_config")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings=mappings)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = _robot_description_with_eoat()
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": _named_srdf(selected_model)
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    if is_isaac_sim:
        world_to_link0_q = ("0", "0", "0.7071067811865475", "0.7071067811865476")
    else:
        world_to_link0_q = ("0", "0", "0", "1")

    # Real RB10 uses world == link0.  Isaac Sim kept a historical +90deg bridge
    # because the authored scene axes were different from the robot URDF axes.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fallback_world_to_link0",
        output="log",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "link0",
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", world_to_link0_q[0],
            "--qy", world_to_link0_q[1],
            "--qz", world_to_link0_q[2],
            "--qw", world_to_link0_q[3],
        ],
    )

    static_tf_world_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_World_bridge",
        output="log",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "World",
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    bringup_share = get_package_share_directory("rbpodo_bringup")
    controllers_yaml_name = (
        "controllers_admittance.yaml" if use_admittance else "controllers.yaml"
    )
    controllers_path = os.path.join(bringup_share, "config", controllers_yaml_name)
    ros2_control_parameters = [moveit_config.robot_description, controllers_path, use_sim_time]
    if use_admittance:
        for profile in admittance_profiles:
            ros2_control_parameters.append(
                os.path.join(
                    bringup_share,
                    "config",
                    "admittance_profiles",
                    f"{profile}.yaml",
                )
            )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=ros2_control_parameters,
        output="both",
        remappings=[
            ("joint_states", "controller_manager/joint_states"),
        ] if is_isaac_sim else [],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "300",
            "--switch-timeout",
            "60",
        ],
    )

    if use_admittance:
        ft_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "force_torque_sensor_broadcaster",
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                "300",
            ],
        )
        ft_broadcaster_raw_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "force_torque_sensor_broadcaster_raw",
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                "300",
            ],
        )
        admittance_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "admittance_controller",
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                "300",
            ],
        )
        spawn_admittance_after_joint_state_broadcaster = RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    ft_broadcaster_spawner,
                    ft_broadcaster_raw_spawner,
                    admittance_spawner,
                ],
            )
        )
        activate_arm_after_admittance = RegisterEventHandler(
            OnProcessExit(
                target_action=admittance_spawner,
                on_exit=[arm_controller_spawner],
            )
        )
        ft_tare_helper_node = Node(
            package="rbpodo_bringup",
            executable="ft_tare_node.py",
            name="rbpodo_ft_tare_helper",
            output="screen",
        )
        controller_startup_actions = [
            spawn_admittance_after_joint_state_broadcaster,
            activate_arm_after_admittance,
            ft_tare_helper_node,
        ]
        if helper_enabled("start_admittance_reset"):
            controller_startup_actions.append(
                Node(
                    package="rbpodo_bringup",
                    executable="admittance_reset_node.py",
                    name="rbpodo_admittance_helper",
                    output="screen",
                )
            )
        if helper_enabled("start_variable_impedance"):
            controller_startup_actions.append(
                Node(
                    package="rbpodo_bringup",
                    executable="variable_impedance_node.py",
                    name="rbpodo_variable_impedance",
                    output="screen",
                )
            )
    else:
        controller_startup_actions = [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            )
        ]

    return [
        static_tf,
        static_tf_world_bridge,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        *controller_startup_actions,
    ]
