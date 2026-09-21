import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


robot_ip = LaunchConfiguration("robot_ip")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
human_collab = LaunchConfiguration("human_collab")
model_id = LaunchConfiguration("model_id")
cb_simulation = LaunchConfiguration("cb_simulation")


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="True if there's no RB Cobot Control Box",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="true",
            description=(
                "When true, rbpodo_hardware enables the ~/inject_ft topic "
                "and uses injected values instead of the cobot's real eft_*. "
                "Default true so that use_fake_hardware:=true on its own "
                "works without requiring a reachable cobot at robot_ip "
                "(otherwise hardware initialisation fails on connect). "
                "Set false to read the real cobot's F/T."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "human_collab",
            default_value="false",
            description=(
                "Enable human-collaboration safety clamp on the published wrench. "
                "Each axis of hw_ft_states_ is hard limited to +/-30 (N for force, "
                "Nm for torque) after bias subtraction, so admittance cannot react "
                "to wrench magnitudes beyond a comfortable interaction range. "
                "Applies to both real eft_* and injected values."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="Select RB Control Box mode, Simulation or Real",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_id",
            default_value="rb10_1300e_u",
            description="RB Series currently using",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_admittance",
            default_value="false",
            description=(
                "Chain MoveIt2 -> JTC -> admittance_controller. "
                "Loads controllers_admittance.yaml and spawns FT broadcaster + admittance."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "admittance_profile",
            default_value="default",
            description=(
                "Admittance gains preset. One or more "
                "rbpodo_bringup/config/admittance_profiles/<name>.yaml files. "
                "Pass a comma-separated list to layer profiles - later "
                "entries override earlier ones (e.g. "
                "admittance_profile:=mass_high,damping_low loads mass_high "
                "first, then damping_low). "
                "Built-ins: default, painting_normal_y, mass_low, mass_high, "
                "damping_low, damping_high. "
                "Tune live via: ros2 param set /admittance_controller admittance.stiffness '[...]'"
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "admittance_use_case",
            default_value="free_push",
            description=(
                "free_push keeps the existing admittance helpers. painting loads "
                "painting_normal_y if needed and disables reset/variable_impedance "
                "unless explicitly requested."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_admittance_reset",
            default_value="auto",
            description=(
                "true/false/auto. auto starts the reset helper for free_push and "
                "keeps it off for painting contact control."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_variable_impedance",
            default_value="auto",
            description=(
                "true/false/auto. auto starts variable_impedance for free_push and "
                "keeps it off for painting contact control."
            ),
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    use_admittance_str = LaunchConfiguration("use_admittance").perform(context)
    use_admittance = use_admittance_str.lower() in ("true", "1", "yes")
    admittance_use_case = (
        LaunchConfiguration("admittance_use_case").perform(context).strip().lower()
    )
    if admittance_use_case not in ("free_push", "painting"):
        raise RuntimeError("admittance_use_case must be 'free_push' or 'painting'")
    # Split comma-separated profile list. Later entries take precedence
    # because ros2_control parameter files are merged in order and the
    # last-written value wins.
    admittance_profile_str = LaunchConfiguration("admittance_profile").perform(context)
    admittance_profiles = [
        p.strip() for p in admittance_profile_str.split(",") if p.strip()
    ]
    if admittance_use_case == "painting" and "painting_normal_y" not in admittance_profiles:
        if admittance_profiles == ["default"]:
            admittance_profiles = ["painting_normal_y"]
        else:
            admittance_profiles.append("painting_normal_y")
    painting_contact_mode = admittance_use_case == "painting" or (
        "painting_normal_y" in admittance_profiles
    )

    def helper_enabled(arg_name: str, default_for_free_push: bool) -> bool:
        value = LaunchConfiguration(arg_name).perform(context).strip().lower()
        if value == "auto":
            return default_for_free_push and not painting_contact_mode
        return value in ("true", "1", "yes", "on")

    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "human_collab": human_collab,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings=mappings)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rbpodo_moveit_config"), "config", rviz_base]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    bringup_share = get_package_share_directory("rbpodo_bringup")
    controllers_yaml_name = "controllers_admittance.yaml" if use_admittance else "controllers.yaml"
    ros2_controllers_path = os.path.join(bringup_share, "config", controllers_yaml_name)

    cm_parameters = [moveit_config.robot_description, ros2_controllers_path]
    if use_admittance:
        for profile in admittance_profiles:
            profile_path = os.path.join(
                bringup_share, "config", "admittance_profiles", f"{profile}.yaml"
            )
            # Profiles are appended after the base file so their values
            # override mass/damping/stiffness. When multiple profiles are
            # given, later ones override earlier ones, which lets users
            # combine orthogonal overrides (e.g. mass_high + damping_low).
            cm_parameters.append(profile_path)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=cm_parameters,
        output="both",
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
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
    ]

    if use_admittance:
        ft_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"],
        )
        ft_broadcaster_raw_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["force_torque_sensor_broadcaster_raw", "-c", "/controller_manager"],
        )
        admittance_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["admittance_controller", "-c", "/controller_manager"],
        )
        # In chained mode JTC claims admittance_controller/<joint>/position reference
        # interfaces, which only exist once admittance_controller is active. Defer JTC
        # until the admittance spawner has finished (= controller is active).
        delay_jtc_after_admittance = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=admittance_spawner,
                on_exit=[arm_controller_spawner],
            )
        )
        nodes_to_start += [ft_broadcaster_spawner, ft_broadcaster_raw_spawner,
                           admittance_spawner, delay_jtc_after_admittance]

        if helper_enabled("start_admittance_reset", True):
            # Auto-reset admittance state at the start of each MoveIt plan&execute.
            # This is for free-push K=0 workflows. In painting/contact mode the
            # wrench reference node must publish zero before any manual reset.
            admittance_reset_node = Node(
                package="rbpodo_bringup",
                executable="admittance_reset_node.py",
                name="rbpodo_admittance_helper",
                output="screen",
            )
            nodes_to_start.append(admittance_reset_node)

        # F/T tare helper. Re-exposes the hardware-side tare service under
        # a clean Python-importable interface (rbpodo_bringup.ft_tare_client.
        # FtTareClient) for use from other scripts. Manual call:
        #   ros2 service call /rbpodo_ft_tare_helper/tare_ft std_srvs/srv/Trigger
        ft_tare_helper_node = Node(
            package="rbpodo_bringup",
            executable="ft_tare_node.py",
            name="rbpodo_ft_tare_helper",
            output="screen",
        )
        nodes_to_start.append(ft_tare_helper_node)

        if helper_enabled("start_variable_impedance", True):
            # Variable impedance is a free-push helper. Keep it out of the
            # controlled-contact painting path unless explicitly requested.
            variable_impedance_node = Node(
                package="rbpodo_bringup",
                executable="variable_impedance_node.py",
                name="rbpodo_variable_impedance",
                output="screen",
            )
            nodes_to_start.append(variable_impedance_node)

        # FT signal source is controlled by 'fake_sensor_commands' at the URDF
        # level: when true, rbpodo_hardware enables the ~/inject_ft topic on its
        # rbpodo_ft_tare node and routes injected values through the tare + spike
        # filter before they reach the broadcaster / admittance. Use:
        #   ros2 topic pub /rbpodo_ft_tare/inject_ft \
        #     std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"
        # (empty data clears the injection.) No separate command controller is
        # needed - that path is superseded.
    else:
        nodes_to_start.append(arm_controller_spawner)

    return nodes_to_start
