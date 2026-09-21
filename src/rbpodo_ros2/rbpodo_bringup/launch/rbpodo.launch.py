# Copyright (c) 2024 Rainbow Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "model_id",
            default_value="rb5_850e",
            description="Model ID for Rainbow Robotics Cobot",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=[
                TextSubstitution(
                    text=os.path.join(
                        get_package_share_directory("rbpodo_description"),
                        "robots",
                        "",
                    )
                ),
                LaunchConfiguration("model_id"),
                TextSubstitution(text=".urdf.xacro"),
            ],
            description="Model path (xacro)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in Rviz",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Fake sensor commands. Only valid when 'use_fake_hardware' is true",
        ),
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="Select RB Control Box mode, Simulation or Real",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="auto",
            description=(
                "Controller mode: auto, normal, or admittance. "
                "auto preserves use_admittance backwards compatibility."
            ),
        ),
        DeclareLaunchArgument(
            "use_admittance",
            default_value="false",
            description=(
                "Chain MoveIt2/JTC commands through admittance_controller. "
                "Loads controllers_admittance.yaml and spawns the FT "
                "broadcaster + admittance_controller."
            ),
        ),
        DeclareLaunchArgument(
            "admittance_profile",
            default_value="default",
            description=(
                "Admittance gains preset. Selects "
                "rbpodo_bringup/config/admittance_profiles/<name>.yaml. "
                "Built-ins: default, painting_normal_y, mass_low, mass_high, "
                "damping_low, damping_high. "
                "Tune live with: ros2 param set /admittance_controller "
                "admittance.stiffness '[...]'"
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    model_path = LaunchConfiguration("model_path")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    cb_simulation = LaunchConfiguration("cb_simulation")
    use_rviz = LaunchConfiguration("use_rviz")
    control_mode_str = LaunchConfiguration("control_mode").perform(context).strip().lower()
    use_admittance_str = LaunchConfiguration("use_admittance").perform(context)
    use_admittance = use_admittance_str.lower() in ("true", "1", "yes")
    admittance_profile = LaunchConfiguration("admittance_profile").perform(context)
    if control_mode_str in ("", "auto"):
        control_mode = "admittance" if use_admittance else "normal"
    else:
        control_mode = control_mode_str
    if control_mode not in ("normal", "admittance"):
        raise RuntimeError(
            "control_mode must be one of: auto, normal, admittance"
        )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            model_path,
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " cb_simulation:=",
            cb_simulation,
        ]
    )

    rviz_file = os.path.join(
        get_package_share_directory("rbpodo_description"), "rviz", "urdf.rviz"
    )

    bringup_share = FindPackageShare("rbpodo_bringup")
    controllers_yaml_by_mode = {
        "normal": "controllers.yaml",
        "admittance": "controllers_admittance.yaml",
    }
    controllers_yaml_name = controllers_yaml_by_mode[control_mode]
    robot_controllers = PathJoinSubstitution([bringup_share, "config", controllers_yaml_name])

    cm_parameters = [robot_controllers]
    if control_mode == "admittance":
        profile_yaml = PathJoinSubstitution(
            [bringup_share, "config", "admittance_profiles", f"{admittance_profile}.yaml"]
        )
        # Profile is loaded after the base file so its values override mass/damping/stiffness.
        cm_parameters.append(profile_yaml)

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=cm_parameters,
            remappings=[
                ("joint_states", "rbpodo/joint_states"),
                ("~/robot_description", "/robot_description"),
            ],
            output="both",
            on_exit=Shutdown(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": ["rbpodo/joint_states"], "rate": 30}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
    ]

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    if control_mode == "admittance":
        ft_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["force_torque_sensor_broadcaster"],
            output="screen",
        )
        ft_broadcaster_raw_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["force_torque_sensor_broadcaster_raw"],
            output="screen",
        )
        admittance_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["admittance_controller"],
            output="screen",
        )
        # JTC's command_joints reference admittance_controller/<joint>/position,
        # which only exists once admittance_controller is active. Wait for the
        # admittance spawner to exit before starting JTC.
        delay_jtc_after_admittance = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=admittance_spawner,
                on_exit=[jtc_spawner],
            )
        )
        nodes += [ft_broadcaster_spawner, ft_broadcaster_raw_spawner,
                  admittance_spawner, delay_jtc_after_admittance]

        # With mock hardware + fake_sensor_commands, mock_components/GenericSystem
        # exposes writable sensor command interfaces that the ft_sensor_command_controller
        # forwards. Publish wrench mock values via:
        #   ros2 topic pub /ft_sensor_command_controller/commands \
        #     std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"
        fake_hw = LaunchConfiguration("use_fake_hardware").perform(
            context
        ).lower() in ("true", "1", "yes")
        fake_sc = LaunchConfiguration("fake_sensor_commands").perform(
            context
        ).lower() in ("true", "1", "yes")
        if fake_hw and fake_sc:
            nodes.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["ft_sensor_command_controller"],
                    output="screen",
                )
            )
    else:
        nodes.append(jtc_spawner)

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_file],
            condition=IfCondition(use_rviz),
        )
    )

    return nodes
