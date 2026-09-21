import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


robot_ip = LaunchConfiguration("robot_ip")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
use_isaac_sim = LaunchConfiguration("use_isaac_sim")
fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
human_collab = LaunchConfiguration("human_collab")
model_id = LaunchConfiguration("model_id")
cb_simulation = LaunchConfiguration("cb_simulation")
use_sim_time = LaunchConfiguration("use_sim_time")


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
    robot_description = _robot_description_with_eoat()
    use_sim_time_param = {"use_sim_time": use_sim_time}

    bringup_share = get_package_share_directory("rbpodo_bringup")
    controllers_path = os.path.join(bringup_share, "config", "controllers.yaml")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fallback_world_to_link0",
        output="log",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "link0",
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
        ],
    )

    static_tf_world_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_World_bridge",
        output="log",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "World",
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time_param],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_path, use_sim_time_param],
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

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="10.0.2.7"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_isaac_sim", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("human_collab", default_value="false"),
        DeclareLaunchArgument("cb_simulation", default_value="false"),
        DeclareLaunchArgument("model_id", default_value="rb10_1300e_u"),
        static_tf,
        static_tf_world_bridge,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
    ])
