"""
Visualization-only launch for rbpodo.

Brings up:
    - robot_state_publisher (URDF -> TF)
    - static_transform_publisher (world -> link0)
    - rviz2 (moveit.rviz config)
    - rbpodo_joint_state_publisher (publishes /joint_states from rbpodo sdata)

ros2_control / joint_state_broadcaster / move_group 는 띄우지 않으므로
moveit.launch.py 와 함께 실행하지 마세요. /joint_states 충돌이 발생합니다.

Usage:
    ros2 launch rbpodo_moveit_config viz_only.launch.py
    ros2 launch rbpodo_moveit_config viz_only.launch.py robot_ip:=10.0.2.7
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file (in rbpodo_moveit_config/config/)",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        ),
        DeclareLaunchArgument(
            "model_id",
            default_value="rb10_1300e_u",
            description="RB series model id",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Pass-through to URDF xacro (viz_only does not run ros2_control)",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Pass-through to URDF xacro",
        ),
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="RB Control Box mode passed to URDF xacro (Simulation or Real)",
        ),
        DeclareLaunchArgument(
            "publish_hz",
            default_value="100.0",
            description="Hz for /joint_states publication from rbpodo sdata",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    model_id = LaunchConfiguration("model_id")
    rviz_base = LaunchConfiguration("rviz_config")
    publish_hz = LaunchConfiguration("publish_hz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    cb_simulation = LaunchConfiguration("cb_simulation")

    # URDF xacro mappings. viz_only 는 ros2_control 을 띄우지 않으므로
    # use_fake_hardware 등은 URDF 처리 단계에만 영향을 줍니다.
    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings=mappings)
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rbpodo_moveit_config"), "config", rviz_base]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description],
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

    rbpodo_joint_state_publisher = Node(
        package="rb10_control",
        executable="rbpodo_joint_state_publisher",
        name="rbpodo_joint_state_publisher",
        output="screen",
        parameters=[{"robot_ip": robot_ip, "publish_hz": publish_hz}],
    )

    return [
        static_tf,
        robot_state_publisher,
        rviz_node,
        rbpodo_joint_state_publisher,
    ]
