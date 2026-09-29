"""
Phase 1 런치: core + Python tkinter 스케치 UI

사용:
  ros2 launch sketch_control phase1_python.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('sketch_control')

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'core.launch.py')
        ),
        launch_arguments={key: LaunchConfiguration(key) for key in
                          ('process_mode', 'model_id', 'spray_tool_axis', 'spray_footprint_width_m',
                           'spray_overlap', 'spray_speed_mps', 'spray_standoff_m')}.items(),
    )

    sketch_ui_node = Node(
        package='sketch_control',
        executable='sketch_ui',
        name='sketch_ui',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('process_mode', default_value='paint', choices=['paint', 'spray']),
        DeclareLaunchArgument('model_id', default_value='rb10_1300e_u'),
        DeclareLaunchArgument('spray_tool_axis', default_value=''),
        DeclareLaunchArgument('spray_footprint_width_m', default_value='0.35'),
        DeclareLaunchArgument('spray_overlap', default_value='0.30'),
        DeclareLaunchArgument('spray_speed_mps', default_value='0.020'),
        DeclareLaunchArgument('spray_standoff_m', default_value='0.5'),
        core_launch,
        sketch_ui_node,
    ])
