"""Fixed ZED camera and geometric sketch planning; no robot/force/executor."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from sketch_control.outpost_camera import camera_status
from sketch_control.robot_models import validate_calibration_files


def _validate_inputs(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    validate_calibration_files({'zed_calibration_file': value('zed_calibration_file')}, 'spray')
    camera_status(value('outpost_http'), value('outpost_zed_hw_id'),
                  value('outpost_zed_serial'), 'zed')
    return []


def generate_launch_description():
    share = Path(get_package_share_directory('sketch_control'))
    return LaunchDescription([
        DeclareLaunchArgument('model_id', default_value='rb10_1300e_u'),
        DeclareLaunchArgument('zed_calibration_file'),
        DeclareLaunchArgument('outpost_http', default_value='http://127.0.0.1:8100'),
        DeclareLaunchArgument('outpost_zed_hw_id'),
        DeclareLaunchArgument('outpost_zed_serial'),
        OpaqueFunction(function=_validate_inputs),
        Node(package='sketch_control', executable='outpost_bridge',
             name='sketch_outpost_zed_bridge', output='screen', parameters=[{
                 'camera_name': 'zed', 'publish_hz': 10.0, 'point_stride': 2,
                 **{key: ParameterValue(LaunchConfiguration(key), value_type=str) for key in
                    ('outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial')},
             }]),
        # The measured calibration uses World == robot base link0. These fixed
        # aliases require no live robot state or fictitious joint positions.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='preview_world_to_link0', arguments=['--frame-id', 'World', '--child-frame-id', 'link0']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/rb10_perception_sketch.launch.py')),
            launch_arguments={
                'process_mode': 'spray', 'model_id': LaunchConfiguration('model_id'),
                'zed_calibration_file': LaunchConfiguration('zed_calibration_file'),
                'use_zed_calibration_file': 'true', 'real_painting_enabled': 'false',
                'dry_run': 'true', 'front_view_source': 'zed',
                'zed_image_reliable': 'true',
                'use_sim_depth_pointcloud': 'false', 'use_sim_d405_depth_pointcloud': 'false',
                'launch_wall_detector': 'false', 'launch_environment_scanner': 'false',
                'use_d405_refinement': 'false', 'use_d405_mount_tf': 'false',
                'use_d405_optical_tf': 'false', 'use_ft_normal_controller': 'false',
            }.items()),
        Node(package='sketch_control', executable='zed_preview', name='zed_preview', output='screen'),
    ])
