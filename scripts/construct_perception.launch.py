"""Original sketch perception consuming official Outpost, with Construct TF.

Start after the official Michelo local camera lease reports STREAMING. Camera
poses come from the Construct model/environment, never a real RB calibration.
The bridge's simulation receipt stamps require a stationary head for selection.
"""
from pathlib import Path
from copy import deepcopy
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


SPRAY_ARGUMENTS = ('model_id', 'spray_tool_axis', 'spray_eoat_profile',
                   'spray_footprint_width_m', 'spray_overlap', 'spray_speed_mps', 'spray_standoff_m')


def construct_perception_config(original, values):
    """Override at the original YAML's node specificity, not wildcard priority."""
    result = deepcopy(original)
    projector = result.setdefault('wall_projector', {}).setdefault('ros__parameters', {})
    projector.update(process_mode='spray', front_view_source='zed',
                     zed_image_reliable=True, use_sim_time=True)
    generator = result.setdefault('sketch_to_waypoints', {}).setdefault('ros__parameters', {})
    generator.update(values)
    generator.update(process_mode='spray', eoat_segment_frame='World',
                     real_painting_enabled=False, dry_run=False, use_sim_time=True)
    return result


def _launch_perception(context, root):
    if LaunchConfiguration('process_mode').perform(context) == 'weld':
        return [Node(package='sketch_control', executable='weld_input', name='weld_input',
                     output='screen', parameters=[{'use_sim_time': True, 'simulation': True,
                     'weld_input_provider': LaunchConfiguration('weld_input_provider').perform(context),
                     'weld_fixture_file': str(root / 'src/sketch_control/config/construct_weld_fixture.json')}])]

    values = {key: LaunchConfiguration(key).perform(context) for key in SPRAY_ARGUMENTS}
    for key in SPRAY_ARGUMENTS[3:]:
        values[key] = float(values[key])
    original = yaml.safe_load((root / 'src/sketch_control/config/painting_system_real.yaml').read_text())
    with tempfile.NamedTemporaryFile(mode='w', prefix='construct-perception-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(construct_perception_config(original, values), stream)
        config_path = Path(stream.name)

    def cleanup(_context):
        config_path.unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root / 'src/sketch_control/launch/rb10_perception_sketch.launch.py')),
            launch_arguments={
                **{key: LaunchConfiguration(key) for key in SPRAY_ARGUMENTS},
                'painting_config_file': str(config_path),
                'process_mode': 'spray', 'eoat_segment_frame': 'World',
                'publish_zed_static_tf': 'false', 'use_zed_calibration_file': 'false',
                'real_painting_enabled': 'false', 'dry_run': 'false',
                'front_view_source': 'zed', 'zed_image_reliable': 'true',
                'use_sim_depth_pointcloud': 'false', 'use_sim_d405_depth_pointcloud': 'false',
                'launch_wall_detector': 'false', 'launch_environment_scanner': 'false',
                'use_d405_refinement': 'false', 'use_d405_mount_tf': 'false',
                'use_d405_optical_tf': 'false', 'use_ft_normal_controller': 'false',
            }.items()),
    ]


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('weld_input_provider', default_value='fixture', choices=['fixture','external']),
        DeclareLaunchArgument('process_mode', default_value='spray', choices=['spray', 'weld']),
        DeclareLaunchArgument('model_id', default_value='construct_left',
                              choices=['construct_left', 'construct_right']),
        DeclareLaunchArgument('spray_tool_axis'),
        DeclareLaunchArgument('spray_eoat_profile'),
        DeclareLaunchArgument('spray_footprint_width_m', default_value='0.35'),
        DeclareLaunchArgument('spray_overlap', default_value='0.30'),
        DeclareLaunchArgument('spray_speed_mps', default_value='0.020'),
        DeclareLaunchArgument('spray_standoff_m', default_value='0.5'),
        DeclareLaunchArgument('outpost_http', default_value='http://127.0.0.1:8100'),
        DeclareLaunchArgument('outpost_zed_hw_id'),
        DeclareLaunchArgument('outpost_zed_serial', default_value='stream://127.0.0.1:30000'),
        SetParameter(name='use_sim_time', value=True),
        Node(package='sketch_control', executable='outpost_bridge',
             name='sketch_outpost_zed_bridge', output='screen', parameters=[{
                 'camera_name': 'zed', 'publish_hz': 10.0, 'point_stride': 2,
                 'stamp_mode': 'ros_clock',
                 **{key: ParameterValue(LaunchConfiguration(key), value_type=str) for key in
                    ('outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial')},
             }]),
        OpaqueFunction(function=_launch_perception, args=[root]),
    ])
