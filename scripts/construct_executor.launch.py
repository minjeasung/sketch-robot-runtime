"""Run the original sketch executor with the Construct equipment contract."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    root = Path(__file__).resolve().parents[1]
    model = LaunchConfiguration('model_id').perform(context)
    return [
        Node(package='sketch_control', executable='moveit_executor',
             name='moveit_executor', output='screen', parameters=[{
                 'model_id': model, 'use_sim_time': True,
                 'execution_backend': 'follow_joint_trajectory',
                 # FJT trajectories use simulation time; the original result
                 # watchdog uses wall time. Allow slower RTX/VNC playback while
                 # retaining a finite watchdog and immediate cancellation.
                 'fjt_result_timeout_margin_s': 120.0,
                 'process_mode': LaunchConfiguration('process_mode').perform(context),
                 'spray_motion_test': LaunchConfiguration('process_mode').perform(context) == 'spray',
                 'weld_input_provider': LaunchConfiguration('weld_input_provider').perform(context),
                 'weld_tool_profile': str(root / 'src/sketch_control/config/weld_sim_profile.json'),
                 'real_painting_enabled': False, 'dry_run': False,
                 'painting_force_enabled': False,
                 'spray_tool_axis': '+z',
                 'spray_eoat_profile': str(root / 'isaac_assets' / (model + '_eoat.json')),
             }]),
        ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'scripts/construct_position_state.py')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('weld_input_provider', default_value='fixture', choices=['fixture','external']),
        DeclareLaunchArgument('process_mode', default_value='spray', choices=['spray', 'weld']),
        DeclareLaunchArgument('model_id', default_value='construct_left',
                              choices=['construct_left', 'construct_right']),
        OpaqueFunction(function=nodes),
    ])
