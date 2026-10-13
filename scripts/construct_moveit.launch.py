"""MoveIt + topic hardware for Construct; no physical robot driver is loaded."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def apply_process_planning_settings(pipeline,process_mode):
    if process_mode!='weld': return
    # The narrow workpiece/torch needs denser OMPL edge checks. Keep final
    # ValidateSolution and limit TOTG's deviation from the checked path.
    for group in ('left_manipulator','right_manipulator'):
        pipeline[group]['longest_valid_segment_fraction']=.0001
    pipeline['planner_configs']['RRTConnectkConfigDefault']['longest_valid_segment_fraction']=.0001
    pipeline['totg']={'path_tolerance':.001}


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    assets = root / 'isaac_assets'
    source = root / 'src/construct_moveit_config/config'
    description = {'robot_description':(assets/'construct_control.urdf').read_text()}
    config = (MoveItConfigsBuilder('construct_robot_0528', package_name='construct_moveit_config')
        .robot_description_semantic(file_path=str(assets/'construct.srdf'))
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs())
    # Original config targets Humble; use installed Jazzy pipeline adapter names.
    defaults = Path(get_package_share_directory('moveit_configs_utils'))/'default_configs/ompl_planning.yaml'
    pipeline = yaml.safe_load(defaults.read_text())
    pipeline['response_adapters'].insert(1, 'default_planning_response_adapters/AddRuckigTrajectorySmoothing')
    pipeline['planner_configs'] = {'RRTConnectkConfigDefault': {'type':'geometric::RRTConnect','range':0.0}}
    for group in ('left_manipulator','right_manipulator','robot_head','dual_arm','whole_robot'):
        pipeline[group] = {'planner_configs':['RRTConnectkConfigDefault']}
    apply_process_planning_settings(pipeline,os.environ.get('CONSTRUCT_PROCESS_MODE','spray'))
    config.planning_pipelines['ompl'] = pipeline
    # KDL requires a chain; the original head group is a two-joint collection.
    config.robot_description_kinematics['robot_description_kinematics'].pop('robot_head',None)
    config.joint_limits = {'robot_description_planning':yaml.safe_load((assets/'moveit_joint_limits.yaml').read_text())}
    parameters = config.to_dict()
    parameters.update(description)
    parameters['use_sim_time'] = True
    parameters['trajectory_execution.allowed_execution_duration_scaling'] = 2.0
    parameters['trajectory_execution.allowed_goal_duration_margin'] = 5.0
    nodes = [
        DeclareLaunchArgument('rviz',default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',parameters=[description,{'use_sim_time':True}]),
        Node(package='controller_manager', executable='ros2_control_node',parameters=[description,str(assets/'controllers.yaml')],output='screen'),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[parameters],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(source/'moveit.rviz')],parameters=[parameters],condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    for name in ('left_manipulator_controller','right_manipulator_controller','robot_head_controller'):
        nodes.append(Node(package='controller_manager',executable='spawner',arguments=[name,'--controller-manager-timeout','60'],output='screen'))
    return LaunchDescription(nodes)
