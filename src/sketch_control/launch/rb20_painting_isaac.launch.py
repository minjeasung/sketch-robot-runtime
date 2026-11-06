"""Original guarded painting stack using only generated RB20 Isaac assets."""
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from functools import partial

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

COMPONENTS = ('robot_control', 'perception', 'force_pipeline', 'executor', 'all')
CONTROLLER_CHAIN = ('joint_state_broadcaster', 'force_torque_sensor_broadcaster',
                    'force_torque_sensor_broadcaster_raw', 'admittance_controller',
                    'joint_trajectory_controller')


def _ready_identity(assets, readiness_file):
    try:
        manifest = json.loads((assets / 'manifest.json').read_text())
        ready = json.loads(Path(readiness_file).read_text())
        age = time.monotonic() - float(ready['updated_monotonic'])
        os.kill(int(ready['pid']), 0)
        for field, variable in (('session_id', 'RB20_PAINTING_SESSION_ID'),
                                ('scene_revision', 'RB20_PAINTING_SCENE_REVISION')):
            expected = os.environ.get(variable)
            if expected and ready[field] != expected:
                raise ValueError(f'{field} does not match supervisor identity')
        if not (0.0 <= age <= 2.0 and ready['session_id'] and ready['scene_revision']
                and ready['model_id'] == manifest['model_id'] == 'rb20_1900es'
                and ready['scene_revision'] == manifest['scene_revision']
                and int(ready['step_index']) > 0
                and all(ready[k] is True for k in ('joints_valid', 'wrench_valid', 'cameras_ready'))):
            raise ValueError('not ready')
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f'Isaac RB20 readiness rejected: {exc}') from exc
    return manifest, ready


def _bound_description(assets, ready):
    root = ET.fromstring((assets / 'robot.urdf').read_text())
    plugins = root.findall('./ros2_control/hardware/plugin')
    if len(plugins) != 1 or plugins[0].text != 'isaac_painting_hardware/IsaacPaintingSystem':
        raise RuntimeError('generated URDF must contain only IsaacPaintingSystem')
    hardware = root.find('./ros2_control/hardware')
    for name in ('session_id', 'scene_revision'):
        param = hardware.find(f"param[@name='{name}']")
        if param is None:
            param = ET.SubElement(hardware, 'param', name=name)
        param.text = ready[name]
    return ET.tostring(root, encoding='unicode')


def _node(package, executable, name=None, parameters=(), **kwargs):
    return Node(package=package, executable=executable, name=name or executable,
                parameters=[*parameters, {'use_sim_time': True}], output='screen', **kwargs)


def _identity_tf(parent, child, name):
    return _node('tf2_ros', 'static_transform_publisher', name,
        arguments=['--frame-id', parent, '--child-frame-id', child,
                   '--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1'])


def _include(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare(package), 'launch', filename])), launch_arguments=arguments.items())


def _moveit_parameters(assets, description, share):
    config = Path(share) / 'config'
    def load(name):
        return yaml.safe_load((config / name).read_text())
    # The same generated geometry feeds MoveIt, RSP and ros2_control. Planning
    # parameters retain the original TRAC-IK and JTC action configuration.
    planning = load('joint_limits.yaml')
    for joint in ET.fromstring(description).findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limit = joint.find('limit')
        entry = planning['joint_limits'].setdefault(joint.get('name'), {})
        entry.update(has_position_limits=True, min_position=float(limit.get('lower')),
                     max_position=float(limit.get('upper')))
    return {
        'robot_description': description,
        'robot_description_semantic': (assets / 'robot.srdf').read_text(),
        'robot_description_kinematics': load('kinematics.yaml'),
        'robot_description_planning': planning,
        **load('moveit_controllers.yaml'),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                                 'default_planning_request_adapters/ValidateWorkspaceBounds',
                                 'default_planning_request_adapters/CheckStartStateBounds',
                                 'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                                  'default_planning_response_adapters/ValidateSolution',
                                  'default_planning_response_adapters/DisplayMotionPath'],
        },
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'moveit_manage_controllers': False,
    }


def _after_controllers_started(event, context, downstream):
    del context
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='RB20 controller activation failed'))]
    return downstream


def build_actions(assets, readiness_file, *, component='all', moveit_share=None):
    if component not in COMPONENTS:
        raise ValueError(f'component must be one of {COMPONENTS}')
    assets = Path(assets).resolve()
    manifest, ready = _ready_identity(assets, readiness_file)
    description = _bound_description(assets, ready)
    config = str(assets / 'painting.yaml')
    # Named YAML sections can override later /** launch dictionaries. Require
    # the generated file itself to carry every strict profile interlock.
    profile = yaml.safe_load(Path(config).read_text())
    required = {
        'moveit_executor': {'real_painting_enabled': True, 'dry_run': False, 'painting_force_enabled': True},
        'sketch_to_waypoints': {'real_painting_enabled': True, 'dry_run': False},
        'painting_wrench_reference': {'dry_run': False, 'enable_force': False},
        'painting_wrench_guard': {'real_painting_enabled': True},
    }
    for node_name, flags in required.items():
        params = profile.get(node_name, {}).get('ros__parameters', {})
        if any(params.get(key) is not value for key, value in flags.items()):
            raise RuntimeError(f'generated strict simulation profile is invalid for {node_name}')
    actions = [SetParameter(name='use_sim_time', value=True)]
    controller_spawner = None
    if component in ('robot_control', 'all'):
        moveit_share = moveit_share or get_package_share_directory('rbpodo_moveit_config')
        controller_spawner = _node('controller_manager', 'spawner', 'isaac_painting_controller_spawner',
            arguments=[*CONTROLLER_CHAIN, '-c', '/controller_manager',
                       '--controller-manager-timeout', '60', '--switch-timeout', '30',
                       '--activate-as-group'])
        actions.extend([
            _identity_tf('world', 'link0', 'isaac_world_to_link0'),
            _identity_tf('world', 'World', 'isaac_world_alias'),
            _node('robot_state_publisher', 'robot_state_publisher',
                  parameters=[{'robot_description': description}]),
            _node('moveit_ros_move_group', 'move_group',
                  parameters=[_moveit_parameters(assets, description, moveit_share)]),
            _node('controller_manager', 'ros2_control_node', 'controller_manager',
                  parameters=[{'robot_description': description}, str(assets / 'controllers.yaml'), config]),
            # One spawner configures all controllers first, then activates them
            # as a group. JTC claims the admittance reference interfaces.
            controller_spawner,
            _node('sketch_control', 'isaac_painting_lifecycle',
                  parameters=[{'session_id': ready['session_id'],
                               'scene_revision': ready['scene_revision']}]),
        ])
    control_action_count = len(actions)
    if component in ('perception', 'all'):
        calibration = manifest.get('calibration', {'zed': 'calibration/zed.json', 'd405': 'calibration/d405.json'})
        paths = {k: str(assets / calibration[k]) for k in ('zed', 'd405')}
        if not all(Path(path).is_file() for path in paths.values()):
            raise RuntimeError('generated simulation calibration is missing')
        actions.extend([
            _identity_tf('paint_d405_color_optical_frame', 'd405_color_optical_frame', 'isaac_d405_color_alias'),
            _identity_tf('paint_d405_depth_optical_frame', 'd405_depth_optical_frame', 'isaac_d405_depth_alias'),
            _include('sketch_control', 'rb10_perception_sketch.launch.py', {
                'painting_config_file': config, 'model_id': 'rb20_1900es', 'process_mode': 'paint',
                'real_painting_enabled': 'true', 'dry_run': 'false',
                'use_sim_depth_pointcloud': 'true', 'use_sim_d405_depth_pointcloud': 'true',
                'launch_wall_detector': 'false', 'launch_environment_scanner': 'false',
                'front_view_source': 'd405', 'use_d405_refinement': 'true',
                'use_ft_normal_controller': 'false', 'use_d405_mount_tf': 'false',
                'use_d405_optical_tf': 'false', 'use_zed_calibration_file': 'true',
                'use_d405_calibration_file': 'true', 'zed_calibration_file': paths['zed'],
                'd405_calibration_file': paths['d405'],
            }),
        ])
    if component in ('force_pipeline', 'all'):
        actions.append(_include('rbpodo_painting_control', 'painting_admittance_control.launch.py', {
            'config_file': config, 'real_painting_enabled': 'true', 'dry_run': 'false',
            # Initial output stays OFF; executor enables the original guarded
            # request only in RAMP_UP/PAINT after runtime free-space tare.
            'enable_force': 'false', 'desired_contact_force_n': '20.0',
            'start_segment_mode_node': 'false',
        }))
    if component in ('executor', 'all'):
        actions.extend([
            _node('sketch_control', 'moveit_executor', parameters=[config, {
                'model_id': 'rb20_1900es', 'process_mode': 'paint',
                'objects_config_file': str(assets / 'objects.yaml'),
                'generated_description_file': str(assets / 'robot.urdf'),
                'real_painting_enabled': True, 'dry_run': False, 'painting_force_enabled': True}]),
            _node('sketch_control', 'interlock_flight_recorder', parameters=[config]),
        ])
    if component == 'all':
        downstream = actions[control_action_count:]
        actions = actions[:control_action_count]
        # Register before the spawner starts so even a fast exit is observed.
        actions.insert(1, RegisterEventHandler(OnProcessExit(target_action=controller_spawner,
            on_exit=partial(_after_controllers_started, downstream=downstream))))
    return actions


def _setup(context):
    return build_actions(LaunchConfiguration('assets').perform(context),
        LaunchConfiguration('readiness_file').perform(context),
        component=LaunchConfiguration('component').perform(context))


def generate_launch_description():
    root = Path(os.environ.get('SKETCH_WORKSPACE', os.getcwd())).expanduser()
    runtime = root / '.runtime/rb20_painting'
    return LaunchDescription([
        DeclareLaunchArgument('component', default_value='all', choices=COMPONENTS),
        DeclareLaunchArgument('assets', default_value=str(runtime / 'assets')),
        DeclareLaunchArgument('readiness_file', default_value=str(runtime / 'managed-ready.json')),
        OpaqueFunction(function=_setup),
    ])
