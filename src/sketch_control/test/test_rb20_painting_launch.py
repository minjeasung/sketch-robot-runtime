"""Exercise real ROS launch actions and session lifecycle without running a GPU."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/sketch_control'))


def launch_module():
    path = ROOT / 'src/sketch_control/launch/rb20_painting_isaac.launch.py'
    assert path.is_file(), 'dedicated generated-model launch is missing'
    spec = importlib.util.spec_from_file_location('rb20_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def assets(tmp_path):
    (tmp_path / 'calibration').mkdir()
    for camera in ('zed', 'd405'):
        (tmp_path / f'calibration/{camera}.json').write_text('{}')
    (tmp_path / 'robot.urdf').write_text('''<robot name="rb20_1900es"><link name="link0"/>
<ros2_control name="IsaacPainting" type="system"><hardware>
<plugin>isaac_painting_hardware/IsaacPaintingSystem</plugin>
<param name="session_id">unbound</param><param name="scene_revision">scene-a</param>
</hardware></ros2_control></robot>''')
    (tmp_path / 'robot.srdf').write_text('<robot name="rb20_1900es"/>')
    import yaml
    (tmp_path / 'painting.yaml').write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'use_sim_time': True}},
        'moveit_executor': {'ros__parameters': {'real_painting_enabled': True,
            'dry_run': False, 'painting_force_enabled': True}},
        'sketch_to_waypoints': {'ros__parameters': {'real_painting_enabled': True, 'dry_run': False}},
        'painting_wrench_reference': {'ros__parameters': {'dry_run': False, 'enable_force': False}},
        'painting_wrench_guard': {'ros__parameters': {'real_painting_enabled': True}},
    }))
    (tmp_path / 'controllers.yaml').write_text('controller_manager:\n  ros__parameters:\n    update_rate: 100\n')
    (tmp_path / 'objects.yaml').write_text('active_target: rb20_painting_wall\nobjects:\n'
        '  - name: rb20_painting_wall\n    shape: box\n    position: [1.48, 0, 1.1]\n'
        '    size: [0.06, 1.2, 1.4]\n    sketch_face: "-x"\n    enabled: true\n')
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'model_id': 'rb20_1900es', 'scene_revision': 'scene-a',
        'calibration': {'zed': 'calibration/zed.json', 'd405': 'calibration/d405.json'}}))
    ready = tmp_path / 'managed-ready.json'
    ready.write_text(json.dumps({'pid': __import__('os').getpid(),
        'session_id': 'session-a', 'scene_revision': 'scene-a', 'model_id': 'rb20_1900es',
        'step_index': 1, 'updated_monotonic': __import__('time').monotonic(),
        'joints_valid': True, 'wrench_valid': True, 'cameras_ready': True}))
    return tmp_path, ready


def node_parameters(node):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    return evaluate_parameters(LaunchContext(), node._Node__parameters)


def text(substitutions):
    if isinstance(substitutions, str):
        return substitutions
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch.utilities import normalize_to_list_of_substitutions
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(substitutions))


def test_generated_control_uses_identity_tf_and_single_measured_joint_owner(assets):
    from launch_ros.actions import Node
    module = launch_module()
    actions = module.build_actions(*assets, component='robot_control',
        moveit_share=ROOT / 'src/rbpodo_ros2/rbpodo_moveit_config')
    nodes = [a for a in actions if isinstance(a, Node)]
    assert sum(text(n.node_executable) == 'robot_state_publisher' for n in nodes) == 1
    assert not any(text(n.node_executable) in ('isaac_sim_rb10', 'joint_state_publisher') for n in nodes)
    tf = next(n for n in nodes if text(n._Node__node_name) == 'isaac_world_to_link0')
    assert [text([a]) for a in tf._Node__arguments][-8:] == ['--qx','0','--qy','0','--qz','0','--qw','1']
    manager = next(n for n in nodes if text(n.node_executable) == 'ros2_control_node')
    params = node_parameters(manager)
    description = next(p['robot_description'] for p in params if isinstance(p, dict) and 'robot_description' in p)
    assert 'IsaacPaintingSystem' in description and 'session-a' in description
    assert 'RBPodoSystem' not in description
    for node in nodes:
        assert any(p.get('use_sim_time') is True for p in node_parameters(node) if isinstance(p, dict))
    chain = module.CONTROLLER_CHAIN
    assert chain == ('joint_state_broadcaster', 'force_torque_sensor_broadcaster',
        'force_torque_sensor_broadcaster_raw', 'admittance_controller', 'joint_trajectory_controller')


def test_perception_reuses_pipeline_without_physical_camera_or_competing_tf(assets):
    from launch.actions import IncludeLaunchDescription
    module = launch_module()
    actions = module.build_actions(*assets, component='perception')
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments['use_sim_depth_pointcloud'] == 'true'
    assert arguments['use_sim_d405_depth_pointcloud'] == 'true'
    assert arguments['use_d405_mount_tf'] == arguments['use_d405_optical_tf'] == 'false'
    assert arguments['zed_calibration_file'] == str(assets[0] / 'calibration/zed.json')
    assert 'real_perception' not in str(include.launch_description_source.location)


def test_all_launch_delays_guarded_pipeline_until_controllers_activate(assets):
    from launch.actions import RegisterEventHandler, EmitEvent
    from launch_ros.actions import Node
    from types import SimpleNamespace
    module = launch_module()
    actions = module.build_actions(*assets, component='all',
        moveit_share=ROOT / 'src/rbpodo_ros2/rbpodo_moveit_config')
    assert any(isinstance(a, RegisterEventHandler) for a in actions), 'unguarded simultaneous startup'
    assert not any(isinstance(a, Node) and text(a.node_executable) == 'moveit_executor' for a in actions)
    downstream = [module._node('sketch_control', 'moveit_executor')]
    assert module._after_controllers_started(SimpleNamespace(returncode=0), None, downstream) == downstream
    assert isinstance(module._after_controllers_started(SimpleNamespace(returncode=1), None, downstream)[0], EmitEvent)


def test_launch_rejects_physical_plugin_or_unready_session(assets):
    module = launch_module()
    assets[1].write_text('{}')
    with pytest.raises((ValueError, RuntimeError), match='readiness'):
        module.build_actions(*assets, component='executor')


def test_launch_rejects_any_physical_hardware_plugin(assets):
    module = launch_module()
    urdf = assets[0] / 'robot.urdf'
    urdf.write_text(urdf.read_text().replace('isaac_painting_hardware/IsaacPaintingSystem', 'rbpodo/RBPodoSystem'))
    with pytest.raises(RuntimeError, match='only IsaacPaintingSystem'):
        module.build_actions(*assets, component='executor')


@pytest.mark.parametrize('name,value', [('RB20_PAINTING_SESSION_ID', 'foreign-session'),
                                      ('RB20_PAINTING_SCENE_REVISION', 'foreign-scene')])
def test_launch_binds_supervisor_expected_identity(assets, monkeypatch, name, value):
    module = launch_module()
    monkeypatch.setenv(name, value)
    with pytest.raises(RuntimeError, match='readiness'):
        module.build_actions(*assets, component='executor')


def test_global_invalidation_clears_both_d405_modes_and_pending_sensor_work():
    from types import SimpleNamespace
    from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
    from sketch_control.plane_lifecycle import PlaneLifecycle
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {name: PlaneLifecycle(name, 'old-area') for name in ('target', 'work_area')}
    for lifecycle in node._lifecycles.values():
        lifecycle.accepted = True
        lifecycle.state = 'accepted'
        lifecycle.set_paint_active(True)
    node._paint_active = True
    node._active_capture_mode = 'target'
    node._active_capture_reference = object()
    node._pending_cloud_capture = object()
    node._external_work_area_id = 'old-area'
    node._publish_status = lambda *args, **kwargs: None
    node._publish_target_status = lambda *args, **kwargs: None
    node._cancel_pending_cloud = lambda mode=None: setattr(node, '_pending_cloud_capture', None)
    assert hasattr(D405SurfaceRefinerNode, '_on_invalidate_all'), 'noncapturing invalidation API missing'
    node._on_invalidate_all(SimpleNamespace(data=True))
    assert all(not item.accepted and item.state == 'invalidated' for item in node._lifecycles.values())
    assert node._active_capture_mode is None and node._pending_cloud_capture is None
    assert node.latest_target_surface is None and node.latest_work_area_plane is None
    assert node._start_capture('target') is False


def test_strict_profile_flags_are_named_and_verified_before_ros_start(assets):
    import os
    import subprocess
    named = assets[0] / 'painting.yaml'
    result = subprocess.run([sys.executable, '-c',
        'import rclpy; rclpy.init(); n=rclpy.create_node("painting_wrench_guard"); '
        'print(n.declare_parameter("real_painting_enabled",False).value); '
        'n.destroy_node(); rclpy.shutdown()', '--ros-args',
        '--params-file', str(named)], capture_output=True, text=True, timeout=5,
        env=dict(os.environ, ROS_DOMAIN_ID='116', ROS_LOCALHOST_ONLY='1'))
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['True']
    named.write_text(named.read_text().replace('real_painting_enabled: true', 'real_painting_enabled: false'))
    with pytest.raises(RuntimeError, match='strict simulation profile'):
        launch_module().build_actions(*assets, component='force_pipeline')


def test_force_monitor_startup_grace_ends_on_first_tf_or_force_activity():
    from rbpodo_painting_control.painting_force_monitor_node import PaintingForceMonitorNode
    from types import SimpleNamespace
    node = SimpleNamespace(_startup_readiness_grace_s=5.0, _startup_started_at=10.0,
        _mode='IDLE', _force_enabled=False, _trajectory_active=False,
        _safety=SimpleNamespace(last_tf_ok_at=None))
    assert hasattr(PaintingForceMonitorNode, '_startup_grace_active'), 'TF discovery grace missing'
    assert PaintingForceMonitorNode._startup_grace_active(node, 13.0)
    assert not PaintingForceMonitorNode._startup_grace_active(node, 15.01)
    node._safety.last_tf_ok_at = 12.0
    assert not PaintingForceMonitorNode._startup_grace_active(node, 13.0)
    node._safety.last_tf_ok_at = None
    node._force_enabled = True
    assert not PaintingForceMonitorNode._startup_grace_active(node, 13.0)
    node._force_enabled = False
    node._mode = 'CONTACT_SEARCH'
    assert not PaintingForceMonitorNode._startup_grace_active(node, 13.0)
    node._mode = 'IDLE'
    node._startup_readiness_grace_s = 0.0
    assert not PaintingForceMonitorNode._startup_grace_active(node, 10.01)


def test_force_monitor_discovery_reports_invalid_and_latches_after_valid_tf():
    from rbpodo_painting_control.painting_force_monitor_node import PaintingForceMonitorNode, _noncontact_limits
    from rbpodo_painting_control.force_safety import ForceSafetyMonitor, SafetyConfig
    from types import SimpleNamespace
    config = SafetyConfig(mode_limits={'IDLE': _noncontact_limits()}, fallback_limits=_noncontact_limits(),
        contact_force_sign=1.0, absolute_normal_force=True, contact_detect_n=3.5,
        contact_release_n=2.0, contact_confirm_duration_s=0.04, force_saturation_n=190.0,
        torque_saturation_nm=14.0, filtered_timeout_s=0.20, raw_timeout_s=0.20,
        tf_timeout_s=0.20, max_message_age_s=0.20, reset_max_force_norm_n=0.75,
        reset_max_torque_norm_nm=0.10)
    safety = ForceSafetyMonitor(config, started_at=10.0)
    node = SimpleNamespace(_startup_readiness_grace_s=5.0, _startup_started_at=10.0,
        _mode='IDLE', _force_enabled=False, _safety=safety)
    PaintingForceMonitorNode._observe_transform_validity(node, False, 12.0)
    status = safety.status(12.0, 'IDLE')
    assert status['tf_valid'] is False and status['ft_valid'] is False
    assert status['abort_latched'] is False
    PaintingForceMonitorNode._observe_transform_validity(node, True, 12.1)
    PaintingForceMonitorNode._observe_transform_validity(node, False, 12.2)
    assert safety.status(12.2, 'IDLE')['reason'] == 'TF_INVALID'


def test_executor_uses_generated_collision_registry_for_acm_identity(assets):
    from sketch_control.moveit_executor import _load_executor_objects_config
    config = _load_executor_objects_config(str(assets[0] / 'objects.yaml'))
    assert config['active_target'] == 'rb20_painting_wall'
    assert config['objects'][0]['position'] == [1.48, 0, 1.1]
    actions = launch_module().build_actions(*assets, component='executor')
    from launch_ros.actions import Node
    executor = next(a for a in actions if isinstance(a, Node) and text(a.node_executable) == 'moveit_executor')
    assert any(p.get('objects_config_file') == str(assets[0] / 'objects.yaml')
               for p in node_parameters(executor) if isinstance(p, dict))


def test_executor_rejects_malformed_explicit_scene_registry(tmp_path):
    from sketch_control.moveit_executor import _load_executor_objects_config
    path = tmp_path / 'objects.yaml'
    path.write_text('active_target: missing\nobjects: []\n')
    with pytest.raises(ValueError, match='active target'):
        _load_executor_objects_config(str(path))


def test_generated_joint_bounds_override_legacy_wrist3_in_both_consumers(tmp_path):
    from sketch_control.moveit_executor import _generated_joint_limits
    from sketch_control.robot_models import JOINT_NAMES
    path = tmp_path / 'robot.urdf'
    path.write_text('<robot name="rb20">' + ''.join(
        f'<joint name="{name}" type="revolute"><limit lower="-3.14" upper="3.14"/></joint>'
        for name in JOINT_NAMES) + '</robot>')
    assert _generated_joint_limits(str(path), JOINT_NAMES)['wrist3'] == (-3.14, 3.14)
    description = path.read_text()
    (tmp_path / 'robot.srdf').write_text('<robot name="rb20"/>')
    parameters = launch_module()._moveit_parameters(tmp_path, description,
        ROOT / 'src/rbpodo_ros2/rbpodo_moveit_config')
    wrist = parameters['robot_description_planning']['joint_limits']['wrist3']
    assert wrist['min_position'] == -3.14 and wrist['max_position'] == 3.14
    assert _generated_joint_limits('', JOINT_NAMES) is None


def test_generated_joint_bounds_reject_wrong_names_and_nonfinite(tmp_path):
    from sketch_control.moveit_executor import _generated_joint_limits
    path = tmp_path / 'robot.urdf'
    path.write_text('<robot><joint name="base" type="revolute"><limit lower="nan" upper="3.14"/></joint></robot>')
    with pytest.raises(ValueError, match='joint limits'):
        _generated_joint_limits(str(path), ('base',))
    with pytest.raises(ValueError, match='joint names'):
        _generated_joint_limits(str(path), ('base', 'shoulder'))


def lifecycle_module():
    path = ROOT / 'src/sketch_control/sketch_control/isaac_painting_lifecycle.py'
    assert path.is_file(), 'wall-time simulator lifecycle is missing'
    from sketch_control import isaac_painting_lifecycle
    return isaac_painting_lifecycle


def test_lifecycle_pause_latches_and_play_does_not_resume():
    guard = lifecycle_module().SessionWatchdog('session-a', 'scene-a', timeout_s=0.20)
    assert guard.observe('session-a', 'scene-a', 1, 1.0, True, True, True, 10.0) is None
    assert guard.observe('session-a', 'scene-a', 2, 1.1, False, True, True, 10.1) == 'paused'
    assert guard.observe('session-a', 'scene-a', 3, 1.2, True, True, True, 10.2) == 'paused'
    assert guard.restart_required


def test_initial_dds_discovery_grace_does_not_weaken_active_source_lease():
    guard = lifecycle_module().SessionWatchdog('s', 'r', now=10.0, startup_timeout_s=5.0)
    assert guard.check(13.0) is None
    guard.observe_clock(1.0, 13.0)
    guard.observe('s', 'r', 1, 1.0, True, True, True, 13.0)
    assert guard.check(13.21) == 'state_stalled'
    guard = lifecycle_module().SessionWatchdog('s', 'r', now=10.0, startup_timeout_s=5.0)
    assert guard.check(15.01) == 'state_missing'


@pytest.mark.parametrize('fault,kwargs', [
    ('session_changed', {'session_id': 'old-session'}),
    ('scene_changed', {'scene_revision': 'different'}),
    ('step_rewound', {'step_index': 0}),
    ('clock_rewound', {'sim_time': 0.5}),
    ('invalid_measurement', {'wrench_valid': False}),
])
def test_lifecycle_rejects_invalid_identity_and_source(fault, kwargs):
    guard = lifecycle_module().SessionWatchdog('session-a', 'scene-a')
    data = dict(session_id='session-a', scene_revision='scene-a', step_index=1,
        sim_time=1.0, playing=True, joints_valid=True, wrench_valid=True, now=10.0)
    guard.observe(**data)
    data.update(step_index=2, sim_time=1.1, now=10.1)
    data.update(kwargs)
    assert guard.observe(**data) == fault


def test_duplicate_source_step_never_refreshes_wall_watchdog():
    guard = lifecycle_module().SessionWatchdog('session-a', 'scene-a', timeout_s=0.20)
    guard.observe('session-a', 'scene-a', 1, 1.0, True, True, True, 10.0)
    guard.observe('session-a', 'scene-a', 1, 1.0, True, True, True, 10.19)
    assert guard.check(10.21) == 'state_stalled'


def test_live_state_cannot_hide_stopped_ros_clock():
    guard = lifecycle_module().SessionWatchdog('session-a', 'scene-a', timeout_s=0.20)
    guard.observe_clock(1.0, 10.0)
    guard.observe('session-a', 'scene-a', 1, 1.0, True, True, True, 10.0)
    guard.observe('session-a', 'scene-a', 2, 1.1, True, True, True, 10.19)
    guard.observe_clock(1.0, 10.19)
    assert guard.check(10.21) == 'clock_stalled'


def test_lifecycle_sigterm_cleans_up_without_second_ros_shutdown():
    import os
    import subprocess
    import time
    env = dict(os.environ, ROS_DOMAIN_ID='116', ROS_LOCALHOST_ONLY='1')
    process = subprocess.Popen([sys.executable, '-c',
        'from sketch_control.isaac_painting_lifecycle import main; main()',
        '--ros-args', '-p', 'session_id:=shutdown-test', '-p', 'scene_revision:=test'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        time.sleep(0.4)
        process.terminate()
        output = process.communicate(timeout=5)[0]
        assert process.returncode == 0, output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_interruption_invalidates_once_and_late_perception_receives_latched_event():
    import os
    import subprocess
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from isaac_painting_msgs.msg import State
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    context = Context()
    context.init(domain_id=116)
    node = rclpy.create_node('lifecycle_event_regression', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, areas, stops, late_invalidations = [], [], [], []
    node.create_subscription(String, '/isaac/rb20_painting/lifecycle_status',
        lambda m: statuses.append(json.loads(m.data)), latched)
    node.create_subscription(String, '/painting_system/work_area_state',
        lambda m: areas.append(json.loads(m.data)), latched)
    node.create_subscription(Bool, '/motion_abort', lambda m: stops.append(m.data), 10)
    source = node.create_publisher(State, '/isaac/rb20_painting/state', 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    process = subprocess.Popen([sys.executable, '-c',
        'from sketch_control.isaac_painting_lifecycle import main; main()',
        '--ros-args', '-p', 'session_id:=event-test', '-p', 'scene_revision:=test',
        '-p', 'use_sim_time:=true'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dict(os.environ, ROS_DOMAIN_ID='116', ROS_LOCALHOST_ONLY='1'), text=True)
    step = 0
    def spin_until(predicate, playing=True, timeout=5):
        nonlocal step
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            step += 1
            msg = State(session_id='event-test', scene_revision='test', step_index=step,
                playing=playing, joints_valid=True, wrench_valid=True)
            msg.header.stamp.sec = step // 100
            msg.header.stamp.nanosec = (step % 100) * 10000000
            clock.publish(Clock(clock=msg.header.stamp))
            source.publish(msg)
            executor.spin_once(timeout_sec=0.005)
            if predicate():
                return
        pytest.fail('lifecycle DDS regression condition timed out')
    try:
        spin_until(lambda: any(s['ok'] for s in statuses))
        spin_until(lambda: len(stops) >= 5, playing=False)
        assert len(areas) == 1, 'invalidation is an event, not the repeated stop lease'
        assert areas[0]['selected'] is False
        node.create_subscription(Bool, '/perception/invalidate_all',
            lambda m: late_invalidations.append(m.data), latched)
        spin_until(lambda: late_invalidations == [True], playing=False)
        assert len(areas) == 1
        assert statuses[-1]['restart_required'] is True
    finally:
        process.terminate()
        output = process.communicate(timeout=5)[0]
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    assert process.returncode == 0, output


def test_lifecycle_timings_distinguish_receipt_gap_and_blocked_publish():
    from sketch_control.isaac_painting_lifecycle import LifecycleTimings
    trace = LifecycleTimings()
    trace.received('clock', 1.0)
    trace.received('clock', 1.25)
    trace.completed('status_publish', 0.24, 0.002)
    result = trace.snapshot(1.26)
    assert result['clock']['count'] == 2
    assert result['clock']['age_s'] == pytest.approx(0.01)
    assert result['clock']['max_gap_s'] == pytest.approx(0.25)
    assert result['status_publish']['max_wall_s'] == pytest.approx(0.24)
    assert result['status_publish']['max_cpu_s'] == pytest.approx(0.002)
    assert trace.snapshot(1.5)['clock']['age_s'] == pytest.approx(0.25)
