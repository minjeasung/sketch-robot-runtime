"""The checker must drain startup discovery before sending workflow input."""
import importlib.util
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import check_rb20_painting_isaac as checker


def observer(monkeypatch, *, recover=True, wrong_session=False, query_delay=0.0):
    node = object.__new__(checker.Observer)
    clock = {'now': 10.0}
    monkeypatch.setattr(checker.time, 'monotonic', lambda: clock['now'])
    names = ['joint_state_broadcaster', 'force_torque_sensor_broadcaster',
             'force_torque_sensor_broadcaster_raw', 'admittance_controller',
             'joint_trajectory_controller']
    controllers = SimpleNamespace(controller=[SimpleNamespace(name=n, state='active') for n in names])
    queries = []

    def query(_request):
        queries.append(clock['now'])
        deadline = clock['now'] + query_delay
        return SimpleNamespace(done=lambda: clock['now'] >= deadline, result=lambda: controllers)

    def spin(_node, timeout_sec):
        clock['now'] += min(timeout_sec, .01)
        if recover and clock['now'] >= 10.06:
            state = SimpleNamespace(playing=True, joints_valid=True, wrench_valid=True,
                                    session_id='current', scene_revision='revision')
            node.store('state', state)
            node.store('hardware', {'inhibited': False, 'session_id': 'foreign' if wrong_session else 'current', 'scene_revision': 'revision'})
            node.store('lifecycle', {'ok': True, 'session_id': 'current', 'scene_revision': 'revision'})
            for key in checker.CAMERA_TOPICS:
                node.store(key, {'width': 640, 'height': 480})
            node.pose_errors = [{'position_m': .0001, 'orientation_deg': .01}]

    node.node = object()
    node.rclpy = SimpleNamespace(spin_once=spin)
    node.latest, node.received, node.counts = {}, {}, {}
    node.pose_errors, node.plane_evidence = [], None
    node.controllers = SimpleNamespace(service_is_ready=lambda: True, call_async=query)
    service = SimpleNamespace(ListControllers=SimpleNamespace(Request=lambda: object()))
    monkeypatch.setitem(sys.modules, 'controller_manager_msgs.srv', service)
    return node, clock, queries


def test_startup_wait_recovers_only_after_all_live_snapshot_checks_pass(monkeypatch):
    node, clock, _ = observer(monkeypatch)
    assert not node.report()['passed']  # Initial discovery is not acceptance.
    report = node.wait_healthy(seconds=.5)
    assert report['passed'] and all(report['checks'].values())
    assert report['session_id'] == 'current'
    assert clock['now'] >= 10.06


def test_startup_health_never_accepts_foreign_identity(monkeypatch):
    node, clock, _ = observer(monkeypatch, wrong_session=True)
    with pytest.raises(RuntimeError, match='hardware_not_inhibited'):
        node.wait_healthy(seconds=.3)
    assert clock['now'] <= 10.32


def test_startup_health_remains_bounded_without_fresh_input(monkeypatch):
    node, clock, _ = observer(monkeypatch, recover=False)
    with pytest.raises(RuntimeError, match='live_measured_state'):
        node.wait_healthy(seconds=.3)
    assert clock['now'] <= 10.32


def test_slow_controller_query_does_not_extend_startup_deadline(monkeypatch):
    node, clock, _ = observer(monkeypatch, query_delay=2.0)
    with pytest.raises(RuntimeError, match='controller query'):
        node.wait_healthy(seconds=.3)
    assert clock['now'] <= 10.32


def test_measurement_dependencies_loaded_before_observer_initializes_ros():
    # Fresh interpreter: test-suite imports must not hide a cold scipy import.
    program = '''
import sys
sys.path.insert(0, sys.argv[1])
import check_rb20_painting_isaac
assert 'numpy' in sys.modules, 'numpy was deferred until ROS callbacks'
assert 'scipy.spatial.transform' in sys.modules, 'Rotation was deferred until ROS callbacks'
assert 'rclpy' not in sys.modules, 'checker initialized ROS while loading numerical dependencies'
'''
    completed = subprocess.run([sys.executable, '-c', program, str(SCRIPTS)], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr


def test_target_catalog_requires_exact_request_and_executor_ack():
    latest = {'catalog': {'generation': 'old', 'planes': [{'id': 'old:1'}]},
              'planes': {'generation': 'old', 'state': 'ready', 'error': ''}}
    received = {'catalog': 11., 'planes': 11.}
    assert not checker.target_catalog_ready(latest, received, 10., 'new')
    latest['catalog'] = {'generation': 'new', 'planes': [{'id': 'new:1'}]}
    assert not checker.target_catalog_ready(latest, received, 10., 'new')
    latest['planes'] = {'generation': 'new', 'state': 'candidates', 'error': '', 'running': False}
    assert checker.target_catalog_ready(latest, received, 10., 'new')
    received['planes'] = 9.
    assert not checker.target_catalog_ready(latest, received, 10., 'new')


@pytest.mark.parametrize('generation,received_at', [('old', 11.), ('new', 9.)])
def test_target_measurement_ignores_busy_from_old_or_preselection_status(generation, received_at):
    latest = {'planes': {'generation': generation, 'error': 'BUSY'}}
    assert not checker.target_measurement_done(latest, {'planes': received_at}, 10., 'new', 'new:1')


def test_target_measurement_keeps_current_busy_fail_closed():
    latest = {'planes': {'generation': 'new', 'error': 'BUSY'}}
    with pytest.raises(RuntimeError, match='BUSY'):
        checker.target_measurement_done(latest, {'planes': 11.}, 10., 'new', 'new:1')


def test_target_measurement_needs_correlated_plane_and_fresh_front_view():
    latest = {'planes': {'generation': 'new', 'error': '', 'state': 'ready',
              'active_id': 'new:1', 'measured': ['new:1'], 'running': False}}
    received = {'planes': 11., 'front': 9.}
    assert not checker.target_measurement_done(latest, received, 10., 'new', 'new:1')
    received['front'] = 11.
    assert checker.target_measurement_done(latest, received, 10., 'new', 'new:1')
    assert not checker.target_measurement_done(latest, received, 10., 'new', 'new:2')


def test_target_request_waits_for_selector_not_just_executor_subscription():
    node = object.__new__(checker.Observer)
    delivered = []
    endpoints = [SimpleNamespace(node_name='moveit_executor')]
    publisher = SimpleNamespace(get_subscription_count=lambda: len(endpoints),
                                publish=lambda msg: delivered.append((msg, list(endpoints))))
    node.publishers = {}
    node.node = SimpleNamespace(create_publisher=lambda *a: publisher,
        get_subscriptions_info_by_topic=lambda topic: endpoints)
    waits = []
    def wait(predicate, seconds, label):
        waits.append(label)
        if not predicate():
            endpoints.append(SimpleNamespace(node_name='target_selector'))
        assert predicate()
    node.wait = wait
    request = object()
    node.publish('/target_selection_pixels', request)
    assert 'target_selector' in {e.node_name for e in delivered[0][1]}
