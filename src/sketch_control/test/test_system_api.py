"""Supervisor tests use disposable Python subprocesses, never ROS/hardware."""
import asyncio
from dataclasses import replace
from pathlib import Path
import signal
import sys
import time

import pytest
pytest.importorskip('fastapi', reason='Install the optional API environment with scripts/setup_system_api.sh')
pytest.importorskip('httpx')
from fastapi.testclient import TestClient
from sketch_control.process_supervisor import ProcessSpec, Supervisor, SupervisorError, build_specs
from sketch_control.system_api import create_app


class Monitor:
    def __init__(self):
        self.nodes = []
        self.fresh = True
        self.aborts = 0
    def snapshot(self):
        return dict(graph_fresh=self.fresh, nodes=self.nodes, readiness=None, execution=None)
    def request_abort(self):
        self.aborts += 1


def make_supervisor(tmp_path, commands=None):
    (tmp_path/'web').mkdir(exist_ok=True)
    (tmp_path/'web/system.html').write_text('dashboard')
    (tmp_path/'web/index.html').write_text('sketch')
    _, specs = build_specs(tmp_path, {'profile': 'fake'})
    command = (sys.executable, '-u', '-c', 'import time; print("TEST_PROCESS", flush=True); time.sleep(90)')
    specs = [replace(s, command=(commands or {}).get(s.name, command)) for s in specs]
    return Supervisor(tmp_path, Monitor(), {'profile': 'fake'}, specs=specs, grace=.06, stop_timeout=.2)


@pytest.fixture
def supervisor(tmp_path):
    return make_supervisor(tmp_path)


@pytest.fixture
def client(supervisor):
    with TestClient(create_app(supervisor), base_url='http://localhost') as client:
        yield client


@pytest.mark.parametrize('profile,fake,dry,force', [
    ('dry_run', 'false', 'true', 'false'), ('work', 'false', 'false', 'true'),
    ('fake', 'true', 'true', 'false'), ('spray_motion_test', 'false', 'false', 'false')])
def test_group_commands_preserve_interlocks(tmp_path, profile, fake, dry, force):
    options, specs = build_specs(tmp_path, {'profile': profile})
    for spec in specs:
        assert 'model_id:=rb10_1300e_u' in spec.command
        assert f'use_fake_hardware:={fake}' in spec.command
        assert f'dry_run:={dry}' in spec.command
        assert f'painting_force_enabled:={force}' in spec.command
        assert f'spray_motion_test:={str(profile == "spray_motion_test").lower()}' in spec.command
        assert f'real_painting_enabled:={str(profile in ("work", "spray_motion_test")).lower()}' in spec.command
        for name in ('robot_control', 'perception', 'force_pipeline', 'executor', 'rosbridge'):
            assert f'launch_{name}:={str(name == spec.name).lower()}' in spec.command
        assert f'enable_interlock_flight_recorder:={str(spec.name == "executor").lower()}' in spec.command


@pytest.mark.parametrize('options', [{'profile': 'custom'}, {'robot_ip': '$(touch /tmp/oops)'},
    {'robot_ip': True}, {'launch_zed_driver': 'false'}, {'dry_run': False}, {'command': 'sh'}])
def test_reject_arbitrary_configuration(tmp_path, options):
    with pytest.raises(SupervisorError):
        build_specs(tmp_path, options)


@pytest.mark.parametrize('profile', ['dry_run', 'work', 'fake', 'spray_motion_test', 'zed_preview'])
def test_eoat_profile_path_forwarding(tmp_path, profile):
    path = str((tmp_path / 'profiles' / 'nozzle outlet.json').resolve())
    options, specs = build_specs(tmp_path, {
        'profile': profile, 'spray_eoat_profile': 'profiles/../profiles/nozzle outlet.json'})
    assert options['spray_eoat_profile'] == path
    launches = [s for s in specs if s.command[:2] == ('ros2', 'launch')]
    assert launches
    assert all('spray_eoat_profile:=' + path in s.command for s in launches)
    options, specs = build_specs(tmp_path, {'profile': profile})
    assert options['spray_eoat_profile'] == ''
    assert all('spray_eoat_profile:=' in s.command for s in specs
               if s.command[:2] == ('ros2', 'launch'))


@pytest.mark.parametrize('profile', ['dry_run', 'work', 'fake', 'spray_motion_test', 'zed_preview'])
def test_eoat_profile_expands_home_before_workspace(tmp_path, profile):
    expected = str(Path.home() / 'tools' / 'tool.json')
    options, specs = build_specs(tmp_path, {
        'profile': profile, 'spray_eoat_profile': '~/tools/tool.json'})
    assert options['spray_eoat_profile'] == expected
    assert all('spray_eoat_profile:=' + expected in spec.command for spec in specs
               if spec.command[:2] == ('ros2', 'launch'))


@pytest.mark.parametrize('value', [True, 123, None, [], 'a\n.json', 'a\x00.json',
                                  'a\x7f.json', 'a\x85.json', 'x' * 4097])
def test_eoat_profile_rejects_invalid_paths(tmp_path, value):
    with pytest.raises(SupervisorError, match='spray_eoat_profile'):
        build_specs(tmp_path, {'spray_eoat_profile': value})


def test_eoat_profile_api_roundtrip_preserve_clear_and_lock(client, supervisor, tmp_path):
    path = str((tmp_path / 'unconfirmed nozzle.json').resolve())
    response = client.post('/configuration', json={'profile': 'zed_preview', 'spray_eoat_profile': path})
    assert response.status_code == 200, response.text
    assert response.json()['spray_eoat_profile'] == path
    assert client.get('/configuration').json()['spray_eoat_profile'] == path
    assert client.get('/status').json()['configuration']['spray_eoat_profile'] == path
    assert client.post('/configuration', json={'profile': 'fake'}).json()['spray_eoat_profile'] == path
    supervisor.records['executor']['process'] = object()
    try:
        assert client.post('/configuration', json={'spray_eoat_profile': ''}).status_code == 409
    finally:
        supervisor.records['executor']['process'] = None
    assert client.post('/configuration', json={'spray_eoat_profile': ''}).json()['spray_eoat_profile'] == ''


@pytest.mark.parametrize('value,status', [(False, 422), (123, 422), ('bad\npath', 400), ('x' * 4097, 400)])
def test_eoat_profile_api_rejects_invalid_values(client, value, status):
    assert client.post('/configuration', json={'spray_eoat_profile': value}).status_code == status


def test_eoat_profile_environment_initializes_server(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from sketch_control import system_api
    (tmp_path / 'web').mkdir()
    captured = {}
    monkeypatch.setenv('SKETCH_SPRAY_EOAT_PROFILE', 'profiles/nozzle.json')
    monkeypatch.setattr(sys, 'argv', ['system-api', '--workspace', str(tmp_path), '--host', '127.0.0.1'])
    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
    monkeypatch.setitem(sys.modules, 'sketch_control.system_ros_monitor',
                        SimpleNamespace(RosMonitor=lambda: SimpleNamespace(close=lambda: None)))
    monkeypatch.setitem(sys.modules, 'uvicorn',
                        SimpleNamespace(run=lambda app, **kwargs: captured.update(app=app)))
    system_api.main()
    assert captured['app'].state.supervisor.options['spray_eoat_profile'] == str(
        (tmp_path / 'profiles/nozzle.json').resolve())


def test_api_model_selection_is_explicit_and_locked_while_running(client, supervisor):
    assert client.post('/configuration',json={'profile':'fake','model_id':'rb20_1900es'}).status_code==200
    assert client.get('/status').json()['configuration']['model_id']=='rb20_1900es'
    assert all('model_id:=rb20_1900es' in r['spec'].command for r in supervisor.records.values())
    # Represent a process already owned by the supervisor, without launching hardware.
    supervisor.records['robot_control']['process']=object()
    try:
        assert client.post('/configuration',json={'profile':'fake','model_id':'rb10_1300e_u'}).status_code==409
    finally:
        supervisor.records['robot_control']['process']=None
    assert client.post('/configuration',json={'model_id':'unknown'}).status_code==400


def test_api_process_selection_reaches_supervisor_and_locks_while_running(client, supervisor):
    response = client.post('/configuration', json={'profile': 'fake', 'process_mode': 'spray'})
    assert response.status_code == 200, response.text
    assert response.json()['process_mode'] == 'spray'
    assert all('process_mode:=spray' in r['spec'].command for r in supervisor.records.values())
    supervisor.records['robot_control']['process'] = object()
    try:
        assert client.post('/configuration', json={'process_mode': 'paint'}).status_code == 409
    finally:
        supervisor.records['robot_control']['process'] = None
    assert client.post('/configuration', json={'process_mode': 'unknown'}).status_code == 400


def test_api_motion_test_defaults_to_spray_and_rejects_explicit_paint(client):
    response = client.post('/configuration', json={'profile': 'spray_motion_test'})
    assert response.status_code == 200, response.text
    assert response.json()['process_mode'] == 'spray'
    assert client.post('/configuration', json={
        'profile': 'spray_motion_test', 'process_mode': 'paint',
    }).status_code == 400


def test_rb20_missing_calibration_rejects_prepare_before_any_process_starts(client, supervisor):
    assert client.post('/configuration',json={'profile':'spray_motion_test','model_id':'rb20_1900es'}).status_code==200
    response=client.post('/prepare-system')
    assert response.status_code==409
    assert 'calibration required' in response.json()['detail']
    assert all(r['process'] is None for r in supervisor.records.values())


def test_preflight_duplicate_and_discovery(supervisor):
    supervisor.monitor.nodes = ['/move_group']
    with pytest.raises(SupervisorError, match='Already running'):
        supervisor.preflight('robot_control')
    supervisor.monitor.nodes = []
    supervisor.monitor.fresh = False
    with pytest.raises(SupervisorError, match='discovery'):
        supervisor.preflight('robot_control')


def test_allow_external_camera_only_when_disabled(supervisor):
    supervisor.monitor.nodes = ['/zed/zed_node']
    supervisor.preflight('perception')  # fake profile: drivers disabled
    supervisor.options['launch_zed_driver'] = True
    with pytest.raises(SupervisorError):
        supervisor.preflight('perception')


def test_api_dependency_order_cascade_and_logs(client, supervisor):
    assert client.post('/processes/executor/start').status_code == 409
    result = client.post('/prepare-system')
    assert result.status_code == 200, result.text
    assert result.json()['started'] == list(supervisor.records)
    state = client.get('/status').json()
    assert state['system_prepared'] and state['ros']['readiness'] is None
    assert client.post('/processes/robot_control/start').json()['start_count'] == 1
    assert client.post('/processes/robot_control/stop').status_code == 409
    assert client.post('/configuration', json={'profile': 'work'}).status_code == 409
    assert 'TEST_PROCESS' in client.get('/processes/executor/logs').json()['lines']
    assert client.post('/processes/robot_control/stop?cascade=true').status_code == 200
    assert supervisor.monitor.aborts == 3
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'
    assert client.get('/processes/rosbridge').json()['state'] == 'RUNNING'
    assert client.post('/shutdown-system').status_code == 200
    assert all(p['pid'] is None for p in client.get('/status').json()['processes'])


def test_failed_prepare_rolls_back_only_new_processes(tmp_path):
    supervisor = make_supervisor(tmp_path, {'force_pipeline': (sys.executable, '-c', 'raise SystemExit(2)')})
    with TestClient(create_app(supervisor), base_url='http://localhost') as client:
        assert client.post('/processes/robot_control/start').status_code == 200
        response = client.post('/prepare-system')
        assert response.status_code == 409
        assert client.get('/processes/robot_control').json()['state'] == 'RUNNING'
        assert client.get('/processes/perception').json()['state'] == 'STOPPED'
        assert client.get('/processes/force_pipeline').json()['state'] == 'FAILED'
        assert client.get('/processes/force_pipeline').json()['pid'] is None


def test_restart_stops_dependents_without_restarting_them(client):
    assert client.post('/prepare-system').status_code == 200
    assert client.post('/processes/robot_control/restart').status_code == 409
    assert client.post('/processes/robot_control/restart?cascade=true').status_code == 200
    assert client.get('/processes/robot_control').json()['start_count'] == 2
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'


def test_dependency_crash_stops_executor(client, supervisor):
    assert client.post('/prepare-system').status_code == 200
    supervisor.records['robot_control']['process'].send_signal(signal.SIGTERM)
    until = time.monotonic() + 5
    while time.monotonic() < until:
        state = client.get('/status').json()
        by_name = {p['name']: p for p in state['processes']}
        if by_name['robot_control']['state'] == 'FAILED' and by_name['executor']['state'] == 'STOPPED':
            break
        time.sleep(.05)
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'
    assert supervisor.monitor.aborts == 3
    assert client.get('/processes/robot_control').json()['state'] == 'FAILED'


def test_lifespan_closes_owned_processes(supervisor):
    with TestClient(create_app(supervisor), base_url='http://localhost') as client:
        assert client.post('/prepare-system').status_code == 200
    assert all(r['process'] is None for r in supervisor.records.values())


def test_stopping_idle_never_aborts_external_robot(client, supervisor):
    assert client.post('/shutdown-system').status_code == 200
    assert supervisor.monitor.aborts == 0


def test_authentication_and_schema(supervisor):
    with TestClient(create_app(supervisor, api_token='test-only-token'), base_url='http://localhost') as client:
        assert client.get('/healthz').status_code == 200
        assert client.get('/status').status_code == 401
        assert client.post('/prepare-system', headers={'Authorization': 'Bearer wrong'}).status_code == 401
        assert client.get('/status', headers={'Authorization': 'Bearer test-only-token'}).status_code == 200
        spec = client.get('/openapi.json').json()
        assert '/prepare-system' in spec['paths']
        assert spec['paths']['/prepare-system']['post']['security']
        assert client.get('/docs').status_code == 200


@pytest.mark.parametrize('method,path,body,headers,expected', [
    ('GET', '/status', None, {'Host': 'evil.example'}, 403),
    ('POST', '/prepare-system', None, {'Origin': 'https://evil.example'}, 403),
    ('POST', '/configuration', {'profile': 'fake', 'dry_run': False}, {}, 422),
    ('POST', '/configuration', {'launch_rviz': 'false'}, {}, 422),
    ('GET', '/processes/not_registered', None, {}, 404),
    ('POST', '/processes/not_registered/start', None, {}, 404),
    ('GET', '/processes/executor/logs?lines=501', None, {}, 422)])
def test_http_rejections(client, method, path, body, headers, expected):
    assert client.request(method, path, json=body, headers=headers).status_code == expected


def test_pages_and_stopped_configuration(client):
    assert client.get('/').text == 'dashboard'
    assert client.get('/sketch/').text == 'sketch'
    result = client.post('/configuration', json={'profile': 'fake'})
    assert result.status_code == 200
    assert result.json()['launch_zed_driver'] is False


def test_zed_preview_registry_has_no_robot_or_force_process(tmp_path):
    options, specs = build_specs(tmp_path, {'profile': 'zed_preview'})
    assert options['process_mode'] == 'spray'
    assert options['camera_backend'] == 'outpost'
    assert options['launch_rviz'] is False
    assert [s.name for s in specs] == ['perception', 'rosbridge']
    assert all(not s.dependencies for s in specs)
    assert 'zed_preview.launch.py' in specs[0].command
    assert not any('robot_ip:=' in arg for s in specs for arg in s.command)
    assert not any(n in {'move_group', 'controller_manager', 'moveit_executor',
                        'painting_force_monitor'} for s in specs for n in s.nodes)


def test_zed_preview_rejects_contact_mode(tmp_path):
    with pytest.raises(SupervisorError, match='requires process_mode=spray'):
        build_specs(tmp_path, {'profile': 'zed_preview', 'process_mode': 'paint'})


def test_zed_preview_cannot_start_robot_process_via_api(client):
    result = client.post('/configuration', json={'profile': 'zed_preview'})
    assert result.status_code == 200, result.text
    for name in ('robot_control', 'force_pipeline', 'executor'):
        assert client.post(f'/processes/{name}/start').status_code == 404
