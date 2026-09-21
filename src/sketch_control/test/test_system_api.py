"""Supervisor tests use disposable Python subprocesses, never ROS/hardware."""
import asyncio
from dataclasses import replace
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
    ('dry_run', 'false', 'true', 'false'), ('work', 'false', 'false', 'true'), ('fake', 'true', 'true', 'false')])
def test_group_commands_preserve_interlocks(tmp_path, profile, fake, dry, force):
    options, specs = build_specs(tmp_path, {'profile': profile})
    for spec in specs:
        assert f'use_fake_hardware:={fake}' in spec.command
        assert f'dry_run:={dry}' in spec.command
        assert f'painting_force_enabled:={force}' in spec.command
        for name in ('robot_stack', 'perception', 'force_pipeline', 'executor', 'rosbridge'):
            assert f'launch_{name}:={str(name == spec.name).lower()}' in spec.command
        assert f'enable_interlock_flight_recorder:={str(spec.name == "executor").lower()}' in spec.command


@pytest.mark.parametrize('options', [{'profile': 'custom'}, {'robot_ip': '$(touch /tmp/oops)'},
    {'robot_ip': True}, {'launch_zed_driver': 'false'}, {'dry_run': False}, {'command': 'sh'}])
def test_reject_arbitrary_configuration(tmp_path, options):
    with pytest.raises(SupervisorError):
        build_specs(tmp_path, options)


def test_preflight_duplicate_and_discovery(supervisor):
    supervisor.monitor.nodes = ['/move_group']
    with pytest.raises(SupervisorError, match='Already running'):
        supervisor.preflight('robot_stack')
    supervisor.monitor.nodes = []
    supervisor.monitor.fresh = False
    with pytest.raises(SupervisorError, match='discovery'):
        supervisor.preflight('robot_stack')


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
    assert client.post('/processes/robot_stack/start').json()['start_count'] == 1
    assert client.post('/processes/robot_stack/stop').status_code == 409
    assert client.post('/configuration', json={'profile': 'work'}).status_code == 409
    assert 'TEST_PROCESS' in client.get('/processes/executor/logs').json()['lines']
    assert client.post('/processes/robot_stack/stop?cascade=true').status_code == 200
    assert supervisor.monitor.aborts == 3
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'
    assert client.get('/processes/rosbridge').json()['state'] == 'RUNNING'
    assert client.post('/shutdown-system').status_code == 200
    assert all(p['pid'] is None for p in client.get('/status').json()['processes'])


def test_failed_prepare_rolls_back_only_new_processes(tmp_path):
    supervisor = make_supervisor(tmp_path, {'force_pipeline': (sys.executable, '-c', 'raise SystemExit(2)')})
    with TestClient(create_app(supervisor), base_url='http://localhost') as client:
        assert client.post('/processes/robot_stack/start').status_code == 200
        response = client.post('/prepare-system')
        assert response.status_code == 409
        assert client.get('/processes/robot_stack').json()['state'] == 'RUNNING'
        assert client.get('/processes/perception').json()['state'] == 'STOPPED'
        assert client.get('/processes/force_pipeline').json()['state'] == 'FAILED'
        assert client.get('/processes/force_pipeline').json()['pid'] is None


def test_restart_stops_dependents_without_restarting_them(client):
    assert client.post('/prepare-system').status_code == 200
    assert client.post('/processes/robot_stack/restart').status_code == 409
    assert client.post('/processes/robot_stack/restart?cascade=true').status_code == 200
    assert client.get('/processes/robot_stack').json()['start_count'] == 2
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'


def test_dependency_crash_stops_executor(client, supervisor):
    assert client.post('/prepare-system').status_code == 200
    supervisor.records['robot_stack']['process'].send_signal(signal.SIGTERM)
    until = time.monotonic() + 5
    while time.monotonic() < until:
        state = client.get('/status').json()
        by_name = {p['name']: p for p in state['processes']}
        if by_name['robot_stack']['state'] == 'FAILED' and by_name['executor']['state'] == 'STOPPED':
            break
        time.sleep(.05)
    assert client.get('/processes/executor').json()['state'] == 'STOPPED'
    assert supervisor.monitor.aborts == 3
    assert client.get('/processes/robot_stack').json()['state'] == 'FAILED'


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
