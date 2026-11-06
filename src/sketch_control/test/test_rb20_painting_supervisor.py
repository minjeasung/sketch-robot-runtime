"""RB20 simulation management using disposable children, never device drivers."""
import asyncio
from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from sketch_control.process_supervisor import Supervisor, SupervisorError, build_specs


@pytest.fixture(autouse=True)
def dedicated_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '106')


class Monitor:
    def snapshot(self):
        return {'graph_fresh': True, 'nodes': [], 'readiness': None}

    def request_abort(self):
        pass


def test_rb20_profile_uses_native_sensors_and_dependency_order(tmp_path):
    options, specs = build_specs(tmp_path, {'profile': 'isaac_rb20_paint'})
    assert options['model_id'] == 'rb20_1900es'
    assert options['process_mode'] == 'paint'
    assert options['camera_backend'] == 'native'
    assert not options['launch_zed_driver'] and not options['launch_d405_driver']
    assert [(s.name, s.dependencies) for s in specs] == [
        ('simulation', ()), ('robot_control', ('simulation',)),
        ('perception', ('robot_control',)), ('force_pipeline', ('robot_control',)),
        ('executor', ('robot_control', 'perception', 'force_pipeline')), ('rosbridge', ())]
    assert specs[0].command == ('bash', 'scripts/run_rb20_painting_isaac.sh')
    for spec in specs[1:5]:
        assert spec.command == ('bash', 'scripts/run_rb20_painting_ros.sh', spec.name)
    assert specs[5].command[:4] == ('ros2', 'run', 'rosbridge_server', 'rosbridge_websocket')
    assert all('10.0.2.7' not in ' '.join(s.command) for s in specs)
    assert Supervisor(tmp_path, Monitor(), options).status()['runtime']['simulation'] is True


@pytest.mark.parametrize('override', [
    {'model_id': 'rb10_1300e_u'}, {'process_mode': 'spray'}, {'process_mode': 'weld'},
    {'camera_backend': 'outpost'}, {'launch_zed_driver': True}, {'launch_d405_driver': True},
    {'outpost_zed_hw_id': 'physical-camera'}, {'outpost_d405_serial': 'physical-camera'},
])
def test_rb20_rejects_physical_or_incompatible_settings(tmp_path, override):
    with pytest.raises(SupervisorError):
        build_specs(tmp_path, {'profile': 'isaac_rb20_paint', **override})


READY = '''
import json, os, pathlib, signal, time
signal.signal(signal.SIGINT, lambda *_: exit(0))
path = pathlib.Path('.runtime/rb20_painting/managed-ready.json')
path.parent.mkdir(parents=True, exist_ok=True)
marker = dict(pid=os.getpid(), session_id=os.environ['RB20_PAINTING_SESSION_ID'],
 scene_revision='scene-hash-123', model_id='rb20_1900es', step_index=1,
 updated_monotonic=time.monotonic(), joints_valid=True, wrench_valid=True, cameras_ready=True)
OVERRIDE
for step in range(10000):
 marker['step_index'] = step + 1
 marker['updated_monotonic'] = time.monotonic()
 UPDATE
 temporary = path.with_suffix('.tmp')
 temporary.write_text(json.dumps(marker)); temporary.replace(path)
 time.sleep(.04)
'''


def simulation_supervisor(tmp_path, override='', update='', timeout=.25, downstream=False):
    manifest = tmp_path / '.runtime/rb20_painting/assets/manifest.json'
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps({'scene_revision': 'scene-hash-123', 'model_id': 'rb20_1900es'}))
    options = {'profile': 'isaac_rb20_paint'}
    _, specs = build_specs(tmp_path, options)
    source = READY.replace('OVERRIDE', override).replace('UPDATE', update)
    specs = [replace(specs[0], command=(sys.executable, '-u', '-c', source))] + (
        [replace(specs[1], command=(sys.executable, '-c', 'import time; time.sleep(90)'))] if downstream else [])
    return Supervisor(tmp_path, Monitor(), options, specs=specs, grace=.01,
                      stop_timeout=.05, readiness_timeout=timeout)


def test_rb20_ready_requires_advancing_feedback_and_exports_owned_identity(tmp_path, monkeypatch):
    async def run():
        supervisor = simulation_supervisor(tmp_path, timeout=2, downstream=True)
        try:
            result = await supervisor.prepare()
            marker = json.loads((tmp_path / '.runtime/rb20_painting/managed-ready.json').read_text())
            assert result['started'] == ['simulation', 'robot_control']
            assert marker['pid'] == supervisor.snapshot('simulation')['pid']
            assert marker['step_index'] >= 2
            assert supervisor.status()['runtime']['session_id'] == marker['session_id']
        finally:
            await supervisor.shutdown()
    asyncio.run(run())


@pytest.mark.parametrize('override,update', [
    ("marker['pid'] = -1", ''), ("marker['session_id'] = 'foreign-session'", ''),
    ("marker['scene_revision'] = 'foreign-hash'", ''), ("marker['model_id'] = 'rb10_1300e_u'", ''),
    ("marker['joints_valid'] = False", ''), ("marker['wrench_valid'] = False", ''),
    ("marker['cameras_ready'] = False", ''), ("marker['wrench_valid'] = 1", ''),
    ('', "marker['updated_monotonic'] = time.monotonic() - 3"),
    ('', "marker['updated_monotonic'] = float('nan')"),
    ('', "marker['updated_monotonic'] = time.monotonic() + 5"),
    ('', "marker['step_index'] = 1"), ('', "marker['step_index'] = True"),
])
def test_rb20_rejects_foreign_stale_invalid_or_stalled_marker(tmp_path, override, update):
    async def run():
        supervisor = simulation_supervisor(tmp_path, override, update)
        with pytest.raises(SupervisorError, match='readiness'):
            await supervisor.start('simulation')
        assert supervisor.snapshot('simulation')['state'] == 'FAILED'
        assert supervisor.records['simulation']['process'] is None
        assert not supervisor._group_alive(supervisor.snapshot('simulation')['last_pid'])
    asyncio.run(run())


def test_rb20_shutdown_interrupts_unready_start_and_never_starts_ros(tmp_path):
    async def run():
        supervisor = simulation_supervisor(tmp_path, "marker['cameras_ready'] = False", timeout=10, downstream=True)
        task = asyncio.create_task(supervisor.prepare())
        try:
            while supervisor.records['simulation']['process'] is None:
                await asyncio.sleep(.01)
            await asyncio.wait_for(supervisor.shutdown(), 2)
            with pytest.raises(SupervisorError, match='cancelled by shutdown'):
                await task
            assert supervisor.snapshot('robot_control')['start_count'] == 0
            assert all(r['process'] is None for r in supervisor.records.values())
        finally:
            if not task.done():
                task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await supervisor.shutdown()
    asyncio.run(run())


def test_api_rb20_switch_sources_dedicated_overlay_before_monitor(tmp_path):
    root = Path(__file__).resolve().parents[3]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    shutil.copy(root / 'scripts/run_system_api.sh', scripts)
    shutil.copy(root / 'scripts/rb20_painting_dds_env.sh', scripts)
    (scripts / 'rb20_painting_ros_env.sh').write_text('source "$(dirname -- "${BASH_SOURCE[0]}")/rb20_painting_dds_env.sh"\nexport OVERLAY_WAS_SOURCED=yes\n')
    python = tmp_path / '.venv-api/bin/python'
    python.parent.mkdir(parents=True)
    python.write_text('#!/usr/bin/env python3\nimport json, os, sys\nprint(json.dumps({"env": dict(os.environ), "argv": sys.argv[1:]}))\n')
    python.chmod(0o755)
    result = subprocess.run(['bash', str(scripts / 'run_system_api.sh'), '--rb20-paint', '--port', '8070'],
                            env={**os.environ, 'ROS_DOMAIN_ID': '106'}, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data['env']['OVERLAY_WAS_SOURCED'] == 'yes'
    assert data['env']['SKETCH_PROFILE'] == 'isaac_rb20_paint'
    assert data['env']['SKETCH_MODEL_ID'] == 'rb20_1900es'
    assert data['env']['SKETCH_PROCESS_MODE'] == 'paint'
    assert data['env']['SKETCH_CAMERA_BACKEND'] == 'native'
    assert 'ROS_LOCALHOST_ONLY' not in data['env']
    assert data['env']['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SYSTEM_DEFAULT'
    assert data['env']['ROS_DOMAIN_ID'] == '106'
    assert '--rb20-paint' not in data['argv']
    assert data['argv'][-2:] == ['--port', '8070']


def test_configuration_api_applies_rb20_defaults_but_rejects_explicit_physical_choice(tmp_path):
    from fastapi.testclient import TestClient
    from sketch_control.system_api import create_app
    (tmp_path / 'web').mkdir()
    supervisor = Supervisor(tmp_path, Monitor(), {'profile': 'fake'})
    with TestClient(create_app(supervisor), base_url='http://localhost') as client:
        response = client.post('/configuration', json={'profile': 'isaac_rb20_paint'})
        assert response.status_code == 200, response.text
        assert response.json()['model_id'] == 'rb20_1900es'
        assert response.json()['camera_backend'] == 'native'
        assert client.get('/status').json()['runtime']['simulation'] is True
        response = client.post('/configuration', json={'profile': 'isaac_rb20_paint', 'model_id': 'rb10_1300e_u'})
        assert response.status_code == 400
        response = client.post('/configuration', json={'profile': 'isaac_rb20_paint', 'camera_backend': 'outpost'})
        assert response.status_code == 400


def test_rb20_child_identity_and_isolation_are_shared_with_ros(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '106')
    async def run():
        supervisor = simulation_supervisor(tmp_path, timeout=2, downstream=True)
        spec = supervisor.records['robot_control']['spec']
        supervisor.records['robot_control']['spec'] = replace(spec, command=(sys.executable, '-u', '-c',
            'import json, os, time; keys = ("RB20_PAINTING_SESSION_ID", "RB20_PAINTING_SCENE_REVISION", '
            '"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE", "FASTRTPS_DEFAULT_PROFILES_FILE"); '
            'print(json.dumps({key: os.environ.get(key) for key in keys}), flush=True); time.sleep(90)'))
        try:
            await supervisor.prepare()
            deadline = asyncio.get_running_loop().time() + 2.0
            while not (lines := supervisor.logs('robot_control')['lines']):
                assert asyncio.get_running_loop().time() < deadline, 'ROS child did not publish its environment'
                await asyncio.sleep(.01)
            env = json.loads(lines[0])
            identity = supervisor.status()['runtime']
            assert env['RB20_PAINTING_SESSION_ID'] == identity['session_id']
            assert env['RB20_PAINTING_SCENE_REVISION'] == 'scene-hash-123'
            assert env['ROS_DOMAIN_ID'] == '106'
            assert env['ROS_LOCALHOST_ONLY'] is None
            assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SYSTEM_DEFAULT'
            assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(tmp_path/'config/rb20_painting_fastdds.xml')
        finally:
            await supervisor.shutdown()
    asyncio.run(run())


def test_rb20_simulation_restarts_with_a_new_session(tmp_path):
    async def run():
        supervisor = simulation_supervisor(tmp_path, timeout=2)
        try:
            await supervisor.start('simulation')
            first = supervisor.status()['runtime']['session_id']
            await supervisor.restart('simulation')
            assert supervisor.status()['runtime']['session_id'] != first
        finally:
            await supervisor.shutdown()
    asyncio.run(run())


@pytest.mark.parametrize('domain', ['0', '105'])
def test_rb20_configuration_requires_dedicated_monitor_domain(tmp_path, monkeypatch, domain):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    async def run():
        supervisor = Supervisor(tmp_path, Monitor(), {'profile': 'fake'})
        with pytest.raises(SupervisorError, match='--rb20-paint'):
            await supervisor.configure({'profile': 'isaac_rb20_paint'})
        assert supervisor.options['profile'] == 'fake'
    asyncio.run(run())


def test_rb20_start_requires_dedicated_monitor_domain(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '105')
    async def run():
        supervisor = simulation_supervisor(tmp_path)
        try:
            with pytest.raises(SupervisorError, match='--rb20-paint'):
                await supervisor.start('simulation')
            assert supervisor.snapshot('simulation')['start_count'] == 0
        finally:
            await supervisor.shutdown()
    asyncio.run(run())


def test_rb20_websocket_port_controls_command_probe_and_public_browser_config(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from sketch_control.system_api import create_app
    monkeypatch.setenv('SKETCH_ROSBRIDGE_PORT', '9091')
    (tmp_path / 'web').mkdir()
    supervisor = Supervisor(tmp_path, Monitor(), {'profile': 'isaac_rb20_paint'})
    command = supervisor.records['rosbridge']['spec'].command
    assert 'port:=9091' in command
    assert supervisor.status()['runtime']['rosbridge_port'] == 9091
    with TestClient(create_app(supervisor, api_token='private'), base_url='http://localhost') as client:
        response = client.get('/runtime-connection')
        assert response.status_code == 200
        assert response.json() == {'rosbridge_port': 9091, 'ros_domain_id': '106', 'simulation': True}


@pytest.mark.parametrize('value', ['bad', '0', '65536', '-1'])
def test_rb20_invalid_websocket_port_is_rejected(tmp_path, monkeypatch, value):
    monkeypatch.setenv('SKETCH_ROSBRIDGE_PORT', value)
    with pytest.raises(SupervisorError, match='SKETCH_ROSBRIDGE_PORT'):
        build_specs(tmp_path, {'profile': 'isaac_rb20_paint'})


def test_rb20_preflight_checks_its_configured_websocket_port(tmp_path, monkeypatch):
    import socket
    with socket.socket() as occupied:
        occupied.bind(('127.0.0.1', 0))
        occupied.listen()
        port = occupied.getsockname()[1]
        monkeypatch.setenv('SKETCH_ROSBRIDGE_PORT', str(port))
        supervisor = Supervisor(tmp_path, Monitor(), {'profile': 'isaac_rb20_paint'})
        with pytest.raises(SupervisorError, match=f'Port {port} is already occupied'):
            supervisor.preflight('rosbridge')
