"""Construct supervision contracts, using disposable processes and no robot hardware."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path
import sys

import pytest
from sketch_control.process_supervisor import Supervisor, SupervisorError, build_specs


class Monitor:
    def snapshot(self):
        return {'graph_fresh': True, 'nodes': [], 'readiness': None}

    def request_abort(self):
        pass


@pytest.mark.parametrize('model', ['construct_left', 'construct_right'])
def test_construct_profile_uses_managed_adapter_and_original_stack(tmp_path, model):
    metadata = tmp_path / '.runtime/construct/zed-camera.json'
    metadata.parent.mkdir(parents=True)
    metadata.write_text(json.dumps({'hw_id': 'camera-123'}))
    options, specs = build_specs(tmp_path, {'profile': 'isaac_construct', 'model_id': model})
    assert options['process_mode'] == 'spray'
    assert options['camera_backend'] == 'outpost'
    assert options['outpost_zed_hw_id'] == 'camera-123'
    assert options['outpost_zed_serial'] == 'stream://127.0.0.1:30000'
    assert options['spray_eoat_profile'] == str(tmp_path / 'isaac_assets' / (model + '_eoat.json'))
    assert not any(options[key] for key in ('launch_rviz', 'launch_zed_driver', 'launch_d405_driver'))
    assert [(s.name, s.dependencies) for s in specs] == [
        ('simulation', ()), ('perception', ('simulation',)),
        ('executor', ('simulation', 'perception')), ('rosbridge', ())]
    assert specs[0].command == ('/usr/bin/python3', '-u', 'scripts/run_construct_demo.py', '--managed-runtime')
    assert specs[0].nodes == ('controller_manager', 'move_group')
    assert specs[1].command == ('bash', 'scripts/run_construct_perception.sh', model)
    assert specs[2].command == ('bash', 'scripts/run_construct_executor.sh', model)
    assert specs[3].command[:4] == ('ros2', 'run', 'rosbridge_server', 'rosbridge_websocket')
    assert all('10.0.2.7' not in ' '.join(s.command) for s in specs)


@pytest.mark.parametrize('override,match', [
    ({'process_mode': 'paint'}, 'process_mode=spray'),
    ({'camera_backend': 'native'}, 'camera_backend=outpost'),
    ({'model_id': 'rb10_1300e_u'}, 'construct_left'),
    ({'outpost_http': 'http://localhost:9999'}, 'outpost_http'),
    ({'outpost_zed_serial': 'stream://10.0.2.7:30000'}, 'outpost_zed_serial'),
])
def test_construct_profile_rejects_incompatible_settings(tmp_path, override, match):
    with pytest.raises(SupervisorError, match=match):
        build_specs(tmp_path, {'profile': 'isaac_construct', 'model_id': 'construct_left', **override})


@pytest.mark.parametrize('profile', ['dry_run', 'work', 'fake', 'spray_motion_test', 'zed_preview'])
def test_construct_cannot_select_physical_or_rb_launch(tmp_path, profile):
    with pytest.raises(SupervisorError, match='isaac_construct'):
        build_specs(tmp_path, {'profile': profile, 'model_id': 'construct_left'})
    with pytest.raises(SupervisorError, match='camera identifier'):
        build_specs(tmp_path, {'profile': profile, 'outpost_zed_serial': 'stream://127.0.0.1:30000'})


def simulation_supervisor(tmp_path, source, timeout=.12, perception=False):
    options = {'profile': 'isaac_construct', 'model_id': 'construct_left'}
    _, specs = build_specs(tmp_path, options)
    specs = [replace(specs[0], command=(sys.executable, '-u', '-c', source))] + (
        [replace(specs[1], command=(sys.executable, '-c', 'import time; time.sleep(90)'))]
        if perception else [])
    return Supervisor(tmp_path, Monitor(), options, specs=specs, grace=.01,
                      stop_timeout=.05, readiness_timeout=timeout)


READY = """
import json, os, pathlib, signal, time
signal.signal(signal.SIGINT, lambda *_: exit(0))
p = pathlib.Path('.runtime/construct/managed-ready.json')
p.parent.mkdir(parents=True, exist_ok=True)
time.sleep(.08)
p.with_name('zed-camera.json').write_text(json.dumps({'hw_id': 'fresh-camera'}))
p.write_text(json.dumps({'pid': os.getpid()}))
time.sleep(90)
"""


def test_simulation_waits_for_own_ready_marker_before_camera_validation(tmp_path, monkeypatch):
    from sketch_control import process_supervisor
    async def run():
        supervisor = simulation_supervisor(tmp_path, READY, timeout=2, perception=True)
        def camera_status(origin, hw_id, serial, kind):
            assert hw_id == 'fresh-camera'
            assert supervisor.snapshot('simulation')['state'] == 'RUNNING'
            marker = json.loads((tmp_path / '.runtime/construct/managed-ready.json').read_text())
            assert marker['pid'] == supervisor.snapshot('simulation')['pid']
        monkeypatch.setattr(process_supervisor, 'camera_status', camera_status)
        try:
            result = await supervisor.prepare()
            assert result['started'] == ['simulation', 'perception']
        finally:
            await supervisor.shutdown()
    asyncio.run(run())


@pytest.mark.parametrize('marker', [None, {'pid': -1}, {'pid': 'invalid'}, []])
def test_missing_or_stale_readiness_times_out_and_reaps_process(tmp_path, marker):
    path = tmp_path / '.runtime/construct/managed-ready.json'
    path.parent.mkdir(parents=True)
    if marker is not None:
        path.write_text(json.dumps(marker))
    async def run():
        supervisor = simulation_supervisor(tmp_path, 'import time; time.sleep(90)')
        with pytest.raises(SupervisorError, match='readiness'):
            await supervisor.start('simulation')
        assert supervisor.snapshot('simulation')['state'] == 'FAILED'
        assert supervisor.records['simulation']['process'] is None
        assert not supervisor._group_alive(supervisor.snapshot('simulation')['last_pid'])
    asyncio.run(run())


def test_simulation_exit_is_reported_before_readiness_timeout(tmp_path):
    async def run():
        supervisor = simulation_supervisor(tmp_path, 'raise SystemExit(7)', timeout=2)
        with pytest.raises(SupervisorError, match='exited during startup'):
            await supervisor.start('simulation')
        assert supervisor.snapshot('simulation')['last_exit_code'] == 7
    asyncio.run(run())


def test_cancelled_readiness_reaps_process(tmp_path):
    async def run():
        supervisor = simulation_supervisor(tmp_path, 'import time; time.sleep(90)', timeout=2)
        task = asyncio.create_task(supervisor.start('simulation'))
        while supervisor.records['simulation']['process'] is None:
            await asyncio.sleep(.01)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert supervisor.records['simulation']['process'] is None
    asyncio.run(run())


def test_simulation_receives_grace_to_clean_children(tmp_path):
    source = READY.replace("lambda *_: exit(0)", "lambda *_: (time.sleep(.2), exit(0))")
    async def run():
        supervisor = simulation_supervisor(tmp_path, source, timeout=2)
        await supervisor.start('simulation')
        await supervisor.stop('simulation')
        assert supervisor.snapshot('simulation')['last_exit_code'] == 0
    asyncio.run(run())


@pytest.mark.parametrize('profile,model,simulation,standoff', [
    ('isaac_construct', 'construct_left', True, .3),
    ('work', 'rb10_1300e_u', False, .5),
    ('fake', 'rb10_1300e_u', True, .5),
])
def test_status_reports_execution_environment_and_standoff(tmp_path, profile, model, simulation, standoff):
    supervisor = Supervisor(tmp_path, Monitor(), {'profile': profile, 'model_id': model})
    assert supervisor.status()['runtime'] == {'simulation': simulation, 'spray_standoff_m': standoff}


def test_shutdown_interrupts_startup_and_queued_start_without_launching_perception(tmp_path):
    async def run():
        supervisor = simulation_supervisor(tmp_path, 'import time; time.sleep(90)',
                                           timeout=30, perception=True)
        preparing = asyncio.create_task(supervisor.prepare())
        tasks = [preparing]
        try:
            while supervisor.records['simulation']['process'] is None:
                await asyncio.sleep(.01)
            pid = supervisor.records['simulation']['process'].pid
            queued_start = asyncio.create_task(supervisor.start('simulation'))
            tasks.append(queued_start)
            await asyncio.sleep(0)
            shutting_down = asyncio.create_task(supervisor.shutdown())
            tasks.append(shutting_down)
            result = await asyncio.wait_for(asyncio.shield(shutting_down), 1)
            assert result['system'] == 'stopped'
            for task in (preparing, queued_start):
                with pytest.raises(SupervisorError, match='cancelled by shutdown'):
                    await task
            assert supervisor.snapshot('perception')['start_count'] == 0
            assert all(record['process'] is None for record in supervisor.records.values())
            assert not supervisor._group_alive(pid)
            # A later deliberate start is not poisoned by the previous shutdown.
            spec = supervisor.records['simulation']['spec']
            supervisor.records['simulation']['spec'] = replace(
                spec, command=(sys.executable, '-u', '-c', READY))
            assert (await supervisor.start('simulation'))['state'] == 'RUNNING'
        finally:
            for task in tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await supervisor.shutdown()
    asyncio.run(run())
