from pathlib import Path
from fastapi.testclient import TestClient
from test_system_api import make_supervisor
from sketch_control.system_api import create_app


def test_runtime_configuration_revision_and_legacy_guard(tmp_path):
    supervisor=make_supervisor(tmp_path)
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        initial=client.get('/runtime/configuration')
        assert initial.status_code==200
        state=initial.json()
        assert state['desired']['tools']=={'left':'spray_v2','right':'none'}
        response=client.post('/runtime/configuration',json={'expected_revision':state['revision'],'selection':{'execution_mode':'mixed','process':'weld','active_arm':'right'}})
        assert response.status_code==200
        assert client.post('/runtime/configuration',json={'expected_revision':0,'selection':{}}).status_code==409
        assert client.post('/configuration',json={'profile':'fake'}).status_code==409
        assert client.get('/status').json()['runtime_configuration']['revision']==1


def test_apply_without_build_never_falls_back_to_real(tmp_path):
    supervisor=make_supervisor(tmp_path)
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        response=client.post('/runtime/configuration',json={'expected_revision':0,'selection':{'execution_mode':'real','process':'weld'}})
        assert response.status_code==200
        response=client.post('/runtime/apply',json={'expected_revision':1})
        assert response.status_code==400
        assert all(r['process'] is None for r in supervisor.records.values())


def test_revisioned_selection_cannot_be_started_through_legacy_endpoints(tmp_path):
    supervisor=make_supervisor(tmp_path)
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        assert client.post('/runtime/configuration',json={'expected_revision':0,'selection':{'process':'weld'}}).status_code==200
        for path in ['/prepare-system','/processes/robot_control/start','/processes/robot_control/restart']:
            assert client.post(path).status_code==409
        assert all(record['process'] is None for record in supervisor.records.values())


def test_advanced_rb20_requires_revision_acknowledgement_but_remains_selectable(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','106')
    supervisor=make_supervisor(tmp_path)
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        client.post('/runtime/configuration',json={'expected_revision':0,'selection':{'process':'paint'}})
        assert client.post('/configuration',json={'profile':'isaac_rb20_paint'}).status_code==409
        response=client.post('/configuration',json={'profile':'isaac_rb20_paint','expected_revision':1})
        assert response.status_code==200,response.text
        assert response.json()['model_id']=='rb20_1900es'
        assert client.get('/runtime/configuration').json()['revision']==2
        assert client.post('/runtime/apply',json={'expected_revision':1}).status_code==409


def test_shutdown_revokes_persisted_runtime_ready(tmp_path):
    import asyncio
    from test_runtime_apply import manager
    supervisor=make_supervisor(tmp_path)
    supervisor.runtime_manager=manager(tmp_path/'runtime')
    asyncio.run(supervisor.runtime_manager.apply(0))
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        assert client.post('/shutdown-system').status_code==200
        state=client.get('/runtime/configuration').json()
        assert state['phase']!='ready'
        assert state['applied'] is not None


def test_legacy_start_in_progress_blocks_runtime_configuration(tmp_path):
    import asyncio
    import pytest
    from sketch_control.runtime_apply import RuntimeApplyError
    async def scenario():
        supervisor=make_supervisor(tmp_path)
        supervisor.grace=.15
        task=asyncio.create_task(supervisor.start('robot_control'))
        try:
            while supervisor.records['robot_control']['process'] is None:await asyncio.sleep(.001)
            with pytest.raises(RuntimeApplyError):
                supervisor.runtime_manager.configure({'expected_revision':0,'selection':{'process':'weld'}})
            await task
        finally:
            await task
            await supervisor.shutdown()
    asyncio.run(scenario())


def test_manual_process_stop_during_apply_requires_transaction_shutdown(tmp_path):
    import asyncio
    import pytest
    from sketch_control.runtime_apply import RuntimeApplyError
    from test_runtime_apply import manager,asset_bundle
    async def scenario():
        supervisor=make_supervisor(tmp_path);entered=asyncio.Event();release=asyncio.Event()
        async def build(selection,directory):entered.set();await release.wait();return asset_bundle(selection,directory)
        supervisor.runtime_manager=manager(tmp_path/'runtime',build=build)
        task=asyncio.create_task(supervisor.runtime_manager.apply(0));await entered.wait()
        try:
            with pytest.raises(RuntimeApplyError):await supervisor.stop('robot_control')
        finally:
            release.set();await task;await supervisor.shutdown()
    asyncio.run(scenario())


def test_live_process_with_expired_platform_readiness_is_not_reported_ready(tmp_path):
    import asyncio,json,time
    from types import SimpleNamespace
    from sketch_control.process_supervisor import ProcessSpec
    from test_runtime_apply import manager
    supervisor=make_supervisor(tmp_path)
    supervisor.runtime_manager=manager(tmp_path/'runtime')
    state=asyncio.run(supervisor.runtime_manager.apply(0))
    bundle=json.loads((Path(state['bundle_directory'])/'bundle.json').read_text())
    supervisor._runtime_bundle=bundle
    supervisor.options['profile']='platform'
    supervisor._set_specs([ProcessSpec('simulation','test',(),('unused',),())])
    supervisor.records['simulation'].update(process=SimpleNamespace(pid=123,returncode=None),state='RUNNING',started=time.monotonic()-20)
    (Path(bundle['directory'])/'managed-ready.json').write_text(json.dumps(dict(pid=123,asset_revision=bundle['revision'],runtime_revision=0,ready=True,step_index=3,updated_monotonic=time.monotonic()-10)))
    state=supervisor.status()['runtime_configuration']
    assert state['phase']=='failed'
    assert 'readiness' in state['error'].lower()


def test_persisted_legacy_selection_is_restored_without_starting_it(tmp_path,monkeypatch):
    import asyncio
    from sketch_control.process_supervisor import Supervisor
    from test_system_api import Monitor
    monkeypatch.setenv('ROS_DOMAIN_ID','106')
    original=Supervisor(tmp_path,Monitor(),{'profile':'fake'})
    original.runtime_manager.configure({'expected_revision':0,'selection':{'process':'paint'}})
    asyncio.run(original.configure({'profile':'isaac_rb20_paint','expected_revision':1}))
    restored=Supervisor(tmp_path,Monitor(),{'profile':'fake'})
    assert restored.options['profile']=='isaac_rb20_paint'
    assert restored.options['model_id']=='rb20_1900es'
    assert all(record['process'] is None for record in restored.records.values())


def test_actual_motion_status_contract_blocks_edit_and_apply(tmp_path):
    supervisor=make_supervisor(tmp_path)
    for source,payload in [('readiness',{'running':True,'state':'IDLE'}),
                           ('welding',{'running':True,'state':'STOPPING'}),
                           ('motion_busy',True)]:
        supervisor.monitor.snapshot=lambda:{'graph_fresh':True,'nodes':[],source:payload}
        with TestClient(create_app(supervisor),base_url='http://localhost') as client:
            assert client.post('/runtime/configuration',json={'expected_revision':0,'selection':{}}).status_code==409
            assert client.post('/runtime/apply',json={'expected_revision':0}).status_code==409
            assert client.get('/runtime/configuration').json()['revision']==0


def test_confirmed_owned_simulator_shutdown_clears_motion_observations(tmp_path):
    import asyncio
    from dataclasses import replace
    supervisor=make_supervisor(tmp_path)
    spec=replace(supervisor.records['robot_control']['spec'],name='simulation')
    supervisor._set_specs([spec])
    cleared=[]
    supervisor.monitor.clear_motion_observations=lambda:cleared.append(True)
    async def scenario():
        await supervisor.start('simulation')
        supervisor.options['profile']='platform'
        await supervisor.shutdown()
    asyncio.run(scenario())
    assert cleared==[True]
    asyncio.run(supervisor.shutdown())
    assert cleared==[True], 'Empty shutdown must not clear foreign motion observations'


def test_platform_startup_reason_reaches_apply_response_and_persisted_state(tmp_path):
    import json,time
    from dataclasses import replace
    from types import SimpleNamespace
    from test_runtime_apply import manager
    supervisor=make_supervisor(tmp_path)
    supervisor._set_specs([replace(next(iter(supervisor.records.values()))['spec'],name='simulation')])
    reason='Paint hardware motion inhibited; full stack restart required. Inspect ros.log.'
    async def start(bundle):
        supervisor._runtime_bundle=bundle
        supervisor.records['simulation']['started']=time.monotonic()-.1
        (Path(bundle['directory'])/'startup-error.json').write_text(json.dumps(dict(error=reason,pid=987654,
            session_id='sim-session',asset_revision=bundle['revision'],runtime_revision=bundle['runtime_revision'],updated_monotonic=time.monotonic())))
        await supervisor._wait_platform_ready(SimpleNamespace(pid=987654,returncode=1))
    supervisor.runtime_manager=manager(tmp_path/'runtime',start=start)
    with TestClient(create_app(supervisor),base_url='http://localhost') as client:
        response=client.post('/runtime/apply',json={'expected_revision':0})
        assert response.status_code==503
        assert response.json()['detail']==reason
        state=client.get('/runtime/configuration').json()
        assert state['phase']=='failed' and state['error']==reason
        assert client.get('/status').json()['runtime_configuration']['error']==reason


def test_foreign_or_prelaunch_startup_error_is_not_used(tmp_path):
    import asyncio,json,time
    from dataclasses import replace
    from types import SimpleNamespace
    from sketch_control.process_supervisor import SupervisorError
    import pytest
    supervisor=make_supervisor(tmp_path)
    supervisor._set_specs([replace(next(iter(supervisor.records.values()))['spec'],name='simulation')])
    supervisor._runtime_bundle={'directory':str(tmp_path),'revision':'current','runtime_revision':3}
    started=time.monotonic();supervisor.records['simulation']['started']=started
    base=dict(error='foreign failure',pid=987654,session_id='session',asset_revision='current',runtime_revision=3,updated_monotonic=started)
    for patch in ({'pid':1},{'asset_revision':'old'},{'runtime_revision':2},{'updated_monotonic':started-1},{'updated_monotonic':started+100},{'session_id':''}):
        (tmp_path/'startup-error.json').write_text(json.dumps({**base,**patch}))
        with pytest.raises(SupervisorError,match='Platform runtime exited'):
            asyncio.run(supervisor._wait_platform_ready(SimpleNamespace(pid=987654,returncode=1)))
