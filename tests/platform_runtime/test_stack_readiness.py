import importlib.util
from pathlib import Path
from types import SimpleNamespace
import socket
import pytest


def module():
    path=Path(__file__).resolve().parents[2]/'scripts/platform_stack.py'
    spec=importlib.util.spec_from_file_location('platform_stack_test',path); m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def test_camera_readiness_requires_same_exposure_rgb_depth_and_both_infos():
    state=module().StackReadiness(['zed'],['jtc'])
    for kind in ('rgb','depth','info_rgb'): state.camera('zed',kind,10,'optical',640,480,1.)
    assert not state.cameras_ready(1.1)
    state.camera('zed','info_depth',11,'optical',640,480,1.)
    assert not state.cameras_ready(1.1)
    state.camera('zed','info_depth',10,'optical',640,480,1.)
    assert state.cameras_ready(1.1)
    assert not state.cameras_ready(2.1)
    assert module().CAMERA_TOPICS['zed'][0]=='/zed/zed_node/rgb/color/rect/image'
    assert module().CAMERA_TOPICS['d405'][0]=='/d405/d405/color/image_raw'


def test_controller_success_expires_and_requires_every_paint_controller():
    state=module().StackReadiness(['zed'],['jtc','admittance'])
    state.controllers([SimpleNamespace(name='jtc',state='active')],1.)
    assert not state.controllers_ready(1.1)
    state.controllers([SimpleNamespace(name=n,state='active') for n in ('jtc','admittance')],1.)
    assert state.controllers_ready(1.5)
    assert not state.controllers_ready(2.1)


def test_foreign_rosbridge_port_is_rejected_without_killing_owner():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0));listener.listen()
        with pytest.raises(RuntimeError,match='already in use'): module().check_rosbridge_port(listener.getsockname()[1])
        assert listener.fileno()>=0


def test_stale_mixed_scene_status_cannot_report_work_ready():
    state=module().StackReadiness(['zed'],['jtc'])
    state.scene({'ready':True,'surface_id':'s','revision':'r','acks':{'isaac':True,'moveit':True}},1.)
    assert state.scene_ready(1.2)
    assert not state.scene_ready(2.1)
    state.scene({'ready':True,'surface_id':'s','revision':'r','acks':{'isaac':True,'moveit':False}},2.)
    assert not state.scene_ready(2.1)


def test_duplicate_camera_messages_do_not_extend_freshness():
    state=module().StackReadiness(['zed'],['jtc'])
    for now in (1.,1.9):
        for kind in ('rgb','depth','info_rgb','info_depth'): state.camera('zed',kind,10,'optical',640,480,now)
    assert not state.cameras_ready(2.1)


def test_shutdown_kills_owned_descendant_after_launch_parent_already_exited(tmp_path):
    import subprocess,sys,os,time
    pidfile=tmp_path/'descendant.pid'
    child="import os,signal,time,pathlib;signal.signal(signal.SIGINT,signal.SIG_IGN);pathlib.Path(%r).write_text(str(os.getpid()));time.sleep(60)"%str(pidfile)
    parent="import subprocess,sys,time;subprocess.Popen([sys.executable,'-c',%r]);time.sleep(.2)"%child
    process=subprocess.Popen([sys.executable,'-c',parent],start_new_session=True)
    try:
        process.wait(timeout=3)
        pid=int(pidfile.read_text());os.kill(pid,0)
        module().shutdown_owned_groups([process],interrupt_s=.1,terminate_s=.1)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            stat=Path('/proc')/str(pid)/'stat'
            if not stat.exists() or stat.read_text().split()[2]=='Z': break
            time.sleep(.02)
        else: pytest.fail('owned descendant survived cleanup')
    finally:
        try: os.killpg(process.pid,9)
        except ProcessLookupError: pass


def test_ground_ack_required_for_every_process_and_revision(tmp_path):
    import json,os
    bundle={'revision':'r','runtime_revision':2}
    m=module()
    assert not m.environment_ready(tmp_path,bundle)
    marker={'asset_revision':'r','runtime_revision':2,'pid':os.getpid(),'applied':True,'mesh_count':0,'ground_applied':True,'collision_object_count':1}
    path=tmp_path/'environment-ready.json';path.write_text(json.dumps(marker))
    assert m.environment_ready(tmp_path,bundle)
    marker['asset_revision']='old';path.write_text(json.dumps(marker))
    assert not m.environment_ready(tmp_path,bundle)


def paint_status():
    monitor=module().PaintStartupReadiness('session','scene')
    hardware={'simulation':True,'session_id':'session','scene_revision':'scene','inhibited':False,'state_fresh':True,'tare_ready':False}
    safety={'ft_valid':True,'tf_valid':True,'abort_latched':False,'nonfinite':False,'saturated':False,
            'filtered_wrench':[0.]*6,'raw_wrench':[0.]*6,'reason':'NONE','bias_ready':False,'mode':'IDLE'}
    monitor.observe('hardware',hardware,10.)
    monitor.observe('safety',safety,10.)
    monitor.observe('guard',guard_status(),10.)
    return monitor,hardware,safety


def test_paint_startup_accepts_healthy_untared_idle_without_target_or_plan():
    monitor,_,_=paint_status()
    assert monitor.blockers(10.1)==[]
    assert monitor.fatal_error() is None


@pytest.mark.parametrize('field,value', [('inhibited',True),('state_fresh',False),('simulation',False),('session_id','old'),('scene_revision','old')])
def test_paint_hardware_bad_status_blocks_startup_even_when_controllers_are_active(field,value):
    monitor,hardware,_=paint_status()
    monitor.observe('hardware',{**hardware,field:value},10.05)
    assert monitor.blockers(10.1)


def test_paint_hardware_inhibit_is_a_terminal_startup_failure():
    monitor,hardware,_=paint_status()
    monitor.observe('hardware',{**hardware,'inhibited':True},10.05)
    assert 'restart' in monitor.fatal_error().lower()
    monitor.observe('hardware',hardware,10.1)
    assert monitor.blockers(10.15), 'Later false sample must not clear a hardware inhibit latch'
    assert monitor.fatal_error()


@pytest.mark.parametrize('field,value', [('ft_valid',False),('tf_valid',False),('nonfinite',True),('saturated',True),('filtered_wrench',[float('nan')]*6),('raw_wrench',[float('inf')]*6)])
def test_paint_invalid_safety_never_reports_startup_ready(field,value):
    monitor,_,safety=paint_status()
    monitor.observe('safety',{**safety,field:value,'reason':'FT_NONFINITE'},10.05)
    assert monitor.blockers(10.1)


def test_paint_startup_status_expiry_uses_existing_leases_and_fault_is_not_ignored():
    monitor,_,_=paint_status()
    assert any('hardware' in reason.lower() for reason in monitor.blockers(10.201))
    assert any('safety' in reason.lower() for reason in monitor.blockers(10.501))
    monitor.observe('guard',guard_status(controller_fault=True,published_monotonic_s=10.05),10.05)
    assert any('controller' in reason.lower() for reason in monitor.work_blockers(10.1))


def test_paint_missing_or_malformed_status_is_not_healthy():
    monitor=module().PaintStartupReadiness('session','scene')
    assert len(monitor.blockers(10.))==3
    for kind in ('hardware','safety','guard'):monitor.observe(kind,{},10.)
    assert monitor.blockers(10.1)


def test_managed_ready_marker_cannot_promote_inhibited_paint_to_ready():
    monitor,hardware,_=paint_status()
    monitor.observe('hardware',{**hardware,'inhibited':True},10.05)
    marker={'ready':True,'controllers_ready':True,'cameras_ready':True,'scene_monitor_ready':True}
    blockers=monitor.gate_marker(marker,10.1)
    assert marker['ready'] is False
    assert marker['paint_startup_ready'] is False
    assert marker['paint_startup_blockers']==blockers
    assert any('inhibited' in item for item in blockers)


def test_paint_health_does_not_override_an_unready_camera_or_scene_marker():
    monitor,_,_=paint_status()
    marker={'ready':False}
    assert monitor.gate_marker(marker,10.1)==[]
    assert marker['ready'] is False
    assert marker['paint_startup_ready'] is True


@pytest.mark.parametrize('fault', ['safety_latch','controller_fault'])
def test_recoverable_latch_allows_measurement_startup_but_not_work(fault):
    monitor,_,safety=paint_status()
    if fault=='safety_latch':
        safety={**safety,'abort_latched':True,'reason':'FT_STALE'}
        monitor.observe('safety',safety,10.05)
    else:monitor.observe('guard',guard_status(controller_fault=True,published_monotonic_s=10.05),10.05)
    marker={'ready':True,'work_ready':True}
    assert monitor.gate_marker(marker,10.1)==[]
    assert marker['ready'] is True
    assert marker['work_ready'] is False
    assert marker['paint_work_blockers']
    if fault=='safety_latch':
        assert safety['abort_latched'] is True, 'Readiness must not reset existing safety latches'
        assert 'FT_STALE' in ' '.join(marker['paint_work_blockers'])
    assert monitor.fatal_error() is None


def test_missing_or_current_invalid_force_data_cannot_be_misclassified_as_historical_latch():
    monitor,_,safety=paint_status()
    monitor.observe('safety',{**safety,'abort_latched':True,'reason':'FT_STALE','ft_valid':False},10.05)
    marker={'ready':True,'work_ready':True}
    monitor.gate_marker(marker,10.1)
    assert marker['ready'] is False
    assert marker['work_ready'] is False


def guard_status(**changes):
    return {'published_monotonic_s':10.,'status_sequence':1,'controller_fault':False,'blockers':[],**changes}


@pytest.mark.parametrize('changes', [
    {'controller_fault':None}, {'controller_fault':'false'},
    {'published_monotonic_s':float('nan')}, {'published_monotonic_s':9.},
    {'published_monotonic_s':11.}, {'status_sequence':True}, {'status_sequence':0},
    {'blockers':['CONTROLLER_STATUS_STALE']}, {'blockers':None},
])
def test_paint_guard_requires_typed_fresh_controller_status(changes):
    monitor,_,_=paint_status()
    monitor.guard_source_time=0.
    monitor.observe('guard',guard_status(**changes),10.05)
    assert any('guard' in item.lower() for item in monitor.blockers(10.1))


def test_paint_guard_old_samples_cannot_clear_new_fault():
    monitor,_,_=paint_status()
    monitor.observe('guard',guard_status(controller_fault=True,published_monotonic_s=10.05,status_sequence=2),10.05)
    monitor.observe('guard',guard_status(),10.06)
    assert monitor.blockers(10.1)==[]
    assert any('controller fault' in item.lower() for item in monitor.work_blockers(10.1))


def test_paint_stack_subscribes_actual_guard_json_not_unpublished_legacy_fault():
    source=(Path(__file__).resolve().parents[2]/'scripts/platform_stack.py').read_text()
    assert "namespace+'/painting_admittance/wrench_guard_status'" in source
    assert "namespace+'/painting_admittance/controller_fault'" not in source


def test_terminal_inhibit_keeps_only_bounded_unready_observation_after_ready():
    window=module().PostInhibitObservation(simulation=True)
    assert not window.update({'ready':True,'work_ready':True},None,10.)
    marker={'ready':True,'work_ready':True}
    assert not window.update(marker,'hardware inhibited',11.)
    assert marker['ready'] is False and marker['work_ready'] is False
    assert marker['terminal_error']=='hardware inhibited'
    assert marker['stop_observation_deadline_monotonic']==16.
    assert marker['stop_observation_only'] is True
    # A later healthy-looking marker or missing failure cannot reopen readiness
    # or extend the first terminal edge's five-second observation bound.
    for now in (11.1,15.999):
        marker={'ready':True,'work_ready':True}
        assert not window.update(marker,None,now)
        assert marker['ready'] is False and marker['work_ready'] is False
        assert marker['stop_observation_deadline_monotonic']==16.
    assert window.update({'ready':True},None,16.)


@pytest.mark.parametrize('simulation,ready',[(True,False),(False,True)])
def test_startup_inhibit_and_non_simulation_keep_immediate_teardown(simulation,ready):
    window=module().PostInhibitObservation(simulation=simulation)
    window.update({'ready':ready},None,10.)
    marker={'ready':True,'work_ready':True}
    assert window.update(marker,'hardware inhibited',11.)
    assert marker['ready'] is False and marker['work_ready'] is False
    assert marker['stop_observation_only'] is False


def test_camera_interleaved_complete_exposures_are_not_lost():
    state=module().StackReadiness(['zed'],[])
    state.camera('zed','rgb',10,'optical',640,480,1.)
    state.camera('zed','rgb',11,'optical',640,480,1.01)
    for kind in ('depth','info_rgb','info_depth'):
        state.camera('zed',kind,10,'optical',640,480,1.02)
    assert state.cameras_ready(1.03)
    assert state.complete['zed']==((10,'optical',640,480),1.02)
    for kind in ('info_depth','depth','info_rgb'):
        state.camera('zed',kind,11,'optical',640,480,1.04)
    assert state.complete['zed']==((11,'optical',640,480),1.04)


@pytest.mark.parametrize('frame,width',[('other_optical',640),('optical',320)])
def test_camera_tuple_does_not_mix_frame_or_dimensions(frame,width):
    state=module().StackReadiness(['zed'],[])
    for kind in ('rgb','depth','info_rgb'):
        state.camera('zed',kind,10,'optical',640,480,1.)
    state.camera('zed','info_depth',10,frame,width,480,1.1)
    assert not state.cameras_ready(1.1)
    state.camera('zed','info_depth',10,'optical',640,480,1.2)
    assert state.cameras_ready(1.2)


def test_camera_older_completion_and_clock_reset_never_renew_lease():
    state=module().StackReadiness(['zed'],[])
    for stamp,now in ((100,1.),(99,1.9),(1,2.1),(100,2.2)):
        for kind in ('rgb','depth','info_rgb','info_depth'):
            state.camera('zed',kind,stamp,'optical',640,480,now)
    assert state.complete['zed']==((100,'optical',640,480),1.)
    assert not state.cameras_ready(2.2)


def test_camera_pending_tuple_storage_is_bounded_and_expired_parts_not_reused():
    state=module().StackReadiness(['zed'],[])
    for stamp in range(1,101):state.camera('zed','rgb',stamp,'optical',640,480,1.)
    assert len(state.parts['zed'])==16
    # An evicted first part cannot complete the oldest exposure.
    for kind in ('depth','info_rgb','info_depth'):
        state.camera('zed',kind,1,'optical',640,480,1.1)
    assert not state.cameras_ready(1.1)
    # Stored parts also expire, even if the tuple count stays below the bound.
    for kind in ('depth','info_rgb','info_depth'):
        state.camera('zed',kind,100,'optical',640,480,2.01)
    assert not state.cameras_ready(2.01)
    for kind in ('rgb','depth','info_rgb','info_depth'):
        state.camera('zed',kind,101,'optical',640,480,2.02)
    assert state.cameras_ready(2.02)


def test_closed_rosbridge_time_wait_is_not_a_live_port_owner():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        listener.bind(('127.0.0.1',0));listener.listen();port=listener.getsockname()[1]
        with socket.create_connection(('127.0.0.1',port)) as client:
            connection,_=listener.accept()
            connection.close()
            assert client.recv(1)==b''
    module().check_rosbridge_port(port)
