"""Force-phase abort evidence is distinct from complete two-stroke acceptance."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import check_platform_workflow as m


def data():
    identity=dict(plan_hash='p',work_area_id='a',plane_generation_id='g')
    latest=dict(execution=dict(state='PAINT',**identity),
        safety=dict(ft_valid=True,tf_valid=True,abort_latched=False,contact_force_n=20.),
        guard=dict(mode='PAINT',forwarding=True,force_enable=True,compliance_active=True,requested_force_norm_n=20.))
    received=dict(execution=10.,safety=10.,guard=10.)
    return latest,received,identity


def test_cancel_trigger_requires_current_measured_force_phase():
    latest,received,identity=data()
    assert m.force_cancel_trigger(latest,received,identity,10.01)
    for key,value in [('contact_force_n',0.),('ft_valid',False),('abort_latched',True)]:
        old=latest['safety'][key];latest['safety'][key]=value
        assert not m.force_cancel_trigger(latest,received,identity,10.01)
        latest['safety'][key]=old
    assert not m.force_cancel_trigger(latest,received,identity,10.3)
    latest['execution']['plan_hash']='old'
    assert not m.force_cancel_trigger(latest,received,identity,10.01)


def stopped_evidence():
    latest,received,identity=data()
    latest.update(execution=dict(state='ABORT',**identity),
                  _cancel_terminal_event=dict(state='ABORT',time=10.05,**identity),
                  guard=dict(mode='ABORT',forwarding=False,force_enable=False,compliance_active=False,requested_force_norm_n=0.),
                  hardware=dict(session_id='s',scene_revision='r',inhibited=True),
                  command=NS(enabled=False,session_id='s',scene_revision='r'),
                  forwarded_wrench=NS(wrench=NS(force=NS(x=0.,y=0.,z=0.),torque=NS(x=0.,y=0.,z=0.))))
    received.update(execution=10.05,guard=10.07,hardware=10.04,command=10.06,forwarded_wrench=10.06)
    samples=[dict(time=10.1+i*.01,sim_time=5+i*.01,positions=[0.]*6,velocities=[0.]*6,
                  valid=True,simulator_state_valid=True,session='s',scene_revision='r',source='isaac_measured_reaction') for i in range(61)]
    return latest,received,samples


def test_forceoff_ack_does_not_substitute_for_measured_stop():
    latest,received,samples=stopped_evidence()
    received.update(guard=10.7,hardware=10.7,command=10.7,forwarded_wrench=10.7)
    result=m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')
    assert result['force_off'] and result['hardware_inhibit_acknowledged'] and result['measured_stop']['passed']
    assert result['retreat']=='not_performed_hard_stop_requires_relaunch'
    for sample in samples:sample['velocities'][0]=.02
    assert not m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')['passed']


@pytest.mark.parametrize('fault',['stale_guard','old_ack','nonzero','wrong_session','source','missing_velocity'])
def test_force_cancel_rejects_stale_foreign_or_incomplete_evidence(fault):
    latest,received,samples=stopped_evidence()
    # Fresh current state of all status channels, while ACK edge was observed earlier.
    received.update(guard=10.7,hardware=10.7,command=10.7,forwarded_wrench=10.7)
    if fault=='stale_guard':received['guard']=10.
    elif fault=='old_ack':received['hardware']=9.
    elif fault=='nonzero':latest['guard']['requested_force_norm_n']=2.
    elif fault=='wrong_session':latest['hardware']['session_id']='other'
    elif fault=='source':samples[-1]['source']='command_echo'
    else:samples[-1]['velocities']=[]
    result=m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')
    assert not result['passed']


def test_guard_zero_claim_requires_actual_forwarded_zero_and_current_command_identity():
    latest,received,samples=stopped_evidence()
    received.update(guard=10.7,hardware=10.7,command=10.7,forwarded_wrench=10.7)
    latest['forwarded_wrench'].wrench.force.y=1.
    assert not m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')['force_off']
    latest['forwarded_wrench'].wrench.force.y=0.;latest['command'].session_id='old'
    assert not m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')['force_off']


def test_cancel_path_requires_execute_even_before_observer_setup(tmp_path):
    with pytest.raises(ValueError,match='--execute'):
        m.run(NS(assets=tmp_path,cancel_during_paint=True,execute=False),{})


@pytest.mark.parametrize('diagnostic_last',[False,True])
def test_cancel_workflow_sends_one_public_abort_and_observes_stop(tmp_path,monkeypatch,diagnostic_last):
    import json
    clock={'now':10.};monkeypatch.setattr(m.time,'monotonic',lambda:clock['now'])
    latest,received,identity=data();after,_,_=stopped_evidence();after.pop('_cancel_terminal_event');sent=[]
    bundle=dict(revision='r',runtime_revision=1,selection=dict(execution_mode='simulation',process='paint'))
    (tmp_path/'bundle.json').write_text(json.dumps(bundle))
    obs=NS(execute_authorized=True,latest=latest,received=received,events=[],samples=[],
           assets=tmp_path,bundle=bundle,session='s',active=dict(joints=[str(i) for i in range(6)]))
    prepared=[]
    def prepare_abort():
        prepared.append(True)
        clock['now']+=.08  # Cold DDS discovery belongs before the loaded trigger.
        obs.received.update({key:clock['now'] for key in received})
    def wait(predicate,timeout,label):
        assert prepared, 'discover the stop channel before awaiting loaded PAINT'
        assert predicate()
    def publish(topic,message):sent.append((topic,message.data))
    def spin(seconds):
        clock['now']+=.02;now=clock['now']
        obs.latest.update(after)
        if not obs.events:
            obs.events.append(dict(state='ABORT',time=now,**identity))
        if diagnostic_last:obs.latest['execution']=dict(state='HARDWARE_MOTION_INHIBITED',**identity)
        obs.received.update({key:now for key in ('execution','guard','hardware','command','forwarded_wrench')})
        obs.samples.append(dict(time=now,sim_time=now*.4,positions=[0.]*6,velocities=[0.]*6,
            valid=True,simulator_state_valid=True,session='s',scene_revision='r',source='isaac_measured_reaction'))
        (tmp_path/'simulation-ready.json').write_text(json.dumps(dict(session_id='s',asset_revision='r',
            runtime_revision=1,updated_monotonic=now,joints_valid=True,wrench_valid=True,ready=False)))
    obs.wait=wait;obs.publish=publish;obs.spin=spin;obs.prepare_abort=prepare_abort
    m.run_force_cancel(obs,identity,0,30.)
    assert sent==[('/motion_abort',True)]
    assert obs.cancel_report['passed'] and obs.cancel_report['force_off_latency_s']<.1
    assert obs.cancel_report['requested_monotonic']==pytest.approx(10.08)
    assert obs.cancel_report['abort_event']['time']==pytest.approx(10.10)
    assert not obs.cancel_report['normal_workflow_complete']
    assert obs.cancel_report['retreat']=='not_performed_hard_stop_requires_relaunch'


def test_cancel_uses_direct_measured_state_after_monitor_invalidation():
    latest,received,samples=stopped_evidence()
    received.update(guard=10.7,hardware=10.7,command=10.7,forwarded_wrench=10.7)
    for sample in samples:
        sample['valid']=False  # Inhibited hardware deliberately invalidates controller FT.
        sample['simulator_state_valid']=True  # Direct physics observation remains live.
    result=m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')
    assert result['passed']
    samples[-1]['simulator_state_valid']=False
    assert not m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')['passed']


def test_direct_state_validity_does_not_reuse_monitor_readiness():
    zero=NS(x=0.,y=0.,z=0.);wrench=NS(force=zero,torque=zero)
    msg=NS(header=NS(stamp=NS(sec=1,nanosec=0)),step_index=5,session_id='s',scene_revision='r',
        positions=[0.]*6,velocities=[0.]*6,wrench=wrench,gravity_wrench=wrench,
        joints_valid=True,wrench_valid=True,playing=True,joint_names=[str(i) for i in range(6)])
    node=object.__new__(m.PlatformObserver)
    node.__dict__.update(session='s',bundle={'revision':'r'},active={'joints':msg.joint_names},samples=[],
        latest={'safety':{'ft_valid':False}},received={},counts={})
    node.store=lambda key,value:m.Observer.store(node,key,value)
    m.PlatformObserver.state_message(node,msg)
    assert node.samples[-1]['simulator_state_valid'] is True
    assert node.samples[-1]['valid'] is False
    for field,value in [('session_id','old'),('scene_revision','old'),('wrench_valid',False),
                        ('positions',[float('nan')]*6),('playing',False)]:
        previous=getattr(msg,field);setattr(msg,field,value)
        m.PlatformObserver.state_message(node,msg)
        assert node.samples[-1]['simulator_state_valid'] is False
        setattr(msg,field,previous)


def test_cancel_abort_edge_ignores_pre_request_and_foreign_execution():
    _,_,identity=data()
    events=[dict(state='ABORT',time=9.,**identity),
            dict(state='ABORT',time=10.01,**{**identity,'plan_hash':'old'}),
            dict(state='HARDWARE_MOTION_INHIBITED',time=10.02,**identity)]
    assert m.cancel_terminal_event(events,identity,10.,10.1) is None
    accepted=dict(state='ABORT',time=10.03,**identity)
    events.extend([accepted,dict(state='HARDWARE_MOTION_INHIBITED',time=10.04,**identity)])
    assert m.cancel_terminal_event(events,identity,10.,10.1)==accepted


def test_latest_abort_status_alone_cannot_replace_reliable_post_request_edge():
    latest,received,samples=stopped_evidence()
    received.update(guard=10.7,hardware=10.7,command=10.7,forwarded_wrench=10.7)
    for event in (None,dict(state='ABORT',time=9.),dict(state='ABORT',time=11.)):
        latest['_cancel_terminal_event']=event
        result=m.force_cancel_evidence(latest,received,samples,[str(i) for i in range(6)],10.,10.7,'s','r')
        assert not result['passed'] and not result['abort_acknowledged']
