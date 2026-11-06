"""Acceptance must use complete, fresh measured execution evidence."""
import importlib.util
from pathlib import Path
import sys
import pytest

SCRIPTS=Path(__file__).resolve().parents[2]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from rb20_painting_report import assess_painting, assess_stop, assess_plane
from check_rb20_painting_isaac import finalize_report, status_is_current, work_area_measurement_done


def evidence():
    phases=['APPROACH_PRECONTACT','PRECONTACT_TARE_COMPLETE','CONTACT_SEARCH','RAMP_UP','PAINT','RAMP_DOWN','RETRACT',
            'APPROACH_PRECONTACT','PRECONTACT_TARE_COMPLETE','CONTACT_SEARCH','RAMP_UP','PAINT','RAMP_DOWN','FINAL_RETRACT','COMPLETE']
    events=[{'state':phase,'time':float(i*10),'timestamp_ns':int(i*10*1e9),'plan_hash':'plan-one',
             'work_area_id':'area-one','plane_generation_id':'plane-one'} for i,phase in enumerate(phases)]
    samples=[]
    for start in (e['time'] for e in events if e['state']=='PAINT'):
        for i in range(651):
            t=start+i*.01
            samples.append({'time':t,'sim_time':t,'step':len(samples)+1,'session':'one',
                'measured_contact_force_n':20.,'state_age_s':.01,'force_age_s':.01,'valid':True})
    return events,samples


def test_two_strokes_need_complete_phases_and_measured_force():
    events,samples=evidence()
    assert assess_painting(events,samples)['passed']
    assert not assess_painting(events[:-1],samples)['passed']
    assert not assess_painting(events,[])['passed']
    events=[e for e in events if e['state']!='RAMP_DOWN']
    assert not assess_painting(events,samples)['passed']


@pytest.mark.parametrize('field,value',[('measured_contact_force_n',40.),('state_age_s',.3),('force_age_s',.3),('valid',False),('session','other')])
def test_bad_feedback_cannot_pass(field,value):
    events,samples=evidence()
    for sample in samples[200:]:sample[field]=value
    assert not assess_painting(events,samples)['passed']


def test_stop_needs_ack_and_measured_stop_after_detection():
    assert assess_stop(1.,1.19,1.20,1.21,1.24)['passed']
    assert not assess_stop(1.,1.19,1.20,None,1.24)['passed']
    assert not assess_stop(1.,1.19,1.20,1.21,1.5)['passed']


def test_measured_plane_requires_distance_and_directed_normal_accuracy():
    assert assess_plane([1.451,.2,1.],[-1,0,0],1.45)['passed']
    assert not assess_plane([1.456,.2,1.],[-1,0,0],1.45)['passed']
    assert not assess_plane([1.45,.2,1.],[1,0,0],1.45)['passed']
    assert not assess_plane([1.45,.2,1.],[-1,.1,0],1.45)['passed']
    assert not assess_plane([float('nan'),.2,1.],[-1,0,0],1.45)['passed']
    assert not assess_plane([1.45,.2,1.],[0,0,0],1.45)['passed']


def test_execution_cannot_pass_on_healthy_graph_without_painting():
    result=finalize_report({'passed':True},[],[],execute=True)
    assert not result['passed']
    assert result['scope']=='painting_execution'
    assert 'final COMPLETE missing' in result['painting']['errors']


def test_every_stroke_needs_its_own_tare_before_contact():
    events,samples=evidence()
    late_tare=next(e for e in events if e['state']=='PRECONTACT_TARE_COMPLETE')
    events=[e for e in events if e['state']!='PRECONTACT_TARE_COMPLETE']
    events.insert(-1,{**late_tare,'time':events[-1]['time']-.1})
    assert not assess_painting(events,samples)['passed']


def test_tare_from_another_execution_cannot_validate_contact():
    events,samples=evidence()
    next(e for e in events if e['state']=='PRECONTACT_TARE_COMPLETE')['plan_hash']='old-plan'
    assert not assess_painting(events,samples)['passed']


def test_workflow_failure_cannot_be_overwritten_by_passing_measurements():
    events,samples=evidence()
    result=finalize_report({'passed':True},events,samples,execute=True,workflow_error='selection failed')
    assert result['painting']['passed']
    assert not result['passed']


def test_old_or_foreign_healthy_status_cannot_pass():
    from types import SimpleNamespace
    state=SimpleNamespace(session_id='current',scene_revision='revision')
    status={'session_id':'current','scene_revision':'revision','ok':True}
    assert status_is_current(status,9.9,10.,state)
    assert not status_is_current(status,9.7,10.,state)
    assert not status_is_current(status,10.1,10.,state)
    assert not status_is_current({**status,'session_id':'previous'},9.9,10.,state)
    assert not status_is_current({**status,'scene_revision':'old'},9.9,10.,state)


def test_work_area_waits_for_executor_completion_after_sensor_acceptance():
    refinement={'accepted':True,'mode':'work_area','work_area_id':'new-area',
                'plane_generation_id':'new-plane'}
    latest={'refinement':refinement,'refine_execution':{'state':'moving'}}
    received={'refinement':11.,'refine_execution':10.5}
    assert not work_area_measurement_done(latest,received,10.,'new-area','old-plane')
    latest['refine_execution']={'state':'done'}
    received['refine_execution']=9.
    assert not work_area_measurement_done(latest,received,10.,'new-area','old-plane')
    received['refine_execution']=11.1
    assert work_area_measurement_done(latest,received,10.,'new-area','old-plane')
    assert not work_area_measurement_done(latest,received,10.,'other-area','old-plane')
    assert not work_area_measurement_done(latest,received,10.,'new-area','new-plane')
    latest['refine_execution']={'state':'failed'}
    with pytest.raises(RuntimeError,match='work-area measurement failed'):
        work_area_measurement_done(latest,received,10.,'new-area','old-plane')


def test_slow_simulation_wall_duration_cannot_substitute_for_five_second_force_hold():
    events,samples=evidence()
    for event in events:event['timestamp_ns']=int(event['time']*.4*1e9)
    for sample in samples:sample['sim_time']*=.4
    result=assess_painting(events,samples)
    assert not result['passed']
    assert all(stroke['simulation_duration_s']<5. for stroke in result['strokes'])
    assert all(stroke['wall_duration_s']>stroke['simulation_duration_s'] for stroke in result['strokes'])


def test_simulation_hold_and_first_second_exclusion_use_source_clock():
    events,samples=evidence()
    for event in events:
        event['timestamp_ns']=int(event['time']*1e9)
        event['time']*=2.5
    for sample in samples:sample['time']*=2.5
    result=assess_painting(events,samples)
    assert result['passed']
    assert result['clock_domain']=='simulation'
    for stroke in result['strokes']:
        assert stroke['simulation_duration_s']==pytest.approx(5.5)
        assert stroke['wall_duration_s']==pytest.approx(13.75)


def test_real_clock_force_hold_uses_steady_progress_without_simulation_clock():
    events,samples=evidence()
    for sample in samples:sample['sim_time']=0.
    result=assess_painting(events,samples,clock_domain='steady')
    assert result['passed'] and result['clock_domain']=='steady'
    assert all(stroke['duration_s']==pytest.approx(5.5) for stroke in result['strokes'])


def test_simulation_report_refuses_missing_source_event_timestamp():
    events,samples=evidence()
    for event in events:event.pop('timestamp_ns',None)
    assert not assess_painting(events,samples)['passed']


def test_slow_simulation_report_keeps_wall_freshness_gap_limit():
    events,samples=evidence()
    for event in events:event['timestamp_ns']=int(event['time']*1e9)
    for sample in samples[201:]:sample['time']+=.3
    assert not assess_painting(events,samples)['passed']
