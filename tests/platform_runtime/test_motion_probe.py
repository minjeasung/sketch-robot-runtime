import importlib.util
from pathlib import Path
import json,os
import pytest


def module():
    path=Path(__file__).resolve().parents[2]/'scripts/check_platform_motion.py'
    spec=importlib.util.spec_from_file_location('motion_probe',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def context(tmp_path):
    bundle={'revision':'rev','runtime_revision':3,'selection':{'execution_mode':'simulation','active_arm':'single','process':'spray'},'arms':{'single':{'joints':['j0','j1'],'group':'arm','controller':'jtc'}},'joint_names':['j0','j1','head'],'initial_positions':{'j0':0.,'j1':0.,'head':0.}}
    (tmp_path/'bundle.json').write_text(json.dumps(bundle))
    (tmp_path/'robot.urdf').write_text('<robot name="r"><ros2_control name="sim"><hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin></hardware></ros2_control></robot>')
    ready={'ready':True,'pid':os.getpid(),'asset_revision':'rev','runtime_revision':3,'session_id':'s','scene_revision':'rev','updated_monotonic':10.,'joints_valid':True,'cameras_ready':True,'wrench_valid':True,'controllers_ready':True,'scene_monitor_ready':True}
    for name in ('simulation-ready.json','managed-ready.json'):(tmp_path/name).write_text(json.dumps(ready))
    return bundle,ready


def test_probe_requires_live_matching_simulator_only_context(tmp_path):
    context(tmp_path);m=module()
    assert m.validate_probe_context(tmp_path,now=10.5)['session_id']=='s'
    with pytest.raises(ValueError,match='expired'):m.validate_probe_context(tmp_path,now=13.)
    p=tmp_path/'robot.urdf';p.write_text(p.read_text().replace('topic_based_ros2_control/TopicBasedSystem','rbpodo/Hardware'))
    with pytest.raises(ValueError,match='hardware'):m.validate_probe_context(tmp_path,now=10.5)


def test_probe_rejects_mixed_or_real_even_with_ready_marker(tmp_path):
    bundle,_=context(tmp_path);bundle['selection']['execution_mode']='mixed';(tmp_path/'bundle.json').write_text(json.dumps(bundle))
    with pytest.raises(ValueError,match='simulation only'):module().validate_probe_context(tmp_path,now=10.5)


def test_tracking_distinguishes_selected_error_from_inactive_motion():
    report=module().tracking_summary({'j0':.02,'j1':0.,'head':1.},{'j0':.019,'j1':.002,'head':1.03},['j0','j1'])
    assert report['selected_max_error_rad']==pytest.approx(.002)
    assert report['inactive_max_drift_rad']==pytest.approx(.03)
    assert report['passed'] is False


def test_cancelled_action_is_insufficient_without_stationary_measurements():
    m=module()
    samples=[{'time':i*.1,'stamp_ns':1_000_000_000+i*100_000_000,'positions':{'j0':i*.002}} for i in range(7)]
    assert not m.stop_summary(samples,['j0'])['passed']
    samples=[{'time':i*.1,'stamp_ns':1_000_000_000+i*100_000_000,'positions':{'j0':.02}} for i in range(7)]
    assert m.stop_summary(samples,['j0'])['passed']
    assert not m.stop_summary(samples[:2],['j0'])['passed']


def test_probe_samples_clock_after_reading_markers(tmp_path,monkeypatch):
    context(tmp_path);m=module();calls=[]
    original=Path.read_text
    def read(path,*args,**kwargs):
        calls.append(path.name);return original(path,*args,**kwargs)
    def clock():
        assert 'managed-ready.json' in calls
        return 10.5
    monkeypatch.setattr(Path,'read_text',read);monkeypatch.setattr(m.time,'monotonic',clock)
    assert m.validate_probe_context(tmp_path)['session_id']=='s'


def test_stop_velocity_uses_measurement_time_despite_bursty_dds_arrival():
    receipts=[10.,10.2,10.20001,10.4,10.6]
    samples=[{'time':received,'stamp_ns':1_000_000_000+i*100_000_000,
              'positions':{'j0':i*.0002},'velocities':{'j0':.002}}
             for i,received in enumerate(receipts)]
    summary=module().stop_summary(samples,['j0'])
    assert summary['passed']
    assert summary['max_velocity_rad_s']==pytest.approx(.002)
    assert summary['position_derived_max_velocity_rad_s']==pytest.approx(.002)
    assert summary['measured_max_velocity_rad_s']==pytest.approx(.002)
    assert summary['duration_s']==pytest.approx(.6)
    assert summary['simulation_duration_s']==pytest.approx(.4)


def test_small_position_span_cannot_hide_real_fast_physics_motion():
    samples=[{'time':10.+i*.2,'stamp_ns':1_000_000_000+i*1_000_000,
              'positions':{'j0':i*.0002},'velocities':{'j0':.2}} for i in range(4)]
    summary=module().stop_summary(samples,['j0'])
    assert not summary['passed']
    assert summary['max_velocity_rad_s']==pytest.approx(.2)
    assert summary['position_span_rad']<.001


def test_measured_velocity_crosschecks_position_derivative():
    samples=[{'time':10.+i*.2,'stamp_ns':1_000_000_000+i*100_000_000,
              'positions':{'j0':.02},'velocities':{'j0':.02}} for i in range(4)]
    summary=module().stop_summary(samples,['j0'])
    assert not summary['passed']
    assert summary['position_derived_max_velocity_rad_s']==0.
    assert summary['measured_max_velocity_rad_s']==.02


@pytest.mark.parametrize('stamp_change',[None,0,-1])
def test_stop_measurements_require_present_strictly_increasing_source_time(stamp_change):
    samples=[{'time':10.+i*.2,'stamp_ns':1_000_000_000+i*100_000_000,
              'positions':{'j0':.02}} for i in range(4)]
    if stamp_change is None:samples[1].pop('stamp_ns')
    else:samples[1]['stamp_ns']=samples[0]['stamp_ns']+stamp_change
    assert not module().stop_summary(samples,['j0'])['passed']


def cancel_samples(start=10., count=151, settling_until=10.5):
    # Realistic continued deceleration after terminal acknowledgment, then hold.
    return [{'time':start+i*.01,'stamp_ns':1_000_000_000+i*10_000_000,
             'positions':{'j0':min(start+i*.01,settling_until)*.008},
             'velocities':{'j0':.008 if start+i*.01<settling_until else 0.}}
            for i in range(count)]


def test_cancel_observation_waits_for_measured_settle_instead_of_one_early_window():
    m=module();samples=cancel_samples()
    early=m.cancel_stop_summary(samples[:71],['j0'],requested=9.9,terminal=10.,now=10.7)
    assert not early['passed']
    assert early['position_span_rad']>.001
    assert early['observed_stop_latency_s'] is None
    settled=m.cancel_stop_summary(samples[:121],['j0'],requested=9.9,terminal=10.,now=11.2)
    assert settled['passed']
    assert settled['observed_stop_latency_s']==pytest.approx(1.3)
    assert settled['confirmation_after_terminal_s']==pytest.approx(1.2)
    assert settled['duration_s']>=.5


def test_cancel_observation_never_accepts_persistent_drift_or_stale_samples():
    m=module();samples=cancel_samples(count=501,settling_until=20.)
    assert not m.cancel_stop_summary(samples,['j0'],9.9,10.,15.)['passed']
    held=cancel_samples(settling_until=9.)
    assert not m.cancel_stop_summary(held,['j0'],9.9,10.,15.)['passed']


def test_cancel_observation_excludes_preterminal_stationary_window():
    m=module();samples=cancel_samples(settling_until=9.)
    assert not m.cancel_stop_summary(samples,['j0'],9.9,11.4,11.5)['passed']


def test_cancel_observation_does_not_weaken_velocity_or_span_limits():
    m=module();samples=cancel_samples(settling_until=9.)
    for sample in samples:sample['velocities']['j0']=.010001
    assert not m.cancel_stop_summary(samples,['j0'],9.9,10.,11.5)['passed']
    for i,sample in enumerate(samples):
        sample['velocities']['j0']=.002
        sample['positions']['j0']=i*.00002
    assert not m.cancel_stop_summary(samples,['j0'],9.9,10.,11.5)['passed']


def test_late_stop_confirmation_cannot_cross_existing_five_second_bound():
    samples=cancel_samples(start=14.5,count=61,settling_until=9.)
    result=module().cancel_stop_summary(samples,['j0'],9.9,10.,15.1)
    assert not result['passed']
    assert result['observed_stop_latency_s'] is None


def test_rejected_readiness_retains_exact_marker_and_distinguishes_not_ready(tmp_path):
    _,ready=context(tmp_path);ready.update(ready=False,paint_startup_blockers=['camera tuple missing'])
    path=tmp_path/'managed-ready.json';path.write_text(json.dumps(ready));observed={}
    with pytest.raises(ValueError,match='runtime readiness false: managed-ready.json'):
        module().validate_probe_context(tmp_path,now=10.5,observed=observed)
    path.write_text(json.dumps({**ready,'ready':True}))
    assert observed['markers']['managed-ready.json']['ready'] is False
    assert observed['markers']['managed-ready.json']['paint_startup_blockers']==['camera tuple missing']
    assert observed['checked_monotonic']==10.5


def test_main_motion_probe_keeps_rejected_marker_before_ros_start(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import rclpy
    _,ready=context(tmp_path);ready['ready']=False
    (tmp_path/'managed-ready.json').write_text(json.dumps(ready))
    m=module();monkeypatch.setattr(m.time,'monotonic',lambda:10.5)
    monkeypatch.setattr(rclpy,'init',lambda *a,**kw:pytest.fail('No ROS startup on rejected context'))
    report={}
    with pytest.raises(ValueError,match='runtime readiness false'):
        m.run(SimpleNamespace(assets=tmp_path),report)
    assert report['readiness_observation']['markers']['managed-ready.json']['ready'] is False


def test_repeated_identical_hardware_validation_skips_xml_parse_but_rereads_files(tmp_path, monkeypatch):
    context(tmp_path);m=module();parses=[];reads=[]
    original_parse=m.ET.fromstring;original_read=Path.read_text
    def parse(description):
        parses.append(description);return original_parse(description)
    def read(path,*args,**kwargs):
        reads.append(path.name);return original_read(path,*args,**kwargs)
    monkeypatch.setattr(m.ET,'fromstring',parse)
    monkeypatch.setattr(Path,'read_text',read)
    for _ in range(2):assert m.validate_probe_context(tmp_path,now=10.5)['session_id']=='s'
    assert len(parses)==1
    for name in ('robot.urdf','bundle.json','simulation-ready.json','managed-ready.json'):
        assert reads.count(name)==2


def test_cached_hardware_validation_revalidates_every_changed_description(tmp_path, monkeypatch):
    context(tmp_path);m=module();parses=[];original=m.ET.fromstring
    def parse(description):
        parses.append(description);return original(description)
    monkeypatch.setattr(m.ET,'fromstring',parse)
    path=tmp_path/'robot.urdf';first=path.read_text()
    assert m.validate_probe_context(tmp_path,now=10.5)['session_id']=='s'
    path.write_text(first.replace('name="r"','name="changed"'))
    assert m.validate_probe_context(tmp_path,now=10.5)['session_id']=='s'
    path.write_text(first.replace('topic_based_ros2_control/TopicBasedSystem','rbpodo/Hardware'))
    with pytest.raises(ValueError,match='hardware'):m.validate_probe_context(tmp_path,now=10.5)
    assert len(parses)==3


def test_hardware_validation_cache_retains_only_four_exact_descriptions(tmp_path, monkeypatch):
    context(tmp_path);m=module();parses=[];original=m.ET.fromstring
    def parse(description):
        parses.append(description);return original(description)
    monkeypatch.setattr(m.ET,'fromstring',parse)
    first=(tmp_path/'robot.urdf').read_text()
    descriptions=[first.replace('name="r"',f'name="r{i}"') for i in range(5)]
    for description in descriptions:m.validate_hardware(description)
    for description in descriptions[1:]:m.validate_hardware(description)
    assert len(parses)==5
    m.validate_hardware(descriptions[0])
    assert len(parses)==6


@pytest.mark.parametrize('change', ['expired', 'session', 'scene', 'not_ready'])
def test_cached_hardware_validation_never_caches_readiness_or_identity(tmp_path, change):
    _,ready=context(tmp_path);m=module()
    assert m.validate_probe_context(tmp_path,now=10.5)['session_id']=='s'
    if change=='expired':ready['updated_monotonic']=7.
    elif change=='session':ready['session_id']='replacement'
    elif change=='scene':ready['scene_revision']='replacement'
    elif change=='not_ready':ready['ready']=False
    (tmp_path/'managed-ready.json').write_text(json.dumps(ready))
    with pytest.raises(ValueError):m.validate_probe_context(tmp_path,now=10.5)
