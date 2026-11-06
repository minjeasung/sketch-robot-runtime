from pathlib import Path
import copy
import importlib.util
import json
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def report_module():
    path=ROOT/'scripts/check_platform_runtime.py'
    assert path.exists(), 'Platform evidence verifier is missing'
    spec=importlib.util.spec_from_file_location('check_platform_runtime',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def selection(mode='simulation',process='spray'):
    return dict(execution_mode=mode,platform='commercial',active_arm='left',process=process)


def evidence(mode='simulation',process='spray'):
    identity=dict(session_id='run:left',asset_revision='asset1',tool_revision='tool1',scene_revision='scene1',
                  runtime_revision=3,calibration_revision='cal1' if mode=='mixed' else None)
    samples=[dict(time=10.+i,stamp=10.+i,valid=True,state_age_s=.01,source='controller_state',
                  session_id='run:left',scene_revision='scene1',joints=['j'+str(n) for n in range(6)],
                  positions=[float(i)]*6,desired_positions=[float(i)]*6) for i in range(2)]
    return dict(selection=selection(mode,process),identity=identity,executed=True,command_accepted=True,terminal_state='COMPLETE',
                camera=dict(connected=True,source='live_camera' if mode=='mixed' else 'isaac_camera',
                            camera_id='camera1',frame_id='optical1',age_s=.01),
                controller=dict(name='left_controller',active=True),joint_samples=samples,
                geometry=dict(normal_error_deg=.5,offset_error_m=.002,source='measured'),
                stop=dict(last_input=10.,detected=10.05,disabled=10.06,cancel_ack=10.07,stopped=10.08),
                events=[],force_samples=[])


def test_command_acceptance_without_measured_tracking_is_not_a_pass():
    module=report_module();data=evidence();data['joint_samples']=[]
    report=module.assess_observations(selection(),data)
    assert report['status']=='failed'
    assert report['checks']['command_accepted']['status']=='passed'
    assert report['checks']['joint_tracking']['status']=='failed'


def test_recorded_fixture_cannot_be_claimed_as_live_mixed_camera():
    module=report_module();data=evidence('mixed')
    data['camera']['source']='recording'
    report=module.assess_observations(selection('mixed'),data)
    assert report['status']=='failed'
    assert report['camera']['source']=='recording'
    data['camera']['connected']=False
    report=module.assess_observations(selection('mixed'),data)
    assert report['status']=='not_run'


def test_stale_input_foreign_revision_or_unacknowledged_stop_fails():
    module=report_module()
    assert module.assess_observations(selection(),evidence())['status']=='passed'
    for change in ('stale','revision','stop','echo'):
        data=evidence()
        if change=='stale':data['joint_samples'][1]['state_age_s']=.201
        if change=='revision':data['joint_samples'][1]['scene_revision']='old'
        if change=='stop':data['stop']['cancel_ack']=None
        if change=='echo':data['joint_samples'][1]['source']='command_echo'
        assert module.assess_observations(selection(),data)['status']=='failed',change


def painting_evidence():
    data=evidence(process='paint');events=[];samples=[]
    for stroke,start in enumerate((0.,10.)):
        identity=dict(plan_hash='plan1',work_area_id='area1',plane_generation_id='plane1')
        for state,offset in [('APPROACH_PRECONTACT',0),('PRECONTACT_TARE_COMPLETE',.1),('CONTACT_SEARCH',.2),
                             ('RAMP_UP',.3),('PAINT',1),('RAMP_DOWN',8),('RETRACT' if stroke==0 else 'FINAL_RETRACT',9)]:
            events.append(dict(state=state,time=start+offset,timestamp_ns=int((start+offset)*1e9),**identity))
        for n in range(70):
            stamp=start+1+n*.1
            samples.append(dict(time=stamp,sim_time=stamp,step=stroke*100+n,session='run:left',session_id='run:left',
                scene_revision='scene1',valid=True,state_age_s=.01,force_age_s=.01,
                measured_contact_force_n=25.,source='isaac_measured_reaction'))
    events.append(dict(state='COMPLETE',time=20.,timestamp_ns=20_000_000_000))
    data.update(events=events,force_samples=samples)
    return data


def test_paint_force_requires_two_strokes_and_measured_five_second_hold():
    module=report_module();data=painting_evidence()
    report=module.assess_observations(selection(process='paint'),data)
    assert report['checks']['roller_force']['status']=='passed'
    assert report['status']=='passed'
    for change in ('short','synthetic','out_of_band','revision'):
        bad=copy.deepcopy(data)
        if change=='short':bad['force_samples']=bad['force_samples'][:30]
        if change=='synthetic':bad['force_samples'][20]['source']='target_force'
        if change=='out_of_band':
            for sample in bad['force_samples']:sample['measured_contact_force_n']=5.
        if change=='revision':bad['force_samples'][20]['scene_revision']='old'
        assert module.assess_observations(selection(process='paint'),bad)['status']=='failed',change


def test_matrix_lists_all_eighteen_without_counting_unobserved_runs():
    module=report_module()
    run=module.assess_observations(selection(),evidence())
    matrix=module.verification_matrix([run])
    assert len(matrix)==18
    assert sum(row['status']=='passed' for row in matrix)==1
    assert sum(row['status']=='not_run' for row in matrix)==17


def test_readiness_does_not_count_as_execution_and_rejects_stale_heartbeat():
    module=report_module()
    identity=evidence()['identity']
    bundle=dict(revision='asset1',runtime_revision=3,arms={'left':dict(tool_revision='tool1')},selection=selection())
    heartbeat=dict(**identity,pid=123,updated_monotonic=100.,ready=True,joints_valid=True,wrench_valid=True,
                   cameras_ready=True,moveit_ready=True,controllers_ready=True)
    report=module.prepare_report(selection(),bundle,heartbeat,heartbeat,now=100.1,pid_alive=lambda pid:True)
    assert report['readiness']['status']=='passed'
    assert report['status']=='not_run'
    report=module.prepare_report(selection(),bundle,heartbeat,heartbeat,now=102.,pid_alive=lambda pid:True)
    assert report['readiness']['status']=='failed'


def test_execute_refuses_absent_or_stale_plan_and_real_mode():
    module=report_module()
    identity=evidence()['identity']
    for plan in (None,{},dict(validated=True,plan_id='p1',identity={**identity,'scene_revision':'old'})):
        try:module.validate_execution_plan(selection(),plan,identity)
        except ValueError:pass
        else:raise AssertionError('unvalidated/stale plan accepted')
    try:module.validate_execution_plan({**selection(),'execution_mode':'real'},None,identity)
    except ValueError:pass
    else:raise AssertionError('real execution accepted')


def test_cli_default_is_read_only_and_emits_not_run_without_assets(tmp_path):
    module=report_module();output=tmp_path/'report.json'
    result=subprocess.run([sys.executable,module.__file__,'--mode','simulation','--platform','rb10',
        '--arm','single','--process','paint','--assets',str(tmp_path/'missing'),'--output',str(output)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    report=json.loads(output.read_text())
    assert report['status']=='not_run' and report['scope']=='readiness'
    assert len(report['verification_matrix'])==18


def test_execute_checks_selected_hardware_is_simulator_and_live_plan_matches(tmp_path):
    module=report_module()
    assert hasattr(module,'validate_simulator_bundle'), 'Live execution simulator isolation missing'
    robot=tmp_path/'robot.urdf'
    joints=''.join(f'<joint name="j{i}"/>' for i in range(6))
    robot.write_text('<robot><ros2_control><hardware><plugin>real_robot/Driver</plugin></hardware>'+joints+'</ros2_control></robot>')
    bundle=dict(urdf=str(robot),selection=selection(process='paint'),arms={'left':dict(joints=['j'+str(i) for i in range(6)])})
    import pytest
    with pytest.raises(ValueError):module.validate_simulator_bundle(selection(process='paint'),bundle)
    robot.write_text(robot.read_text().replace('real_robot/Driver','isaac_painting_hardware/IsaacPaintingSystem'))
    module.validate_simulator_bundle(selection(process='paint'),bundle)
    plan=dict(plan_id='path1',plan_hash='hash1')
    assert module.live_plan_matches(selection(),plan,dict(ready=True,path_id='path1',plan_hash='hash1',process_mode='spray'))
    assert not module.live_plan_matches(selection(),plan,dict(ready=True,path_id='old',plan_hash='hash1',process_mode='spray'))


def test_incomplete_execution_and_unverified_camera_never_pass():
    module=report_module()
    data=evidence();data['terminal_state']='ABORT'
    assert module.assess_observations(selection(),data)['status']=='failed'
    data=evidence();data['camera']['source']='unverified'
    assert module.assess_observations(selection(),data)['status']=='failed'


def test_live_observation_identity_requires_advancing_current_runtime(tmp_path):
    module=report_module()
    assert hasattr(module,'require_current_runtime'), 'Observation runtime identity check missing'
    identity=evidence()['identity']
    bundle=dict(directory=str(tmp_path))
    import pytest
    with pytest.raises(RuntimeError):module.require_current_runtime(bundle,identity,now=10.)
    heartbeat=dict(**identity,ready=True,updated_monotonic=10.)
    (tmp_path/'managed-ready.json').write_text(json.dumps(heartbeat))
    module.require_current_runtime(bundle,identity,now=10.1)
    heartbeat['session_id']='new-run'
    (tmp_path/'managed-ready.json').write_text(json.dumps(heartbeat))
    with pytest.raises(RuntimeError):module.require_current_runtime(bundle,identity,now=10.1)


def test_invalid_measurements_remain_reportable_without_json_nan():
    module=report_module()
    assert hasattr(module,'json_safe'), 'Invalid sensor evidence must serialize explicitly'
    invalid={'positions':[float('nan'),float('inf'),-.5],'valid':False}
    assert module.json_safe(invalid)=={'positions':[None,None,-.5],'valid':False}
    json.dumps(module.json_safe(invalid),allow_nan=False)
