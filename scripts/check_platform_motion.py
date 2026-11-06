#!/usr/bin/env python3
"""Simulation-only infrastructure probe; never a process or hardware validation."""
import argparse
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

SIM_PLUGINS={'topic_based_ros2_control/TopicBasedSystem','isaac_painting_hardware/IsaacPaintingSystem'}


# Cache only this pure XML check by exact content; every guard still rereads
# the file and revalidates all live readiness, session, revision and leases.
@lru_cache(maxsize=4)
def validate_hardware(description):
    plugins=ET.fromstring(description).findall('./ros2_control/hardware/plugin')
    if not plugins or any(p.text not in SIM_PLUGINS for p in plugins):
        raise ValueError('physical or unknown hardware forbidden')


def validate_probe_context(assets,now=None,*,observed=None):
    assets=Path(assets)
    bundle=json.loads((assets/'bundle.json').read_text())
    if bundle['selection']['execution_mode']!='simulation':raise ValueError('probe is simulation only')
    validate_hardware((assets/'robot.urdf').read_text())
    marker_names=('simulation-ready.json','managed-ready.json');markers=[]
    if observed is not None:observed.update(markers={},expected_asset_revision=bundle['revision'])
    for name in marker_names:
        marker=json.loads((assets/name).read_text());markers.append(marker)
        if observed is not None:observed['markers'][name]=marker
    now=time.monotonic() if now is None else now
    if observed is not None:observed['checked_monotonic']=now
    for name,marker in zip(marker_names,markers):
        os.kill(int(marker['pid']),0)
        if not 0<=now-float(marker['updated_monotonic'])<=2.:raise ValueError('runtime readiness expired')
        if marker.get('scene_revision')!=bundle['revision'] or marker.get('asset_revision')!=bundle['revision'] or marker.get('runtime_revision')!=bundle.get('runtime_revision',0):raise ValueError('runtime readiness identity mismatch: '+name)
        if marker.get('ready') is not True:raise ValueError('runtime readiness false: '+name)
        if not all(marker.get(k) is True for k in ('joints_valid','cameras_ready')):raise ValueError('runtime sensors unavailable')
    if not markers[0].get('session_id') or markers[0]['session_id']!=markers[1].get('session_id'):raise ValueError('simulator session mismatch')
    if not all(markers[1].get(k) is True for k in ('controllers_ready','scene_monitor_ready')):raise ValueError('ROS runtime unavailable')
    if bundle['selection']['process']=='paint' and markers[0].get('wrench_valid') is not True:raise ValueError('simulator wrench unavailable')
    return {'bundle':bundle,'session_id':markers[0]['session_id'],'markers':markers}


def tracking_summary(reference,measured,active):
    active=set(active)
    if set(reference)-set(measured) or any(not math.isfinite(float(v)) for v in measured.values()):raise ValueError('incomplete measured joint state')
    selected=max((abs(measured[n]-reference[n]) for n in active),default=0.)
    inactive=max((abs(measured[n]-q) for n,q in reference.items() if n not in active),default=0.)
    return dict(selected_max_error_rad=selected,inactive_max_drift_rad=inactive,passed=selected<=.005 and inactive<=.005)


def stop_summary(samples,active):
    """Observe for wall time, differentiate only stamped simulation measurements."""
    if len(samples)<3:return {'passed':False,'reason':'insufficient measured duration'}
    try:
        receipts=[float(sample['time']) for sample in samples]
        stamps=[sample['stamp_ns'] for sample in samples]
        if not all(math.isfinite(value) for value in receipts):raise ValueError('nonfinite receipt time')
        if any(not isinstance(value,int) or isinstance(value,bool) or value<=0 for value in stamps):
            raise ValueError('invalid measurement timestamp')
        if any(right<=left for left,right in zip(receipts,receipts[1:])):raise ValueError('nonmonotonic receipt times')
        if any(right<=left for left,right in zip(stamps,stamps[1:])):raise ValueError('nonmonotonic measurement timestamps')
        duration=receipts[-1]-receipts[0]
        if duration<.5:return {'passed':False,'reason':'insufficient measured duration'}
        speed=0.;span=0.;measured_speed=None;velocity_difference=0.
        for name in active:
            values=[float(sample['positions'][name]) for sample in samples]
            if not all(math.isfinite(value) for value in values):raise ValueError('nonfinite position')
            span=max(span,max(values)-min(values))
            velocities=[]
            for sample in samples:
                measured=sample.get('velocities',{})
                velocity=float(measured[name]) if measured else None
                if velocity is not None:
                    if not math.isfinite(velocity):raise ValueError('nonfinite measured velocity')
                    measured_speed=max(measured_speed or 0.,abs(velocity))
                velocities.append(velocity)
            for i in range(1,len(samples)):
                dt=(stamps[i]-stamps[i-1])*1e-9
                derivative=(values[i]-values[i-1])/dt
                speed=max(speed,abs(derivative))
                if velocities[i] is not None and velocities[i-1] is not None:
                    velocity_difference=max(velocity_difference,abs(derivative-(velocities[i]+velocities[i-1])*.5))
        combined_speed=max(speed,measured_speed or 0.)
        return dict(passed=combined_speed<=.01 and span<=.001,max_velocity_rad_s=combined_speed,
                    position_derived_max_velocity_rad_s=speed,measured_max_velocity_rad_s=measured_speed,
                    max_velocity_difference_rad_s=velocity_difference if measured_speed is not None else None,
                    velocity_time_basis='joint_states.header.stamp',position_span_rad=span,
                    duration_s=duration,simulation_duration_s=(stamps[-1]-stamps[0])*1e-9)
    except (KeyError,TypeError,ValueError) as exc:
        return {'passed':False,'reason':'invalid measured stop evidence: '+str(exc)}


def cancel_stop_summary(samples,active,requested,terminal,now):
    """Confirm measured stop after terminal cancel, never infer it from the ACK.

    JTC sets a hold command on cancellation; the mechanism may still be settling.
    Keep the existing 0.6 s observation window and all stop_summary thresholds.
    The caller bounds observation to the existing five-second wall-time budget.
    """
    after=[sample for sample in samples if sample['time']>=terminal]
    window=[sample for sample in after if sample['time']>=after[-1]['time']-.6] if after else []
    summary=stop_summary(window,active)
    if not window or not 0<=now-window[-1]['time']<.5:
        summary.update(passed=False,reason='measured stop samples missing or stale')
    if not 0<=now-terminal<=5.:
        summary.update(passed=False,reason='measured stop not confirmed within 5 s after terminal cancellation')
    summary.update(terminal_latency_s=terminal-requested,
                   observation_window_start_s=window[0]['time']-requested if window else None,
                   observation_window_end_s=window[-1]['time']-requested if window else None,
                   observation_after_terminal_s=now-terminal,
                   observed_stop_latency_s=now-requested if summary['passed'] else None,
                   confirmation_after_terminal_s=now-terminal if summary['passed'] else None)
    return summary


def run(args,report):
    import rclpy
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints,JointConstraint
    from moveit_msgs.srv import GetStateValidity
    from rcl_interfaces.srv import GetParameters
    from action_msgs.msg import GoalStatus
    report['readiness_observation']={}
    context=validate_probe_context(args.assets,observed=report['readiness_observation']);bundle=context['bundle'];session=context['session_id']
    active=bundle['arms'][bundle['selection']['active_arm']];names=active['joints'];all_names=bundle['joint_names']
    report.update(session_id=session,selection=bundle['selection'],readiness=context['markers'])
    rclpy.init();node=rclpy.create_node('platform_motion_probe');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    state={};samples=[];latest=[0.,-1];pending=[None]
    def observe(msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp<=latest[1]:return
        state.update(zip(msg.name,msg.position));latest[:]=[time.monotonic(),stamp]
        if all(n in msg.name for n in all_names):
            samples.append({'time':latest[0],'stamp_ns':stamp,'positions':{n:state[n] for n in all_names},
                            'velocities':dict(zip(msg.name,msg.velocity))})
        if len(samples)>3000:del samples[:1000]
    node.create_subscription(JointState,'/joint_states',observe,10)
    def guard():
        if validate_probe_context(args.assets)['session_id']!=session:raise RuntimeError('simulator restarted during probe')
    def until(predicate,timeout=30,check=True):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.03)
            if predicate():return
            if check:guard()
        raise TimeoutError('probe timeout')
    def future(f,timeout=30):until(f.done,timeout);return f.result()
    def fresh():
        until(lambda:all(n in state for n in all_names) and time.monotonic()-latest[0]<.5)
        return {n:state[n] for n in all_names}
    def goal(target,scale=.1):
        g=MoveGroup.Goal();g.request.group_name=active['group'];g.request.pipeline_id='ompl';g.request.allowed_planning_time=5.;g.request.num_planning_attempts=3
        g.request.max_velocity_scaling_factor=scale;g.request.max_acceleration_scaling_factor=scale;g.request.start_state.is_diff=True
        g.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(target[n]),tolerance_above=.001,tolerance_below=.001,weight=1.) for n in names])]
        g.planning_options.plan_only=False
        return g
    client=ActionClient(node,MoveGroup,'/move_action')
    def send(target,scale=.1):
        guard();fresh();handle=future(client.send_goal_async(goal(target,scale)))
        if not handle.accepted:raise RuntimeError('MoveGroup rejected probe')
        pending[0]=handle;return handle
    def complete(target):
        handle=send(target);result=future(handle.get_result_async(),90);pending[0]=None
        if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val!=1:raise RuntimeError('MoveGroup failed: '+str(result.result.error_code.val))
        until(lambda:tracking_summary(target,fresh(),names)['passed'],5)
        return tracking_summary(target,fresh(),names)
    try:
        initial=fresh();report['initial_joint_state']=initial
        parameters=node.create_client(GetParameters,'/move_group/get_parameters')
        if not parameters.wait_for_service(timeout_sec=20):raise RuntimeError('MoveGroup parameters unavailable')
        description=future(parameters.call_async(GetParameters.Request(names=['robot_description']))).values[0].string_value
        validate_hardware(description)
        actual=ET.fromstring(description);expected=ET.fromstring((args.assets/'robot.urdf').read_text())
        if actual.get('name')!=expected.get('name') or {x.get('name') for x in actual.findall('joint')}!={x.get('name') for x in expected.findall('joint')}:raise RuntimeError('live MoveGroup robot differs from bundle')
        validity=node.create_client(GetStateValidity,'/check_state_validity')
        if not validity.wait_for_service(timeout_sec=20):raise RuntimeError('state validity service unavailable')
        request=GetStateValidity.Request();request.group_name='';request.robot_state.joint_state=JointState(name=list(initial),position=list(initial.values()))
        valid=future(validity.call_async(request));report['initial_state_validity']={'valid':valid.valid,'contacts':[{'body_1':c.contact_body_1,'body_2':c.contact_body_2,'depth':c.depth,'position':[c.position.x,c.position.y,c.position.z]} for c in valid.contacts]}
        if not valid.valid:raise RuntimeError('full robot initial state collides; inspect contacts')
        if not args.execute:report['passed']=True;report['motion_executed']=False;return
        if not client.wait_for_server(timeout_sec=20):raise RuntimeError('MoveGroup action unavailable')
        target=dict(initial);joint=expected.find("joint[@name='%s']/limit"%names[0]);delta=.02
        if joint is not None and initial[names[0]]+delta>=float(joint.get('upper','inf'))-.005:delta=-delta
        if joint is not None and initial[names[0]]+delta<=float(joint.get('lower','-inf'))+.005:raise RuntimeError('no modest probe offset within joint limits')
        target[names[0]]+=delta
        report['outbound']=complete(target);report['return']=complete(initial)
        handle=send(target,.01);result_future=handle.get_result_async()
        until(lambda:abs(state[names[0]]-initial[names[0]])>.002 or result_future.done(),30)
        if result_future.done():pending[0]=None;raise RuntimeError('trajectory finished before measured cancellation; cancellation unverified')
        requested=time.monotonic();cancel=future(handle.cancel_goal_async(),5)
        report['cancel']={'acknowledged':bool(cancel.goals_canceling)}
        result=future(result_future,15);pending[0]=None
        report['cancel']['terminal_status']=result.status
        if not cancel.goals_canceling or result.status!=GoalStatus.STATUS_CANCELED:raise RuntimeError('cancellation did not terminate canceled')
        terminal=time.monotonic()
        report['cancel']['stop_observation_timeout_s']=5.
        def measured_stop():
            report['cancel'].update(cancel_stop_summary(samples,names,requested,terminal,time.monotonic()))
            return report['cancel']['passed']
        try:until(measured_stop,5)
        except TimeoutError:
            measured_stop()
            # Expiring the deadline is not stop proof, even if a late callback
            # happens to expose a qualifying window after the observation bound.
            report['cancel'].update(passed=False,observed_stop_latency_s=None,
                                    confirmation_after_terminal_s=None,
                                    reason='measured stop not confirmed within 5 s after terminal cancellation')
        report['cancel']['inactive_tracking']=tracking_summary({**initial,**{n:state[n] for n in names}},fresh(),names)
        if not report['cancel']['passed'] or not report['cancel']['inactive_tracking']['passed']:raise RuntimeError('canceled action has unverified measured stop or inactive drift')
        report['final_return']=complete(initial);report.update(passed=True,motion_executed=True)
    finally:
        if pending[0] is not None:
            f=pending[0].cancel_goal_async()
            try:until(f.done,5,check=False)
            except Exception:pass
        node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',required=True,type=Path);parser.add_argument('--execute',action='store_true',help='execute modest selected-arm motion and cancellation in verified simulation');parser.add_argument('--output',type=Path);args=parser.parse_args();args.assets=args.assets.resolve()
    report={'kind':'infrastructure_probe','passed':False,'motion_executed':False}
    try:run(args,report)
    except Exception as exc:report['error']=str(exc)
    output=args.output or args.assets/'motion-probe.json';output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
