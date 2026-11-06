#!/usr/bin/env python3
"""Simulation CAD-fixture weld probe through public target/selection/plan topics.

Default is read-only. --plan-only builds a reviewed fixture plan without motion;
--execute builds and executes it, assessing measured TCP feedback. This is not
camera seam recognition or a physical welding-power test.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
from uuid import uuid4
import xml.etree.ElementTree as ET
from check_platform_motion import validate_probe_context,validate_hardware
from sketch_control.weld_contract import load_tool_profile,validate_target,validate_settings
from sketch_control.weld_input_node import load_fixture
from sketch_control.weld_report import report_segment


def target_request(frame,session,server,request):
    if (frame.get('generation')!=session or frame.get('valid') is not True
            or frame.get('source')!='simulation' or frame.get('camera_id')!='zed'
            or frame.get('clock_domain')!='ros' or frame.get('frame_id')!='zed_left_camera_frame_optical'):
        raise ValueError('current simulated camera identity required')
    width,height=frame.get('width',0),frame.get('height',0)
    if width<10 or height<10:raise ValueError('camera identity dimensions invalid')
    payload={**frame,'request_id':request,'target_revision':request,'provider':'fixture',
             'server_session_id':server,'polygon':[[.1*width,.1*height],[.9*width,.1*height],[.9*width,.9*height],[.1*width,.9*height]]}
    validate_target(payload)
    return payload


def selected_ids(catalog,requested):
    available=[seam.seam_id for seam in catalog.seams];ids=list(requested or available[:2])
    if not ids or len(ids)!=len(set(ids)) or not set(ids)<=set(available):raise ValueError('fixture seam selection missing, duplicate or unknown')
    return ids


def validate_live_description(expected,actual):
    validate_hardware(actual);validate_hardware(expected)
    left,right=ET.fromstring(expected),ET.fromstring(actual)
    def contents(root,tag):
        return sorted(ET.canonicalize(ET.tostring(element,encoding='unicode'),strip_text=True) for element in root.findall(tag))
    if left.get('name')!=right.get('name') or any(contents(left,tag)!=contents(right,tag) for tag in ('link','joint','ros2_control')):
        raise ValueError('live MoveGroup model differs from generated simulator bundle')


def authorize_command(topic,plan_only,execute):
    if topic=='/welding/execute' and not execute:raise ValueError('--execute required for motion')
    if not (plan_only or execute):raise ValueError('--plan-only or --execute required for workflow requests')


def matching_command_ack(status,command_id):
    """An absent/null acknowledgment is pending, never an acceptance."""
    ack=status.get('command_ack')
    return ack if isinstance(ack,dict) and ack.get('command_id')==command_id else None


def run(args,report):
    import rclpy
    from std_msgs.msg import String,Bool
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from tf2_ros import Buffer,TransformListener
    context=validate_probe_context(args.assets);bundle=context['bundle'];session=context['session_id']
    if bundle['selection']['process']!='weld':raise ValueError('selected process must be weld')
    arm=bundle['arms'][bundle['selection']['active_arm']]
    profile=load_tool_profile(args.assets/'weld_tool.json',arm['model_id'])
    fixture_path=args.assets/'weld_fixture.json';fixture=json.loads(fixture_path.read_text())
    if fixture.get('scene_revision')!=bundle['revision'] or fixture.get('frame_id')!=arm['base_frame']:raise ValueError('CAD fixture frame/revision differs from selected bundle')
    report.update(selection=bundle['selection'],session_id=session,scene_revision=bundle['revision'],fixture_provenance=fixture.get('provenance'),
                  model_id=arm['model_id'],tcp_link=arm['tcp_link'],base_frame=arm['base_frame'])
    rclpy.init();node=rclpy.create_node('platform_weld_probe');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node);seen={};received={};starts={};feedback=[];trace=[];publishers={};server=[None];executed=[False]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(topic,msg):
        value=json.loads(msg.data);seen[topic]=value;received[topic]=time.monotonic()
        if topic=='/welding/output_status' and value.get('on'):
            if value.get('simulation') is not True:raise RuntimeError('physical welding output refused')
            start=value.get('start_sim_time_s')
            if start is not None:starts.setdefault(value['seam_id'],float(start))
    for topic in ('/welding/status','/welding/output_status','/perception/zed_frame_identity'):
        node.create_subscription(String,topic,lambda msg,t=topic:receive(t,msg),10 if topic.endswith('frame_identity') else qos)
    abort=node.create_publisher(Bool,'/motion_abort',10)
    def guard():
        current=validate_probe_context(args.assets)
        if (current['session_id']!=session or current['bundle']['revision']!=bundle['revision']
                or current['bundle']['selection']!=bundle['selection']):raise RuntimeError('simulator identity changed during weld probe')
        state=seen.get('/welding/status',{})
        if server[0] and state.get('server_session_id')!=server[0]:raise RuntimeError('weld executor restarted during probe')
    def spin():
        rclpy.spin_once(node,timeout_sec=.01)
        try:
            msg=tf.lookup_transform(arm['base_frame'],arm['tcp_link'],rclpy.time.Time());p=msg.transform
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp>0 and (not feedback or stamp>feedback[-1]['stamp']):feedback.append(dict(stamp=stamp,xyz=[p.translation.x,p.translation.y,p.translation.z],quaternion=[p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w]))
        except Exception:pass
    def wait(predicate,timeout=30.,guarded=True):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            spin()
            if guarded:guard()
            if predicate():return
        raise TimeoutError('weld probe deadline: '+json.dumps(seen.get('/welding/status',{})))
    def send(topic,payload,command=True):
        authorize_command(topic,args.plan_only,args.execute);guard()
        if time.monotonic()-received.get('/welding/status',0)>1.:raise RuntimeError('weld status stale')
        if command:
            status=seen['/welding/status'];payload={**payload,'command_id':uuid4().hex,'server_session_id':server[0],'expected':status['identity']}
        if topic not in publishers:publishers[topic]=node.create_publisher(String,topic,10)
        wait(lambda:publishers[topic].get_subscription_count()>=(2 if topic=='/welding/target_request' else 1),10.)
        publishers[topic].publish(String(data=json.dumps(payload)))
        if command:
            wait(lambda:matching_command_ack(seen['/welding/status'],payload['command_id']) is not None,15.)
            ack=matching_command_ack(seen['/welding/status'],payload['command_id'])
            if ack.get('ok') is not True:raise RuntimeError('weld command rejected: '+str(ack.get('error') or 'unspecified rejection'))
    try:
        wait(lambda:seen.get('/welding/status',{}).get('runtime_ready') is True and '/perception/zed_frame_identity' in seen,45.)
        status=seen['/welding/status'];server[0]=status['server_session_id']
        if status.get('provider')!='fixture':raise RuntimeError('simulation CAD fixture provider required')
        if status.get('running'):raise RuntimeError('existing weld execution/planning is active')
        parameters=node.create_client(GetParameters,'/move_group/get_parameters');wait(parameters.service_is_ready,20.)
        query=parameters.call_async(GetParameters.Request(names=['robot_description']));wait(query.done,10.)
        validate_live_description((args.assets/'robot.urdf').read_text(),query.result().values[0].string_value)
        report['live_model_verified']=True
        if time.monotonic()-received['/perception/zed_frame_identity']>1.:raise RuntimeError('rendered camera identity stale')
        target=target_request(seen['/perception/zed_frame_identity'],session,server[0],uuid4().hex)
        catalog=load_fixture(fixture_path,validate_target(target),bundle['revision'])
        ids=selected_ids(catalog,args.seam);report.update(selected_ids=ids,target=target)
        if not (args.plan_only or args.execute):report.update(passed=True,scope='read_only_weld_fixture');return
        send('/welding/target_request',target,False)
        wait(lambda:seen['/welding/status']['identity']['target_revision']==target['target_revision'] and
             {s['seam_id'] for s in seen['/welding/status'].get('seams',[])}=={s.seam_id for s in catalog.seams},30.)
        send('/welding/selection',dict(selection=[dict(seam_id=s,reverse=False) for s in ids]))
        send('/process_settings/request',dict(values=json.loads(args.settings.read_text()) if args.settings else {}))
        send('/welding/plan_request',{})
        wait(lambda:seen['/welding/status']['state']!='PLANNING',args.plan_timeout)
        status=seen['/welding/status']
        if not status.get('ready') or not status.get('plan_id'):raise RuntimeError(status.get('error') or 'weld plan not ready')
        identity=dict(status['identity']);plan_id=status['plan_id'];settings=validate_settings(status['settings'],profile)
        report.update(plan_id=plan_id,plan_identity=identity,settings=status['settings'],planned=True)
        if args.plan_only:report.update(passed=True,scope='validated_weld_plan');return
        wait(lambda:seen['/welding/status'].get('runtime_ready') and seen['/welding/status'].get('ready'),30.)
        if seen['/welding/status']['identity']!=identity or seen['/welding/status']['plan_id']!=plan_id:raise RuntimeError('weld plan changed before execution')
        starts.clear();feedback.clear();executed[0]=True
        send('/welding/execute',dict(plan_id=plan_id))
        wait(lambda:seen['/welding/status']['state'] in ('COMPLETED','STOPPED'),args.execution_timeout)
        status=seen['/welding/status']
        if status['state']!='COMPLETED':raise RuntimeError(status.get('error') or 'weld execution stopped')
        wait(lambda:seen.get('/welding/output_status',{}).get('on') is False,5.)
        segments=[];by_id={seam.seam_id:seam for seam in catalog.seams}
        for seam_id in ids:
            seam=by_id[seam_id];start=starts.get(seam_id)
            if start is None:raise RuntimeError('measured welding output start missing: '+seam_id)
            duration=sum(math.dist(a,b) for a,b in zip(seam.points_xyz,seam.points_xyz[1:]))/settings.speed_mps
            measured=[dict(time_s=p['stamp']-start,xyz=p['xyz'],quaternion=p['quaternion']) for p in feedback if start-.02<=p['stamp']<=start+duration+.02]
            trace.extend(dict(seam_id=seam_id,**sample) for sample in measured)
            segments.append(report_segment(seam,settings,profile,measured))
        report.update(scope='measured_simulation_weld',segments=segments,observed_ids=list(starts),
                      passed=list(starts)==ids and all(segment['passed'] for segment in segments))
    except Exception:
        if executed[0] and seen.get('/welding/status',{}).get('running'):
            abort.publish(Bool(data=True))
            try:wait(lambda:not seen.get('/welding/status',{}).get('running') and seen.get('/welding/output_status',{}).get('on') is False,30.,guarded=False)
            except Exception:report['stop_unconfirmed']=True
        raise
    finally:
        report['execute_requested']=executed[0];report['last_status']=seen.get('/welding/status',{})
        args.output.with_suffix('.trace.json').write_text(json.dumps(dict(feedback=feedback,seam_starts=starts),indent=2)+'\n')
        with args.output.with_suffix('.trace.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['seam_id','time_s','x','y','z','qx','qy','qz','qw'])
            for p in trace:writer.writerow([p['seam_id'],p['time_s'],*p['xyz'],*p['quaternion']])
        node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',type=Path,required=True)
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--plan-only',action='store_true');mode.add_argument('--execute',action='store_true')
    parser.add_argument('--seam',action='append',default=[],help='fixture seam ID; repeat to select order, default first two actual IDs')
    parser.add_argument('--settings',type=Path);parser.add_argument('--plan-timeout',type=float,default=540.)
    parser.add_argument('--execution-timeout',type=float,default=900.);parser.add_argument('--output',type=Path)
    args=parser.parse_args();args.assets=args.assets.resolve();args.output=args.output or args.assets/'weld-probe.json';args.output.parent.mkdir(parents=True,exist_ok=True)
    if any(not math.isfinite(value) or value<=0 for value in (args.plan_timeout,args.execution_timeout)):parser.error('timeouts must be finite and positive')
    report=dict(kind='simulation_cad_weld_probe',passed=False,planned=False,execute_requested=False)
    try:run(args,report)
    except Exception as exc:report['error']=str(exc)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
