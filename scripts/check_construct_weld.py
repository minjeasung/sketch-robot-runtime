#!/usr/bin/env python3
"""Exercise the public seam workflow and record actual Isaac TF feedback."""
import argparse
import csv
import json
import os
from pathlib import Path
import time
from uuid import uuid4
from dataclasses import asdict
import urllib.request
import rclpy
from std_msgs.msg import String,Bool
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
from sketch_control.weld_contract import load_tool_profile,validate_target,validate_settings
from sketch_control.weld_input_node import load_fixture
from sketch_control.weld_report import report_segment,check_isaac_hardware

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['complete','stop','invalid-frame','unreachable','reset'],default='complete')
    parser.add_argument('--report')
    parser.add_argument('--trace')
    parser.add_argument('--plan-only',action='store_true')
    args=parser.parse_args()
    suffix='' if args.case=='complete' and not args.plan_only else '-'+('plan' if args.plan_only else args.case)
    args.report=args.report or f'logs/construct-weld{suffix}-report.json'
    args.trace=args.trace or f'logs/construct-weld{suffix}-trace.csv'
    config=json.load(urllib.request.urlopen('http://127.0.0.1:8081/status'))
    if config['configuration']['profile']!='isaac_construct' or config['configuration']['process_mode']!='weld' or os.environ.get('ROS_DOMAIN_ID')!='105': raise RuntimeError('Construct weld runtime in domain 105 required')
    model=config['configuration']['model_id']; profile=load_tool_profile(ROOT/'src/sketch_control/config/weld_sim_profile.json',model)
    rclpy.init(); node=rclpy.create_node('construct_weld_check');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node); seen={};pubs={}; trace=[]; active={}; starts={}; feedback=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(topic,msg):
        seen[topic]=json.loads(msg.data)
    for topic in ['/welding/status','/welding/output_status','/perception/zed_frame_identity']:
        node.create_subscription(String,topic,lambda m,t=topic:receive(t,m),qos if topic!='/perception/zed_frame_identity' else 10)
    stop=node.create_publisher(Bool,'/motion_abort',10)
    def spin():
        rclpy.spin_once(node,timeout_sec=.01)
        out=seen.get('/welding/output_status',{})
        if out.get('on') and out.get('start_sim_time_s') is not None:
            starts.setdefault(out['seam_id'],out['start_sim_time_s'])
        try:
            msg=tf.lookup_transform('World',model.replace('construct_','')+'_manipulator_ee_point',rclpy.time.Time())
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9; tr=msg.transform
            p=dict(stamp=stamp,xyz=[tr.translation.x,tr.translation.y,tr.translation.z],quaternion=[tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
            if not feedback or stamp>feedback[-1]['stamp']: feedback.append(p)
        except Exception: pass
    def wait(predicate,timeout=30):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            spin()
            if predicate():return
        raise TimeoutError(json.dumps(seen.get('/welding/status',{})))
    def send(topic,payload,command=True):
        if command:
            s=seen['/welding/status'];payload={**payload,'command_id':uuid4().hex,'server_session_id':s['server_session_id'],'expected':s['identity']}
        if topic not in pubs:
            pubs[topic]=node.create_publisher(String,topic,qos if topic=='/welding/seams' else 10)
            wait(lambda:pubs[topic].get_subscription_count()>=(2 if topic=='/welding/target_request' else 1))
        pubs[topic].publish(String(data=json.dumps(payload)))
    report={'case':args.case,'passed':False,'simulation':True,'model_id':model}
    try:
        wait(lambda:all(k in seen for k in ['/welding/status','/perception/zed_frame_identity']) and seen['/welding/status'].get('runtime_ready'))
        from controller_manager_msgs.srv import ListHardwareComponents
        from rosidl_runtime_py.convert import message_to_ordereddict
        hardware=node.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components')
        wait(hardware.service_is_ready)
        query=hardware.call_async(ListHardwareComponents.Request());wait(query.done)
        components=[message_to_ordereddict(c) for c in query.result().component];check_isaac_hardware(components)
        heartbeat=ROOT/'logs/construct-heartbeat.json'
        if not heartbeat.exists() or time.time()-heartbeat.stat().st_mtime>5: raise ValueError('live Isaac heartbeat required')
        report['hardware']=components[0]['plugin_name']
        stamp=seen['/perception/zed_frame_identity'];request=uuid4().hex
        target={**stamp,'request_id':request,'target_revision':request,'provider':'fixture',
                'polygon':[[100,100],[1180,100],[1180,650],[100,650]],'server_session_id':seen['/welding/status']['server_session_id']}
        if args.case=='invalid-frame': target['frame_id']='invalid_optical_frame'
        send('/welding/target_request',target,False)
        if args.case=='invalid-frame':
            wait(lambda:'frame identity mismatch' in seen['/welding/status'].get('error',''))
            report.update(passed=not seen['/welding/status']['ready'] and not seen['/welding/status']['seams'],rejection=seen['/welding/status']['error']);return
        wait(lambda:seen['/welding/status']['identity']['target_revision']==request and len(seen['/welding/status'].get('seams',[]))==3)
        if args.case=='unreachable':
            data=asdict(load_fixture(ROOT/'src/sketch_control/config/construct_weld_fixture.json',validate_target(target),'1'))
            for seam in data['seams']: seam['points_xyz']=[[p[0]+50,p[1],p[2]] for p in seam['points_xyz']]
            revision=seen['/welding/status']['identity']['selection_revision']
            send('/welding/seams',data,False)
            wait(lambda:seen['/welding/status']['identity']['selection_revision']>revision)
        send('/welding/selection',{'selection':[{'seam_id':'S1','reverse':False},{'seam_id':'S3','reverse':False}]})
        wait(lambda:len(seen['/welding/status']['selection'])==2)
        revision=seen['/welding/status']['settings_revision']
        send('/process_settings/request',{'values':{}})
        wait(lambda:seen['/welding/status']['settings_revision']>revision)
        send('/welding/plan_request',{})
        wait(lambda:seen['/welding/status']['state']=='PLANNING' or bool(seen['/welding/status'].get('error')))
        wait(lambda:seen['/welding/status']['state']!='PLANNING',180)
        state=seen['/welding/status'];print('PLAN',json.dumps(state),flush=True)
        if args.case=='unreachable':
            report.update(passed=not state['ready'] and 'IK' in state.get('error',''),rejection=state.get('error'));return
        if not state['ready']:raise RuntimeError(state.get('error') or 'plan not ready')
        report['plan_id']=state['plan_id']
        if args.plan_only:report.update(planned=True,passed=True);return
        if args.case=='reset':
            old=dict(command_id=uuid4().hex,server_session_id=state['server_session_id'],expected=state['identity'],plan_id=state['plan_id'])
            for endpoint in ('shutdown-system','prepare-system'):
                req=urllib.request.Request('http://127.0.0.1:8081/'+endpoint,method='POST')
                with urllib.request.urlopen(req,timeout=180) as response: response.read()
            wait(lambda:seen['/welding/status']['server_session_id']!=old['server_session_id'] and seen['/welding/status'].get('runtime_ready'),60)
            send('/welding/execute',old,False)
            wait(lambda:'stale session' in seen['/welding/status'].get('error',''))
            report.update(passed=not seen['/welding/status']['ready'] and not seen['/welding/status']['running'],rejection=seen['/welding/status']['error']);return
        wait(lambda:seen['/welding/status'].get('runtime_ready') and seen['/welding/status'].get('ready'))
        send('/welding/execute',{'plan_id':state['plan_id']})
        wait(lambda:seen['/welding/status']['running'])
        if args.case=='stop':
            wait(lambda:seen.get('/welding/output_status',{}).get('on'),180);stop.publish(Bool(data=True))
            wait(lambda:seen['/welding/status']['state']=='STOPPED' and not seen['/welding/output_status'].get('on'),30)
            report['passed']=not seen['/welding/output_status'].get('on');return
        wait(lambda:seen['/welding/status']['state'] in ('COMPLETED','STOPPED'),240)
        if seen['/welding/status']['state']!='COMPLETED':raise RuntimeError(seen['/welding/status']['error'])
        catalog=load_fixture(ROOT/'src/sketch_control/config/construct_weld_fixture.json',validate_target(target),'1')
        settings=validate_settings({},profile)
        for seam in catalog.seams:
            if seam.seam_id not in starts: continue
            start=starts[seam.seam_id]; duration=sum(__import__('math').dist(a,b) for a,b in zip(seam.points_xyz,seam.points_xyz[1:]))/settings.speed_mps
            active[seam.seam_id]=[dict(time_s=p['stamp']-start,xyz=p['xyz'],quaternion=p['quaternion']) for p in feedback if start-.02<=p['stamp']<=start+duration+.02]
            trace.extend(dict(seam_id=seam.seam_id,**p) for p in active[seam.seam_id])
        report['segments']=[report_segment(s,settings,profile,active.get(s.seam_id,[])) for s in catalog.seams if s.seam_id in ('S1','S3')]
        report['selected_ids']=['S1','S3'];report['observed_ids']=list(starts)
        report['passed']=list(starts)==['S1','S3'] and all(s['passed'] for s in report['segments'])
    except Exception as exc:
        report['error']=str(exc);print('FAILED',str(exc),flush=True)
        if seen.get('/welding/status',{}).get('running'):
            stop.publish(Bool(data=True))
            try: wait(lambda:not seen['/welding/status']['running'] and not seen.get('/welding/output_status',{}).get('on'),30)
            except TimeoutError: report['stop_unconfirmed']=True
    finally:
        Path(args.report).parent.mkdir(parents=True,exist_ok=True);Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
        Path(args.trace).parent.mkdir(parents=True,exist_ok=True)
        with open(args.trace,'w') as f:
            w=csv.writer(f);w.writerow(['seam_id','time_s','x','y','z','qx','qy','qz','qw'])
            for p in trace:w.writerow([p['seam_id'],p['time_s'],*p['xyz'],*p['quaternion']])
        node.destroy_node();rclpy.shutdown();print('REPORT',json.dumps(report),flush=True)
        if not report['passed']:raise SystemExit(1)
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
