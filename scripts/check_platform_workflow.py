#!/usr/bin/env python3
"""Measured simulator paint workflow through original UI pixel topics.

Default inspects and projects the requested wall ROI without publishing workflow
requests. --execute permits D405 wrist motion and the complete two-stroke run.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from check_platform_motion import validate_probe_context,validate_hardware,cancel_stop_summary
from check_rb20_painting_isaac import Observer,run_workflow,status_is_current,manual_strokes
from rb20_painting_report import assess_painting
from rbpodo_painting_control.runtime_topics import painting_control_remappings


def authorize_workflow(execute):
    if execute is not True:raise ValueError('--execute is required: D405 selection itself moves the simulated wrist')


def cleanup_step(report,stage,action):
    """Keep the primary failure while attempting every independent cleanup step."""
    try:action()
    except Exception as exc:
        report.setdefault('cleanup_errors',[]).append(dict(stage=stage,error=str(exc)))
        report['passed']=False
        report.setdefault('error',f'cleanup failed ({stage}): {exc}')


def workflow_remappings(arm):
    if arm not in ('left','right','single'):raise ValueError('unknown active arm')
    namespace='/platform/'+arm
    return painting_control_remappings(namespace)+[
        ('/isaac/rb20_painting/'+name,namespace+'/isaac_painting/'+name)
        for name in ('state','command','measured_ft_pose','hardware_status','lifecycle_status')]+[
        ('/controller_manager/list_controllers',namespace+'/controller_manager/list_controllers')]


def wall_fixture(scene,objects,fraction=.3):
    if not 0<fraction<=.8:raise ValueError('ROI fraction must be in (0,.8]')
    wall=scene['wall'];active=objects['active_target']
    entry=next((item for item in objects['objects'] if item['name']==active),None)
    if entry is None or active!=wall['id']:raise ValueError('generated wall is not the active target')
    face=entry['sketch_face']
    if face not in ('+x','-x','+y','-y','+z','-z'):raise ValueError('unsupported wall face')
    axis='xyz'.index(face[1]);sign=1 if face[0]=='+' else -1
    normal=np.zeros(3);normal[axis]=sign
    center=np.asarray(wall['center'],dtype=float);size=np.asarray(wall['size'],dtype=float)
    if center.shape!=(3,) or size.shape!=(3,) or not np.all(np.isfinite([center,size])) or np.any(size<=0):raise ValueError('invalid wall geometry')
    point=center+normal*size/2
    camera=np.asarray(scene['zed_world_optical'],dtype=float)[:3,3]
    if np.dot(camera-point,normal)<=.02:raise ValueError('rendered camera sees the opposite face of the active wall')
    axes=[i for i in range(3) if i!=axis];corners=[]
    for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:
        corner=point.copy();corner[axes[0]]+=u*size[axes[0]]*fraction/2;corner[axes[1]]+=v*size[axes[1]]*fraction/2
        corners.append(corner.tolist())
    return dict(point=point.tolist(),normal=normal.tolist(),corners=corners,face=face,target=active)


def project_fixture(fixture,world_optical,k,width,height):
    inverse=np.linalg.inv(np.asarray(world_optical,dtype=float));k=np.asarray(k,dtype=float).reshape(3,3);pixels=[]
    for corner in fixture['corners']:
        camera=inverse@np.r_[corner,1.]
        if not np.all(np.isfinite(camera)) or camera[2]<=0:raise ValueError('target is not visible in rendered camera')
        pixel=k@camera[:3];u,v=pixel[:2]/pixel[2]
        if not (0<=u<width and 0<=v<height):raise ValueError('target is not visible inside the rendered pixel grid')
        pixels.append([float(u),float(v)])
    return pixels


def free_space_evidence(sample,age,tcp,fixture):
    try:
        residual=np.asarray(sample['wrench'])-np.asarray(sample['gravity_wrench'])
        velocity=np.asarray(sample['velocities']);point=np.asarray(tcp)
        finite=(residual.shape==(6,) and velocity.shape==(6,) and point.shape==(3,)
                and np.all(np.isfinite(residual)) and np.all(np.isfinite(velocity)) and np.all(np.isfinite(point)))
        force=float(np.linalg.norm(residual[:3]));torque=float(np.linalg.norm(residual[3:]));speed=float(np.max(np.abs(velocity)))
        clearance=float(np.dot(point-np.asarray(fixture['point']),fixture['normal']))
        passed=bool(finite and sample['valid'] and 0<=age<=.2 and force<=5. and torque<=2. and speed<=.01 and clearance>=.02)
        return dict(passed=passed,force_n=force,torque_nm=torque,max_velocity_rad_s=speed,wall_clearance_m=clearance,state_age_s=age)
    except (ValueError,TypeError,KeyError):return dict(passed=False,reason='incomplete measured free-space evidence')


class PlatformObserver(Observer):
    def __init__(self,assets,context,execute):
        self.assets=Path(assets);self.context=context;self.bundle=context['bundle'];self.session=context['session_id']
        self.active=self.bundle['arms'][self.bundle['selection']['active_arm']]
        self.execute_authorized=execute;self.mutated=False;self.free_space_proof=None
        super().__init__(remappings=workflow_remappings(self.bundle['selection']['active_arm']),
                         base_frame=self.active['base_frame'],wrench_frame=self.active['wrench_frame'])
        from geometry_msgs.msg import WrenchStamped
        self.node.create_subscription(WrenchStamped,'/admittance_controller/wrench_reference',lambda msg:self.store('forwarded_wrench',msg),10)

    def guard(self):
        observed={}
        try:current=validate_probe_context(self.assets,observed=observed)
        except (OSError,ValueError) as exc:
            self.readiness_rejection=dict(error=str(exc),**observed)
            evidence={}
            try:
                failure=json.loads((self.assets/'startup-error.json').read_text())
                if (failure.get('session_id')==self.session and failure.get('asset_revision')==self.bundle['revision']
                        and failure.get('runtime_revision')==self.bundle.get('runtime_revision',0)):
                    evidence['stack_failure']=failure
            except (OSError,ValueError,AttributeError):pass
            safety=self.latest.get('safety',{})
            if safety.get('abort_latched') is True:evidence['last_received_safety']=safety
            if evidence:
                self.runtime_failure=evidence
                raise RuntimeError(f'{exc}; runtime failure context: '+json.dumps(evidence)) from exc
            raise
        if (current['session_id']!=self.session or current['bundle']['revision']!=self.bundle['revision']
                or current['bundle']['selection']!=self.bundle['selection']):raise RuntimeError('runtime changed during workflow probe')

    def wait(self,predicate,seconds=30.,label='condition'):
        started=time.monotonic()
        self.last_wait=dict(label=label,limit_wall_s=seconds,started_monotonic=started)
        def guarded():self.guard();return predicate()
        try:return super().wait(guarded,seconds,label)
        except Exception as exc:raise RuntimeError(f'{label}: {exc}') from exc
        finally:self.last_wait['elapsed_wall_s']=time.monotonic()-started

    def _abort_destination_current(self):
        """A stop may bypass readiness, but never target a replacement runtime."""
        bundle=json.loads((self.assets/'bundle.json').read_text())
        marker=json.loads((self.assets/'simulation-ready.json').read_text())
        now=time.monotonic()
        if (bundle['selection']!=self.bundle['selection'] or bundle['revision']!=self.bundle['revision']
            or bundle.get('runtime_revision',0)!=self.bundle.get('runtime_revision',0)
            or marker.get('session_id')!=self.session or marker.get('asset_revision')!=self.bundle['revision']
            or marker.get('scene_revision')!=self.bundle['revision']
            or marker.get('runtime_revision')!=self.bundle.get('runtime_revision',0)
            or not 0<=now-marker.get('updated_monotonic',0)<=2.):
            raise ValueError('abort destination identity or lease changed')
        os.kill(int(marker['pid']),0)

    def prepare_abort(self):
        """Discover the stop channel early, without sending a command."""
        from std_msgs.msg import Bool
        topic='/motion_abort'
        self._abort_destination_current()
        if topic not in self.publishers:
            self.publishers[topic]=self.node.create_publisher(Bool,topic,10)
        def subscriber():
            self._abort_destination_current()
            return self.publishers[topic].get_subscription_count()>0
        if not subscriber():
            # Call the base wait explicitly: normal readiness may be the reason
            # for this stop. Recheck only the captured destination during discovery.
            Observer.wait(self,subscriber,2.,'/motion_abort subscriber')
        self._abort_destination_current()
        return self.publishers[topic]

    def publish(self,topic,msg):
        if topic=='/motion_abort':
            publisher=self.prepare_abort()
            self.abort_publish_monotonic=time.monotonic()
            return publisher.publish(msg)
        if topic!='/motion_abort':
            authorize_workflow(self.execute_authorized);self.guard()
            if topic=='/painting_admittance/free_space_confirmed':
                state=self.latest.get('state')
                if not state or not self.samples:raise RuntimeError('measured free-space state unavailable')
                sample=self.samples[-1];received=self.received['state']
                stamp=self.rclpy.time.Time.from_msg(state.header.stamp)
                # Joint feedback can arrive one ROS callback before its TF.
                # Wait for that exact exposure, retaining its original age.
                self.wait(lambda:self.tf.can_transform(self.active['base_frame'],self.active['tcp_link'],stamp),
                          .15,'free-space stamped TCP transform')
                transform=self.tf.lookup_transform(self.active['base_frame'],self.active['tcp_link'],stamp)
                t=transform.transform.translation
                now=time.monotonic()
                self.free_space_proof=free_space_evidence(sample,now-received,[t.x,t.y,t.z],self.fixture)
                recent=[sample for sample in self.samples if now-.5<=sample['time']<=now]
                stable=bool(len(recent)>=3 and recent[-1]['time']-recent[0]['time']>=.3
                    and all(b['time']-a['time']<=.2 for a,b in zip(recent,recent[1:]))
                    and all(free_space_evidence(sample,0.,[t.x,t.y,t.z],self.fixture)['passed'] for sample in recent))
                self.free_space_proof.update(stable_measured_window=stable,samples=len(recent))
                self.free_space_proof['passed']=self.free_space_proof['passed'] and stable
                if not self.free_space_proof['passed']:raise RuntimeError('free-space confirmation refused: '+json.dumps(self.free_space_proof))
            self.mutated=True
        return super().publish(topic,msg)

    def state_message(self,msg):
        super().state_message(msg)
        sample=self.samples[-1];sample['scene_revision']=msg.scene_revision;sample['source']='isaac_measured_reaction'
        sample['simulator_state_valid']=bool(msg.joints_valid and msg.wrench_valid and msg.playing
            and all(math.isfinite(value) for value in [*sample['positions'],*sample['velocities'],
                                                       *sample['wrench'],*sample['gravity_wrench']]))
        if (msg.session_id!=self.session or msg.scene_revision!=self.bundle['revision']
                or tuple(msg.joint_names)!=tuple(self.active['joints'])):
            sample['valid']=False;sample['simulator_state_valid']=False

    def report(self,*,controller_timeout_s=5.):
        from controller_manager_msgs.srv import ListControllers
        self.guard();active=[]
        if self.controllers.service_is_ready():
            future=self.controllers.call_async(ListControllers.Request());self.wait(future.done,controller_timeout_s,'selected controllers')
            active=[c.name for c in future.result().controller if c.state=='active']
        now=time.monotonic();state=self.latest.get('state')
        checks={'canonical_runtime':True,'measured_state':bool(state and state.playing and state.joints_valid and state.wrench_valid
            and tuple(state.joint_names)==tuple(self.active['joints']) and state.session_id==self.session and state.scene_revision==self.bundle['revision']
            and 0<=now-self.received.get('state',0)<.2)}
        required={'joint_state_broadcaster','force_torque_sensor_broadcaster','force_torque_sensor_broadcaster_raw','admittance_controller',self.active['controller']}
        checks['controllers_active']=required.issubset(active)
        for key in ('zed_rgb','zed_depth','d405_rgb','d405_depth'):
            checks[key]=self.counts.get(key,0)>2 and 0<=now-self.received.get(key,0)<1.
        for key,field in [('hardware','inhibited'),('lifecycle','ok')]:
            value=self.latest.get(key,{})
            checks[key]=bool(value.get(field) is (field=='ok') and status_is_current(value,self.received.get(key,0.),now,state))
        return dict(passed=all(checks.values()),checks=checks,session_id=self.session,scene_revision=self.bundle['revision'],
                    active_controllers=active,counts=self.counts,d405_plane=self.plane_evidence)

    def record_fixture_accuracy(self):
        plane=self.latest['refinement']['refined_plane'];stamp=plane['stamp']
        transform=self.tf.lookup_transform(self.active['base_frame'],plane['frame_id'],self.rclpy.time.Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec']))
        t=transform.transform.translation;q=transform.transform.rotation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]);point=rotation.apply(plane['position'])+np.array([t.x,t.y,t.z])
        normal=rotation.apply(Rotation.from_quat(plane['orientation']).apply([0,0,1]))
        distance=abs(float(np.dot(point-self.fixture['point'],self.fixture['normal'])))
        angle=math.degrees(math.acos(float(np.clip(np.dot(normal,self.fixture['normal'])/np.linalg.norm(normal),-1,1))))
        self.plane_evidence=dict(passed=distance<=.005 and angle<=2.,distance_error_m=distance,normal_error_deg=angle,
                                point=point.tolist(),normal=normal.tolist(),capture_stamp=stamp)
        if not self.plane_evidence['passed']:raise RuntimeError('measured D405 plane disagrees with selected wall: '+json.dumps(self.plane_evidence))


def force_cancel_trigger(latest,received,identity,now):
    """Only cancel an identity-matched PAINT phase carrying fresh measured load."""
    execution=latest.get('execution') or {};safety=latest.get('safety') or {};guard=latest.get('guard') or {}
    force=safety.get('contact_force_n')
    return bool(execution.get('state')=='PAINT' and all(execution.get(k)==identity[k] for k in ('plan_hash','work_area_id','plane_generation_id'))
        and all(0<=now-received.get(k,0.)<=.2 for k in ('safety','guard'))
        and safety.get('ft_valid') is True and safety.get('tf_valid') is True and safety.get('abort_latched') is False
        and isinstance(force,(int,float)) and not isinstance(force,bool) and math.isfinite(force) and 18<=force<=32
        and guard.get('mode')=='PAINT' and guard.get('forwarding') is True
        and guard.get('force_enable') is True and guard.get('compliance_active') is True)


def cancel_terminal_event(events,identity,requested,now):
    """Retain an actual post-request reliable ABORT edge for this execution."""
    return next((event for event in events if event.get('state')=='ABORT'
        and type(event.get('time')) in (int,float) and requested<event['time']<=now
        and all(event.get(key)==identity[key] for key in ('plan_hash','work_area_id','plane_generation_id'))),None)


def force_cancel_evidence(latest,received,samples,joints,requested,now,session,revision):
    """Hard stop acknowledges inhibit/zero then independently proves measured rest."""
    guard=latest.get('guard') or {};hardware=latest.get('hardware') or {}
    fresh=lambda key: requested<received.get(key,0.)<=now and 0<=now-received.get(key,0.)<=.2
    norm=guard.get('requested_force_norm_n')
    forwarded=getattr(latest.get('forwarded_wrench'),'wrench',None)
    try:
        values=[getattr(vector,axis) for vector in (forwarded.force,forwarded.torque) for axis in ('x','y','z')]
        forwarded_zero=all(math.isfinite(value) and abs(value)<=1e-9 for value in values)
    except (AttributeError,TypeError,ValueError):forwarded_zero=False
    command=latest.get('command')
    zero=bool(fresh('guard') and fresh('command') and guard.get('forwarding') is False
        and guard.get('force_enable') is False and guard.get('compliance_active') is False
        and isinstance(norm,(int,float)) and not isinstance(norm,bool) and math.isfinite(norm) and abs(norm)<=1e-9
        and getattr(command,'enabled',None) is False
        and getattr(command,'session_id',None)==session and getattr(command,'scene_revision',None)==revision
        and fresh('forwarded_wrench') and forwarded_zero)
    inhibit=bool(fresh('hardware') and hardware.get('session_id')==session
        and hardware.get('scene_revision')==revision and hardware.get('inhibited') is True)
    # Latest status may already be HARDWARE_MOTION_INHIBITED. Only the latched
    # reliable event acknowledges the abort; diagnostic updates cannot erase it.
    terminal_event=latest.get('_cancel_terminal_event') or {}
    terminal_time=terminal_event.get('time')
    terminal=bool(terminal_event.get('state')=='ABORT' and type(terminal_time) in (int,float)
                  and requested<terminal_time<=now)
    post=[sample for sample in samples if sample['time']>=requested]
    invalid=any(sample.get('simulator_state_valid') is not True or sample.get('session')!=session or sample.get('scene_revision')!=revision
        or sample.get('source')!='isaac_measured_reaction' or len(sample.get('positions',[]))!=len(joints)
        or len(sample.get('velocities',[]))!=len(joints) for sample in post)
    converted=[dict(time=sample['time'],stamp_ns=int(round(sample['sim_time']*1e9)),
                    positions=dict(zip(joints,sample['positions'])),velocities=dict(zip(joints,sample['velocities'])))
               for sample in post] if not invalid else []
    stop=cancel_stop_summary(converted,joints,requested,terminal_time,now) if terminal else dict(passed=False,reason='ABORT acknowledgment missing')
    if invalid:stop.update(passed=False,reason='invalid/foreign measured state after cancel')
    return dict(passed=zero and inhibit and terminal and stop['passed'],force_off=zero,
        hardware_inhibit_acknowledged=inhibit,abort_acknowledged=terminal,
        abort_event=terminal_event if terminal else None,measured_stop=stop,
        retreat='not_performed_hard_stop_requires_relaunch',normal_workflow_complete=False,
        force_off_semantics='forwarded reference zero and simulated drive disabled; contact load may remain',
        final_measured_contact_force_n=post[-1].get('measured_contact_force_n') if post and post[-1].get('valid') else None)


def run_force_cancel(obs,identity,event_start,timeout):
    """A separate authorized public hard-stop test; never auto-retract after inhibit."""
    from std_msgs.msg import Bool
    authorize_workflow(obs.execute_authorized)
    # DDS discovery must finish before loaded PAINT, not after the stop trigger.
    # The timing budget still begins before publish and includes dispatch/checks.
    obs.prepare_abort()
    def active():
        if any(event.get('state') in {'ABORT','FAILED','PLAN_REJECTED','NOT_READY'} for event in obs.events[event_start:]):
            raise RuntimeError('workflow failed before requested force-phase cancellation')
        return force_cancel_trigger(obs.latest,obs.received,identity,time.monotonic())
    obs.wait(active,timeout,'fresh loaded PAINT before cancellation')
    cancel_event_start=len(obs.events)
    obs.latest.pop('_cancel_terminal_event',None)
    requested=time.monotonic();obs.publish('/motion_abort',Bool(data=True))
    first_off=None;first_terminal=None
    obs.cancel_report=dict(passed=False,requested_monotonic=requested,scope='force_phase_hard_stop')
    while time.monotonic()-requested<=5.:
        obs.spin(.01);now=time.monotonic()
        # Read-only observation may continue while managed readiness is false
        # because of our requested inhibit. Identity and sensor leases still apply.
        marker=json.loads((obs.assets/'simulation-ready.json').read_text())
        bundle=json.loads((obs.assets/'bundle.json').read_text())
        if (marker.get('session_id')!=obs.session or marker.get('asset_revision')!=obs.bundle['revision']
            or marker.get('runtime_revision')!=obs.bundle.get('runtime_revision',0)
            or bundle['revision']!=obs.bundle['revision'] or bundle['selection']!=obs.bundle['selection']
            or not 0<=now-marker.get('updated_monotonic',0)<=2. or marker.get('joints_valid') is not True
            or marker.get('wrench_valid') is not True):raise RuntimeError('cancel observation runtime identity/measurement lease lost')
        if first_terminal is None:
            first_terminal=cancel_terminal_event(obs.events[cancel_event_start:],identity,requested,now)
            if first_terminal is not None:obs.latest['_cancel_terminal_event']=dict(first_terminal)
        evidence=force_cancel_evidence(obs.latest,obs.received,obs.samples,obs.active['joints'],requested,now,obs.session,obs.bundle['revision'])
        if evidence['force_off'] and first_off is None:first_off=now
        evidence.update(scope='force_phase_hard_stop',requested_monotonic=requested,
                        published_monotonic=getattr(obs,'abort_publish_monotonic',None),
                        force_off_latency_s=first_off-requested if first_off is not None else None,
                        force_off_deadline_s=.1,stop_observation_deadline_s=5.)
        evidence['passed']=evidence['passed'] and first_off is not None and first_off-requested<=.1
        obs.cancel_report=evidence
        if evidence['passed']:return
    raise RuntimeError('force-phase cancel failed measured acceptance: '+json.dumps(obs.cancel_report))


def live_model_proof(obs):
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState
    client=obs.node.create_client(GetParameters,'/move_group/get_parameters')
    obs.wait(client.service_is_ready,20.,'live MoveGroup description')
    future=client.call_async(GetParameters.Request(names=['robot_description']));obs.wait(future.done,10.,'live robot description')
    actual_text=future.result().values[0].string_value;validate_hardware(actual_text)
    actual=ET.fromstring(actual_text);expected=ET.parse(obs.assets/'robot.urdf').getroot()
    for tag in ('joint','link'):
        if {x.get('name') for x in actual.findall(tag)}!={x.get('name') for x in expected.findall(tag)}:raise RuntimeError('live MoveGroup model differs from generated bundle')
    if actual.get('name')!=expected.get('name'):raise RuntimeError('live MoveGroup identity differs from bundle')
    obs.wait(lambda:'joints' in obs.latest and time.monotonic()-obs.received['joints']<.2,10.,'measured full robot joints')
    measured=obs.latest['joints'];positions=dict(zip(measured.name,measured.position));names=obs.bundle['joint_names']
    if not all(n in positions and math.isfinite(positions[n]) for n in names):raise RuntimeError('full measured robot state incomplete')
    client=obs.node.create_client(GetStateValidity,'/check_state_validity');obs.wait(client.service_is_ready,20.,'full state collision service')
    request=GetStateValidity.Request();request.group_name='';request.robot_state.joint_state=JointState(name=names,position=[positions[n] for n in names])
    future=client.call_async(request);obs.wait(future.done,10.,'full robot initial validity');result=future.result()
    proof=dict(valid=result.valid,contacts=[dict(body_1=c.contact_body_1,body_2=c.contact_body_2,depth=c.depth) for c in result.contacts])
    if not result.valid:raise RuntimeError('initial full robot collision: '+json.dumps(proof))
    return proof


def run(args,report):
    import yaml
    cancel=getattr(args,'cancel_during_paint',False)
    if cancel:authorize_workflow(args.execute)
    report['readiness_observation']={}
    context=validate_probe_context(args.assets,observed=report['readiness_observation']);bundle=context['bundle']
    if bundle['selection']['process']!='paint':raise ValueError('pixel probe currently requires paint; spray has no D405 refinement workflow')
    scene=json.loads((args.assets/'scene.json').read_text());objects=yaml.safe_load((args.assets/'objects.yaml').read_text())
    fixture=wall_fixture(scene,objects,args.roi_fraction)
    report.update(selection=bundle['selection'],session_id=context['session_id'],scene_revision=bundle['revision'],fixture=fixture)
    obs=PlatformObserver(args.assets,context,args.execute);obs.fixture=fixture
    try:
        report['initial_health']=obs.wait_healthy(45.);report['initial_state_validity']=live_model_proof(obs)
        obs.wait(lambda:'zed_info' in obs.latest,10.,'rendered camera intrinsics')
        info=obs.latest['zed_info'];polygon=project_fixture(fixture,scene['zed_world_optical'],info.k,info.width,info.height)
        report['target_pixels']=polygon
        if not args.execute:report.update(passed=True,scope='read_only_pixel_plan',workflow_executed=False);return
        authorize_workflow(args.execute)
        report['plan_identity']=run_workflow(obs,args.workflow,execute=True,scene=scene,selection_polygon=polygon,plane_accuracy=obs.record_fixture_accuracy,workflow_time_scale=args.workflow_time_scale,execution_wait=run_force_cancel if cancel else None,stroke_inset_fraction=getattr(args,'stroke_inset',.25))
        if cancel:
            report.update(passed=obs.cancel_report['passed'],scope='force_phase_hard_stop',workflow_executed=True,
                          normal_workflow_complete=False,force_cancel=obs.cancel_report)
        else:
            report['painting']=assess_painting(obs.events,obs.samples)
            report.update(passed=report['painting']['passed'],scope='measured_painting_workflow',workflow_executed=True,d405_plane=obs.plane_evidence,free_space=obs.free_space_proof)
    except Exception as exc:
        report['error']=str(exc)
        report['failure_wait']=getattr(obs,'last_wait',None)
        report['runtime_failure']=getattr(obs,'runtime_failure',None)
        report['readiness_rejection']=getattr(obs,'readiness_rejection',None)
        if obs.mutated:
            def abort():
                from std_msgs.msg import Bool
                obs.publish('/motion_abort',Bool(data=True))
            cleanup_step(report,'motion_abort',abort)
        raise
    finally:
        if hasattr(obs,'cancel_report'):report['force_cancel']=obs.cancel_report
        report['workflow_inputs_published']=obs.mutated
        report['d405_plane']=obs.plane_evidence
        report['free_space']=obs.free_space_proof
        report['last_statuses']={k:v for k,v in obs.latest.items() if isinstance(v,dict)}
        cleanup_step(report,'trace_write',lambda:args.output.with_suffix('.trace.json').write_text(json.dumps(dict(events=obs.events,samples=obs.samples),indent=2)+'\n'))
        def samples_write():
            with args.output.with_suffix('.samples.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=['time','sim_time','step','session','scene_revision','source','measured_contact_force_n','force_age_s','state_age_s','valid'],extrasaction='ignore')
                writer.writeheader();writer.writerows(obs.samples)
        cleanup_step(report,'samples_write',samples_write)
        cleanup_step(report,'observer_close',obs.close)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',required=True,type=Path)
    parser.add_argument('--cancel-during-paint',action='store_true',help='separate force-phase hard stop; requires --execute, no automatic retreat')
    parser.add_argument('--execute',action='store_true');parser.add_argument('--workflow',choices=['manual','fill'],default='manual')
    parser.add_argument('--workflow-time-scale',type=float,default=3.,help='scale only long D405 measurement and execution waits (default: 3); freshness/readiness unchanged')
    parser.add_argument('--stroke-inset',type=float,default=.25,help='manual stroke endpoint inset inside 10–90%% work area; .15 gives longer strokes')
    parser.add_argument('--roi-fraction',type=float,default=.3);parser.add_argument('--output',type=Path)
    args=parser.parse_args();args.assets=args.assets.resolve();args.output=args.output or args.assets/'pixel-workflow-report.json';args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.cancel_during_paint and not args.execute:parser.error('--cancel-during-paint requires --execute')
    if not math.isfinite(args.workflow_time_scale) or args.workflow_time_scale<=0:parser.error('--workflow-time-scale must be finite and positive')
    try:manual_strokes(1,1,args.stroke_inset)
    except ValueError as exc:parser.error(str(exc))
    report=dict(passed=False,workflow_executed=False,kind='platform_pixel_workflow',workflow_time_scale=args.workflow_time_scale,stroke_inset=args.stroke_inset)
    try:run(args,report)
    except Exception as exc:report['error']=str(exc)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
