#!/usr/bin/env python3
"""Collect original ROS runtime and measured Isaac feedback; no synthetic acceptance."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

# Cold numerical-library initialization must precede DDS subscriptions. Loading
# scipy inside the first pose callback can starve the checker's own receipt loop.
import numpy as np
from scipy.spatial.transform import Rotation

from rb20_painting_report import assess_painting, assess_plane

ROOT=Path(__file__).resolve().parents[1]
JSON_TOPICS={
    'readiness':'/painting_system/readiness',
    'execution':'/painting_system/execution_status',
    'execution_event':'/painting_system/execution_events',
    'plan':'/painting_system/plan_status',
    'planes':'/painting_system/planes',
    'catalog':'/perception/target_planes',
    'refinement':'/perception/d405_surface_refinement_status',
    'refine_execution':'/work_area_refine_status',
    'target_refinement':'/target_refine_status',
    'work_area':'/painting_system/work_area_state',
    'lifecycle':'/isaac/rb20_painting/lifecycle_status',
    'hardware':'/isaac/rb20_painting/hardware_status',
    'safety':'/painting_admittance/safety_status',
    'guard':'/painting_admittance/wrench_guard_status',
}
CAMERA_TOPICS={
    'zed_rgb':'/zed/zed_node/rgb/color/rect/image',
    'zed_depth':'/zed/zed_node/depth/depth_registered',
    'd405_rgb':'/d405/d405/color/image_raw',
    'd405_depth':'/d405/d405/depth/image_rect_raw',
}


def finalize_report(report, events, samples, *, execute=False, record_painting=False, workflow_error=None):
    if workflow_error:
        report.update(passed=False,workflow_error=workflow_error)
    if execute or record_painting:
        report['painting']=assess_painting(events,samples)
        report['passed']=report['passed'] and report['painting']['passed']
        report['scope']='painting_execution'
    return report


def status_is_current(status, received, now, state):
    return bool(state and 0<=now-received<=.20 and
                status.get('session_id')==state.session_id and
                status.get('scene_revision')==state.scene_revision)


def target_catalog_ready(latest,received,started,generation):
    """The selection header stamp is the producer's catalog generation ID."""
    catalog=latest.get('catalog',{})
    state=latest.get('planes',{})
    if (catalog.get('generation')!=generation or not catalog.get('planes')
            or received.get('catalog',0)<=started
            or state.get('generation')!=generation
            or received.get('planes',0)<=started):
        return False
    if state.get('error'):
        raise RuntimeError('Target catalog acceptance: '+str(state['error']))
    # The executor publishes candidates only after storing this catalog. Wait
    # for that explicit acknowledgement before selecting across another topic.
    return state.get('state')=='candidates' and not state.get('running')


def target_measurement_done(latest,received,started,generation,plane_id,*,require_front=True):
    state=latest.get('planes',{})
    if (state.get('generation')!=generation
            or received.get('planes',0)<=started):
        return False
    if state.get('error'):
        raise RuntimeError('D405 target selection: '+str(state['error']))
    return bool(plane_id in state.get('measured',[]) and
                state.get('active_id')==plane_id and state.get('state')=='ready'
                and not state.get('running') and (not require_front or received.get('front',0)>started))


def work_area_measurement_done(latest, received, started, area_id, previous_generation):
    refinement=latest.get('refinement',{})
    execution=latest.get('refine_execution',{})
    if received.get('refine_execution',0)>started and execution.get('state') in {
            'failed','timeout','aborted','abort_latched','busy'}:
        raise RuntimeError('Original work-area measurement failed: '+json.dumps(execution))
    return bool(refinement.get('accepted') is True and
                refinement.get('mode')=='work_area' and
                refinement.get('work_area_id')==area_id and
                refinement.get('plane_generation_id') and
                refinement['plane_generation_id']!=previous_generation and
                received.get('refinement',0)>started and
                execution.get('state')=='done' and
                received.get('refine_execution',0)>started)


class Observer:
    def __init__(self, *, remappings=(), base_frame='link0', wrench_frame='ft_link'):
        import rclpy
        from isaac_painting_msgs.msg import State, Command
        from std_msgs.msg import String
        from sensor_msgs.msg import Image,CameraInfo,JointState
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
        from controller_manager_msgs.srv import ListControllers
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy
        rclpy.init()
        cli_args=['--ros-args']
        for source,target in remappings:cli_args.extend(['-r',source+':='+target])
        self.node=rclpy.create_node('rb20_painting_check',cli_args=cli_args)
        self.base_frame=base_frame;self.wrench_frame=wrench_frame
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.latest={};self.received={};self.counts={};self.events=[];self.samples=[];self.images={};self.publishers={};self.pose_errors=[];self.pending_poses=[]
        self.plane_evidence=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # A live event stream retains rapid phases while latest status remains
        # transient-local depth one for progress and reconnect diagnostics.
        execution_history=QoSProfile(depth=100,durability=DurabilityPolicy.VOLATILE,
                                     reliability=ReliabilityPolicy.RELIABLE)
        for key,topic in JSON_TOPICS.items():
            qos=(execution_history if key=='execution_event' else
                 10 if key in {'safety','guard','refine_execution'} else latched)
            self.node.create_subscription(String,topic,lambda msg,k=key:self.json_message(k,msg),qos)
        self.node.create_subscription(State,'/isaac/rb20_painting/state',self.state_message,qos_profile_sensor_data)
        self.node.create_subscription(Command,'/isaac/rb20_painting/command',lambda m:self.store('command',m),1)
        self.node.create_subscription(JointState,'/joint_states',lambda m:self.store('joints',m),10)
        self.node.create_subscription(PoseStamped,'/isaac/rb20_painting/measured_ft_pose',self.pose_message,qos_profile_sensor_data)
        for key,topic in CAMERA_TOPICS.items():
            self.node.create_subscription(Image,topic,lambda m,k=key:self.image_message(k,m),1)
        self.node.create_subscription(Image,'/perception/wall_front_view',lambda m:self.image_message('front',m),1)
        for key,topic in [('zed_info','/zed/zed_node/rgb/color/rect/camera_info'),('d405_info','/d405/d405/color/camera_info')]:
            self.node.create_subscription(CameraInfo,topic,lambda m,k=key:self.store(k,m),1)
        self.controllers=self.node.create_client(ListControllers,'/controller_manager/list_controllers')

    def store(self,key,value):
        self.latest[key]=value;self.received[key]=time.monotonic();self.counts[key]=self.counts.get(key,0)+1

    def json_message(self,key,msg):
        try:value=json.loads(msg.data)
        except ValueError:return
        self.store(key,value)
        if key=='execution_event':
            self.events.append({**value,'time':time.monotonic()})
            print('EXECUTION '+json.dumps(value,ensure_ascii=False),flush=True)

    def image_message(self,key,msg):
        self.images[key]=msg;self.store(key,{'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'width':msg.width,'height':msg.height})

    def pose_message(self,msg):
        self.store('ft_pose',msg)
        self.pending_poses.append(msg)
        self.pending_poses=self.pending_poses[-20:]
        # RSP follows the measured joint message, so its transform for this
        # same timestamp commonly arrives after the independent PhysX pose.
        msg=self.pending_poses[0]
        try:
            tf=self.tf.lookup_transform(self.base_frame,self.wrench_frame,self.rclpy.time.Time.from_msg(msg.header.stamp))
        except Exception:return
        self.pending_poses.pop(0)
        p=msg.pose.position;q=msg.pose.orientation;t=tf.transform.translation;r=tf.transform.rotation
        position=float(np.linalg.norm([p.x-t.x,p.y-t.y,p.z-t.z]))
        angle=float(np.degrees((Rotation.from_quat([q.x,q.y,q.z,q.w]).inv()*Rotation.from_quat([r.x,r.y,r.z,r.w])).magnitude()))
        self.pose_errors.append({'position_m':position,'orientation_deg':angle})

    def state_message(self,msg):
        self.store('state',msg)
        now=time.monotonic(); safety=self.latest.get('safety',{})
        command=self.latest.get('command')
        wrench=lambda value:[value.force.x,value.force.y,value.force.z,
                             value.torque.x,value.torque.y,value.torque.z]
        self.samples.append({'time':now,'sim_time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'step':msg.step_index,'session':msg.session_id,'positions':list(msg.positions),
            'velocities':list(msg.velocities),'wrench':wrench(msg.wrench),
            'gravity_wrench':wrench(msg.gravity_wrench),
            'commanded_positions':list(command.positions) if command else None,
            'command_enabled':bool(command.enabled) if command else False,
            'command_age_s':now-self.received['command'] if command else None,
            'measured_contact_force_n':safety.get('contact_force_n'),
            'force_age_s':now-self.received.get('safety',0.),'state_age_s':0.,
            'valid':bool(msg.joints_valid and msg.wrench_valid and msg.playing and safety.get('ft_valid'))})

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.rclpy.spin_once(self.node,timeout_sec=.01)

    def wait(self,predicate,seconds=30.,label='condition'):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.rclpy.spin_once(self.node,timeout_sec=.01)
            if predicate():return
        raise RuntimeError(f'{label} timed out: '+json.dumps({k:v for k,v in self.latest.items() if k in JSON_TOPICS},ensure_ascii=False))

    def publish(self,topic,msg):
        if topic not in self.publishers:
            self.publishers[topic]=self.node.create_publisher(type(msg),topic,10)
        self.wait(lambda:self.publishers[topic].get_subscription_count()>0,10.,f'{topic} subscriber')
        required_nodes={
            # The executor also listens for target invalidation; its early
            # discovery does not mean the plane extractor received the input.
            '/target_selection_pixels':{'target_selector'},
            '/work_area_pixels':{'wall_projector','sketch_to_waypoints'},
            '/refine_work_area':{'moveit_executor','d405_surface_refiner'},
        }.get(topic,set())
        if required_nodes:
            self.wait(lambda:required_nodes.issubset({endpoint.node_name for endpoint in
                self.node.get_subscriptions_info_by_topic(topic)}),10.,f'{topic} workflow consumers')
        self.publishers[topic].publish(msg)

    def wait_healthy(self,seconds=45.):
        """Wait for an actually current complete snapshot, never extend leases."""
        end=time.monotonic()+seconds
        last_report=None
        last_error=None
        while time.monotonic()<end:
            try:
                last_report=self.report(controller_timeout_s=min(5.,end-time.monotonic()))
                last_error=None
                if last_report['passed']:
                    return last_report
            except RuntimeError as exc:
                last_error=str(exc)
            # Drain subscriptions between queries rather than flooding the
            # controller service while camera/TF discovery is still catching up.
            self.spin(max(0.,min(.10,end-time.monotonic())))
        raise RuntimeError('Pre-workflow integration health timed out: '+json.dumps({
            'checks':last_report['checks'] if last_report else {},
            'controller_query_error':last_error},ensure_ascii=False))

    def report(self,*,controller_timeout_s=5.):
        from controller_manager_msgs.srv import ListControllers
        active=[]
        if self.controllers.service_is_ready():
            future=self.controllers.call_async(ListControllers.Request())
            self.wait(future.done,controller_timeout_s,'controller query')
            active=[c.name for c in future.result().controller if c.state=='active']
        # Querying controllers spins callbacks and can take seconds. All
        # freshness/identity checks must use the final snapshot after it.
        now=time.monotonic();state=self.latest.get('state');checks={}
        checks['live_measured_state']=bool(state and state.playing and state.joints_valid and state.wrench_valid and 0<=now-self.received['state']<.2)
        for key in CAMERA_TOPICS:checks[key]=self.counts.get(key,0)>2 and 0<=now-self.received.get(key,0)<1.
        hardware=self.latest.get('hardware',{});lifecycle=self.latest.get('lifecycle',{})
        checks['hardware_not_inhibited']=(hardware.get('inhibited') is False and
            status_is_current(hardware,self.received.get('hardware',0.),now,state))
        checks['lifecycle_ok']=(lifecycle.get('ok') is True and
            status_is_current(lifecycle,self.received.get('lifecycle',0.),now,state))
        required={'joint_state_broadcaster','force_torque_sensor_broadcaster','force_torque_sensor_broadcaster_raw','admittance_controller','joint_trajectory_controller'}
        checks['controllers_active']=required.issubset(active)
        checks['physics_tf_agrees']=bool(self.pose_errors and max(p['position_m'] for p in self.pose_errors)<=.005 and
                                        max(p['orientation_deg'] for p in self.pose_errors)<=2.)
        return {'passed':all(checks.values()),'scope':'live_integration_health','checks':checks,'active_controllers':active,
                'counts':self.counts,'statuses':{k:v for k,v in self.latest.items() if k in JSON_TOPICS},
                'physics_tf_errors':self.pose_errors,
                'd405_plane':self.plane_evidence,
                'session_id':state.session_id if state else None,'scene_revision':state.scene_revision if state else None}

    def record_plane_accuracy(self,surface_x):
        status=self.latest['refinement'];plane=status['refined_plane'];stamp=plane['stamp']
        tf=self.tf.lookup_transform('link0',plane['frame_id'],
            self.rclpy.time.Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec']))
        t=tf.transform.translation;q=tf.transform.rotation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])
        point=rotation.apply(plane['position'])+np.array([t.x,t.y,t.z])
        normal=rotation.apply(Rotation.from_quat(plane['orientation']).apply([0,0,1]))
        self.plane_evidence={**assess_plane(point.tolist(),normal.tolist(),surface_x),
            'work_area_id':status['work_area_id'],'plane_generation_id':status['plane_generation_id'],
            'capture_stamp':stamp}
        if not self.plane_evidence['passed']:
            raise RuntimeError('Measured D405 plane inaccurate: '+json.dumps(self.plane_evidence))

    def close(self):
        self.node.destroy_node();self.rclpy.try_shutdown()


def manual_strokes(width, height, inset=.25):
    """Two opposite pixel strokes strictly inside the 10–90% work-area box."""
    if not math.isfinite(inset) or not .1 < inset < .5:
        raise ValueError('stroke inset must be finite and strictly between .1 and .5')
    return [[(.35*width,inset*height),(.35*width,(1-inset)*height)],
            [(.65*width,(1-inset)*height),(.65*width,inset*height)]]


def run_workflow(obs, mode, execute=False, *, scene=None, selection_polygon=None, plane_accuracy=None, workflow_time_scale=1., execution_wait=None, stroke_inset_fraction=.25, experiment_geometry=None):
    """Use the same pixel/selection/Run interfaces as the original browser.

    The fixture is only a user-input polygon projected into the rendered ZED.
    Plane geometry, D405 acceptance and trajectories come from the original nodes.
    """
    if not math.isfinite(workflow_time_scale) or workflow_time_scale<=0:
        raise ValueError('workflow time scale must be finite and positive')
    manual_strokes(1,1,stroke_inset_fraction)  # Reject invalid input before ROS or motion.
    from geometry_msgs.msg import PoseArray,Pose
    from std_msgs.msg import String,Bool,Empty
    if scene is None:scene=json.loads((ROOT/'.runtime/rb20_painting/assets/scene.json').read_text())
    obs.wait_healthy(45.)
    obs.wait(lambda:bool(obs.latest.get('readiness')),10.,'original runtime readiness discovery')
    obs.wait(lambda:'zed_info' in obs.latest and obs.counts.get('zed_depth',0)>3,20.,'rendered ZED')
    def pixels(topic,frame,strokes):
        msg=PoseArray();msg.header.frame_id=frame;msg.header.stamp=obs.node.get_clock().now().to_msg()
        for index,stroke in enumerate(strokes):
            for u,v in stroke:
                p=Pose();p.position.x=float(u);p.position.y=float(v);p.position.z=float(index);p.orientation.w=1.;msg.poses.append(p)
        obs.publish(topic,msg)
        return str(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
    k=np.array(obs.latest['zed_info'].k).reshape(3,3)
    optical_from_world=np.linalg.inv(np.array(scene['zed_world_optical']))
    polygon=selection_polygon
    if polygon is None:
        polygon=[]
        for y,z in [(-.45,.55),(.45,.55),(.45,1.65),(-.45,1.65)]:
            point=optical_from_world@np.array([scene['wall']['surface_x'],y,z,1.])
            pixel=k@point[:3];polygon.append((pixel[0]/pixel[2],pixel[1]/pixel[2]))
    target_started=time.monotonic()
    requested_generation=pixels('/target_selection_pixels','zed_raw',[polygon])
    obs.wait(lambda:target_catalog_ready(obs.latest,obs.received,target_started,requested_generation),
             20.,'request-bound target catalog and executor acknowledgement')
    catalog=obs.latest['catalog'];plane=max(catalog['planes'],key=lambda p:np.linalg.norm(np.cross(np.array(p['corners'])[1]-p['corners'][0],np.array(p['corners'])[3]-p['corners'][0])))
    selection_started=time.monotonic()
    obs.publish('/painting_system/select_planes',String(data=json.dumps({'generation':catalog['generation'],'ids':[plane['id']]})))
    def target_measured():
        return target_measurement_done(obs.latest,obs.received,selection_started,
                                       requested_generation,plane['id'],require_front=experiment_geometry is None)
    obs.wait(target_measured,120.*workflow_time_scale,'original D405 target measurement')
    if mode=='select-only':return
    previous_area=obs.latest.get('work_area',{}).get('work_area_id')
    if experiment_geometry:
        area_frame,area_polygon=experiment_geometry('area_request',obs,0,0)
    else:
        front=obs.latest['front'];w,h=front['width'],front['height']
        area_frame='wall_front'
        area_polygon=[(.10*w,.10*h),(.90*w,.10*h),(.90*w,.90*h),(.10*w,.90*h)]
    pixels('/work_area_pixels',area_frame,[area_polygon])
    obs.wait(lambda:bool(obs.latest.get('work_area',{}).get('selected')) and
                   bool(obs.latest['work_area'].get('work_area_id')) and
                   obs.latest['work_area']['work_area_id']!=previous_area,10.,'new original work area')
    area_id=obs.latest['work_area']['work_area_id']
    if experiment_geometry:
        # Convert the raw-camera selection to its rectified full-view rectangle
        # before refinement, so projector and path generator share one selection.
        full_area=experiment_geometry('front_area',obs,0,0)
        pixels('/work_area_pixels','wall_front',[full_area])
        obs.wait(lambda:obs.latest.get('work_area',{}).get('selected') and
                 obs.latest['work_area'].get('work_area_id')!=area_id,10.,'rectified work-area selection')
        area_id=obs.latest['work_area']['work_area_id']
    previous_generation=obs.latest.get('refinement',{}).get('plane_generation_id')
    refine_started=time.monotonic()
    obs.publish('/refine_work_area',Bool(data=True))
    # Sensor acceptance precedes the executor releasing its prescan lock.
    # Follow the UI's executor completion status before submitting a path.
    obs.wait(lambda:work_area_measurement_done(obs.latest,obs.received,refine_started,
                                             area_id,previous_generation),
             120.*workflow_time_scale,'original D405 work-area measurement')
    generation=obs.latest['refinement']['plane_generation_id']
    if plane_accuracy is None:obs.record_plane_accuracy(scene['wall']['surface_x'])
    else:plane_accuracy()
    previous_path=obs.latest.get('plan',{}).get('path_id')
    if mode=='fill':obs.publish('/fill_work_area',Empty())
    else:
        if experiment_geometry:
            front=obs.latest['front'];w,h=front['width'],front['height']
        path=(experiment_geometry('strokes',obs,w,h) if experiment_geometry else manual_strokes(w,h,stroke_inset_fraction))
        pixels('/sketch_pixels','wall_front',path)
    obs.wait(lambda:bool(obs.latest.get('plan',{}).get('path_id')) and
                   obs.latest['plan']['path_id']!=previous_path and bool(obs.latest['plan'].get('plan_hash')) and
                   obs.latest['plan'].get('work_area_id')==area_id and
                   obs.latest['plan'].get('plane_generation_id')==generation,60.,'new original path')
    plan=obs.latest['plan']
    identity={key:plan[key] for key in ('path_id','plan_hash','work_area_id','plane_generation_id')}
    def ready_plan():
        status=obs.latest.get('readiness',{})
        return (status.get('checks',{}).get('current_plan_validated') is True and
                all(status.get(key)==value for key,value in identity.items()))
    obs.wait(ready_plan,60.,'original path validation')
    if execute:
        obs.publish('/painting_admittance/free_space_confirmed',Bool(data=True))
        obs.wait(lambda:ready_plan() and obs.latest['readiness'].get('ready') is True,20.,'original Run readiness')
        event_start=len(obs.events)
        obs.publish('/sketch_execute',Bool(data=True))
        def completed():
            for event in obs.events[event_start:]:
                if event.get('state') in {'ABORT','FAILED','NOT_READY','PLAN_REJECTED','PRECONTACT_TARE_FAILED'}:
                    raise RuntimeError('Original Run failed: '+json.dumps(event,ensure_ascii=False))
                if event.get('state')=='COMPLETE' and all(event.get(key)==identity[key]
                        for key in ('plan_hash','work_area_id','plane_generation_id')):return True
            return False
        if execution_wait is None:obs.wait(completed,300.*workflow_time_scale,'original painting completion')
        else:execution_wait(obs,identity,event_start,300.*workflow_time_scale)
    return identity


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--record-painting',action='store_true',help='Require two complete strokes from the original executor')
    parser.add_argument('--workflow',choices=['select-only','manual','fill'],help='Request the original UI workflow with a wall pixel fixture')
    parser.add_argument('--execute',action='store_true',help='Run the validated simulation painting plan (requires --workflow)')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/rb20-painting/integration-report.json')
    args=parser.parse_args()
    if args.execute and not args.workflow:parser.error('--execute requires --workflow')
    if args.execute and args.workflow=='select-only':parser.error('--execute requires manual or fill workflow')
    if os.environ.get('ROS_DOMAIN_ID')!='106':raise RuntimeError('Source scripts/rb20_painting_ros_env.sh (domain106) first')
    ready=json.loads((ROOT/'.runtime/rb20_painting/managed-ready.json').read_text())
    if time.monotonic()-ready['updated_monotonic']>2:raise RuntimeError('No live RB20 simulator')
    obs=Observer()
    try:
        workflow_error=None
        if args.workflow:
            try:run_workflow(obs,args.workflow,args.execute)
            except Exception as exc:
                workflow_error=str(exc)
                from std_msgs.msg import Bool
                try:obs.publish('/motion_abort',Bool(data=True));obs.spin(.5)
                except RuntimeError:pass  # Preserve the original workflow failure if the graph has exited.
        obs.spin(args.duration);report=obs.report()
        if report['session_id']!=ready['session_id']:raise RuntimeError('Simulator session changed during check')
        report=finalize_report(report,obs.events,obs.samples,execute=args.execute,
                               record_painting=args.record_painting,workflow_error=workflow_error)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps({'events':obs.events,'samples':obs.samples},indent=2)+'\n')
        with args.output.with_suffix('.samples.csv').open('w',newline='') as stream:
            fields=['time','sim_time','step','session','measured_contact_force_n','force_age_s','state_age_s','valid']
            writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(obs.samples)
        print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
        return 0 if report['passed'] else 1
    finally:obs.close()

if __name__=='__main__':raise SystemExit(main())
