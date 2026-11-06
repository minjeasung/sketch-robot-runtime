#!/usr/bin/env python3
"""Simulation-only public spray pixel workflow and measured nozzle-path probe.

Default is read-only. --execute permits target/work-area/sketch/Run requests.
The CAD face supplies a visible ROI; ZED extraction supplies the actual plane.
Requires live spray_motion_test=true: no gun output or coating quality is tested.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from check_platform_motion import validate_probe_context
from platform_weld_fixture import triangles,occluded
from platform_visibility import robot_occluders,patch_visibility


def authorize(topic,execute):
    if execute is not True:raise ValueError('--execute required for public workflow requests: '+topic)


WORKFLOW_CONSUMERS={
    '/target_selection_pixels':{'target_selector','moveit_executor'},
    '/painting_system/select_planes':{'moveit_executor'},
    '/painting_system/zed_work_area_request':{'wall_projector'},
    '/work_area_pixels':{'wall_projector','sketch_to_waypoints'},
    '/painting_system/zed_sketch_request':{'sketch_to_waypoints'},
    '/sketch_execute':{'moveit_executor'},
}


def workflow_consumers_ready(topic,discovered,matched):
    required=WORKFLOW_CONSUMERS[topic]
    return matched>=len(required) and required<=set(discovered)


def cad_fixture(geometry,world_optical,k,width,height,prim=None,robot_geometry=None):
    if geometry.get('frame_id')!='World':raise ValueError('H-steel CAD World geometry required')
    transform=np.asarray(world_optical,dtype=float);inverse=np.linalg.inv(transform);view=transform[:3,3]
    k=np.asarray(k,dtype=float).reshape(3,3);all_tri=np.concatenate([triangles(m)[2] for m in geometry['meshes']]);candidates=[]
    for mesh in geometry['meshes']:
        if '/HSteel/' not in mesh['prim'] or (prim and mesh['prim']!=prim):continue
        if 'IfcFastener' in mesh['prim']:continue
        _,_,tri=triangles(mesh);cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);areas=np.linalg.norm(cross,axis=1)*.5
        groups=[]
        for index,triangle in enumerate(tri):
            if areas[index]<1e-12:continue
            normal=cross[index]/(2*areas[index])
            for group in groups:
                first=group[0];n=cross[first]/(2*areas[first])
                if np.dot(normal,n)>1.-1e-8 and np.max(abs((triangle-tri[first,0])@n))<1e-6:
                    group.append(index);break
            else:groups.append([index])
        for group in groups:
            normal=cross[group[0]]/(2*areas[group[0]]);points=tri[group].reshape(-1,3)
            edges=np.concatenate([tri[group,1]-tri[group,0],tri[group,2]-tri[group,1],tri[group,0]-tri[group,2]])
            # Try source CAD edge axes; only an area-complete rectangle qualifies.
            rectangle=None
            for edge in sorted(edges,key=lambda e:np.linalg.norm(e),reverse=True):
                u=edge/np.linalg.norm(edge);v=np.cross(normal,u);origin=points[0]
                x=(points-origin)@u;y=(points-origin)@v;dx=x.max()-x.min();dy=y.max()-y.min()
                area=float(areas[group].sum())
                if min(dx,dy)<.06 or abs(area-dx*dy)>max(1e-8,area*.001):continue
                if any(np.min(np.hypot(x-a,y-b))>2e-5 for a in (x.min(),x.max()) for b in (y.min(),y.max())):continue
                center=origin+u*(x.min()+x.max())*.5+v*(y.min()+y.max())*.5
                rectangle=(center,u,v,min(.18,dx*.4),min(.18,dy*.4),area,dx,dy);break
            if rectangle is None:continue
            midpoint,u,v,half_u,half_v,area,dx,dy=rectangle
            for sx,sy in ((0,0),(-.8,0),(.8,0),(0,-.8),(0,.8),(-.8,-.8),(-.8,.8),(.8,-.8),(.8,.8)):
                center=midpoint+u*sx*(dx/2-half_u)+v*sy*(dy/2-half_v)
                if np.dot(view-center,normal)<=.02:continue
                corners=[center+a*half_u*u+b*half_v*v for a,b in ((-1,-1),(1,-1),(1,1),(-1,1))]
                if any(occluded(point+normal*1e-6,view,all_tri) for point in [center,*corners]):continue
                pixels=[]
                for point in corners:
                    camera=(inverse@np.r_[point,1.])[:3]
                    if camera[2]<=0:break
                    image=k@camera;x,y=image[:2]/image[2]
                    if not (2<=x<width-2 and 2<=y<height-2):break
                    pixels.append([float(x),float(y)])
                if len(pixels)!=4:continue
                pixel_array=np.asarray(pixels);pixel_area=abs(np.dot(pixel_array[:,0],np.roll(pixel_array[:,1],1))-np.dot(pixel_array[:,1],np.roll(pixel_array[:,0],1)))*.5
                if pixel_area<1600:continue
                visibility=patch_visibility(corners,view,robot_geometry) if robot_geometry is not None else None
                if visibility is not None and not visibility['clear']:continue
                candidates.append((pixel_area,area,dict(source='simulation_cad_fixture',recognition=False,source_prim=mesh['prim'],triangle_indices=group,
                    point=center.tolist(),normal=normal.tolist(),corners=[p.tolist() for p in corners],pixels=pixels,
                    robot_visibility=visibility,
                    geometry_sha256=hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest())))
    if not candidates:raise ValueError('no supported visible exposed H-steel CAD face in camera grid')
    return max(candidates,key=lambda item:item[:2])[2]


def ready_plan(status,plan,age):
    keys=('path_id','plan_hash','work_area_id','plane_generation_id')
    return bool(0<=age<=1. and status.get('ready') is True and status.get('process_mode')=='spray'
        and status.get('checks',{}).get('current_plan_validated') is True
        and all(plan.get(k) and status.get(k)==plan[k] for k in keys))


def receive_json(key,payload,seen,received,events,diagnostics,now):
    """Invalidate cached status on clear/invalid messages; never reuse old readiness."""
    def invalid_constant(value):
        raise ValueError('nonfinite JSON value: '+value)
    try:
        data=json.loads(payload,parse_constant=invalid_constant)
        if not isinstance(data,dict):raise ValueError('JSON object required')
    except (ValueError,TypeError) as exc:
        seen[key]={};received.pop(key,None)
        diagnostics.append(dict(key=key,kind='cleared' if key=='segments' and not payload.strip() else 'invalid',
                                received_monotonic=now,detail=str(exc),payload=str(payload)[:160]))
        del diagnostics[:-100]
        return False
    seen[key]=data;received[key]=now
    if key=='event':events.append(data)
    return True


def plan_rejection(seen,received,started):
    status=seen.get('plan',{})
    if received.get('plan',0.)>=started and status.get('state') in ('rejected','failed'):
        return 'spray plan rejected: '+json.dumps(status)
    return ''


def tracking(plan,feedback):
    if plan.get('process_mode')!='spray' or plan.get('point_semantics')!='surface_point' or plan.get('spray_tool_axis')!='+z':raise ValueError('native +z surface-point spray plan required')
    rows=[row for row in plan['rows'] if row['mode']=='SPRAY']
    if len(rows)<2 or len(feedback)<3:return dict(passed=False,reason='insufficient measured spray path')
    normals=np.array([[r['nx'],r['ny'],r['nz']] for r in rows]);points=np.array([[r['x'],r['y'],r['z']] for r in rows])+normals*np.array([r['offset_m'] for r in rows])[:,None]
    offsets=np.asarray(plan['spray_endpoint_tcp_m']);measured=np.array([s['xyz'] for s in feedback]);rotations=Rotation.from_quat([s['quaternion'] for s in feedback])
    stamps=np.array([s['stamp'] for s in feedback])
    if offsets.shape!=(3,) or not np.isfinite(points).all() or not np.isfinite(measured).all() or not np.isfinite(offsets).all() or not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0):raise ValueError('invalid measured path evidence')
    endpoints=measured+rotations.apply(np.broadcast_to(offsets,measured.shape));axis=rotations.apply(np.tile([0.,0.,1.],(len(feedback),1)))
    starts=points[:-1];edges=np.diff(points,axis=0);lengths=np.linalg.norm(edges,axis=1);valid=lengths>1e-8
    starts=starts[valid];edges=edges[valid];lengths=lengths[valid];ns=normals[:-1][valid]
    if not len(edges):raise ValueError('nonzero spray path required')
    progress=[];errors=[];angles=[];cumulative=np.r_[0.,np.cumsum(lengths)]
    for point,direction in zip(endpoints,axis):
        fraction=np.clip(np.einsum('ij,ij->i',point-starts,edges)/(lengths**2),0.,1.)
        distances=np.linalg.norm(point-(starts+edges*fraction[:,None]),axis=1);index=int(np.argmin(distances))
        errors.append(float(distances[index]));progress.append(float(cumulative[index]+fraction[index]*lengths[index]))
        angles.append(math.degrees(math.acos(float(np.clip(np.dot(direction,-ns[index]),-1.,1.)))))
    coverage=(max(progress)-min(progress))/cumulative[-1]
    return dict(passed=bool(max(errors)<=.01 and max(angles)<=5. and coverage>=.90 and np.max(np.diff(stamps))<=.2),max_endpoint_error_m=max(errors),max_axis_error_deg=max(angles),coverage=coverage,
        samples=len(feedback),max_sample_gap_sim_s=float(np.max(np.diff(stamps))),simulation_duration_s=float(stamps[-1]-stamps[0]),feedback_source='measured_joint_state_TF',endpoint_offset_applied_once=offsets.tolist())


def run(args,report):
    import rclpy
    from std_msgs.msg import String,Bool
    from geometry_msgs.msg import PoseArray,Pose
    from sensor_msgs.msg import CameraInfo,JointState
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetStateValidity
    from tf2_ros import Buffer,TransformListener
    from check_platform_weld import validate_live_description
    context=validate_probe_context(args.assets);bundle=context['bundle'];session=context['session_id']
    if bundle['selection']['process']!='spray':raise ValueError('selected process must be spray')
    arm=bundle['arms'][bundle['selection']['active_arm']]
    if arm['base_frame'] not in ('World','link0'):raise ValueError('unsupported generated root frame')
    model=ET.parse(args.assets/'robot.urdf').getroot()
    if any(j.find('child').get('link')==arm['base_frame'] for j in model.findall('joint')):raise ValueError('CAD World requires the colocated generated root frame')
    scene=json.loads((args.assets/'scene.json').read_text());geometry=json.loads((args.assets/'h_steel_collision.json').read_text())
    report.update(selection=bundle['selection'],session_id=session,scene_revision=bundle['revision'],source='simulation_cad_roi_and_rendered_rgbd',spray_output='disabled_motion_test')
    rclpy.init();node=rclpy.create_node('platform_spray_probe');node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node);seen={};received={};events=[];diagnostics=[];feedback=[];joints=[];publishers={};mutated=False
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics={'readiness':'/painting_system/readiness','plan':'/painting_system/plan_status','catalog':'/perception/target_planes','planes':'/painting_system/planes','lock':'/perception/zed_target_lock','surface':'/perception/zed_surface_status','segments':'/sketch_eoat_segments','event':'/painting_system/execution_events','frame':'/perception/zed_frame_identity'}
    def receive(key,msg):
        receive_json(key,msg.data,seen,received,events,diagnostics,time.monotonic())
    for key,topic in topics.items():node.create_subscription(String,topic,lambda msg,key=key:receive(key,msg),100 if key=='event' else 10 if key in ('segments','frame') else latched)
    def store(key,msg):seen[key]=msg;received[key]=time.monotonic()
    node.create_subscription(CameraInfo,'/zed/zed_node/rgb/color/rect/camera_info',lambda msg:store('info',msg),10)
    def joint(msg):
        store('joints',msg);joints.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,positions=dict(zip(msg.name,msg.position))))
    node.create_subscription(JointState,'/joint_states',joint,10)
    abort=node.create_publisher(Bool,'/motion_abort',10)
    def guard():
        current=validate_probe_context(args.assets)
        if current['session_id']!=session or current['bundle']['revision']!=bundle['revision'] or current['bundle']['selection']!=bundle['selection']:raise RuntimeError('simulation identity changed')
    def wait(predicate,seconds=30.,label='condition'):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01);guard()
            try:
                transform=tf.lookup_transform(arm['base_frame'],arm['tcp_link'],rclpy.time.Time());p=transform.transform
                stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                if stamp>0 and (not feedback or stamp>feedback[-1]['stamp']):feedback.append(dict(stamp=stamp,xyz=[p.translation.x,p.translation.y,p.translation.z],quaternion=[p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w]))
            except Exception:pass
            if predicate():return
        raise TimeoutError(label+': '+json.dumps({k:v for k,v in seen.items() if k in topics},default=str))
    def send(topic,msg):
        nonlocal mutated
        authorize(topic,args.execute);guard()
        if topic not in publishers:publishers[topic]=node.create_publisher(type(msg),topic,10)
        def consumers():return {endpoint.node_name for endpoint in node.get_subscriptions_info_by_topic(topic)}
        wait(lambda:workflow_consumers_ready(topic,consumers(),publishers[topic].get_subscription_count()),10.,topic+' intended workflow consumers')
        report.setdefault('workflow_subscribers',{})[topic]=sorted(consumers())
        mutated=True;publishers[topic].publish(msg)
    def query(name,parameters):
        client=node.create_client(GetParameters,name+'/get_parameters');wait(client.service_is_ready,20.,name+' parameters')
        future=client.call_async(GetParameters.Request(names=parameters));wait(future.done,10.,name+' parameter result');return future.result().values
    def stamp():
        value=node.get_clock().now().to_msg();return {'sec':value.sec,'nanosec':value.nanosec}
    def pixel_message(frame,points,when):
        msg=PoseArray();msg.header.frame_id=frame;msg.header.stamp.sec=when['sec'];msg.header.stamp.nanosec=when['nanosec']
        for x,y in points:
            pose=Pose();pose.position.x=float(x);pose.position.y=float(y);pose.orientation.w=1.;msg.poses.append(pose)
        return msg
    try:
        wait(lambda:'info' in seen and 'frame' in seen and 'joints' in seen and 'readiness' in seen,45.,'live camera/joint/readiness discovery')
        validate_live_description((args.assets/'robot.urdf').read_text(),query('/move_group',['robot_description'])[0].string_value)
        params=query('/moveit_executor',['spray_motion_test','process_mode','dry_run'])
        if not params[0].bool_value or params[1].string_value!='spray' or params[2].bool_value:raise RuntimeError('live output-disabled spray executor required')
        frame=seen['frame']
        if frame.get('generation')!=session or frame.get('source')!='simulation' or frame.get('valid') is not True or time.monotonic()-received['frame']>1.:raise RuntimeError('fresh current simulated camera identity required')
        if seen['readiness'].get('running') or seen['readiness'].get('stopping'):raise RuntimeError('another motion or cancellation is active')
        positions=dict(zip(seen['joints'].name,seen['joints'].position));names=bundle['joint_names']
        if not all(n in positions and math.isfinite(positions[n]) for n in names):raise RuntimeError('full measured model state unavailable')
        initial=dict(positions);validity=node.create_client(GetStateValidity,'/check_state_validity');wait(validity.service_is_ready,20.)
        request=GetStateValidity.Request();request.group_name='';request.robot_state.joint_state=JointState(name=names,position=[positions[n] for n in names])
        future=validity.call_async(request);wait(future.done,10.);result=future.result();report['initial_state_validity']=dict(valid=result.valid,contacts=[dict(body_1=c.contact_body_1,body_2=c.contact_body_2,depth=c.depth) for c in result.contacts])
        if not result.valid:raise RuntimeError('full model initially collides')
        info=seen['info'];visibility_stamp=seen['joints'].header.stamp
        visibility_stamp_ns=visibility_stamp.sec*1_000_000_000+visibility_stamp.nanosec
        occluders=robot_occluders(model,positions)
        fixture=cad_fixture(geometry,scene['zed_world_optical'],info.k,info.width,info.height,args.cad_prim,robot_geometry=occluders);report['fixture']=fixture
        report['robot_visibility_basis']=dict(joint_stamp_ns=visibility_stamp_ns,positions=positions,geometry='generated_URDF_visual_and_collision',shape_count=len(occluders))
        # CPU mesh work must not allow queued feedback to authorize mutation.
        # Resume subscriptions and require a new, fresh unchanged measured pose.
        def fresh_visibility_pose():
            current=seen['joints'];stamp=current.header.stamp
            now_ns=node.get_clock().now().nanoseconds
            age=(now_ns-stamp.sec*1_000_000_000-stamp.nanosec)*1e-9
            frame_stamp=seen.get('frame',{}).get('stamp',{})
            frame_age=(now_ns-frame_stamp.get('sec',0)*1_000_000_000-frame_stamp.get('nanosec',0))*1e-9
            return (stamp.sec*1_000_000_000+stamp.nanosec>visibility_stamp_ns and 0<=age<.2
                    and 0<=frame_age<.2 and time.monotonic()-received.get('frame',0.)<.2)
        wait(fresh_visibility_pose,10.,'fresh measured pose after visibility check')
        current=dict(zip(seen['joints'].name,seen['joints'].position))
        if any(name not in current or not math.isfinite(current[name]) or abs(current[name]-value)>.005 for name,value in positions.items()):raise RuntimeError('robot moved during ROI visibility check')
        if not args.execute:report.update(passed=True,scope='read_only_spray_cad_roi');return
        when=stamp();generation=str(when['sec']*1_000_000_000+when['nanosec'])
        send('/target_selection_pixels',pixel_message('zed_raw',fixture['pixels'],when))
        wait(lambda:seen.get('catalog',{}).get('generation')==generation and bool(seen['catalog'].get('planes')) and seen.get('planes',{}).get('generation')==generation,30.,'request-bound ZED extraction')
        catalog=seen['catalog'];world=np.asarray(scene['zed_world_optical']);matches=[]
        for plane in catalog['planes']:
            center=world[:3,:3]@np.asarray(plane['center'])+world[:3,3];normal=world[:3,:3]@np.asarray(plane['normal'])
            distance=abs(float(np.dot(center-fixture['point'],fixture['normal'])));angle=math.degrees(math.acos(float(np.clip(np.dot(normal,fixture['normal']),-1,1))))
            if distance<=.01 and angle<=5.:matches.append((distance,plane))
        if not matches:raise RuntimeError('extracted ZED plane disagrees with exposed CAD face')
        plane=min(matches,key=lambda item:item[0])[1]
        send('/painting_system/select_planes',String(data=json.dumps(dict(generation=generation,ids=[plane['id']]))))
        wait(lambda:seen.get('lock',{}).get('accepted') is True and seen['lock'].get('catalog_generation')==generation and seen['lock'].get('plane_id')==plane['id'] and seen.get('surface',{}).get('plane_generation_id')==seen['lock'].get('plane_generation_id') and seen['surface'].get('view_width',0)>0,30.,'locked ZED front view')
        lock=seen['lock'];surface=seen['surface'];w,h=surface['view_width'],surface['view_height'];when=stamp();boundary=[[.2*w,.2*h],[.8*w,.2*h],[.8*w,.8*h],[.2*w,.8*h]]
        area_request=dict(source='zed',plane_generation_id=lock['plane_generation_id'],header=dict(frame_id='wall_front',stamp=when),pixels=boundary)
        send('/painting_system/zed_work_area_request',String(data=json.dumps(area_request)))
        send('/work_area_pixels',pixel_message('wall_front',boundary,when))
        selection=str(when['sec']*1_000_000_000+when['nanosec'])
        wait(lambda:seen.get('surface',{}).get('accepted') is True and seen['surface'].get('mode')=='work_area' and seen['surface'].get('selection_id')==selection and seen['surface'].get('plane_generation_id')==lock['plane_generation_id'],30.,'request-bound ZED work area')
        area=seen['surface'];request={k:area[k] for k in ('work_area_id','selection_id','plane_generation_id')};request.update(source='zed',header=dict(frame_id='wall_front',stamp=stamp()),strokes=[[[.35*w,.5*h],[.65*w,.5*h]]])
        plan_started=time.monotonic()
        send('/painting_system/zed_sketch_request',String(data=json.dumps(request)))
        def validated_plan():
            rejection=plan_rejection(seen,received,plan_started)
            if rejection:raise RuntimeError(rejection)
            return seen.get('segments',{}).get('work_area_id')==area['work_area_id'] and seen['segments'].get('plane_generation_id')==area['plane_generation_id'] and ready_plan(seen.get('readiness',{}),seen['segments'],time.monotonic()-received.get('readiness',0.))
        wait(validated_plan,90.,'exact validated spray plan')
        plan=dict(seen['segments']);report['plan']=plan
        if plan['frame_id']!=arm['base_frame']:raise RuntimeError('spray plan frame mismatch')
        start=len(events);send('/sketch_execute',Bool(data=True))
        def complete():
            for event in events[start:]:
                if event.get('state') in ('ABORT','FAILED','NOT_READY','PLAN_REJECTED'):raise RuntimeError('spray workflow failed: '+json.dumps(event))
                if event.get('state')=='COMPLETE' and event.get('plan_hash')==plan['plan_hash']:return True
            return False
        wait(complete,args.timeout,'public spray execution completion')
        relevant=[event for event in events[start:] if event.get('plan_hash')==plan['plan_hash']]
        windows=[]
        for index,event in enumerate(relevant[:-1]):
            if event.get('state')=='SPRAY':windows.append((event['timestamp_ns']*1e-9,relevant[index+1]['timestamp_ns']*1e-9))
        measured=[sample for sample in feedback if any(a<=sample['stamp']<=b for a,b in windows)]
        report['tracking']=tracking(plan,measured)
        inactive=set(names)-set(arm['joints']);drift=max((abs(sample['positions'][name]-initial[name]) for sample in joints for name in inactive if name in sample['positions']),default=0.)
        report.update(inactive_max_drift_rad=drift,workflow_executed=True,passed=report['tracking']['passed'] and drift<=.005,scope='output_disabled_spray_public_workflow')
    finally:
        if mutated and not report.get('passed'):
            abort.publish(Bool(data=True))
            deadline=time.monotonic()+5.
            report['abort_requested']=True
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.03)
                status=seen.get('readiness',{})
                if status.get('running') is False and status.get('stopping') is False and time.monotonic()-received.get('readiness',0.)<.5:
                    report['abort_terminal_idle_observed']=True;break
        report['workflow_inputs_published']=mutated
        report['topic_diagnostics']=diagnostics
        report['last_statuses']={key:seen[key] for key in topics if key in seen and key!='segments'}
        args.output.with_suffix('.trace.json').write_text(json.dumps(dict(events=events,tcp_feedback=feedback,joint_samples=joints,topic_diagnostics=diagnostics),indent=2)+'\n')
        node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',required=True,type=Path);parser.add_argument('--execute',action='store_true');parser.add_argument('--cad-prim');parser.add_argument('--timeout',type=float,default=300.);parser.add_argument('--output',type=Path)
    args=parser.parse_args();args.assets=args.assets.resolve();args.output=args.output or args.assets/'spray-workflow-report.json'
    if not math.isfinite(args.timeout) or args.timeout<=0:parser.error('--timeout must be finite and positive')
    report=dict(passed=False,workflow_executed=False,kind='platform_spray_workflow')
    try:run(args,report)
    except Exception as exc:report['error']=str(exc)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
