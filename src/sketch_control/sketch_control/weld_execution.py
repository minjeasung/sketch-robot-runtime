"""Weld-only adapter within the original executor, sharing its MoveIt/controller contract."""
from dataclasses import asdict
from copy import deepcopy
from pathlib import Path
from uuid import uuid4
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .weld_contract import (validate_target,validate_catalog,load_tool_profile, WeldPlan,PlannedSegment,WeldSettings)
from .weld_session import WeldSession
from .weld_output import SimWeldOutput
from .weld_geometry import build_weld_samples,tool_outward_axes
from .process_settings import settings_snapshot
from .weld_trajectory import retime_travel


def check_cartesian_result(result):
    if result.error_code.val!=1 or result.fraction<1.-1e-9: raise ValueError('complete collision-free Cartesian path required')


def check_fk_pose(result,position,quaternion):
    if result.error_code.val!=1 or not result.pose_stamped: raise ValueError('FK unavailable')
    pose=result.pose_stamped[0].pose
    actual=np.array([pose.position.x,pose.position.y,pose.position.z])
    q=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    if (not np.isfinite(actual).all() or np.linalg.norm(actual-position)>.001 or
            (Rotation.from_quat(q).inv()*Rotation.from_quat(quaternion)).magnitude()>np.deg2rad(1)):
        raise ValueError('Cartesian path violates torch pose')


def check_start_state(start,current,tolerance=.01):
    if not start or any(n not in current or not np.isfinite(current[n]) or abs(current[n]-v)>tolerance for n,v in start):
        raise ValueError('robot start state changed; plan again')


def check_command(payload,session_id,expected,seen):
    if payload.get('server_session_id')!=session_id or payload.get('expected')!=expected: raise ValueError('stale session or revision')
    cid=payload.get('command_id')
    if not isinstance(cid,str) or not cid or len(cid)>128 or cid in seen: raise ValueError('invalid or duplicate command')
    if len(seen)>4096: raise ValueError('command capacity exceeded; reconnect runtime')
    seen.add(cid)


def validate_joint_motion(positions,times,limits,names,max_speed=.5,max_accel=1.):
    p=np.asarray(positions,float); t=np.asarray(times,float)
    if p.shape!=(len(t),len(names)) or len(t)<2 or not np.isfinite(p).all() or not np.isfinite(t).all() or np.any(np.diff(t)<=0): raise ValueError('invalid joint samples')
    if set(names)!=set(limits): raise ValueError('wrong controller joints')
    for i,n in enumerate(names):
        low,high=limits[n]
        if np.any(p[:,i]<low) or np.any(p[:,i]>high): raise ValueError('joint limit')
    delta=np.diff(p,axis=0); dt=np.diff(t)
    if np.max(abs(delta))>.25 or np.max(abs(delta/dt[:,None]))>max_speed: raise ValueError('joint jump or velocity')
    velocity=delta/dt[:,None]
    if len(velocity)>1:
        acceleration=abs(np.diff(velocity,axis=0)/((dt[1:]+dt[:-1])/2)[:,None])
        if np.max(acceleration)>max_accel:
            sample,joint=np.unravel_index(np.argmax(acceleration),acceleration.shape)
            raise ValueError(f'joint acceleration {names[joint]} at sample {sample+1}: {acceleration[sample,joint]:.4f} > {max_accel} rad/s²')


def scene_signature(scene):
    """Compare complete collision snapshots without live robot joints or timestamps."""
    from rosidl_runtime_py.convert import message_to_ordereddict
    def canonical(value):
        if isinstance(value,dict):
            return {k:canonical(v) for k,v in value.items() if k!='stamp'}
        if isinstance(value,list): return [canonical(v) for v in value]
        return value
    p=message_to_ordereddict(scene)
    snapshot={k:p[k] for k in ('world','allowed_collision_matrix','link_padding','link_scale','fixed_frame_transforms')}
    snapshot['attached']=p['robot_state']['attached_collision_objects']
    snapshot['world']['collision_objects'].sort(key=lambda o:o['id'])
    snapshot['attached'].sort(key=lambda o:o['object']['id'])
    return json.dumps(canonical(snapshot),sort_keys=True)


def tool_attachment(profile,ee_link):
    from moveit_msgs.msg import AttachedCollisionObject,CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    xyz=np.array(profile.tip_translation_m); length=float(np.linalg.norm(xyz))
    z=xyz/length; axis=np.cross([0,0,1],z); quat=np.r_[axis,1+z[2]]
    if np.linalg.norm(quat)<1e-8: quat=np.array([1.,0,0,0])
    quat=quat/np.linalg.norm(quat)
    pose=Pose(); pose.position.x,pose.position.y,pose.position.z=map(float,xyz/2)
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat)
    attached=AttachedCollisionObject(); attached.link_name=ee_link; attached.touch_links=[ee_link,ee_link.replace('_ee_point','_link_ee')]
    attached.object.header.frame_id=ee_link; attached.object.id='weld_demo_torch'; attached.object.operation=CollisionObject.ADD
    attached.object.primitives=[SolidPrimitive(type=SolidPrimitive.CYLINDER,dimensions=[length,profile.radius_m])]; attached.object.primitive_poses=[pose]
    return attached


def check_tool_attachment(scene,profile,ee_link):
    expected=tool_attachment(profile,ee_link)
    matches=[a for a in scene.robot_state.attached_collision_objects if a.object.id==expected.object.id]
    if len(matches)!=1: raise ValueError('torch collision attachment missing or duplicated')
    actual=matches[0]; obj=actual.object; want=expected.object
    if (actual.link_name!=ee_link or obj.header.frame_id!=ee_link or
            set(actual.touch_links)!=set(expected.touch_links) or len(obj.primitives)!=1 or
            len(obj.primitive_poses)!=1 or obj.meshes or obj.planes or
            obj.primitives[0].type!=want.primitives[0].type or
            not np.allclose(obj.primitives[0].dimensions,want.primitives[0].dimensions,atol=1e-7,rtol=0)):
        raise ValueError('torch collision geometry mismatch')
    def pose(p):
        return np.array([p.position.x,p.position.y,p.position.z]),Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    origin,rotation=pose(obj.pose); offset,local=pose(obj.primitive_poses[0]); wanted,wanted_rotation=pose(want.primitive_poses[0])
    if (not np.allclose(origin+rotation.apply(offset),wanted,atol=1e-7,rtol=0) or
            ((rotation*local).inv()*wanted_rotation).magnitude()>1e-7):
        raise ValueError('torch collision pose mismatch')


class WeldExecutionMixin:
    def _init_legacy_process_settings(self):
        from rcl_interfaces.srv import GetParameters
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self._settings_session=uuid4().hex; self._settings_values={}; self._settings_query_pending=False
        self._settings_pub=self.create_publisher(String,'/process_settings/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._settings_client=self.create_client(GetParameters,'/sketch_to_waypoints/get_parameters')
        def tick():
            mode=getattr(self,'process_mode','paint')
            self._settings_pub.publish(String(data=json.dumps(settings_snapshot(mode,self._settings_values,self._settings_session))))
            if self._settings_query_pending or not self._settings_client.service_is_ready(): return
            names=['spray_speed_mps','spray_standoff_m','spray_footprint_width_m','spray_overlap','default_paint_force_n']
            self._settings_query_pending=True
            def done(future):
                self._settings_query_pending=False
                try:
                    for name,value in zip(names,future.result().values):
                        if value.type==3: self._settings_values['target_force_n' if name=='default_paint_force_n' else name]=value.double_value
                except Exception: self._settings_values={}
            self._settings_client.call_async(GetParameters.Request(names=names)).add_done_callback(done)
        self.create_timer(1.,tick)

    def _init_weld_execution(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from std_msgs.msg import String,Bool
        from sensor_msgs.msg import JointState
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.srv import GetCartesianPath,GetPositionFK,GetPositionIK,GetStateValidity,ApplyPlanningScene,GetPlanningScene
        from moveit_msgs.msg import PlanningScene
        from control_msgs.action import FollowJointTrajectory
        from tf2_ros import Buffer,TransformListener
        if not self.model_id.startswith('construct_') or not self.get_parameter('use_sim_time').value:
            raise ValueError('welding currently requires Construct simulation')
        self.process_mode='weld'; self.executing=False
        self._weld_provider=self.declare_parameter('weld_input_provider','fixture').value
        if self._weld_provider not in ('fixture','external'): raise ValueError('invalid seam provider')
        path=self.declare_parameter('weld_tool_profile','').value
        self._weld=WeldSession(load_tool_profile(path,self.model_id)); self._weld_output=SimWeldOutput()
        self._weld_pending_results={}
        self._weld_command_ack=None
        self._weld_seen=set(); self._weld_joints={}; self._weld_joint_stamp=0.; self._weld_joint_arrival=0.
        self._weld_goal=None; self._weld_pending_goal=False; self._weld_cancel_sent=False; self._weld_scene_signature=None
        self._weld_head=None; self._weld_generation=None; self._weld_frame_arrival=0.; self._weld_clock=0.
        self._weld_feedback_token=None; self._weld_execution_token=None; self._weld_stationary_joints=(); self._weld_attach_pending=False
        self._weld_scene_ready=False; self._weld_frame=None; self._weld_scene_refreshing=False; self._weld_scene_dirty=True
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._weld_pubs={name:self.create_publisher(String,name,qos) for name in
            ['/welding/status','/welding/preview','/welding/output_status','/process_settings/status','/painting_system/process_mode']}
        self._weld_move=ActionClient(self,MoveGroup,'/move_action')
        self._weld_follow=ActionClient(self,FollowJointTrajectory,self.follow_joint_trajectory_action)
        self._weld_cart=self.create_client(GetCartesianPath,'/compute_cartesian_path')
        self._weld_fk=self.create_client(GetPositionFK,'/compute_fk')
        self._weld_ik=self.create_client(GetPositionIK,'/compute_ik')
        self._weld_valid=self.create_client(GetStateValidity,'/check_state_validity')
        self._weld_apply=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self._weld_get_scene=self.create_client(GetPlanningScene,'/get_planning_scene')
        self.create_subscription(JointState,'/joint_states',self._weld_on_joints,10)
        self.create_subscription(String,'/perception/zed_frame_identity',self._weld_on_frame,10)
        self.create_subscription(PlanningScene,'/monitored_planning_scene',self._weld_on_scene,10)
        for topic,op in [('/welding/target_request','target'),('/welding/seams','seams'),('/welding/selection','select'),('/welding/invalidate','invalidate'),
                         ('/process_settings/request','settings'),('/welding/plan_request','plan'),('/welding/execute','execute')]:
            self.create_subscription(String,topic,lambda msg,op=op:self._weld_message(op,msg),qos if op=='seams' else 10)
        self.create_subscription(Bool,'/motion_abort',lambda msg:self._weld_abort('operator stop') if msg.data else None,10)
        self.create_subscription(String,'/painting_system/set_process_mode',lambda msg:self._weld_publish_mode('전체 종료 후 공정을 변경하세요.'),10)
        from rclpy.clock import Clock,ClockType
        self.create_timer(.2,self._weld_tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._weld_publish_mode()

    def _weld_publish(self,topic,payload):
        from std_msgs.msg import String
        self._weld_pubs[topic].publish(String(data=json.dumps(payload,allow_nan=False)))

    def _weld_publish_mode(self,error=''):
        self._weld_publish('/painting_system/process_mode',dict(mode='weld',error=error,simulation=True,spray_motion_test=False,planning_only=False))

    def _weld_on_joints(self,msg):
        values=dict(zip(msg.name,msg.position)); now=self.get_clock().now().nanoseconds*1e-9
        if all(np.isfinite(v) for v in values.values()):
            head={n:v for n,v in values.items() if 'head' in n}
            if self._weld_head is not None and any(abs(v-self._weld_head.get(n,v))>.001 for n,v in head.items()): self._weld_invalidate('camera head moved',clear_target=True)
            if self._weld_head is None or self._weld.target is None: self._weld_head=head
            if self._weld.state in ('PLANNING','READY','RUNNING') and any(
                    n in values and abs(values[n]-v)>.001 for n,v in self._weld_stationary_joints):
                self._weld_invalidate('non-commanded robot joints moved')
            self._weld_joints.update(values); self._weld_joint_arrival=time.monotonic(); self._weld_joint_stamp=now

    def _weld_on_frame(self,msg):
        try:
            p=json.loads(msg.data); generation=p['generation']
            if self._weld_generation is not None and generation!=self._weld_generation: self._weld_invalidate('ZED stream changed',clear_target=True)
            self._weld_generation=generation; self._weld_frame=p; self._weld_frame_arrival=time.monotonic()
        except (ValueError,KeyError,TypeError): self._weld_invalidate('invalid ZED metadata',clear_target=True)

    def _weld_on_scene(self,msg):
        # Monitor messages can be diffs, and the initial full scene may predate us.
        if not msg.is_diff and scene_signature(msg)==self._weld_scene_signature: return
        if msg.is_diff and msg.robot_state.is_diff and not (msg.world.collision_objects or msg.world.octomap.octomap.id or msg.world.octomap.octomap.data or
                msg.robot_state.attached_collision_objects or msg.fixed_frame_transforms or
                msg.allowed_collision_matrix.entry_names or msg.allowed_collision_matrix.default_entry_names or
                msg.link_padding or msg.link_scale): return
        self._weld_scene_dirty=True

    async def _weld_refresh_scene(self):
        from moveit_msgs.srv import GetPlanningScene
        try:
            request=GetPlanningScene.Request(); request.components.components=1023
            result=await self._weld_wait(self._weld_get_scene.call_async(request))
            check_tool_attachment(result.scene,self._weld.profile,self.ee_link)
            signature=scene_signature(result.scene)
            if self._weld_scene_signature is not None and signature!=self._weld_scene_signature:
                self._weld.scene_revision=uuid4().hex; self._weld_invalidate('planning scene changed')
            self._weld_scene_signature=signature; self._weld_scene_ready=True
        except Exception as exc:
            self._weld_scene_ready=False; self._weld_scene_signature=None; self._weld_invalidate('scene unavailable: '+str(exc))
        finally:
            self._weld_scene_refreshing=False
            self._weld_publish_status()

    def _weld_invalidate(self,reason,clear_target=False):
        if self._weld.busy: self._weld_abort(reason)
        else: self._weld.invalidate(reason)
        if clear_target:
            self._weld.target=None; self._weld.catalog=None; self._weld.selection=(); self._weld.selection_revision+=1
        self._weld_publish('/welding/preview',{'invalidated':True})

    def _weld_message(self,op,msg):
        p={}
        try:
            p=json.loads(msg.data)
            if not isinstance(p,dict): raise ValueError('object required')
            if op not in ('target','seams'):
                check_command(p,self._weld.server_session_id,asdict(self._weld.identity()),self._weld_seen)
            if op=='target':
                if p.get('server_session_id')!=self._weld.server_session_id: raise ValueError('stale session')
                self._weld.request_target(validate_target(p)); self._weld_publish('/welding/preview',{'invalidated':True})
                result=self._weld_pending_results.pop(p['request_id'],None)
                if result: self._weld_accept_result(result)
            elif op=='seams':
                if not self._weld.target or p.get('request_id')!=self._weld.target.request_id:
                    if not isinstance(p.get('request_id'),str) or len(p['request_id'])>256: return
                    if len(self._weld_pending_results)>=8: self._weld_pending_results.pop(next(iter(self._weld_pending_results)))
                    self._weld_pending_results[p['request_id']]=p
                    return
                self._weld_accept_result(p)
            elif op=='invalidate': self._weld_invalidate('target edited',clear_target=True)
            elif op=='select': self._weld.select(p['selection']); self._weld_publish('/welding/preview',{'invalidated':True})
            elif op=='settings':
                self._weld_output.configure(self._weld.apply_settings(p['values'])); self._weld_publish('/welding/preview',{'invalidated':True})
            elif op=='plan':
                if self._weld.state=='PLANNING': raise ValueError('planning already in progress')
                identity=self._weld.begin_plan()
                self._weld_stationary_joints=tuple((n,v) for n,v in self._weld_joints.items() if n not in self.arm_joint_names)
                self.executor.create_task(self._weld_plan,identity)
            elif op=='execute':
                self._weld_ready_to_move(); check_start_state(self._weld.plan.start_joints if self._weld.plan else (),self._weld_joints)
                plan=self._weld.begin_execution(p['plan_id'],self._weld.identity())
                self._weld_execution_token=plan.plan_id; self.executor.create_task(self._weld_execute,plan)
            self._weld.error=''
            if p.get('command_id'): self._weld_command_ack=dict(command_id=p['command_id'],ok=True,error='')
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self._weld.error=str(exc)
            if isinstance(p,dict) and p.get('command_id'): self._weld_command_ack=dict(command_id=p['command_id'],ok=False,error=str(exc))
        self._weld_tick()

    def _weld_accept_result(self,payload):
        if payload.get('error'): raise ValueError(payload['error'])
        catalog=validate_catalog(payload,self._weld.target)
        if catalog!=self._weld.catalog: self._weld.accept_catalog(catalog)

    def _weld_ready_to_move(self):
        if (self._weld_scene_ready is not True or not self._weld_scene_signature or
                self._weld_scene_dirty or self._weld_scene_refreshing): raise ValueError('collision scene not ready')
        if not set(self.arm_joint_names)<=self._weld_joints.keys(): raise ValueError('robot joints unavailable')
        if time.monotonic()-self._weld_joint_arrival>.5 or time.monotonic()-self._weld_frame_arrival>3: raise ValueError('robot/camera feedback stale')

    def _weld_tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if now<self._weld_clock: self._weld_invalidate('simulation reset',clear_target=True)
        if self._weld.busy and (time.monotonic()-self._weld_joint_arrival>.5 or time.monotonic()-self._weld_frame_arrival>3): self._weld_abort('feedback lost')
        if self._weld_output.on and not self._weld_output.status()['on']: self._weld_abort('controller feedback lost')
        self._weld_clock=now
        if not self._weld_scene_ready and not self._weld_attach_pending and self._weld_apply.service_is_ready():
            self._weld_attach_pending=True; self.executor.create_task(self._weld_attach_tool)
        if self._weld_scene_dirty and not self._weld_scene_refreshing and self._weld_get_scene.service_is_ready():
            self._weld_scene_dirty=False; self._weld_scene_refreshing=True; self.executor.create_task(self._weld_refresh_scene)
        self._weld_publish_status()

    def _weld_publish_status(self):
        snap=self._weld.snapshot(); snap['command_ack']=self._weld_command_ack; snap['provider']=self._weld_provider
        try: self._weld_ready_to_move(); snap['runtime_ready']=True
        except ValueError: snap['runtime_ready']=False
        snap['seams']=[dict(seam_id=s.seam_id,length_m=float(np.linalg.norm(np.diff(s.points_xyz,axis=0),axis=1).sum()),
            frame_id=self._weld.catalog.frame_id,pose_ready=bool(s.pose_profile_id or s.reference_up_vectors or s.quaternions_xyzw or (self._weld.settings and self._weld.settings.pose_profile_id))) for s in self._weld.catalog.seams] if self._weld.catalog else []
        self._weld_publish('/welding/status',snap)
        self._weld_publish('/welding/output_status',self._weld_output.status())
        values=asdict(self._weld.settings or WeldSettings())
        self._weld_publish('/process_settings/status',settings_snapshot('weld',values,self._weld.server_session_id,self._weld.settings_revision,self._weld.settings is not None,profile=self._weld.profile))
        self._weld_publish_mode()

    async def _weld_scene_barrier(self):
        # A periodic monitor update may overlap a phase boundary. Let its full
        # snapshot resolve before checking readiness instead of using stale data.
        from rclpy.task import Future
        from rclpy.clock import Clock,ClockType
        settled=Future()
        def check():
            if not self._weld_scene_dirty and not self._weld_scene_refreshing and not settled.done(): settled.set_result(True)
        timer=self.create_timer(.01,check,clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            check(); await self._weld_wait(settled,5.)
            self._weld_ready_to_move()
        finally: self.destroy_timer(timer)

    async def _weld_wait(self,future,timeout=30.):
        from rclpy.task import Future
        from rclpy.clock import Clock,ClockType
        result=Future()
        def finished(source):
            if result.done(): return
            try: result.set_result(source.result())
            except Exception as exc: result.set_exception(exc)
        def expired():
            if not result.done(): result.set_exception(TimeoutError('ROS request timed out'))
        future.add_done_callback(finished)
        timer=self.create_timer(timeout,expired,clock=Clock(clock_type=ClockType.STEADY_TIME))
        try: return await result
        finally: self.destroy_timer(timer)

    async def _weld_attach_tool(self):
        from moveit_msgs.srv import ApplyPlanningScene
        try:
            attached=tool_attachment(self._weld.profile,self.ee_link)
            request=ApplyPlanningScene.Request(); request.scene.is_diff=True; request.scene.robot_state.is_diff=True
            request.scene.robot_state.attached_collision_objects=[attached]
            response=await self._weld_wait(self._weld_apply.call_async(request))
            if not response.success: raise ValueError('tool collision attachment rejected')
            self._weld_scene_dirty=True
        except Exception as exc: self._weld_scene_ready=False; self._weld.error=str(exc)
        finally: self._weld_attach_pending=False

    def _weld_pose(self,xyz,q):
        from geometry_msgs.msg import Pose
        p=Pose(); p.position.x,p.position.y,p.position.z=map(float,xyz)
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q); return p

    def _weld_robot_state(self,joints):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        return RobotState(joint_state=JointState(name=list(joints),position=list(joints.values())),is_diff=True)

    def _weld_plan_current(self,identity):
        if self._weld.state!='PLANNING' or identity!=self._weld.identity(): raise ValueError('planning inputs changed')

    async def _weld_plan(self,identity):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.srv import GetCartesianPath,GetPositionFK,GetPositionIK,GetStateValidity
        from moveit_msgs.msg import Constraints,JointConstraint
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint
        try:
            await self._weld_scene_barrier(); self._weld_plan_current(identity)
            if not self._weld_move.server_is_ready() or not all(c.service_is_ready() for c in (self._weld_cart,self._weld_fk,self._weld_ik,self._weld_valid)): raise ValueError('MoveIt services not ready')
            joints=dict(self._weld_joints); initial=tuple(sorted(joints.items())); segments=[]; previews=[]
            catalog={s.seam_id:s for s in self._weld.catalog.seams}; settings=self._weld.settings
            if self._weld.catalog.frame_id!=self.base_frame: raise ValueError('seams must be transformed to planning frame')
            for sid,reverse in self._weld.selection:
                self._weld_plan_current(identity)
                samples=build_weld_samples(catalog[sid],settings,self._weld.profile,reverse)
                normal=tool_outward_axes(samples.tcp_quat_xyzw,self._weld.profile)
                approach=samples.tcp_xyz_m[0]+normal[0]*settings.approach_m
                goal=MoveGroup.Goal(); goal.planning_options.plan_only=True
                req=goal.request; req.group_name=self.planning_group; req.start_state=self._weld_robot_state(joints)
                req.num_planning_attempts=5; req.allowed_planning_time=10.; req.max_velocity_scaling_factor=.1; req.max_acceleration_scaling_factor=.1
                ik=GetPositionIK.Request(); ik.ik_request.group_name=self.planning_group
                ik.ik_request.ik_link_name=self.ee_link; ik.ik_request.robot_state=self._weld_robot_state(joints)
                ik.ik_request.avoid_collisions=True; ik.ik_request.timeout.sec=3
                ik.ik_request.pose_stamped.header.frame_id=self.base_frame
                ik.ik_request.pose_stamped.pose=self._weld_pose(approach,samples.tcp_quat_xyzw[0])
                solved=await self._weld_wait(self._weld_ik.call_async(ik)); self._weld_plan_current(identity)
                if solved.error_code.val!=1: raise ValueError('approach has no collision-free IK solution')
                positions=dict(zip(solved.solution.joint_state.name,solved.solution.joint_state.position))
                positions={n:self._nearest_joint_equivalent(n,v,joints[n],self._robot_joint_limits) if n in self.arm_joint_names else v for n,v in positions.items()}
                req.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=positions[n],tolerance_above=.001,tolerance_below=.001,weight=1.) for n in self.arm_joint_names])]
                handle=await self._weld_wait(self._weld_move.send_goal_async(goal)); self._weld_plan_current(identity)
                if not handle.accepted: raise ValueError('approach plan rejected')
                response=await self._weld_wait(handle.get_result_async()); self._weld_plan_current(identity)
                if response.result.error_code.val!=1: raise ValueError('approach is unreachable or colliding')
                trajectory=response.result.planned_trajectory; jt=trajectory.joint_trajectory
                joints.update(zip(jt.joint_names,jt.points[-1].positions)); segments.append(PlannedSegment(sid,'TRANSFER' if segments else 'APPROACH',trajectory))
                for phase,xyz,qs,times in [
                    ('ENTRY',np.array([samples.tcp_xyz_m[0]]),np.array([samples.tcp_quat_xyzw[0]]),None),
                    ('WELD',samples.tcp_xyz_m[1:],samples.tcp_quat_xyzw[1:],samples.time_s),
                    ('RETRACT',np.array([samples.tcp_xyz_m[-1]+normal[-1]*settings.retreat_m]),np.array([samples.tcp_quat_xyzw[-1]]),None)]:
                    request=GetCartesianPath.Request(); request.header.frame_id=self.base_frame; request.group_name=self.planning_group
                    request.link_name=self.ee_link; request.start_state=self._weld_robot_state(joints); request.avoid_collisions=True
                    request.max_step=.005; request.jump_threshold=0.; request.waypoints=[self._weld_pose(x,q) for x,q in zip(xyz,qs)]
                    cart=await self._weld_wait(self._weld_cart.call_async(request)); self._weld_plan_current(identity); check_cartesian_result(cart)
                    trajectory=cart.solution; jt=trajectory.joint_trajectory
                    if times is not None:
                        # MoveIt retimes/resamples Cartesian output. Reconstruct
                        # exact requested poses/times with continuous IK seeds.
                        seed=dict(joints); points=[]
                        for i,(position,quaternion) in enumerate(zip(samples.tcp_xyz_m,samples.tcp_quat_xyzw)):
                            if i:
                                ik=GetPositionIK.Request(); ik.ik_request.group_name=self.planning_group
                                ik.ik_request.ik_link_name=self.ee_link; ik.ik_request.robot_state=self._weld_robot_state(seed)
                                ik.ik_request.avoid_collisions=True; ik.ik_request.timeout.nanosec=100000000
                                ik.ik_request.pose_stamped.header.frame_id=self.base_frame
                                ik.ik_request.pose_stamped.pose=self._weld_pose(position,quaternion)
                                solved=await self._weld_wait(self._weld_ik.call_async(ik)); self._weld_plan_current(identity)
                                if solved.error_code.val!=1: raise ValueError('weave sample has no collision-free IK')
                                values=dict(zip(solved.solution.joint_state.name,solved.solution.joint_state.position))
                                for n in jt.joint_names: seed[n]=self._nearest_joint_equivalent(n,values[n],seed[n],self._robot_joint_limits)
                            points.append(JointTrajectoryPoint(positions=[float(seed[n]) for n in jt.joint_names]))
                        jt.points=points
                    else:
                        times=retime_travel([p.positions for p in jt.points],
                            [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points])
                    try: validate_joint_motion([v.positions for v in jt.points],times,self._robot_joint_limits,jt.joint_names)
                    except ValueError as exc: raise ValueError(sid+'/'+phase+': '+str(exc)) from exc
                    for point,t in zip(jt.points,times):
                        ns=int(t*1e9); point.time_from_start=Duration(sec=ns//1000000000,nanosec=ns%1000000000)
                        point.velocities=[]; point.accelerations=[]
                    if phase=='WELD':
                        for i,point in enumerate(jt.points):
                            state=dict(joints); state.update(zip(jt.joint_names,point.positions))
                            fk=GetPositionFK.Request(); fk.header.frame_id=self.base_frame; fk.fk_link_names=[self.ee_link]; fk.robot_state=self._weld_robot_state(state)
                            actual=await self._weld_wait(self._weld_fk.call_async(fk)); self._weld_plan_current(identity)
                            check_fk_pose(actual,samples.tcp_xyz_m[i],samples.tcp_quat_xyzw[i])
                            if i:
                                midpoint=(np.array(jt.points[i-1].positions)+point.positions)/2
                                state.update(zip(jt.joint_names,midpoint)); check=GetStateValidity.Request(); check.robot_state=self._weld_robot_state(state); check.group_name=self.planning_group
                                valid=await self._weld_wait(self._weld_valid.call_async(check)); self._weld_plan_current(identity)
                                if not valid.valid: raise ValueError('interpolated trajectory collides')
                                fk.robot_state=self._weld_robot_state(state)
                                actual=await self._weld_wait(self._weld_fk.call_async(fk)); self._weld_plan_current(identity)
                                previous=Rotation.from_quat(samples.tcp_quat_xyzw[i-1]); current=Rotation.from_quat(samples.tcp_quat_xyzw[i])
                                midpoint_q=(previous*Rotation.from_rotvec((previous.inv()*current).as_rotvec()/2)).as_quat()
                                check_fk_pose(actual,(samples.tcp_xyz_m[i-1]+samples.tcp_xyz_m[i])/2,midpoint_q)
                    joints.update(zip(jt.joint_names,jt.points[-1].positions)); segments.append(PlannedSegment(sid,phase,trajectory,samples if phase=='WELD' else None))
                previews.append(dict(seam_id=sid,tip_xyz_m=samples.tip_xyz_m.tolist(),center_xyz_m=list(catalog[sid].points_xyz),
                    axes=[[samples.tip_xyz_m[i].tolist(),(samples.tip_xyz_m[i]+normal[i]*.04).tolist()] for i in range(0,len(samples.time_s),max(1,len(samples.time_s)//8))]))
            await self._weld_scene_barrier(); self._weld_plan_current(identity)
            check_start_state(initial,self._weld_joints)
            plan=WeldPlan(uuid4().hex,identity,initial,tuple(segments))
            if self._weld.accept_plan(identity,plan): self._weld_publish('/welding/preview',dict(plan_id=plan.plan_id,segments=previews))
        except Exception as exc:
            if self._weld.state=='PLANNING' and self._weld.identity()==identity: self._weld.invalidate(str(exc))
            self.get_logger().error('Weld planning: '+str(exc))

    async def _weld_execute(self,plan):
        from control_msgs.action import FollowJointTrajectory
        def owned(): return self._weld_execution_token==plan.plan_id and self._weld.plan is plan
        try:
            for segment in plan.segments:
                if not owned() or self._weld.state!='RUNNING': break
                await self._weld_scene_barrier()
                if not owned() or self._weld.state!='RUNNING': break
                jt=segment.trajectory.joint_trajectory
                check_start_state(tuple(zip(jt.joint_names,jt.points[0].positions)),self._weld_joints)
                goal=FollowJointTrajectory.Goal(trajectory=deepcopy(jt))
                self._weld_pending_goal=True
                token=uuid4().hex; self._weld_feedback_token=token
                request=self._weld_follow.send_goal_async(goal,feedback_callback=lambda msg,t=token,s=segment:self._weld_feedback(t,plan.plan_id,s,msg))
                try: handle=await self._weld_wait(request,10)
                except Exception:
                    request.add_done_callback(lambda f:self._weld_late_goal(f,plan.plan_id)); raise
                if not owned():
                    if handle.accepted: handle.cancel_goal_async()
                    return
                self._weld_pending_goal=False
                if not handle.accepted: raise ValueError('controller rejected trajectory')
                self._weld_goal=handle; self._weld_cancel_sent=False
                if self._weld.state=='STOPPING': self._weld_abort('stop pending')
                duration=jt.points[-1].time_from_start.sec+jt.points[-1].time_from_start.nanosec*1e-9
                terminal=handle.get_result_async()
                terminal.add_done_callback(lambda f,h=handle:self._weld_terminal(h,f))
                result=await self._weld_wait(terminal,duration*10+120)
                if not owned(): return
                self._weld_feedback_token=None; self._weld_goal=None; self._weld_output.stop_segment(segment.phase+' finished')
                if self._weld.state in ('STOPPING','STOPPED'):
                    if self._weld.state=='STOPPING': self._weld.confirm_stop()
                    break
                if result.status!=4 or result.result.error_code!=0: raise ValueError('trajectory failed: '+str(result.result.error_code))
                self._weld.complete_segment()
        except Exception as exc:
            if owned(): self._weld_abort(str(exc)); self._weld.error=str(exc)
        finally:
            if owned():
                self._weld_feedback_token=None; self._weld_output.stop_segment('execution ended')
                self._weld_publish('/welding/output_status',self._weld_output.status())

    def _weld_feedback(self,token,plan_id,segment,msg):
        if token!=self._weld_feedback_token or self._weld.state!='RUNNING' or segment.phase!='WELD': return
        fb=msg.feedback
        stamp=fb.header.stamp.sec+fb.header.stamp.nanosec*1e-9
        elapsed=fb.desired.time_from_start.sec+fb.desired.time_from_start.nanosec*1e-9
        if not self._weld_output.on:
            self._weld_output.start_segment(plan_id,segment.seam_id,stamp-elapsed)
            self._weld_publish('/welding/output_status',self._weld_output.status())
        else: self._weld_output.renew()

    def _weld_late_goal(self,future,execution_token):
        if execution_token!=self._weld_execution_token:
            try:
                handle=future.result()
                if handle.accepted: handle.cancel_goal_async()
            except Exception: pass
            return
        self._weld_pending_goal=False
        try:
            handle=future.result()
            if handle.accepted:
                self._weld_goal=handle; self._weld_cancel_sent=False
                handle.get_result_async().add_done_callback(lambda f:self._weld_terminal(handle,f))
                self._weld_abort('late controller acceptance cancelled')
            elif self._weld.state=='STOPPING': self._weld.confirm_stop()
        except Exception as exc:
            # Transport failure does not establish that the controller stopped.
            self._weld.error='controller stop unconfirmed: '+str(exc)

    def _weld_terminal(self,handle,future):
        if self._weld_goal is handle and self._weld.state=='STOPPING':
            try:
                if future.result().status in (4,5,6):
                    self._weld_goal=None; self._weld.confirm_stop()
            except Exception: pass

    def _weld_abort(self,reason):
        self._weld_output.stop_segment(reason); self._weld.request_stop(); self._weld.error=reason
        if self._weld_goal is not None:
            if not self._weld_cancel_sent: self._weld_goal.cancel_goal_async(); self._weld_cancel_sent=True
        elif not self._weld_pending_goal: self._weld.confirm_stop()
