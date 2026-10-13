from types import SimpleNamespace as NS
import pytest
from sketch_control.weld_execution import check_cartesian_result, check_start_state, check_command, validate_joint_motion

def test_partial_cartesian_and_collision_are_rejected():
    with pytest.raises(ValueError): check_cartesian_result(NS(fraction=.99,error_code=NS(val=1)))
    with pytest.raises(ValueError): check_cartesian_result(NS(fraction=1.,error_code=NS(val=-1)))
    check_cartesian_result(NS(fraction=1.,error_code=NS(val=1)))

def test_wrong_arm_start_state_and_scene_reject_execute():
    with pytest.raises(ValueError): check_start_state((('left',0.),),{'right':0.})
    with pytest.raises(ValueError): check_start_state((('left',0.),),{'left':.1})
    check_start_state((('left',0.),),{'left':.001})

def test_old_session_or_revision_cannot_start():
    with pytest.raises(ValueError): check_command({'server_session_id':'old','command_id':'x','expected':{'r':1}},'new',{'r':1},set())
    with pytest.raises(ValueError): check_command({'server_session_id':'s','command_id':'x','expected':{'r':0}},'s',{'r':1},set())
    seen=set(); p={'server_session_id':'s','command_id':'x','expected':{'r':1}}
    check_command(p,'s',{'r':1},seen)
    with pytest.raises(ValueError): check_command(p,'s',{'r':1},seen)

def test_joint_jump_velocity_and_acceleration_rejected():
    with pytest.raises(ValueError): validate_joint_motion([[0],[1]],[0,.01],{'j':(-2,2)},['j'])
    with pytest.raises(ValueError): validate_joint_motion([[0],[3]],[0,10],{'j':(-2,2)},['j'])
    validate_joint_motion([[0],[.01],[.02]],[0,.1,.2],{'j':(-2,2)},['j'])

def test_pending_collision_attachment_cannot_enable_motion():
    from sketch_control.weld_execution import WeldExecutionMixin
    fake=NS(_weld_scene_ready='pending',_weld_scene_signature='scene')
    with pytest.raises(ValueError,match='collision scene'):
        WeldExecutionMixin._weld_ready_to_move(fake)

@pytest.mark.parametrize('dirty,refreshing',[(True,False),(False,True)])
def test_pending_scene_refresh_blocks_motion(dirty,refreshing):
    import time
    from sketch_control.weld_execution import WeldExecutionMixin
    fake=NS(_weld_scene_ready=True,_weld_scene_signature='scene',
        _weld_scene_dirty=dirty,_weld_scene_refreshing=refreshing,
        arm_joint_names=['j'],_weld_joints={'j':0},
        _weld_joint_arrival=time.monotonic(),_weld_frame_arrival=time.monotonic())
    with pytest.raises(ValueError,match='collision scene'):
        WeldExecutionMixin._weld_ready_to_move(fake)

def test_scene_requires_matching_torch_collision_geometry():
    from copy import deepcopy
    from moveit_msgs.msg import PlanningScene
    from sketch_control.weld_execution import tool_attachment,check_tool_attachment
    from test_weld_geometry import P
    scene=PlanningScene()
    with pytest.raises(ValueError,match='torch'): check_tool_attachment(scene,P,'left_manipulator_ee_point')
    attached=tool_attachment(P,'left_manipulator_ee_point')
    scene.robot_state.attached_collision_objects=[attached]
    check_tool_attachment(scene,P,'left_manipulator_ee_point')
    for change in ('link','size','pose','touch','object_pose'):
        bad=deepcopy(scene); a=bad.robot_state.attached_collision_objects[0]
        if change=='link': a.link_name='other_arm'
        elif change=='size': a.object.primitives[0].dimensions[1]*=.5
        elif change=='pose': a.object.primitive_poses[0].position.x+=.1
        elif change=='touch': a.touch_links.append('table')
        else: a.object.pose.position.x+=.1
        with pytest.raises(ValueError,match='torch'): check_tool_attachment(bad,P,'left_manipulator_ee_point')
    # MoveIt may normalize the shape offset into CollisionObject.pose.
    scene.robot_state.attached_collision_objects[0].object.pose=deepcopy(attached.object.primitive_poses[0])
    from geometry_msgs.msg import Pose
    scene.robot_state.attached_collision_objects[0].object.primitive_poses=[Pose()]
    check_tool_attachment(scene,P,'left_manipulator_ee_point')

def test_non_commanded_arm_drift_invalidates_cached_plan():
    from sketch_control.weld_execution import WeldExecutionMixin
    from sketch_control.weld_session import WeldSession
    from test_weld_geometry import P
    calls=[]
    fake=NS(_weld=WeldSession(P),_weld_head={},_weld_joints={'left':0.,'right':0.},
        _weld_stationary_joints=(('right',0.),),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=1)),
        _weld_invalidate=lambda *a,**k:calls.append(a[0]))
    fake._weld.state='RUNNING'
    WeldExecutionMixin._weld_on_joints(fake,NS(name=['left','right'],position=[1.,.0005]))
    assert not calls
    WeldExecutionMixin._weld_on_joints(fake,NS(name=['left','right'],position=[2.,.003]))
    assert calls==['non-commanded robot joints moved']

def test_scene_signature_ignores_robot_motion_but_tracks_objects_and_attachments():
    from copy import deepcopy
    from moveit_msgs.msg import PlanningScene,CollisionObject,AttachedCollisionObject
    from sketch_control.weld_execution import scene_signature
    scene=PlanningScene(); obj=CollisionObject(id='table');scene.world.collision_objects=[obj]
    original=scene_signature(scene)
    scene.robot_state.joint_state.name=['j'];scene.robot_state.joint_state.position=[1.]
    scene.world.collision_objects[0].header.stamp.sec=5
    assert scene_signature(scene)==original
    scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name='tcp',object=CollisionObject(id='torch'))]
    assert scene_signature(scene)!=original
    changed=deepcopy(scene);changed.world.collision_objects=[]
    assert scene_signature(changed)!=scene_signature(scene)

def test_late_goal_acceptance_after_timeout_is_cancelled_and_stop_waits_for_terminal():
    import asyncio
    from sketch_control.weld_execution import WeldExecutionMixin
    from sketch_control.weld_session import WeldSession
    from sketch_control.weld_output import SimWeldOutput
    from sketch_control.weld_contract import WeldPlan,PlannedSegment
    from test_weld_geometry import P
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    async def scenario():
        pending=asyncio.get_running_loop().create_future(); terminal=asyncio.get_running_loop().create_future(); cancelled=[]
        handle=NS(accepted=True,get_result_async=lambda:terminal,cancel_goal_async=lambda:cancelled.append(True))
        class Executor(WeldExecutionMixin):
            def __init__(self):
                self._weld=WeldSession(P);self._weld.state='RUNNING';self._weld_output=SimWeldOutput();self._weld_execution_token='p'
                self._weld_goal=None;self._weld_pending_goal=False;self._weld_cancel_sent=False;self._weld_joints={'j':0.}
                self._weld_follow=NS(send_goal_async=lambda *a,**k:pending)
            async def _weld_scene_barrier(self):pass
            def _weld_publish(self,*a):pass
            async def _weld_wait(self,future,timeout=30):
                if not future.done():raise TimeoutError('delayed acceptance')
                return future.result()
        ex=Executor();tr=RobotTrajectory();tr.joint_trajectory.joint_names=['j']
        tr.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.],time_from_start=Duration(sec=1))]
        plan=WeldPlan('p',ex._weld.identity(),(('j',0.),),(PlannedSegment('s','APPROACH',tr),))
        ex._weld.plan=plan
        await ex._weld_execute(plan)
        assert ex._weld.state=='STOPPING' and not ex._weld_output.on
        pending.set_result(handle);await asyncio.sleep(0)
        assert cancelled and ex._weld.state=='STOPPING'
        terminal.set_result(NS(status=5,result=NS(error_code=0)));await asyncio.sleep(0)
        assert ex._weld.state=='STOPPED'
    asyncio.run(scenario())

def test_virtual_output_requires_feedback_of_active_weld_segment():
    from sketch_control.weld_execution import WeldExecutionMixin
    from sketch_control.weld_output import SimWeldOutput
    from sketch_control.weld_contract import PlannedSegment
    ex=NS(_weld=NS(state='RUNNING'),_weld_output=SimWeldOutput(),_weld_feedback_token='active',_weld_publish=lambda *a:None)
    msg=NS(feedback=NS(header=NS(stamp=NS(sec=20,nanosec=0)),desired=NS(time_from_start=NS(sec=2,nanosec=0))))
    segment=PlannedSegment('S1','WELD',None)
    WeldExecutionMixin._weld_feedback(ex,'old','plan',segment,msg)
    assert not ex._weld_output.on
    WeldExecutionMixin._weld_feedback(ex,'active','plan',PlannedSegment('S1','ENTRY',None),msg)
    assert not ex._weld_output.on
    WeldExecutionMixin._weld_feedback(ex,'active','plan',segment,msg)
    assert ex._weld_output.status()['start_sim_time_s']==18.
    assert ex._weld_output.on
    ex._weld.state='STOPPING';ex._weld_output.stop_segment('stop')
    WeldExecutionMixin._weld_feedback(ex,'active','plan',segment,msg)
    assert not ex._weld_output.on

def test_seam_result_arriving_before_target_is_not_lost():
    import json
    from sketch_control.weld_execution import WeldExecutionMixin
    from sketch_control.weld_session import WeldSession
    from test_weld_geometry import P
    from test_weld_contract import catalog
    class Executor(WeldExecutionMixin):
        def __init__(self):
            self._weld=WeldSession(P);self._weld_pending_results={}
        def _weld_tick(self):pass
        def _weld_publish(self,*a):pass
    ex=Executor();p=catalog();ex._weld_message('seams',NS(data=json.dumps(p)))
    target=dict(request_id=p['request_id'],target_revision=p['target_revision'],generation=p['generation'],frame_id='optical',stamp={'sec':1,'nanosec':0},clock_domain='ros',width=1280,height=720,polygon=[[0,0],[100,0],[100,100]],provider='fixture',server_session_id=ex._weld.server_session_id)
    ex._weld_message('target',NS(data=json.dumps(target)))
    assert ex._weld.catalog is not None and ex._weld.state=='SEAMS_READY'

@pytest.mark.parametrize('barrier_error',[False,True])
@pytest.mark.parametrize('new_state',['PLANNING','RUNNING'])
def test_old_execution_waiting_for_scene_cannot_dispatch_or_clean_up_new_run(barrier_error,new_state):
    import asyncio
    from sketch_control.weld_execution import WeldExecutionMixin
    from sketch_control.weld_contract import WeldPlan,PlannedSegment
    from sketch_control.weld_output import SimWeldOutput
    from test_weld_session import prepared
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    async def scenario():
        gate=asyncio.Event(); entered=asyncio.Event(); sent=[]
        class Executor(WeldExecutionMixin):
            def __init__(self):
                self._weld=prepared();self._weld.state='RUNNING';self._weld_execution_token='old'
                self._weld_output=SimWeldOutput();self._weld_goal=None;self._weld_pending_goal=False;self._weld_cancel_sent=False
                self._weld_joints={'j':0.};self._weld_follow=NS(send_goal_async=self.send)
            async def _weld_scene_barrier(self):
                entered.set();await gate.wait()
                if barrier_error: raise TimeoutError('old scene query timed out')
            async def _weld_wait(self,future,timeout=30): return await future
            def _weld_publish(self,*a):pass
            def send(self,*a,**k):
                sent.append(True);f=asyncio.get_running_loop().create_future();f.set_result(NS(accepted=False));return f
        ex=Executor();tr=RobotTrajectory();tr.joint_trajectory.joint_names=['j'];tr.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.])]
        plan=WeldPlan('old',ex._weld.identity(),(('j',0.),),(PlannedSegment('S1','APPROACH',tr),))
        ex._weld.plan=plan
        task=asyncio.create_task(ex._weld_execute(plan));await entered.wait()
        ex._weld_abort('stop');assert ex._weld.state=='STOPPED'
        ex._weld.state=new_state;ex._weld_feedback_token='new-feedback'
        if new_state=='RUNNING':
            ex._weld_execution_token='new';ex._weld_output.start_segment('new','S3')
        gate.set();await task
        assert not sent and ex._weld.state==new_state
        assert ex._weld_output.on==(new_state=='RUNNING') and ex._weld_feedback_token=='new-feedback'
    asyncio.run(scenario())

def test_new_plan_captures_stationary_joints_before_async_barrier():
    import json
    from dataclasses import asdict
    from sketch_control.weld_execution import WeldExecutionMixin
    from test_weld_session import prepared
    class Executor(WeldExecutionMixin):
        def __init__(self):
            self._weld=prepared();self._weld_seen=set();self._weld_joints={'left':0.,'right':.1}
            self.arm_joint_names=['left'];self._weld_stationary_joints=(('right',0.),)
            self.executor=NS(create_task=lambda *a:None)
        def _weld_tick(self):pass
    ex=Executor()
    payload=dict(command_id='new',server_session_id=ex._weld.server_session_id,expected=asdict(ex._weld.identity()))
    ex._weld_message('plan',NS(data=json.dumps(payload)))
    assert ex._weld.state=='PLANNING' and ex._weld_stationary_joints==( ('right',.1), )

def test_interpolated_fk_must_keep_requested_torch_pose():
    from geometry_msgs.msg import Pose
    from sketch_control.weld_execution import check_fk_pose
    response=NS(error_code=NS(val=1),pose_stamped=[NS(pose=Pose())])
    check_fk_pose(response,[0,0,0],[0,0,0,1])
    response.pose_stamped[0].pose.position.x=.0011
    with pytest.raises(ValueError,match='torch pose'): check_fk_pose(response,[0,0,0],[0,0,0,1])

def test_joint_only_scene_updates_do_not_block_verified_collision_scene():
    from sketch_control.weld_execution import WeldExecutionMixin,scene_signature
    from moveit_msgs.msg import PlanningScene,CollisionObject
    scene=PlanningScene();fake=NS(_weld_scene_dirty=False,_weld_scene_signature=scene_signature(scene))
    scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.joint_state.name=['j'];scene.robot_state.joint_state.position=[1.]
    WeldExecutionMixin._weld_on_scene(fake,scene)
    assert not fake._weld_scene_dirty
    scene.is_diff=False
    WeldExecutionMixin._weld_on_scene(fake,scene)
    assert not fake._weld_scene_dirty
    scene.is_diff=True;scene.world.collision_objects=[CollisionObject(id='obstacle')]
    WeldExecutionMixin._weld_on_scene(fake,scene)
    assert fake._weld_scene_dirty
