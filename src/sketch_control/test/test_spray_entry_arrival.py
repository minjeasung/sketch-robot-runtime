"""Active spray requires measured arrival, not a nominal cached endpoint."""
import time
from concurrent.futures import Future
from types import MethodType, SimpleNamespace as NS
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from sketch_control.moveit_executor import MoveItExecutor
from sketch_control.spray_execution import SprayExecutionMixin
from test_multi_plane_spray import gun_stub


def pose(x=0.):
    p=Pose();p.position.x=x;p.orientation.w=1.;return p


def entry_node():
    n=gun_stub();n.spray_motion_test=True;n.executing=True
    n.current_joint_state_time=time.monotonic();n.current_joint_state=JointState(name=['j'],position=[0.],velocity=[0.])
    n._runtime_tare_tcp_pose_sample=lambda:((np.zeros(3),np.array([0.,0.,0.,1.]),0.),'')
    n._robot_stationary_for_bias=lambda:True
    n._process_row_tcp_poses={1:pose()};n._process_step_index=0;n._execution_snapshot=object()
    n.scheduled=[];n.events=[];n.plans=[];n.aborts=[]
    n._schedule_process_once=lambda delay,cb,**kw:n.scheduled.append((cb,kw))
    n._publish_painting_command=lambda *a,**kw:None
    n._transition_execution_state=lambda *a:n.events.append(a)
    n._plan_process_motion_step=lambda step:n.plans.append(step)
    n._request_motion_abort=n.aborts.append
    return n


def test_spray_entry_waits_for_measured_pose_before_active_phase():
    n=entry_node();step=NS(mode='SPRAY',rows=[NS(row_number=1)])
    n._runtime_tare_tcp_pose_sample=lambda:((np.array([.012,0.,0.]),np.array([0.,0.,0.,1.]),0.),'')
    n._spray_execute_step(step)
    assert not n.events and not n.plans and not n._spray_on
    assert n.scheduled and n.scheduled[-1][1].get('steady') is True
    n._runtime_tare_tcp_pose_sample=lambda:((np.array([.0007,0.,0.]),np.array([0.,0.,0.,1.]),0.),'')
    n.scheduled[-1][0]()
    assert n.events[0][0]=='SPRAY' and n.plans==[step]


@pytest.mark.parametrize('fault',['stale_joint','stale_tf','nonfinite','orientation','moving'])
def test_spray_arrival_rejects_bad_or_moving_measured_evidence(fault):
    n=entry_node()
    if fault=='stale_joint':n.current_joint_state_time=time.monotonic()-1
    if fault=='stale_tf':n._runtime_tare_tcp_pose_sample=lambda:(None,'actual TCP TF stale')
    if fault=='nonfinite':n._runtime_tare_tcp_pose_sample=lambda:((np.array([np.nan,0,0]),np.array([0,0,0,1]),0.),'')
    if fault=='orientation':n._runtime_tare_tcp_pose_sample=lambda:((np.zeros(3),np.array([0,0,1,0]),0.),'')
    if fault=='moving':n._robot_stationary_for_bias=lambda:False
    assert n._spray_arrival_reason(pose())


def test_spray_arrival_timeout_never_starts_output(monkeypatch):
    import sketch_control.spray_execution as module
    n=entry_node();now=[time.monotonic()];monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    n._runtime_tare_tcp_pose_sample=lambda:(None,'missing TF')
    n._spray_execute_step(NS(mode='SPRAY',rows=[NS(row_number=1)]))
    now[0]+=5.01;n._compliance_time=now[0];n.current_joint_state_time=now[0];n.scheduled[-1][0]()
    assert n.aborts and 'SPRAY_ENTRY' in n.aborts[0]
    assert not n.events and not n.plans and not n._spray_on


def test_nominal_zero_distance_spray_approach_still_plans_measured_12mm_correction(monkeypatch):
    n=entry_node();n.stationary_paint_hold_s=0.;n._process_last_tcp_pose=pose();n._motion_abort_requested=False
    n._spray_arrival_reason=lambda target,**kw:'SPRAY_ENTRY_POSITION_ERROR'
    n._process_motion_done=lambda target:pytest.fail('nominal pose incorrectly treated as measured arrival')
    n.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time()))
    calls=[];n.cartesian_client=NS(service_is_ready=lambda:True,call_async=lambda req:(calls.append(req) or Future()))
    monkeypatch.setattr(MoveItExecutor,'_arm_segment_cartesian_timeout',lambda *a:None)
    MoveItExecutor._plan_process_motion_step(n,NS(mode='SPRAY_APPROACH',rows=[NS(row_number=1)],speed_mps=.03))
    assert len(calls)==1 and calls[0].avoid_collisions is True
    assert calls[0].waypoints[0].position.x==0.


def test_stale_spray_entry_callback_cannot_dispatch_replaced_execution():
    n=entry_node();n._runtime_tare_tcp_pose_sample=lambda:(None,'missing TF')
    n._spray_execute_step(NS(mode='SPRAY',rows=[NS(row_number=1)]))
    n._execution_snapshot=object()
    n._runtime_tare_tcp_pose_sample=lambda:((np.zeros(3),np.array([0.,0.,0.,1.]),0.),'')
    n.scheduled[-1][0]()
    assert not n.events and not n.plans


def test_spray_planned_endpoint_rejects_coarse_ompl_goal_even_inside_general_tolerance():
    from test_d405_prescan_planning import _dual_ik_executor, _trajectory, _joint_state
    n,*_= _dual_ik_executor();n.process_mode='spray'
    record=dict(joint_state=_joint_state([0.]*6),candidate=dict(name='spray_tool_axis'))
    endpoint=[0.]*6;endpoint[0]=.01966
    valid,reason=MoveItExecutor._d405_plan_endpoint_matches_ik(n,_trajectory([[0.]*6,endpoint]),record)
    assert not valid and reason
    endpoint[0]=.0001
    valid,reason=MoveItExecutor._d405_plan_endpoint_matches_ik(n,_trajectory([[0.]*6,endpoint]),record)
    assert valid and not reason
