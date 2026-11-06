"""Cone entry must be chosen for the whole stroke, before any robot motion."""
import copy
from concurrent.futures import Future
from types import SimpleNamespace as NS, MethodType
import numpy as np
import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from sketch_control.spray_approach import SprayApproachMixin


def node():
    n=NS(_stage1_attempt_token=object(),_stage1_ik_candidate_generation=3,
         _motion_abort_requested=False,_scene_revision=2,_scene_confirmed_revision=2,
         scene_confirmed=True,base_frame='World',planning_group='left',ee_link='tcp',
         _stage1_ik_seed_state=JointState(name=['arm','head'],position=[0.,1.35]),
         _stage1_ik_candidate_timer=None,failures=[],activated=[],requests=[],futures=[])
    def call(req):
        n.requests.append(req);f=Future();n.futures.append(f);return f
    n.cartesian_client=NS(service_is_ready=lambda:True,call_async=call)
    n._fail_stage1_before_motion=n.failures.append
    n._activate_stage1_orientation_rank=n.activated.append
    n.get_logger=lambda:NS(info=lambda *a:None,warn=lambda *a:None)
    n.create_timer=lambda period,cb,**kw:NS(callback=cb,cancel=lambda:None)
    n._cancel_stage1_ik_candidate_timer=lambda:setattr(n,'_stage1_ik_candidate_timer',None)
    for key in ('_preflight_spray_orientation_paths','_spray_preflight_next','_spray_preflight_done','_spray_preflight_valid'):
        setattr(n,key,MethodType(getattr(SprayApproachMixin,key),n))
    return n


def record(name):
    poses={1:Pose(),2:Pose(),3:Pose()};poses[3].position.x=.5
    return dict(candidate=dict(name=name,row_tcp_poses=poses,retreat_tcp_pose=Pose()),joint_state=JointState(name=['arm'],position=[.2]))


def response(fraction=1.,code=1):
    return NS(fraction=fraction,error_code=NS(val=code),solution=NS(joint_trajectory=NS(points=[object()])))


def test_rejects_partial_primary_stroke_and_selects_complete_roll_before_motion():
    n=node();records=[record('primary'),record('roll180')]
    n._preflight_spray_orientation_paths(records,3,n._stage1_attempt_token)
    assert not n.activated
    req=n.requests[0]
    assert req.avoid_collisions and list(req.start_state.joint_state.position)==[.2,1.35]
    assert len(req.waypoints)==4 and req.waypoints[2].position.x==.5
    n.futures[0].set_result(response(.60))
    assert len(n.requests)==2 and not n.activated
    n.futures[1].set_result(response())
    assert n.activated==[0] and n._stage1_orientation_ranked==[records[1]]
    assert not n.failures and n._stage1_ik_candidate_timer is None


@pytest.mark.parametrize('problem',['partial','error','empty','nan','exception'])
def test_no_motion_for_unverified_whole_stroke(problem):
    n=node();n._preflight_spray_orientation_paths([record('primary')],3,n._stage1_attempt_token)
    r=response()
    if problem=='partial':r.fraction=.999
    elif problem=='error':r.error_code.val=-1
    elif problem=='empty':r.solution.joint_trajectory.points=[]
    elif problem=='nan':r.fraction=float('nan')
    if problem=='exception':n.futures[0].set_exception(RuntimeError('transport'))
    else:n.futures[0].set_result(r)
    assert n.failures and not n.activated


@pytest.mark.parametrize('change',['token','generation','scene','abort'])
def test_stale_preflight_cannot_activate_motion(change):
    n=node();n._preflight_spray_orientation_paths([record('primary')],3,n._stage1_attempt_token)
    if change=='token':n._stage1_attempt_token=object()
    elif change=='generation':n._stage1_ik_candidate_generation+=1
    elif change=='scene':n._scene_revision+=1
    else:n._motion_abort_requested=True
    n.futures[0].set_result(response())
    assert not n.activated


def test_timeout_fails_closed_and_late_success_cannot_activate():
    n=node();n._preflight_spray_orientation_paths([record('primary')],3,n._stage1_attempt_token)
    n._stage1_ik_candidate_timer.callback();n.futures[0].set_result(response())
    assert n.failures and not n.activated


def test_cone_ik_seeds_explore_elbow_without_changing_target_or_inactive_joints():
    n=node();n.arm_joint_names=['j1','j2','j3','j4','j5','j6']
    n._robot_joint_limits={name:(-6.28,6.28) for name in n.arm_joint_names}
    seed=JointState(name=n.arm_joint_names+['head'],position=[.5,-.8,-1.57,0.,-.8,0.,1.35])
    original=[record('primary')['candidate'],record('roll180')['candidate']]
    candidates=SprayApproachMixin._spray_ik_seed_candidates(n,original,seed)
    assert len(candidates)==4
    assert candidates[1]['ik_seed_positions']=={'j2':.8}
    assert candidates[3]['ik_seed_positions']=={'j2':.8}
    assert candidates[0]['row_tcp_poses']==candidates[1]['row_tcp_poses']
    assert 'ik_seed_positions' not in original[0]
    assert seed.position[1]==-.8 and seed.position[-1]==1.35


def test_ik_seed_reflection_respects_joint_limits():
    n=node();n.arm_joint_names=['j1','j2'];n._robot_joint_limits={'j2':(-2.,-.1)}
    seed=JointState(name=n.arm_joint_names,position=[0.,-.8]);original=[record('primary')['candidate']]
    assert SprayApproachMixin._spray_ik_seed_candidates(n,original,seed)==original
