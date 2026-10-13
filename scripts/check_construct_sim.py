#!/usr/bin/env python3
"""Simulation-only integration check. Moves each controller by 0.03 rad and returns."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT/'isaac_assets/construct_manifest.json').read_text())
rclpy.init()
node = rclpy.create_node('construct_integration_check')
node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
state = {}
node.create_subscription(JointState,'/joint_states',lambda m: state.update(zip(m.name,m.position)),10)

def until(predicate,timeout=45):
    end = time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate(): return
    raise TimeoutError('Timed out waiting for simulation/action')

def future_result(future,timeout=45):
    until(future.done,timeout)
    return future.result()

def send(client,goal):
    assert client.wait_for_server(timeout_sec=30), client._action_name
    accepted = future_result(client.send_goal_async(goal))
    assert accepted.accepted, 'Goal rejected'
    response = future_result(accepted.get_result_async(),90)
    return response

try:
    until(lambda: len(state)==14)
    initial = dict(state)
    expected = manifest['initial_positions']
    error = max(abs(state[n]-expected[n]) for n in expected)
    print('INITIAL_STATE '+json.dumps({'joints':len(state),'max_error_rad':error}),flush=True)
    assert error < .03, (error,state)
    report={'initial_max_error_rad':error,'controllers':{}}
    parameters = node.create_client(GetParameters,'/move_group/get_parameters')
    assert parameters.wait_for_service(timeout_sec=30)
    adapters = future_result(parameters.call_async(GetParameters.Request(names=['ompl.response_adapters']))).values[0].string_array_value
    assert 'default_planning_response_adapters/AddRuckigTrajectorySmoothing' in adapters, adapters
    report['ruckig_enabled'] = True
    for controller,names in manifest['groups'].items():
        client = ActionClient(node,FollowJointTrajectory, f'/{controller}/follow_joint_trajectory')
        for delta in (.03,0.0):
            targets=[expected[n]+(delta if i==0 else 0) for i,n in enumerate(names)]
            goal=FollowJointTrajectory.Goal()
            goal.trajectory.joint_names=names
            point=JointTrajectoryPoint(positions=targets,velocities=[0.0]*len(names))
            point.time_from_start.sec=3
            goal.trajectory.points=[point]
            result=send(client,goal)
            assert result.result.error_code == 0,(controller,result)
            until(lambda:max(abs(state[n]-q) for n,q in zip(names,targets))<.025)
            measured=max(abs(state[n]-q) for n,q in zip(names,targets))
            print('TRAJECTORY_OK '+json.dumps({'controller':controller,'offset_rad':delta,'error_rad':measured}),flush=True)
        report['controllers'][controller]={'success':True,'return_error_rad':measured}
    validity=node.create_client(GetStateValidity,'/check_state_validity')
    assert validity.wait_for_service(timeout_sec=20)
    request=GetStateValidity.Request()
    request.robot_state.joint_state=JointState(name=list(state),position=list(state.values()))
    request.group_name='whole_robot'
    valid=future_result(validity.call_async(request))
    print('STATE_VALIDITY '+str(valid.valid)+' contacts='+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]),flush=True)
    report['initial_state_collision_free']=valid.valid
    assert valid.valid, 'Source initial state has a collision; inspect contact list'
    client=ActionClient(node,MoveGroup,'/move_action')
    for group in ('left_manipulator','right_manipulator','robot_head'):
        names=manifest['groups'][group+'_controller']
        for delta in (.03,0.0):
            goal=MoveGroup.Goal()
            goal.request.group_name=group
            goal.request.pipeline_id='ompl'
            goal.request.allowed_planning_time=5.0
            goal.request.num_planning_attempts=3
            goal.request.max_velocity_scaling_factor=.1
            goal.request.max_acceleration_scaling_factor=.1
            goal.request.start_state.is_diff=True
            goal.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(expected[n]+(delta if i==0 else 0)),tolerance_above=.001,tolerance_below=.001,weight=1.0) for i,n in enumerate(names)])]
            goal.planning_options.plan_only=False
            result=send(client,goal)
            assert result.result.error_code.val==1,(group,result.result.error_code)
            print('MOVEIT_EXECUTION_OK '+group+' offset='+str(delta),flush=True)
        report.setdefault('moveit',{})[group]=True
    (ROOT/'logs/construct-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('CONSTRUCT_VALIDATION_OK',flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
