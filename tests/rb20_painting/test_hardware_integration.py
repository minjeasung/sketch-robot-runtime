"""Real controller_manager/plugin with disposable simulator wire fixtures.

This verifies transport and interlocks, not physics. Actual Isaac validation is
collected separately by check_rb20_painting_isaac.py.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest


def test_plugin_measured_state_tare_command_and_foreign_session_stop():
    rclpy = pytest.importorskip('rclpy')
    State = pytest.importorskip('isaac_painting_msgs.msg').State
    from isaac_painting_msgs.msg import Command
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Bool, Float64MultiArray, String
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    import yaml
    root=Path(__file__).resolve().parents[2]
    plugin=root/'.runtime/rb20_painting/install/isaac_painting_hardware/lib/libisaac_painting_hardware.so'
    if not plugin.is_file(): pytest.skip('Build the RB20 simulation overlay first')
    # Fresh private ROS context; never contact a running stack in the user domain.
    context=rclpy.context.Context()
    context.init(domain_id=117)
    node=rclpy.create_node('rb20_hardware_contract_test',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    pub=node.create_publisher(State,'/isaac/rb20_painting/state',
                              QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    description_pub=node.create_publisher(String,'/robot_description',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    controls=[node.create_publisher(Bool,t,10) for t in [
        '/painting_admittance/trajectory_active','/painting_admittance/enable_force',
        '/admittance_controller/compliance_enable','/admittance_controller/compliance_active']]
    free_space=node.create_publisher(Bool,'/painting_admittance/free_space_confirmed',10)
    confirmed=False
    commands=[]; joints=[]; wrenches=[]; inhibited=[]; diagnostics=[]; source_stamps=set()
    node.create_subscription(Command,'/isaac/rb20_painting/command',commands.append,1)
    node.create_subscription(JointState,'/joint_states',joints.append,10)
    node.create_subscription(WrenchStamped,'/force_torque_sensor_broadcaster_raw/wrench',wrenches.append,10)
    node.create_subscription(Bool,'/painting_system/hardware_motion_inhibited',inhibited.append,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/isaac/rb20_painting/hardware_status',lambda msg:diagnostics.append(json.loads(msg.data)),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    control=node.create_publisher(Float64MultiArray,'/test_positions/commands',10)
    tare=node.create_client(Trigger,'/rbpodo_ft_tare/runtime_free_space_tare')
    state=State(session_id='test',scene_revision='fixture',joint_names=['base','shoulder','elbow','wrist1','wrist2','wrist3'],
        positions=[0.,.1,.2,.3,.4,.5],velocities=[0.]*6,efforts=[0.]*6,
        joints_valid=True,wrench_valid=True,playing=True)
    state.header.frame_id='ft_link'; state.wrench.force.z=1.2; state.gravity_wrench.force.z=1.
    def tick():
        state.step_index+=1;state.header.stamp=node.get_clock().now().to_msg()
        source_stamps.add((state.header.stamp.sec,state.header.stamp.nanosec));pub.publish(state)
        for p in controls:p.publish(Bool(data=False))
        free_space.publish(Bool(data=confirmed))
    timer=node.create_timer(.01,tick)
    def wait(predicate, seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate():return
        raise AssertionError('condition timed out; inspect hardware-integration.log')
    env={**os.environ,'ROS_DOMAIN_ID':'117','ROS_LOCALHOST_ONLY':'1','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
    children=[]; logdir=root/'logs/rb20-painting'; logdir.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as temp, (logdir/'hardware-integration.log').open('w') as log:
        description=ET.parse(root/'.runtime/rb20_painting/assets/robot.urdf').getroot()
        hardware=description.find('./ros2_control/hardware')
        for name,value in [('session_id','test'),('scene_revision','fixture')]:
            param=hardware.find(f"param[@name='{name}']")
            if param is None:param=ET.SubElement(hardware,'param',name=name)
            param.text=value
        description_pub.publish(String(data=ET.tostring(description,encoding='unicode')))
        params={'controller_manager':{'ros__parameters':{'robot_description':ET.tostring(description,encoding='unicode'),
            'update_rate':100,'use_sim_time':False,
            'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},
            'force_torque_sensor_broadcaster_raw':{'type':'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'},
            'test_positions':{'type':'forward_command_controller/ForwardCommandController'}}},
            'force_torque_sensor_broadcaster_raw':{'ros__parameters':{'sensor_name':'ft_sensor_raw','frame_id':'ft_link'}},
            'test_positions':{'ros__parameters':{'joints':list(state.joint_names),'interface_name':'position'}}}
        config=Path(temp)/'params.yaml';config.write_text(yaml.safe_dump(params))
        try:
            children.append(subprocess.Popen(['/opt/ros/jazzy/lib/controller_manager/ros2_control_node','--ros-args','--params-file',str(config)],env=env,stdout=log,stderr=log,start_new_session=True))
            wait(lambda:tare.service_is_ready())
            children.append(subprocess.Popen(['ros2','run','controller_manager','spawner','joint_state_broadcaster','force_torque_sensor_broadcaster_raw','test_positions','--activate-as-group'],env=env,stdout=log,stderr=log,start_new_session=True))
            wait(lambda:len(joints)>5 and children[-1].poll()==0,15)
            # Measured state remains fixture state even after different command.
            for _ in range(10):
                control.publish(Float64MultiArray(data=[.2]*6)); executor.spin_once(timeout_sec=.02)
            wait(lambda:any(m.enabled and list(m.positions)==[.2]*6 for m in commands))
            assert all((m.header.stamp.sec,m.header.stamp.nanosec) in source_stamps for m in commands if m.enabled), 'Command must echo the exact source State stamp without floating-point loss'
            actual=dict(zip(joints[-1].name,joints[-1].position));assert actual['shoulder']==pytest.approx(.1)
            # A real untared sample remains observable; tare readiness is a
            # separate execution gate, not a reason to invent invalid sensing.
            wait(lambda:bool(wrenches) and math.isfinite(wrenches[-1].wrench.force.z),2)
            assert wrenches[-1].wrench.force.z==pytest.approx(.2)
            assert diagnostics[-1]['tare_ready'] is False
            assert diagnostics[-1]['session_id']=='test' and diagnostics[-1]['scene_revision']=='fixture'
            wait(lambda:state.step_index>100)
            future=tare.call_async(Trigger.Request());wait(future.done,8)
            assert not future.result().success, 'Tare accepted without free-space confirmation'
            confirmed=True
            start=state.step_index;wait(lambda:state.step_index>start+5)
            future=tare.call_async(Trigger.Request());wait(future.done,8)
            assert future.result().success, future.result().message
            wait(lambda:bool(wrenches) and abs(wrenches[-1].wrench.force.z)<1e-6)
            wait(lambda:diagnostics[-1]['tare_ready'] is True)
            state.wrench.force.x=2.
            wait(lambda:bool(wrenches) and abs(wrenches[-1].wrench.force.x-2.)<1e-6)
            state.session_id='foreign'
            wait(lambda:bool(inhibited) and inhibited[-1].data and bool(commands) and not commands[-1].enabled)
            state.session_id='test'
            start=state.step_index;wait(lambda:state.step_index>start+30)
            assert inhibited[-1].data and not commands[-1].enabled
            (logdir/'hardware-integration.json').write_text(json.dumps({'passed':True,'physics':False,
                'checks':['measured_state_not_command_echo','command_transport','gravity_compensation','runtime_tare','raw_wrench','foreign_session_latched_stop']}))
        finally:
            timer.cancel()
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=4)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            executor.shutdown();node.destroy_node();context.shutdown()
