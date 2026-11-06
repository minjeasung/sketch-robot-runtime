"""Actual ROS executor dispatch with frozen simulation time on an isolated domain."""
import json
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool,String
from rbpodo_painting_control.painting_wrench_reference_node import PaintingWrenchReferenceNode
from rbpodo_painting_control.painting_wrench_guard_node import PaintingWrenchGuardNode
from rbpodo_painting_control.painting_force_monitor_node import PaintingForceMonitorNode


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def spin_until(executor,predicate,timeout=.7):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline and not predicate():executor.spin_once(timeout_sec=.02)
    assert predicate(), 'steady callback did not run while ROS clock was frozen'


@pytest.fixture
def ros_executor():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=119,
               signal_handler_options=SignalHandlerOptions.NO)
    executor=SingleThreadedExecutor()
    try:yield executor
    finally:executor.shutdown();rclpy.shutdown()


@pytest.mark.parametrize('node_type,publisher',[(PaintingWrenchReferenceNode,'_ramp_status_pub'),
    (PaintingWrenchGuardNode,'_status_pub'),(PaintingForceMonitorNode,'_status_pub')])
def test_all_force_nodes_dispatch_without_any_clock_message(ros_executor,node_type,publisher):
    node=node_type();capture=Capture();setattr(node,publisher,capture);ros_executor.add_node(node)
    try:
        assert node.get_clock().ros_time_is_active and node.get_clock().now().nanoseconds==0
        spin_until(ros_executor,lambda:len(capture.messages)>=3)
        assert node.get_clock().now().nanoseconds==0
    finally:ros_executor.remove_node(node);node.destroy_node()


def test_actual_reference_timer_keeps_ramp_on_ros_time_and_aborts_while_frozen(ros_executor):
    node=PaintingWrenchReferenceNode();capture=Capture();node._requested_wrench_pub=capture
    node._transform_force_tcp_to_ft=lambda values:values
    ros_executor.add_node(node)
    try:
        node.get_clock().set_ros_time_override(Time(seconds=10))
        node._last_time=10.;node._ramp_up=2.;node._desired_force=4.;node._max_force_slew_rate=100.;node._dry_run=False
        node._on_enable_force(Bool(data=True));node._on_mode(String(data='RAMP_UP'))
        spin_until(ros_executor,lambda:len(capture.messages)>=3)
        assert node._current_force_y==0.
        node.get_clock().set_ros_time_override(Time(seconds=10.5))
        spin_until(ros_executor,lambda:abs(node._current_force_y)>0)
        force=node._current_force_y;count=len(capture.messages)
        spin_until(ros_executor,lambda:len(capture.messages)>=count+3)
        assert node._current_force_y==force
        assert capture.messages[-1].header.stamp.sec==10
        assert capture.messages[-1].header.stamp.nanosec==500000000
        node._on_abort(Bool(data=True));count=len(capture.messages)
        spin_until(ros_executor,lambda:len(capture.messages)>count)
        assert node._current_force_y==0.
        assert capture.messages[-1].wrench.force.y==0.
    finally:ros_executor.remove_node(node);node.destroy_node()


def test_actual_guard_timer_expires_requested_wrench_with_frozen_ros_clock(ros_executor):
    node=PaintingWrenchGuardNode();capture=Capture();node._status_pub=capture
    node._check_tf=lambda now:None
    node._requested_received_at=time.monotonic();node._requested_frame_valid=True
    ros_executor.add_node(node)
    try:
        assert node._config.requested_wrench_timeout_s==pytest.approx(.1)
        spin_until(ros_executor,lambda:bool(capture.messages) and
            'REQUESTED_WRENCH_STALE' in json.loads(capture.messages[-1].data)['blockers'])
        assert json.loads(capture.messages[-1].data)['ages_s']['requested_wrench']>=.1
        assert node.get_clock().now().nanoseconds==0
    finally:ros_executor.remove_node(node);node.destroy_node()


def test_actual_monitor_timer_expires_measured_ft_without_advancing_ros_time(ros_executor):
    node=PaintingForceMonitorNode();capture=Capture();node._status_pub=capture
    node.get_clock().set_ros_time_override(Time(seconds=10))
    now=time.monotonic();node._mode='IDLE'
    node._safety.observe_tf(True,now)
    node._safety.process_filtered((0.,)*6,now,'IDLE')
    node._safety.process_raw((0.,)*6,now,'IDLE')
    ros_executor.add_node(node)
    try:
        spin_until(ros_executor,lambda:bool(capture.messages))
        assert json.loads(capture.messages[0].data)['ft_valid'] is True
        spin_until(ros_executor,lambda:json.loads(capture.messages[-1].data)['reason']=='FT_STALE')
        status=json.loads(capture.messages[-1].data)
        assert status['ft_valid'] is False
        assert status['filtered_age_s']>node._safety.config.filtered_timeout_s
        assert node.get_clock().now().nanoseconds==10000000000
    finally:ros_executor.remove_node(node);node.destroy_node()


def test_actual_monitor_startup_timer_waits_for_first_pair_then_permanently_arms(ros_executor):
    from geometry_msgs.msg import WrenchStamped
    node=PaintingForceMonitorNode();capture=Capture();node._status_pub=capture
    node.get_clock().set_ros_time_override(Time(seconds=10))
    node._startup_readiness_grace_s=5.;node._mode='IDLE'
    node._startup_started_at=node._safety.started_at=time.monotonic()-1.65
    message=WrenchStamped();message.header.frame_id=node._tcp_frame
    message.header.stamp.sec=10
    node._on_wrench(message)
    assert node._safety.last_filtered_at is not None and node._safety.last_raw_at is None
    ros_executor.add_node(node)
    try:
        spin_until(ros_executor,lambda:len(capture.messages)>=3)
        status=json.loads(capture.messages[-1].data)
        assert not status['ft_valid'] and not status['abort_latched']
        node._on_raw_wrench(message);count=len(capture.messages)
        spin_until(ros_executor,lambda:len(capture.messages)>count)
        status=json.loads(capture.messages[-1].data)
        assert status['ft_valid'] and not status['abort_latched']
        assert node._startup_pair_ready_seen
        spin_until(ros_executor,lambda:json.loads(capture.messages[-1].data)['reason']=='FT_STALE')
        assert not node._startup_pair_grace_active(time.monotonic())
    finally:ros_executor.remove_node(node);node.destroy_node()
