"""Executor leases tick in steady time even when simulation progress pauses."""
import time
from types import SimpleNamespace as NS
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from sketch_control.moveit_executor import MoveItExecutor as M
from rbpodo_painting_control.wrench_guard import GuardConfig, GuardInputs, evaluate_guard


class Capture:
    def __init__(self):self.samples=[]
    def publish(self,message):self.samples.append((time.monotonic(),message.data))


@pytest.mark.parametrize('rate',[0.,.1])
def test_real_heartbeat_timer_renews_leases_with_frozen_or_slow_clock(rate):
    context=Context();context.init(domain_id=119)
    node=rclpy.create_node('executor_heartbeat_clock_test',context=context,
        parameter_overrides=[Parameter('use_sim_time',value=True)])
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    publishers={name:Capture() for name in ('executor_heartbeat_pub','trajectory_active_pub','robot_stationary_pub',
        'painting_force_pub','painting_enable_pub','painting_mode_pub')}
    subject=NS(create_timer=node.create_timer,executing=True,_motion_abort_requested=False,
        _fjt_motion_state_unknown=False,_hardware_motion_inhibited=False,_painting_command_mode='PAINT',
        _painting_command_force_n=20.,_painting_command_enable=True,_trajectory_command_active=lambda:True,
        _robot_stationary_for_bias=lambda:False,**publishers)
    subject._publish_executor_heartbeat=lambda:M._publish_executor_heartbeat(subject)
    try:
        M._start_executor_heartbeat(subject)
        started=time.monotonic()
        while time.monotonic()-started<.34:
            node.get_clock().set_ros_time_override(Time(seconds=10.+rate*(time.monotonic()-started)))
            executor.spin_once(timeout_sec=.01)
        for name in ('executor_heartbeat_pub','painting_enable_pub','painting_mode_pub'):
            samples=publishers[name].samples
            assert len(samples)>=5
            assert all(0<b[0]-a[0]<.2 for a,b in zip(samples,samples[1:]))
        now=time.monotonic()
        # A renewed executor lease does not renew the physically measured FT.
        # Pausing simulated measurements still blocks force after original .2 s.
        config=GuardConfig(.1,.2,.2,.2,.2,.2,.2,.2,35.)
        inputs=GuardInputs(now,True,(0.,20.,0.,0.,0.,0.),now,True,
            'PAINT',publishers['painting_mode_pub'].samples[-1][0],True,
            publishers['painting_enable_pub'].samples[-1][0],True,
            publishers['executor_heartbeat_pub'].samples[-1][0],True,started,
            True,now,True,now,False,False,now)
        decision=evaluate_guard(config,inputs)
        assert decision.blockers==('FT_STALE',)
        assert not decision.forwarding and decision.output_wrench==(0.,)*6
    finally:
        executor.shutdown();node.destroy_node();context.shutdown()


@pytest.mark.parametrize('kind,period,duration,minimum',[
    ('readiness',.5,1.65,3),('spray',.05,.38,5)])
def test_operational_publication_continues_when_simulation_clock_is_frozen(kind,period,duration,minimum):
    from sketch_control.spray_execution import SprayExecutionMixin
    context=Context();context.init(domain_id=119)
    node=rclpy.create_node('operational_clock_test',context=context,
        parameter_overrides=[Parameter('use_sim_time',value=True)])
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    samples=[]
    subject=NS(create_timer=node.create_timer,
        _update_and_publish_readiness=lambda:samples.append(time.monotonic()),
        _spray_tick=lambda:samples.append(time.monotonic()),
        _publish_process_mode=lambda:None)
    try:
        if kind=='readiness':
            assert hasattr(M,'_start_readiness_timer'), 'readiness must use a wall-time renewal timer'
            M._start_readiness_timer(subject)
        else:
            assert hasattr(SprayExecutionMixin,'_start_spray_timers'), 'spray leases must use wall-time renewal timers'
            SprayExecutionMixin._start_spray_timers(subject)
        started=time.monotonic()
        while time.monotonic()-started<duration:
            node.get_clock().set_ros_time_override(Time(seconds=10.))
            executor.spin_once(timeout_sec=.01)
        assert len(samples)>=minimum
        assert all(0<b-a<period*2 for a,b in zip(samples,samples[1:]))
    finally:
        executor.shutdown();node.destroy_node();context.shutdown()
