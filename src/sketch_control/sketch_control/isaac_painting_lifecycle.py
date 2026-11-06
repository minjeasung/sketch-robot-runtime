"""Session-bound simulation watchdog; interruption requires stack restart."""
import json
import math
import time


class LifecycleTimings:
    """Bounded receiver diagnostics; never used to refresh safety leases."""
    def __init__(self):
        self.events = {}

    def received(self, name, now):
        event = self.events.setdefault(name, {'count': 0, 'max_gap_s': 0.0})
        if 'last' in event:
            event['max_gap_s'] = max(event['max_gap_s'], now - event['last'])
        event['last'] = now
        event['count'] += 1

    def completed(self, name, wall_s, cpu_s):
        event = self.events.setdefault(name, {})
        event['max_wall_s'] = max(event.get('max_wall_s', 0.0), wall_s)
        event['max_cpu_s'] = max(event.get('max_cpu_s', 0.0), cpu_s)

    def snapshot(self, now):
        return {name: {**{key: value for key, value in event.items() if key != 'last'},
                       **({'age_s': now - event['last']} if 'last' in event else {})}
                for name, event in self.events.items()}


class SessionWatchdog:
    """Original step/stamp progress and ROS clock, measured against wall time."""
    def __init__(self, session_id, scene_revision, timeout_s=0.20, now=None, startup_timeout_s=5.0):
        self.session_id = session_id
        self.scene_revision = scene_revision
        self.timeout_s = timeout_s
        self.startup_timeout_s = startup_timeout_s
        self.started = time.monotonic() if now is None else now
        self.step = None
        self.stamp = None
        self.last_state_progress = None
        self.clock = None
        self.last_clock_progress = None
        self.fault = None

    @property
    def restart_required(self):
        return self.fault is not None

    def _fail(self, reason):
        if self.fault is None:
            self.fault = reason
        return self.fault

    def observe(self, session_id, scene_revision, step_index, sim_time,
                playing, joints_valid, wrench_valid, now):
        if self.fault:
            return self.fault
        if session_id != self.session_id:
            return self._fail('session_changed')
        if scene_revision != self.scene_revision:
            return self._fail('scene_changed')
        if not playing:
            return self._fail('paused')
        if not joints_valid or not wrench_valid or not math.isfinite(sim_time):
            return self._fail('invalid_measurement')
        if self.step is not None and step_index < self.step:
            return self._fail('step_rewound')
        if self.stamp is not None and sim_time < self.stamp:
            return self._fail('clock_rewound')
        if self.step is None or (step_index > self.step and sim_time > self.stamp):
            self.last_state_progress = now
            self.step, self.stamp = step_index, sim_time
        return self.check(now)

    def observe_clock(self, sim_time, now):
        if self.fault:
            return self.fault
        if not math.isfinite(sim_time) or (self.clock is not None and sim_time < self.clock):
            return self._fail('clock_rewound')
        if self.clock is None or sim_time > self.clock:
            self.clock = sim_time
            self.last_clock_progress = now
        return None

    def check(self, now):
        if self.fault:
            return self.fault
        # A discovery window applies only before the first input;
        # once a source is observed its exact 0.20 s lease applies immediately.
        if self.last_state_progress is None:
            if now - self.started > self.startup_timeout_s:
                return self._fail('state_missing')
        elif now - self.last_state_progress > self.timeout_s:
            return self._fail('state_stalled')
        if self.last_clock_progress is None:
            if now - self.started > self.startup_timeout_s:
                return self._fail('clock_missing')
        elif now - self.last_clock_progress > self.timeout_s:
            return self._fail('clock_stalled')
        return None


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import WrenchStamped
    from isaac_painting_msgs.msg import State
    from rosgraph_msgs.msg import Clock as ClockMessage
    from std_msgs.msg import Bool, String

    class Lifecycle(Node):
        def __init__(self):
            super().__init__('isaac_painting_lifecycle')
            self.declare_parameter('session_id', '')
            self.declare_parameter('scene_revision', '')
            self.declare_parameter('startup_timeout_s', 5.0)
            self.guard = SessionWatchdog(self.get_parameter('session_id').value,
                self.get_parameter('scene_revision').value,
                startup_timeout_s=self.get_parameter('startup_timeout_s').value)
            if not self.guard.session_id or not self.guard.scene_revision:
                raise ValueError('lifecycle requires ready session/revision')
            self.cancel_state = 'not_requested'
            self.cancel_future = None
            self.cancel_started = None
            self.fault_started = None
            self.first_state_received = False
            self.timings = LifecycleTimings()
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status = self.create_publisher(String, '/isaac/rb20_painting/lifecycle_status', latched)
            self.work_area = self.create_publisher(String, '/painting_system/work_area_state', latched)
            self.invalidate_perception = self.create_publisher(Bool, '/perception/invalidate_all', latched)
            self.booleans = {
                topic: (self.create_publisher(Bool, topic, 10), value)
                for topic, value in {
                    '/motion_abort': True, '/painting_admittance/abort': True,
                    '/painting_admittance/enable_force': False,
                    '/painting_admittance/free_space_confirmed': False,
                    '/admittance_controller/compliance_enable': False,
                }.items()}
            self.mode = self.create_publisher(String, '/painting_admittance/mode', 10)
            self.wrench = self.create_publisher(WrenchStamped,
                '/painting_admittance/requested_wrench_reference', 10)
            self.cancel = self.create_client(CancelGoal,
                '/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')
            self.create_subscription(State, '/isaac/rb20_painting/state', self.on_state, qos_profile_sensor_data)
            self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)
            # Explicit steady clock is essential: /clock stops during pause.
            self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_state(self, msg):
            started, cpu = time.monotonic(), time.thread_time()
            self.timings.received('state', started)
            if not self.first_state_received:
                self.first_state_received = True
                self.get_logger().info(f'First simulator state: session={msg.session_id}, step={msg.step_index}')
            self.guard.observe(msg.session_id, msg.scene_revision, msg.step_index,
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                msg.playing, msg.joints_valid, msg.wrench_valid, started)
            self.timings.completed('state_guard', time.monotonic() - started, time.thread_time() - cpu)
            if self.guard.fault and self.fault_started is None:
                self.tick()

        def on_clock(self, msg):
            started, cpu = time.monotonic(), time.thread_time()
            self.timings.received('clock', started)
            self.guard.observe_clock(msg.clock.sec + msg.clock.nanosec * 1e-9, started)
            self.timings.completed('clock_guard', time.monotonic() - started, time.thread_time() - cpu)

        def publish_timed(self, name, publisher, message):
            started, cpu = time.monotonic(), time.thread_time()
            publisher.publish(message)
            elapsed = time.monotonic() - started
            self.timings.completed(name, elapsed, time.thread_time() - cpu)
            if elapsed > 0.05:
                self.get_logger().warning(f'Lifecycle {name} blocked {elapsed:.6f}s; '
                    f'timings={json.dumps(self.timings.snapshot(time.monotonic()))}')

        def tick(self):
            now = time.monotonic()
            self.timings.received('tick', now)
            fault = self.guard.check(now)
            if fault:
                if self.fault_started is None:
                    self.fault_started = now
                    self.get_logger().error(f'Isaac interruption: {fault}; restart required; '
                        f'timings={json.dumps(self.timings.snapshot(now))}')
                    self.publish_timed('invalidate_publish', self.invalidate_perception, Bool(data=True))
                    self.publish_timed('work_area_publish', self.work_area, String(data=json.dumps({
                        'selected': False, 'work_area_id': '', 'plane_generation_id': '',
                        'invalidation_seq': f'{self.guard.session_id}:{self.fault_started}',
                        'reason': f'ISAAC_{fault.upper()}'})))
                for publisher, value in self.booleans.values():
                    self.publish_timed('stop_publish', publisher, Bool(data=value))
                self.publish_timed('mode_publish', self.mode, String(data='ABORT'))
                zero = WrenchStamped()
                zero.header.stamp = self.get_clock().now().to_msg()
                zero.header.frame_id = 'ft_link'
                self.publish_timed('wrench_publish', self.wrench, zero)
                if self.cancel_state == 'not_requested' and self.cancel.service_is_ready():
                    self.cancel_started = now
                    self.cancel_state = 'pending'
                    self.cancel_future = self.cancel.call_async(CancelGoal.Request())
                if self.cancel_future is not None and self.cancel_future.done():
                    try:
                        result = self.cancel_future.result()
                        self.cancel_state = ('acknowledged' if result.return_code == CancelGoal.Response.ERROR_NONE
                                             else 'failed')
                    except Exception:
                        self.cancel_state = 'failed'
                    self.cancel_future = None
                if (self.cancel_state == 'pending' and now - self.cancel_started > 1.0
                        or self.cancel_state == 'not_requested' and now - self.fault_started > 1.0):
                    self.cancel_state = 'failed'
            self.publish_timed('status_publish', self.status, String(data=json.dumps({
                'session_id': self.guard.session_id, 'scene_revision': self.guard.scene_revision,
                'ok': fault is None and self.guard.last_state_progress is not None
                      and self.guard.last_clock_progress is not None,
                'reason': fault or '', 'restart_required': self.guard.restart_required,
                'cancel_state': self.cancel_state, 'step_index': self.guard.step,
                'state_received': self.first_state_received,
                'updated_monotonic': now, 'timings': self.timings.snapshot(now)})))

    rclpy.init(args=args)
    node = None
    try:
        node = Lifecycle()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # A SIGTERM can invalidate the context between checking a timer and
        # publishing its stop lease. Other ROS errors remain visible.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
