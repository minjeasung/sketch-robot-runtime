"""Non-contact process mode and leased spray-gun ROS interface.

The hardware adapter is deliberately external. It must independently expire ON
commands, default OFF on disconnect, and report actual output acknowledgement.
"""
import json
import time
import uuid
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor


class SprayExecutionMixin:
    def _init_spray(self):
        # Startup-only commissioning mode: real motion, no gun attached and no
        # ON command possible. Never substitute a simulated hardware ACK.
        self.spray_motion_test = self.declare_parameter(
            'spray_motion_test', False, ParameterDescriptor(read_only=True)
        ).value
        if type(self.spray_motion_test) is not bool:
            raise ValueError('spray_motion_test must be a boolean')
        if self.spray_motion_test and self.painting_force_enabled:
            raise ValueError('spray_motion_test requires painting_force_enabled=false')
        self.process_mode = "spray" if self.spray_motion_test else "paint"
        self._paint_force_configured = self.painting_force_enabled
        self._spray_session = str(uuid.uuid4())
        self._spray_seq = 0
        self._spray_on = False
        self._spray_dispatch = False
        self._spray_edge_time = time.monotonic()
        self._spray_status = {}
        self._spray_status_time = 0.
        self._compliance_active = None
        self._compliance_time = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.process_mode_pub = self.create_publisher(String, '/painting_system/process_mode', latched)
        self.spray_command_pub = self.create_publisher(String, '/spray_gun/command', 1)
        self.create_subscription(String, '/painting_system/set_process_mode', self._set_process_mode, 10)
        self.create_subscription(String, '/spray_gun/status', self._on_spray_status, 1)
        self.create_subscription(Bool, '/admittance_controller/compliance_active', self._on_spray_compliance, 1)
        self.create_timer(.05, self._spray_tick)
        self.create_timer(.25, self._publish_process_mode)
        self._publish_process_mode()

    def _is_spray(self):
        return getattr(self, 'process_mode', 'paint') == 'spray'

    def _is_spray_motion_test(self):
        return getattr(self, 'spray_motion_test', False) is True

    def _publish_process_mode(self, error=''):
        self.process_mode_pub.publish(String(data=json.dumps(dict(
            mode=self.process_mode, error=error,
            spray_motion_test=self._is_spray_motion_test()))))

    def _set_process_mode(self, msg):
        mode = str(msg.data).strip()
        if mode not in {'paint', 'spray'}:
            self._publish_process_mode('INVALID_MODE')
            return
        if self._is_spray_motion_test() and mode != 'spray':
            self._publish_process_mode('MOTION_TEST_REQUIRES_SPRAY_MODE')
            return
        if self.executing or self._active_trajectory_goal_token is not None or self._d405_prescan_active or getattr(self, "_multi_queue", []) or getattr(self, "_multi_current", None) is not None:
            self._publish_process_mode('BUSY')
            return
        if mode == self.process_mode:
            self._publish_process_mode()
            return
        self._spray_off()
        if self._is_spray() and not self.dry_run and self._spray_io_blockers():
            self._publish_process_mode('GUN_OFF_NOT_ACKNOWLEDGED')
            return
        self._reset_painting_process()
        self.process_mode = mode
        self.painting_force_enabled = self._paint_force_configured if mode == 'paint' else False
        self._execution_tare_ready = False
        self._segment_path = None
        self._accepted_plan_hash = ''
        self._accepted_plan_path_id = ''
        self.current_waypoints = []
        self._publish_painting_command('IDLE', 0., enable=False)
        self._publish_process_mode()
        self._publish_execution_status('PLAN_INVALIDATED', 'PROCESS_MODE_CHANGED')

    def _on_spray_compliance(self, msg):
        self._compliance_active = bool(msg.data)
        self._compliance_time = time.monotonic()

    def _on_spray_status(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('session_id') != self._spray_session:
                return
            if data.get('command_id') != self._spray_seq:
                return
            if type(data.get('is_on')) is not bool or type(data.get('ready')) is not bool:
                return
            self._spray_status = data
            self._spray_status_time = time.monotonic()
        except (ValueError, TypeError):
            return

    def _spray_io_blockers(self):
        if self.dry_run:
            return ()
        if self._is_spray_motion_test():
            # No hardware ACK is claimed. The only supported equipment for
            # this mode is the current EOAT without a spray gun.
            return ('SPRAY_TEST_OUTPUT_NOT_OFF',) if self._spray_on else ()
        s = self._spray_status
        blockers = []
        if time.monotonic()-self._spray_status_time > .3:
            blockers.append('SPRAY_ADAPTER_STALE_OR_UNCONNECTED')
        if s.get('ready') is not True or s.get('fault') not in ('', None):
            blockers.append('SPRAY_ADAPTER_NOT_READY')
        if (s.get('session_id') != self._spray_session or s.get('command_id') != self._spray_seq
                or s.get('is_on') is not self._spray_on):
            blockers.append('SPRAY_OUTPUT_NOT_ACKNOWLEDGED')
        return tuple(blockers)

    def _spray_motion_blockers(self):
        if not self._is_spray() or self.dry_run:
            return ()
        blockers = list(self._spray_io_blockers())
        if self._compliance_active is not False or time.monotonic()-self._compliance_time > .3:
            blockers.append('SPRAY_REQUIRES_FRESH_COMPLIANCE_OFF')
        if self._painting_command_enable or self.painting_force_enabled:
            blockers.append('SPRAY_FORCE_MUST_BE_DISABLED')
        return tuple(blockers)

    def _spray_set_output(self, enabled):
        enabled = bool(enabled and not self.dry_run and self._is_spray()
                       and not self._is_spray_motion_test())
        if enabled != self._spray_on:
            self._spray_on = enabled
            self._spray_seq += 1
            self._spray_edge_time = time.monotonic()
        self.spray_command_pub.publish(String(data=json.dumps(dict(
            session_id=self._spray_session, command_id=self._spray_seq,
            on=self._spray_on, lease_ms=250,
            path_id=getattr(getattr(self, '_active_segment_path', None), 'path_id', ''),
            process_mode=self.process_mode,
            spray_motion_test=self._is_spray_motion_test()))))

    def _spray_off(self):
        if not hasattr(self, '_spray_session'):
            return
        self._spray_dispatch = False
        self._spray_set_output(False)

    def _spray_tick(self):
        active = bool(self._is_spray() and self._spray_dispatch and
                      self._active_trajectory_goal_handle is not None and
                      not self._motion_abort_requested)
        self._spray_set_output(active)
        if not self._is_spray() or not self.executing or self.dry_run:
            return
        blockers = list(self._spray_motion_blockers())
        # Allow a bounded acknowledgement delay only at an output edge.
        if time.monotonic()-self._spray_edge_time < .25:
            blockers = [b for b in blockers if b == 'SPRAY_REQUIRES_FRESH_COMPLIANCE_OFF'
                        or b == 'SPRAY_FORCE_MUST_BE_DISABLED']
        if blockers:
            self._spray_off()
            self._request_motion_abort(','.join(blockers))

    def _spray_execute_step(self, step):
        self._spray_off()
        self._publish_painting_command('IDLE', 0., enable=False)
        if step.mode == 'SPRAY_FINISH':
            self._complete_process_step()
            return
        started = time.monotonic()
        def wait_off():
            if self._motion_abort_requested or not self.executing:
                return
            blockers = self._spray_motion_blockers()
            if blockers:
                if time.monotonic()-started > 1.:
                    self._request_motion_abort(','.join(blockers))
                else:
                    self._schedule_process_once(.05, wait_off)
                return
            self._transition_execution_state(step.mode, 'non-contact spray path; 0.5 m')
            self._plan_process_motion_step(step)
        wait_off()
