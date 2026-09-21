"""Event-triggered recorder for RB controller interlock diagnosis.

The recorder keeps recent high-rate robot and force samples in memory.  No
disk traffic is generated during normal motion.  When the RB controller loses
its ready state, the hardware inhibit latches, or another motion abort is
published, the pre-trigger window is written immediately and a short
post-trigger tail is appended.  This keeps diagnostic I/O outside the
ros2_control real-time process while preserving the samples that led to a
stop.
"""

from __future__ import annotations

from collections import Counter, deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Deque, Dict, Iterable, Optional

from geometry_msgs.msg import WrenchStamped
from rbpodo_msgs.msg import SystemState
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, String


INTERLOCK_REASON_NAMES = {
    0: 'collision',
    1: 'self_collision',
    2: 'sos',
    3: 'soft_estop',
    4: 'ems',
    5: 'safety_board_sos',
    6: 'safety_ems2',
    7: 'safety_prs',
    8: 'safety_hss',
    9: 'safety_sss',
}


def _finite_or_none(value: Any) -> Any:
    """Return JSON-safe primitive values without emitting NaN/Infinity."""
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _finite_or_none(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_or_none(item) for item in value]
    return str(value)


def _number_list(values: Iterable[Any]) -> list[Optional[float]]:
    result: list[Optional[float]] = []
    for value in values:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            result.append(None)
            continue
        result.append(numeric if math.isfinite(numeric) else None)
    return result


def decode_system_state_interlocks(state: Dict[str, Any]) -> Dict[str, Any]:
    """Decode the same RB safety predicates used by the hardware plugin."""
    collision_raw = int(state.get('collision_status_raw', 0))
    self_collision_raw = int(state.get('self_collision_status_raw', 0))
    sos = int(state.get('sos_flag', 0))
    ems = int(state.get('ems_flag', 0))
    chunk_1 = int(state.get('information_chunk_1', 0)) & 0xFFFFFFFF
    chunk_3 = int(state.get('information_chunk_3', 0)) & 0xFFFFFFFF

    mask = 0
    if collision_raw & 0x3:
        mask |= 1 << 0
    if self_collision_raw & 0x3:
        mask |= 1 << 1
    if sos & 0x3F:
        mask |= 1 << 2
    if bool(state.get('soft_estop', False)):
        mask |= 1 << 3
    if ems & 0x3F:
        mask |= 1 << 4
    if chunk_1 & (1 << 12):
        mask |= 1 << 5
    for offset in range(4):
        if chunk_3 & (1 << (22 + offset)):
            mask |= 1 << (6 + offset)

    reasons = [
        name for bit, name in INTERLOCK_REASON_NAMES.items() if mask & (1 << bit)
    ]
    arm_power_on = bool(chunk_1 & (1 << 6))
    direct_teach_pressed = bool(chunk_1 & (1 << 7))
    init_state = int(state.get('init_state_info', 0)) & 0x3F
    init_error = int(state.get('init_error', 0)) & 0xFFF
    ready = bool(
        not state.get('is_freedrive_mode', False)
        and init_state == 6
        and init_error == 0
        and mask == 0
        and arm_power_on
        and not direct_teach_pressed
    )
    return {
        'severe_reason_mask': mask,
        'severe_reasons': reasons,
        'ready': ready,
        'arm_power_on': arm_power_on,
        'direct_teach_pressed': direct_teach_pressed,
        'init_state_masked': init_state,
        'init_error_masked': init_error,
    }


class InterlockFlightBuffer:
    """In-memory pre-trigger ring with an event-triggered JSONL dump."""

    def __init__(
        self,
        output_directory: Path,
        pretrigger_seconds: float = 12.0,
        posttrigger_seconds: float = 3.0,
        monotonic_clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self.output_directory = Path(output_directory).expanduser()
        self.pretrigger_seconds = max(1.0, float(pretrigger_seconds))
        self.posttrigger_seconds = max(0.0, float(posttrigger_seconds))
        self._monotonic_clock = monotonic_clock
        self._wall_clock = wall_clock
        self._events: Deque[Dict[str, Any]] = deque()
        self._sequence = 0
        self._capture_directory: Optional[Path] = None
        self._timeline = None
        self._trigger_monotonic_s = 0.0
        self._trigger_wall_s = 0.0
        self._post_deadline_s = 0.0
        self._trigger_events: list[Dict[str, Any]] = []
        self._source_counts: Counter[str] = Counter()
        self._written_count = 0

    @property
    def active(self) -> bool:
        return self._timeline is not None

    @property
    def capture_directory(self) -> Optional[Path]:
        return self._capture_directory

    def _make_event(
        self,
        source: str,
        payload: Dict[str, Any],
        monotonic_s: float,
        wall_time_s: float,
    ) -> Dict[str, Any]:
        self._sequence += 1
        return {
            'sequence': self._sequence,
            'source': str(source),
            'monotonic_s': float(monotonic_s),
            'wall_time_s': float(wall_time_s),
            'payload': _finite_or_none(payload),
        }

    def _prune(self, now_s: float) -> None:
        cutoff = now_s - self.pretrigger_seconds
        while self._events and self._events[0]['monotonic_s'] < cutoff:
            self._events.popleft()

    def _write_event(self, event: Dict[str, Any]) -> None:
        if self._timeline is None:
            return
        output = dict(event)
        output['relative_to_trigger_s'] = (
            float(event['monotonic_s']) - self._trigger_monotonic_s
        )
        self._timeline.write(json.dumps(output, separators=(',', ':')) + '\n')
        self._timeline.flush()
        self._written_count += 1
        self._source_counts[str(event['source'])] += 1

    def add(
        self,
        source: str,
        payload: Dict[str, Any],
        *,
        monotonic_s: Optional[float] = None,
        wall_time_s: Optional[float] = None,
    ) -> None:
        now_s = self._monotonic_clock() if monotonic_s is None else float(monotonic_s)
        wall_s = self._wall_clock() if wall_time_s is None else float(wall_time_s)
        event = self._make_event(source, payload, now_s, wall_s)
        self._events.append(event)
        self._prune(now_s)
        if self.active:
            self._write_event(event)

    def trigger(
        self,
        reason: str,
        detail: Dict[str, Any],
        *,
        monotonic_s: Optional[float] = None,
        wall_time_s: Optional[float] = None,
    ) -> Path:
        now_s = self._monotonic_clock() if monotonic_s is None else float(monotonic_s)
        wall_s = self._wall_clock() if wall_time_s is None else float(wall_time_s)
        trigger_payload = {'reason': str(reason), 'detail': _finite_or_none(detail)}
        self.add(
            'trigger',
            trigger_payload,
            monotonic_s=now_s,
            wall_time_s=wall_s,
        )
        trigger_event = {
            'reason': str(reason),
            'detail': _finite_or_none(detail),
            'monotonic_s': now_s,
            'wall_time_s': wall_s,
        }
        self._trigger_events.append(trigger_event)
        if self.active:
            return self._capture_directory  # type: ignore[return-value]

        stamp = datetime.fromtimestamp(wall_s).astimezone().strftime(
            '%Y%m%d_%H%M%S_%f'
        )
        capture_dir = self.output_directory / f'interlock_{stamp}_{os.getpid()}'
        capture_dir.mkdir(parents=True, exist_ok=False)
        self._capture_directory = capture_dir
        self._trigger_monotonic_s = now_s
        self._trigger_wall_s = wall_s
        self._post_deadline_s = now_s + self.posttrigger_seconds
        self._timeline = (capture_dir / 'timeline.jsonl').open(
            'w', encoding='utf-8'
        )
        for event in self._events:
            self._write_event(event)

        self.output_directory.mkdir(parents=True, exist_ok=True)
        latest_tmp = self.output_directory / '.latest.tmp'
        latest_tmp.write_text(str(capture_dir) + '\n', encoding='utf-8')
        os.replace(latest_tmp, self.output_directory / 'LATEST.txt')
        self._write_summary('capturing_post_trigger')
        return capture_dir

    def _write_summary(self, status: str) -> None:
        if self._capture_directory is None:
            return
        summary = {
            'schema_version': 1,
            'status': str(status),
            'capture_directory': str(self._capture_directory),
            'trigger_wall_time_s': self._trigger_wall_s,
            'trigger_monotonic_s': self._trigger_monotonic_s,
            'pretrigger_seconds': self.pretrigger_seconds,
            'posttrigger_seconds': self.posttrigger_seconds,
            'written_event_count': self._written_count,
            'source_counts': dict(sorted(self._source_counts.items())),
            'trigger_events': _finite_or_none(self._trigger_events),
        }
        temp = self._capture_directory / '.summary.tmp'
        temp.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        os.replace(temp, self._capture_directory / 'summary.json')

    def finish_if_due(self, now_s: Optional[float] = None) -> bool:
        if not self.active:
            return False
        now = self._monotonic_clock() if now_s is None else float(now_s)
        if now < self._post_deadline_s:
            return False
        self.finalize('complete')
        return True

    def finalize(self, status: str = 'shutdown') -> None:
        if self._timeline is None:
            return
        self._timeline.flush()
        self._timeline.close()
        self._timeline = None
        self._write_summary(status)


class InterlockFlightRecorderNode(Node):
    """ROS adapter around :class:`InterlockFlightBuffer`."""

    def __init__(self) -> None:
        super().__init__('interlock_flight_recorder')
        self.declare_parameter('enabled', True)
        self.declare_parameter('pretrigger_seconds', 12.0)
        self.declare_parameter('posttrigger_seconds', 3.0)
        self.declare_parameter('output_directory', '')
        configured_output = str(self.get_parameter('output_directory').value).strip()
        output = (
            Path(configured_output)
            if configured_output
            else Path.home() / '.ros' / 'painting_interlock_records'
        )
        self._enabled = bool(self.get_parameter('enabled').value)
        self._buffer = InterlockFlightBuffer(
            output,
            pretrigger_seconds=float(
                self.get_parameter('pretrigger_seconds').value
            ),
            posttrigger_seconds=float(
                self.get_parameter('posttrigger_seconds').value
            ),
        )
        self._has_seen_ready = False
        self._last_system_ready: Optional[bool] = None
        self._last_severe_mask = 0
        self._last_hardware_inhibited = False
        self._last_motion_abort = False

        regular_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SystemState,
            '/rbpodo_hardware/system_state',
            self._on_system_state,
            regular_qos,
        )
        self.create_subscription(
            Bool,
            '/painting_system/hardware_motion_inhibited',
            self._on_hardware_inhibited,
            latched_qos,
        )
        self.create_subscription(Bool, '/motion_abort', self._on_motion_abort, 100)
        self.create_subscription(
            Log, '/rosout', self._on_rosout, 100
        )

        for topic, source in (
            ('/force_torque_sensor_broadcaster/wrench', 'ft_link_filtered'),
            ('/force_torque_sensor_broadcaster_raw/wrench', 'ft_link_raw'),
            ('/painting_admittance/force_tcp_filtered', 'tcp_filtered'),
            ('/painting_admittance/force_tcp_raw', 'tcp_raw'),
        ):
            self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, name=source: self._on_wrench(name, msg),
                regular_qos,
            )

        for topic, source in (
            ('/painting_admittance/mode', 'painting_mode'),
            ('/painting_system/execution_status', 'execution_status'),
            ('/painting_system/readiness', 'readiness'),
            ('/painting_admittance/safety_status', 'safety_status'),
            ('/painting_admittance/wrench_guard_status', 'wrench_guard_status'),
            ('/painting_admittance/abort_reason', 'force_abort_reason'),
        ):
            self.create_subscription(
                String,
                topic,
                lambda msg, name=source: self._record(name, {'data': msg.data}),
                100,
            )

        for topic, source in (
            ('/painting_admittance/trajectory_active', 'trajectory_active'),
            ('/painting_admittance/enable_force', 'force_enable'),
            ('/admittance_controller/compliance_enable', 'compliance_enable'),
            ('/admittance_controller/compliance_active', 'compliance_active'),
            ('/painting_admittance/contact_confirmed', 'contact_confirmed'),
            (
                '/admittance_controller/roller_balance/contact_valid',
                'roller_contact_valid',
            ),
            (
                '/admittance_controller/roller_balance/correction_active',
                'roller_correction_active',
            ),
            (
                '/admittance_controller/roller_balance/rate_saturated',
                'roller_rate_saturated',
            ),
            (
                '/admittance_controller/roller_balance/limit_reached',
                'roller_limit_reached',
            ),
            (
                '/admittance_controller/roller_balance/soft_limit_active',
                'roller_soft_limit_active',
            ),
        ):
            self.create_subscription(
                Bool,
                topic,
                lambda msg, name=source: self._record(name, {'data': bool(msg.data)}),
                100,
            )
        for topic, source in (
            ('/painting_admittance/desired_force_n', 'desired_force_n'),
            (
                '/admittance_controller/roller_balance/contact_torque_nm',
                'roller_contact_torque_nm',
            ),
            (
                '/admittance_controller/roller_balance/cop_offset_m',
                'roller_cop_offset_m',
            ),
            (
                '/admittance_controller/roller_balance/rotation_trim_rad',
                'roller_rotation_trim_rad',
            ),
            (
                '/admittance_controller/roller_balance/commanded_trim_rad',
                'roller_commanded_trim_rad',
            ),
        ):
            self.create_subscription(
                Float64,
                topic,
                lambda msg, name=source: self._record(
                    name, {'data': float(msg.data)}
                ),
                100,
            )
        self.create_timer(0.1, self._finish_capture_if_due)
        self.get_logger().info(
            'Interlock flight recorder ready: pre=%.1fs post=%.1fs output=%s enabled=%s'
            % (
                self._buffer.pretrigger_seconds,
                self._buffer.posttrigger_seconds,
                self._buffer.output_directory,
                self._enabled,
            )
        )

    def _record(self, source: str, payload: Dict[str, Any]) -> None:
        if self._enabled:
            self._buffer.add(source, payload)

    def _trigger(self, reason: str, detail: Dict[str, Any]) -> None:
        if not self._enabled:
            return
        was_active = self._buffer.active
        capture = self._buffer.trigger(reason, detail)
        if not was_active:
            self.get_logger().error(
                '[INTERLOCK FLIGHT RECORDER] capture started: %s reason=%s'
                % (capture, reason)
            )

    @staticmethod
    def _state_payload(msg: SystemState) -> Dict[str, Any]:
        collision_raw = int(
            getattr(
                msg,
                'op_stat_collision_status_raw',
                1 if bool(msg.op_stat_collision_occur) else 0,
            )
        )
        self_collision_raw = int(
            getattr(
                msg,
                'op_stat_self_collision_status_raw',
                1 if bool(msg.op_stat_self_collision) else 0,
            )
        )
        payload = {
            'controller_time_s': float(msg.time),
            'jnt_ref_rad': _number_list(msg.jnt_ref),
            'jnt_ang_rad': _number_list(msg.jnt_ang),
            'jnt_cur_a': _number_list(msg.jnt_cur),
            'jnt_temperature_c': _number_list(msg.jnt_temperature),
            'jnt_info_raw': [int(value) for value in msg.jnt_info],
            'tcp_ref_m_rad': _number_list(msg.tcp_ref),
            'tcp_pos_m_rad': _number_list(msg.tcp_pos),
            'eft_raw': _number_list(msg.eft),
            'robot_state': int(msg.robot_state),
            'task_state': int(msg.task_state),
            'collision_detect_onoff': bool(msg.collision_detect_onoff),
            'is_freedrive_mode': bool(msg.is_freedrive_mode),
            'real_vs_simulation_mode': bool(msg.real_vs_simulation_mode),
            'init_state_info': int(msg.init_state_info),
            'init_error': int(msg.init_error),
            'collision_status_raw': collision_raw,
            'self_collision_status_raw': self_collision_raw,
            'sos_flag': int(msg.op_stat_sos_flag),
            'soft_estop': bool(msg.op_stat_soft_estop_occur),
            'ems_flag': int(msg.op_stat_ems_flag),
            'information_chunk_1': int(msg.information_chunk_1),
            'information_chunk_2': int(msg.information_chunk_2),
            'information_chunk_3': int(msg.information_chunk_3),
            'information_chunk_4': int(msg.information_chunk_4),
            'safety_board_stat_info': int(
                getattr(msg, 'safety_board_stat_info', 0)
            ),
        }
        joint_error = []
        for reference, measured in zip(payload['jnt_ref_rad'], payload['jnt_ang_rad']):
            joint_error.append(
                None if reference is None or measured is None else reference - measured
            )
        payload['jnt_ref_minus_ang_rad'] = joint_error
        payload['interlock_decode'] = decode_system_state_interlocks(payload)
        return payload

    def _on_system_state(self, msg: SystemState) -> None:
        payload = self._state_payload(msg)
        self._record('system_state', payload)
        decoded = payload['interlock_decode']
        ready = bool(decoded['ready'])
        severe_mask = int(decoded['severe_reason_mask'])
        severe = severe_mask != 0
        if ready:
            self._has_seen_ready = True
        lost_ready = bool(
            self._has_seen_ready
            and self._last_system_ready is True
            and not ready
            and not bool(msg.is_freedrive_mode)
        )
        if severe and severe_mask != self._last_severe_mask:
            self._trigger(
                'robot_internal_interlock',
                {
                    'decoded_reasons': decoded['severe_reasons'],
                    'system_state': payload,
                },
            )
        elif lost_ready:
            self._trigger(
                'robot_ready_state_lost',
                {'system_state': payload},
            )
        self._last_severe_mask = severe_mask
        self._last_system_ready = ready

    def _on_hardware_inhibited(self, msg: Bool) -> None:
        value = bool(msg.data)
        self._record('hardware_motion_inhibited', {'data': value})
        if value and not self._last_hardware_inhibited:
            self._trigger('hardware_motion_inhibited', {'data': True})
        self._last_hardware_inhibited = value

    def _on_motion_abort(self, msg: Bool) -> None:
        value = bool(msg.data)
        self._record('motion_abort', {'data': value})
        if value and not self._last_motion_abort:
            self._trigger('motion_abort', {'data': True})
        self._last_motion_abort = value

    def _on_wrench(self, source: str, msg: WrenchStamped) -> None:
        self._record(
            source,
            {
                'frame_id': msg.header.frame_id,
                'stamp_ns': int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec),
                'force': [
                    float(msg.wrench.force.x),
                    float(msg.wrench.force.y),
                    float(msg.wrench.force.z),
                ],
                'torque': [
                    float(msg.wrench.torque.x),
                    float(msg.wrench.torque.y),
                    float(msg.wrench.torque.z),
                ],
            },
        )

    def _on_rosout(self, msg: Log) -> None:
        # WARN and above are sparse and include the exact C++ interlock
        # snapshot. Keeping INFO traffic out prevents UI/perception chatter
        # from evicting high-rate robot samples from the pre-trigger ring.
        if int(msg.level) < int(Log.WARN):
            return
        self._record(
            'rosout',
            {
                'stamp_ns': int(msg.stamp.sec) * 1_000_000_000
                + int(msg.stamp.nanosec),
                'level': int(msg.level),
                'name': msg.name,
                'message': msg.msg,
                'file': msg.file,
                'function': msg.function,
                'line': int(msg.line),
            },
        )

    def _finish_capture_if_due(self) -> None:
        if self._buffer.finish_if_due():
            self.get_logger().info(
                '[INTERLOCK FLIGHT RECORDER] capture complete: %s'
                % self._buffer.capture_directory
            )

    def destroy_node(self) -> bool:
        self._buffer.finalize('node_shutdown')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterlockFlightRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
