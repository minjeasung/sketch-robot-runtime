import json
from pathlib import Path

from sketch_control.interlock_flight_recorder import (
    decode_system_state_interlocks,
    InterlockFlightBuffer,
)
import yaml


def healthy_state():
    return {
        'collision_status_raw': 0,
        'self_collision_status_raw': 0,
        'sos_flag': 0,
        'soft_estop': False,
        'ems_flag': 0,
        'information_chunk_1': 1 << 6,
        'information_chunk_3': 0,
        'init_state_info': 6,
        'init_error': 0,
        'is_freedrive_mode': False,
    }


def test_system_state_decoder_reports_exact_simultaneous_interlocks():
    state = healthy_state()
    state.update(
        collision_status_raw=2,
        sos_flag=5,
        ems_flag=3,
        information_chunk_3=(1 << 22) | (1 << 25),
    )

    decoded = decode_system_state_interlocks(state)

    assert decoded['ready'] is False
    assert decoded['severe_reason_mask'] == (
        (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 9)
    )
    assert decoded['severe_reasons'] == [
        'collision',
        'sos',
        'ems',
        'safety_ems2',
        'safety_sss',
    ]


def test_system_state_decoder_distinguishes_ready_loss_from_severe_stop():
    decoded = decode_system_state_interlocks(healthy_state())
    assert decoded['ready'] is True
    assert decoded['severe_reasons'] == []

    power_lost = healthy_state()
    power_lost['information_chunk_1'] = 0
    decoded = decode_system_state_interlocks(power_lost)
    assert decoded['ready'] is False
    assert decoded['arm_power_on'] is False
    assert decoded['severe_reasons'] == []


def test_flight_buffer_writes_pretrigger_and_posttrigger_timeline(tmp_path):
    recorder = InterlockFlightBuffer(
        tmp_path,
        pretrigger_seconds=5.0,
        posttrigger_seconds=2.0,
        monotonic_clock=lambda: 0.0,
        wall_clock=lambda: 1_700_000_000.0,
    )
    recorder.add('system_state', {'sample': 'too_old'}, monotonic_s=1.0)
    recorder.add('system_state', {'sample': 'kept'}, monotonic_s=5.0)
    recorder.add('tcp_raw', {'force': [1.0, float('nan'), 3.0]}, monotonic_s=7.0)

    capture = recorder.trigger(
        'robot_internal_interlock',
        {'decoded_reasons': ['collision']},
        monotonic_s=8.0,
        wall_time_s=1_700_000_008.0,
    )
    assert capture.is_dir()
    assert (tmp_path / 'LATEST.txt').read_text().strip() == str(capture)

    recorder.add('hardware_motion_inhibited', {'data': True}, monotonic_s=8.1)
    assert recorder.finish_if_due(9.9) is False
    assert recorder.finish_if_due(10.0) is True

    events = [
        json.loads(line)
        for line in (capture / 'timeline.jsonl').read_text().splitlines()
    ]
    samples = [event['payload'].get('sample') for event in events]
    assert 'too_old' not in samples
    assert 'kept' in samples
    assert any(event['source'] == 'trigger' for event in events)
    assert any(event['source'] == 'hardware_motion_inhibited' for event in events)
    tcp = next(event for event in events if event['source'] == 'tcp_raw')
    assert tcp['payload']['force'] == [1.0, None, 3.0]

    summary = json.loads((capture / 'summary.json').read_text())
    assert summary['status'] == 'complete'
    assert summary['trigger_events'][0]['reason'] == 'robot_internal_interlock'
    assert summary['source_counts']['system_state'] == 1


def test_active_capture_coalesces_followup_abort_evidence(tmp_path):
    recorder = InterlockFlightBuffer(
        Path(tmp_path), pretrigger_seconds=5.0, posttrigger_seconds=1.0
    )
    first = recorder.trigger(
        'robot_internal_interlock', {}, monotonic_s=10.0, wall_time_s=20.0
    )
    second = recorder.trigger(
        'motion_abort', {}, monotonic_s=10.1, wall_time_s=20.1
    )
    assert first == second
    recorder.finalize('test')

    summary = json.loads((first / 'summary.json').read_text())
    assert [event['reason'] for event in summary['trigger_events']] == [
        'robot_internal_interlock',
        'motion_abort',
    ]


def test_real_launch_enables_event_recorder_with_bounded_windows():
    package_root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (package_root / 'config' / 'painting_system_real.yaml').read_text()
    )
    params = config['interlock_flight_recorder']['ros__parameters']
    assert params['enabled'] is True
    assert params['pretrigger_seconds'] >= 10.0
    assert 1.0 <= params['posttrigger_seconds'] <= 5.0

    launch_source = (
        package_root / 'launch' / 'rb10_painting_system.launch.py'
    ).read_text()
    assert '"enable_interlock_flight_recorder"' in launch_source
    assert 'executable="interlock_flight_recorder"' in launch_source
