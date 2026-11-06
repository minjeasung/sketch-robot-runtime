"""Native domain-118 fixture: a burst of execution edges must survive spin delay."""
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def test_observer_keeps_every_rapid_execution_edge_in_separate_domain():
    program = '''
import json, os, sys, time
from types import SimpleNamespace, MethodType
assert os.environ['ROS_DOMAIN_ID'] == '118'
sys.path.insert(0, sys.argv[1])
from check_rb20_painting_isaac import Observer
from sketch_control import moveit_executor
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

observer = Observer()
publisher_node = rclpy.create_node('execution_edge_fixture_118')
late_node = rclpy.create_node('late_execution_fixture_118')
try:
    assert observer.node.context.get_domain_id() == 118
    status_pub = publisher_node.create_publisher(String, '/painting_system/execution_status', moveit_executor.LATCHED_QOS)
    event_pub = publisher_node.create_publisher(String, '/painting_system/execution_events',
        getattr(moveit_executor, 'EXECUTION_EVENT_QOS', QoSProfile(depth=100)))
    path=SimpleNamespace(plan_hash='fixture-plan')
    executor=SimpleNamespace(execution_status_pub=status_pub, execution_event_pub=event_pub,
        _active_segment_path=path, _segment_path=path, _execution_snapshot=None,
        _current_plane_generation_id='fixture-generation', _current_work_area_id='fixture-area',
        ft_normal_force_n=None, _painting_contact_confirmed=False, get_clock=publisher_node.get_clock)
    executor._publish_execution_status=MethodType(moveit_executor.MoveItExecutor._publish_execution_status,executor)
    deadline = time.monotonic() + 8.
    while event_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(publisher_node, timeout_sec=.01)
    assert event_pub.get_subscription_count() == 1, 'isolated fixture event observer not discovered'
    states = ['APPROACH_PRECONTACT', 'APPROACH_PRECONTACT', 'PRECONTACT_TARE',
              'PRECONTACT_TARE_COMPLETE', 'CONTACT_SEARCH', 'PAINT']
    for index, state in enumerate(states):
        # Exercise the actual executor producer without any yield between
        # phases, as happens in its no-op APPROACH -> TARE callback.
        executor._publish_execution_status(state, fixture_edge=index)
    time.sleep(.10)
    observer.spin(.4)
    observed = [(e['state'], e['fixture_edge']) for e in observer.events]
    expected = list(zip(states, range(len(states))))
    assert observed == expected, f'execution edges lost: {observed}; expected {expected}'
    assert [e['execution_sequence'] for e in observer.events] == [1,2,3,4,5,6]
    assert observer.latest['execution']['state'] == 'PAINT'
    late_events=[]; late_status=[]
    late_node.create_subscription(String,'/painting_system/execution_events',
        lambda msg:late_events.append(json.loads(msg.data)),QoSProfile(depth=100))
    late_node.create_subscription(String,'/painting_system/execution_status',
        lambda msg:late_status.append(json.loads(msg.data)),moveit_executor.LATCHED_QOS)
    deadline=time.monotonic()+.4
    while time.monotonic()<deadline:rclpy.spin_once(late_node,timeout_sec=.01)
    assert late_events == [], 'volatile events replayed old phases to a late checker'
    assert len(late_status)==1 and late_status[0]['state']=='PAINT', late_status
    # The existing latest-only status is not an event: a reconnect cannot
    # reconstruct an old execution into the observer event history.
    assert observer.events[-1]['execution_sequence']==6
    print('ALL_EXECUTION_EDGES_RECORDED_WITH_NO_LATE_REPLAY')
finally:
    late_node.destroy_node()
    publisher_node.destroy_node()
    observer.close()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='118')
    result = subprocess.run([sys.executable, '-c', program, str(ROOT / 'scripts')],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=20.)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'ALL_EXECUTION_EDGES_RECORDED' in result.stdout
