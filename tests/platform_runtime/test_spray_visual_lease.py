from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))


def test_spray_visual_needs_matching_plan_fresh_clock_and_live_lease():
    from platform_simulation_app import spray_visual_active
    payload = dict(enabled=True, simulation_only=True, timestamp_ns=1_000_000_000,
                   plan_hash='a'*64, lease_ms=250)
    def active(data=payload, stamp=1.1, now=10.1, plan='a'*64):
        return spray_visual_active(data, received=10., now=now, stamp=stamp, plan_hash=plan)
    assert active()
    assert not active(plan='b'*64)
    assert not active(now=10.3)
    assert not active(stamp=1.3)
    assert not active(stamp=.9)
    assert not active(data={**payload, 'enabled':False})
    assert not active(data={**payload, 'simulation_only':False})
    assert not active(data={**payload, 'timestamp_ns':None})
    assert not active(data={**payload, 'lease_ms':999999})


def test_nozzle_pose_uses_measured_ancestor_and_retains_fixed_offset():
    from platform_simulation_app import spray_nozzle_body
    import xml.etree.ElementTree as ET
    import numpy as np
    robot=ET.fromstring('''<robot><joint name="adapter" type="fixed">
      <parent link="wrist"/><child link="nozzle"/>
      <origin xyz=".1 0 .2" rpy="0 0 0"/></joint></robot>''')
    index, transform = spray_nozzle_body(robot, {'wrist':3}, 'nozzle')
    assert index == 3
    np.testing.assert_allclose(transform[:3,3], [.1,0,.2])


def test_candidate_plan_does_not_replace_accepted_execution_visualization():
    from platform_simulation_app import SprayVisualState
    state = SprayVisualState('profile')
    def plan(name):
        return dict(process_mode='spray', plan_hash=name*64,
                    spray_envelope_profile_sha256='profile')
    state.plan(plan('a'))
    state.receive(dict(enabled=True,simulation_only=True,timestamp_ns=1_000_000_000,
                       plan_hash='a'*64,lease_ms=250), 10.)
    assert state.current(10.1,1.1) == (True,'a'*64)
    state.plan(plan('b'))
    assert state.current(10.1,1.1) == (True,'a'*64)
    assert state.current(10.5,1.5) == (False,'a'*64)
    state.receive(dict(enabled=False,simulation_only=True,timestamp_ns=1_000_000_000,
                       plan_hash='',lease_ms=250), 10.5)
    assert state.current(10.5,1.5) == (False,'a'*64)


def test_spray_plan_subscription_matches_volatile_generator():
    from platform_simulation_app import spray_plan_qos
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    qos=spray_plan_qos()
    assert qos.durability==DurabilityPolicy.VOLATILE
    assert qos.reliability==ReliabilityPolicy.RELIABLE
