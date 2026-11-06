import json
from types import SimpleNamespace
from test_spray_motion_test import motion_test


class Capture:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(json.loads(msg.data))


def test_simulated_spray_is_on_only_during_accepted_stroke_and_gun_stays_off():
    node = motion_test()
    node._spray_visual_pub = Capture()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000000000))
    node._active_segment_path = SimpleNamespace(path_id='p', plan_hash='a'*64)
    node.executing = True
    node._spray_dispatch = True
    node._spray_tick()
    assert node._spray_visual_pub.messages[-1]['enabled'] is False
    node._active_trajectory_goal_handle = object()
    node._spray_tick()
    active = node._spray_visual_pub.messages[-1]
    assert active['enabled'] is True and active['plan_hash'] == 'a'*64
    assert active['timestamp_ns'] == 1000000000
    node._spray_off()
    assert node._spray_visual_pub.messages[-1]['enabled'] is False
    assert all(json.loads(m.data)['on'] is False for m in node.spray_command_pub.messages)
    node._spray_dispatch = True
    node._motion_abort_requested = True
    node._spray_tick()
    assert node._spray_visual_pub.messages[-1]['enabled'] is False

