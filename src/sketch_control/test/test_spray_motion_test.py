"""Commissioning without a gun must never enable output or contact control."""
import json
import time
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from sketch_control.moveit_executor import MoveItExecutor
from test_multi_plane_spray import gun_stub
from test_painting_system_config import _load_module, WORKSPACE_SRC


def motion_test():
    node = gun_stub()
    node.spray_motion_test = True
    return node


def test_real_motion_test_accepts_absent_gun_but_keeps_force_and_compliance_checks():
    node = motion_test()
    assert not node.dry_run
    assert node._spray_motion_blockers() == ()
    node._compliance_active = True
    assert 'SPRAY_REQUIRES_FRESH_COMPLIANCE_OFF' in node._spray_motion_blockers()
    node._compliance_active = False
    node._compliance_time = time.monotonic() - 1
    assert 'SPRAY_REQUIRES_FRESH_COMPLIANCE_OFF' in node._spray_motion_blockers()
    node._compliance_time = time.monotonic()
    node.painting_force_enabled = True
    assert 'SPRAY_FORCE_MUST_BE_DISABLED' in node._spray_motion_blockers()
    node.painting_force_enabled = False
    node._painting_command_enable = True
    assert 'SPRAY_FORCE_MUST_BE_DISABLED' in node._spray_motion_blockers()


def test_motion_test_never_emits_on_during_accepted_trajectory_or_direct_request():
    node = motion_test()
    node.executing = True
    node._spray_dispatch = True
    node._active_trajectory_goal_handle = object()
    aborts = []
    node._request_motion_abort = aborts.append
    node._spray_set_output(True)
    node._spray_tick()
    node._motion_abort_requested = True
    node._spray_tick()
    assert not aborts
    assert not node._spray_on
    for message in node.spray_command_pub.messages:
        output = json.loads(message.data)
        assert output['on'] is False
        assert output['spray_motion_test'] is True
    assert node._spray_status_time == 0  # no fabricated hardware acknowledgement


@pytest.mark.parametrize('fault', ['active', 'stale', 'force'])
def test_motion_test_aborts_on_compliance_or_force_fault(fault):
    node = motion_test()
    node.executing = True
    node._active_trajectory_goal_handle = object()
    node._spray_dispatch = True
    if fault == 'active':
        node._compliance_active = True
    elif fault == 'stale':
        node._compliance_time = 0
    else:
        node.painting_force_enabled = True
    aborts = []
    node._request_motion_abort = aborts.append
    node._spray_tick()
    assert aborts and not node._spray_on


def test_motion_test_cannot_switch_to_contact_paint():
    node = motion_test()
    errors = []
    node._publish_process_mode = errors.append
    node._set_process_mode(String(data='paint'))
    assert node.process_mode == 'spray'
    assert errors == ['MOTION_TEST_REQUIRES_SPRAY_MODE']


def test_motion_test_preserves_geometry_and_robot_interlocks():
    node = motion_test()
    node._real_geometry_runtime_blockers = lambda: ('CONTROLLER_FAULT', 'PLANNING_SCENE_UNCONFIRMED')
    assert MoveItExecutor._real_runtime_blockers(node) == (
        'CONTROLLER_FAULT', 'PLANNING_SCENE_UNCONFIRMED')


def test_motion_test_launch_rejects_force_and_missing_real_motion_gate():
    module = _load_module('motion_test_launch', WORKSPACE_SRC / 'sketch_control' / 'launch' / 'rb10_painting_system.launch.py')
    flags = dict(use_fake_hardware=False, use_isaac_sim=False,
                 real_painting_enabled=True, dry_run=False,
                 painting_force_enabled=False, spray_motion_test=True)
    module._validate_interlock_values(**flags)
    with pytest.raises(RuntimeError, match='painting_force_enabled=false'):
        module._validate_interlock_values(**(flags | {'painting_force_enabled': True}))
    with pytest.raises(RuntimeError, match='real_painting_enabled=true'):
        module._validate_interlock_values(**(flags | {'real_painting_enabled': False}))


def test_motion_test_startup_is_explicit_read_only_and_initializes_spray():
    node = motion_test()
    declared = []
    def declare(name, default, descriptor):
        declared.append((name, default, descriptor.read_only))
        return SimpleNamespace(value=True)
    node.declare_parameter = declare
    node.create_publisher = lambda *args: node.spray_command_pub
    node.create_subscription = lambda *args: None
    node.create_timer = lambda *args: None
    node._init_spray()
    assert declared == [('spray_motion_test', False, True)]
    assert node.process_mode == 'spray'
    assert json.loads(node.spray_command_pub.messages[-1].data)['spray_motion_test'] is True
    node.painting_force_enabled = True
    with pytest.raises(ValueError, match='painting_force_enabled=false'):
        node._init_spray()
