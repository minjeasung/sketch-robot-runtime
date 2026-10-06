import json
import time
import pytest
from test_contracts import module

JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')


def robot():
    links = ''.join('<link name="%s"/>' % n for n in ('link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'spray_eoat', 'tcp'))
    joints = ''.join('<joint name="%s" type="revolute"><parent link="link%d"/><child link="link%d"/><limit lower="-3" upper="3" velocity="1" effort="100"/></joint>' % (n, i, i+1) for i, n in enumerate(JOINTS))
    return '<robot name="rb20_spray">' + links + joints + '<joint name="spray_mount_joint" type="fixed"><parent link="link6"/><child link="spray_eoat"/></joint><joint name="tcp_joint" type="fixed"><parent link="spray_eoat"/><child link="tcp"/><origin xyz="0 0 .3"/></joint></robot>'


def semantic():
    return '<robot name="rb20_spray"><group name="manipulator"><chain base_link="link0" tip_link="tcp"/></group><disable_collisions link1="link0" link2="link1"/></robot>'


def limits():
    return {n: dict(has_velocity_limits=True, max_velocity=.5, has_acceleration_limits=True, max_acceleration=.5) for n in JOINTS}


def test_external_model_uses_tcp_zero_and_changes_on_mount_or_semantics():
    m = module('model')
    model = m.parse_stack_model(robot(), semantic(), limits(), {})
    assert model['model_id'] == 'rb20_1900es'
    assert model['endpoint_tcp_m'] == [0., 0., 0.]
    assert model['spray_tool_axis'] == '+z'
    assert model['fingerprint'] != m.parse_stack_model(robot().replace('0 0 .3', '0 0 .4'), semantic(), limits(), {})['fingerprint']
    assert model['fingerprint'] != m.parse_stack_model(robot(), semantic().replace('link2="link1"', 'link2="link2"'), limits(), {})['fingerprint']


def test_model_rejects_missing_limits_foreign_pairs_and_wrong_robot():
    m = module('model')
    for urdf, srdf, joint_limits in [(robot().replace('rb20_spray', 'rb20_ft_preview'), semantic(), limits()), (robot(), semantic().replace('link2="link1"', 'link2="foreign"'), limits()), (robot(), semantic(), {})]:
        with pytest.raises(ValueError):
            m.parse_stack_model(urdf, srdf, joint_limits, {})


def test_guard_revokes_on_competition_stale_or_changed_model_without_resume():
    m = module('execution_guard')
    g = m.ExecutionGuard()
    g.update(model='m', source='s', controllers={'joint_trajectory_controller': 'active', 'joint_state_broadcaster':'active'}, competitors=[], now=10)
    assert not g.blockers(10.1, model='m', source='s')
    g.arm('m', 's', now=10.1)
    g.update(model='m', source='s', controllers={'joint_trajectory_controller': 'active'}, competitors=['rb20_spray_executor'], now=10.2)
    assert 'EXTERNAL_MOTION_OWNER' in g.blockers(10.2, model='m', source='s')
    g.update(model='m', source='s', controllers={'joint_trajectory_controller': 'active'}, competitors=[], now=10.3)
    assert 'REARM_REQUIRED' in g.blockers(10.3, model='m', source='s')
    g.reset()
    assert 'EXTERNAL_STATE_STALE' in g.blockers(12, model='m', source='s')


def test_guard_rejects_missing_model_source_and_compliance():
    g = module('execution_guard').ExecutionGuard()
    g.update(model='', source='', controllers={'joint_trajectory_controller':'active', 'admittance_controller':'active'}, competitors=[], now=1)
    assert set(g.blockers(1.1)) >= {'EXTERNAL_MODEL_MISSING', 'EXTERNAL_SOURCE_MISSING', 'COMPETING_CONTROLLER'}
