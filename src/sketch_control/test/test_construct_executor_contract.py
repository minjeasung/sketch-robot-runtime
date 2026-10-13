"""Construct uses original executor with its own joints, frames and collision model."""
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sketch_control import robot_models
from sketch_control.moveit_executor import MoveItExecutor

SRC = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_construct_model_loads_own_semantics_and_joint_limits(side):
    model = f'construct_{side}'
    assert robot_models.validate_model(model) == model
    contract = robot_models.model_execution_contract(model)
    assert contract['planning_group'] == f'{side}_manipulator'
    assert contract['ee_link'] == f'{side}_manipulator_ee_point'
    assert contract['base_frame'] == 'World'
    limits = robot_models.model_joint_limits(model, SRC / 'construct_description', SRC / 'construct_moveit_config')
    assert set(limits) == {f'{side}_manipulator_joint{i}' for i in range(1, 7)}
    root = ET.fromstring(robot_models.model_srdf(model, SRC / 'construct_moveit_config/config/construct_robot_0528.srdf'))
    assert root.find(f"group[@name='{side}_manipulator']") is not None
    assert any(e.get('reason') == 'Never' for e in root.findall('disable_collisions'))


def test_construct_joint_guard_accepts_selected_arm_and_rejects_other_arm():
    names = tuple(f'left_manipulator_joint{i}' for i in range(1, 7))
    errors = []
    node = NS(arm_joint_names=names, _robot_joint_limits={name: (-3.14, 3.14) for name in names},
              get_logger=lambda: NS(error=errors.append))
    state = JointState(name=list(names), position=[0.] * 6)
    assert MoveItExecutor._joint_state_within_limits(node, state, 'construct')
    traj = JointTrajectory(joint_names=list(names), points=[JointTrajectoryPoint(positions=[0.] * 6)])
    assert MoveItExecutor._trajectory_within_joint_limits(node, traj, 'construct')
    state.name = [name.replace('left_', 'right_') for name in names]
    assert not MoveItExecutor._joint_state_within_limits(node, state, 'wrong arm')
    traj.points[0].positions[0] = 4.
    assert not MoveItExecutor._trajectory_within_joint_limits(node, traj, 'out of bounds')


def test_construct_pose_constraints_target_original_tcp_in_world():
    node = NS(ee_link='left_manipulator_ee_point', base_frame='World')
    constraints = MoveItExecutor._make_pose_constraints(node, Pose())
    assert constraints.position_constraints[0].link_name == 'left_manipulator_ee_point'
    assert constraints.orientation_constraints[0].header.frame_id == 'World'


def test_construct_nearest_equivalent_respects_construct_limits():
    assert MoveItExecutor._nearest_joint_equivalent('left_manipulator_joint1', 6.2, 0.,
        {'left_manipulator_joint1': (-3.14, 3.14)}) == pytest.approx(6.2 - 2 * 3.141592653589793)


def test_construct_keeps_native_urdf_tool_collision_without_duplicate_attachment():
    node = NS(model_id='construct_left')
    profile = NS(vertices_tcp_m=[[0., 0., 0.], [0., 1., 0.], [1., 0., 0.]], faces=[[0, 1, 2]])
    assert MoveItExecutor._spray_eoat_collision_object(node, profile) is None


def test_rb_spray_still_attaches_verified_tool_mesh():
    node = NS(model_id='rb10_1300e_u')
    profile = NS(vertices_tcp_m=[[0., 0., 0.], [0., 1., 0.], [1., 0., 0.]], faces=[[0, 1, 2]])
    attached = MoveItExecutor._spray_eoat_collision_object(node, profile)
    assert attached.link_name == 'tcp'
    assert len(attached.object.meshes[0].vertices) == 3


def test_tcp_diagnostic_never_blocks_compliance_callback_on_missing_world_alias():
    """A debug TF lookup must not occupy the 300ms compliance freshness window."""
    calls = []
    tf = NS(transform=NS(translation=NS(x=0., y=0., z=0.),
                         rotation=NS(x=0., y=0., z=0., w=1.)))
    def lookup(target, source, stamp, *, timeout):
        calls.append((target, source, timeout.nanoseconds))
        if target == 'world':
            raise LookupError('Construct only has World')
        return tf
    node = NS(ee_link='left_manipulator_ee_point', tf_buffer=NS(lookup_transform=lookup),
              get_logger=lambda: NS(info=lambda message: None))
    MoveItExecutor._log_current_tcp(node)
    assert calls
    assert all(timeout == 0 for _, _, timeout in calls)
    assert any(target == 'World' and source == 'left_manipulator_ee_point'
               for target, source, _ in calls)
