"""Supervisor status must preserve pending-action ownership after high-level abort."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from sketch_control.moveit_executor import MoveItExecutor


@pytest.mark.parametrize('values,expected', [
    ({'executing':True}, {'running':True,'stopping':False}),
    ({'executing':False,'_active_trajectory_goal_token':object()}, {'running':False,'stopping':True}),
    ({'executing':False,'_active_trajectory_cancel_requested':True}, {'running':False,'stopping':True}),
    ({'executing':False,'_fjt_motion_state_unknown':True}, {'running':False,'stopping':True}),
    ({'executing':False,'_joint_command_timer':object()}, {'running':False,'stopping':True}),
    ({'executing':False,'_active_trajectory_goal_token':None,'_active_trajectory_cancel_requested':False}, {'running':False,'stopping':False}),
])
def test_motion_activity_reports_actual_pending_action_even_after_abort(values,expected):
    assert MoveItExecutor._motion_activity_status(SimpleNamespace(**values))==expected
