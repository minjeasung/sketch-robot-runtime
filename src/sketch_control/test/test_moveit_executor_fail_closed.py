from types import MethodType, SimpleNamespace
import json
import time

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint


moveit_executor = pytest.importorskip("sketch_control.moveit_executor")


class _Logger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(str(message))

    def info(self, message):
        self.infos.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))


class _MissingSegmentExecutor:
    """Minimal adapter proving the execute callback returns before planning."""

    real_painting_enabled = True
    active_target_name = "wall"
    _acm_baseline_verified = True
    _acm_baseline_verified_target_name = "wall"
    dry_run = False
    current_waypoints = [object()]

    def __init__(self):
        self.statuses = []
        self.plan_or_action_calls = 0
        self._logger = _Logger()
        self._acm_baseline_verified_time = time.monotonic()

    def _matching_segment_path(self):
        return None

    def _current_real_plan_blockers(self):
        return ("MATCHING_V3_SEGMENT_REQUIRED",)

    def _publish_execution_status(self, state, reason):
        self.statuses.append((state, reason))

    def get_logger(self):
        return self._logger

    def _plan_joint_goal(self, *args, **kwargs):
        self.plan_or_action_calls += 1

    def _send_follow_joint_trajectory(self, *args, **kwargs):
        self.plan_or_action_calls += 1


class _ReadyCartesianClient:
    def wait_for_service(self, **_kwargs):
        return True


class _ReadyEndpoint:
    def service_is_ready(self):
        return True

    def server_is_ready(self):
        return True


class _ControlledFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None
        self._done = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def done(self):
        return self._done

    def resolve(self, result):
        self._result = result
        self._done = True
        for callback in list(self._callbacks):
            callback(self)


class _DeferredCallbackFuture(_ControlledFuture):
    """Future whose result is ready while callback delivery is still queued."""

    def set_result_without_callbacks(self, result):
        self._result = result
        self._done = True

    def fire_callbacks(self):
        for callback in list(self._callbacks):
            callback(self)


class _QueuedIKClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def wait_for_service(self, **_kwargs):
        return True

    def call_async(self, request):
        future = _ControlledFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


class _QueuedActionClient:
    def __init__(self):
        self.goals = []
        self.futures = []

    def wait_for_server(self, **_kwargs):
        return True

    def send_goal_async(self, goal):
        future = _ControlledFuture()
        self.goals.append(goal)
        self.futures.append(future)
        return future


class _FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def _pose(x=0.0, y=0.0, z=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


def _robot_trajectory(position_rows, times=None):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(
        moveit_executor.READY_POSE_JOINTS
    )
    if times is None:
        times = [0.1 * (index + 1) for index in range(len(position_rows))]
    for positions, stamp in zip(position_rows, times):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = int(stamp)
        point.time_from_start.nanosec = int(
            round((float(stamp) - int(stamp)) * 1e9)
        )
        trajectory.joint_trajectory.points.append(point)
    return trajectory


def _work_area_corners_message(points, frame_id="link0"):
    message = moveit_executor.PoseArray()
    message.header.frame_id = str(frame_id)
    for point in points:
        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2])
        pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def _work_area_scene_executor():
    logger = _Logger()
    identity_transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    executor = SimpleNamespace(
        executing=False,
        _execution_snapshot=None,
        _work_area_corners_signature=None,
        dynamic_work_area_corners=None,
        dynamic_surface_source="zed",
        scene_confirmed=True,
        scene_initialized=True,
        get_logger=lambda: logger,
        _canonical_world_frame=(
            moveit_executor.MoveItExecutor._canonical_world_frame
        ),
        _lookup_transform_to_base=lambda *_args, **_kwargs: identity_transform,
    )
    executor._set_dynamic_work_area_corners = lambda points: (
        moveit_executor.MoveItExecutor._set_dynamic_work_area_corners(
            executor, points
        )
    )
    return executor


@pytest.mark.parametrize("frame_id", ("link0", "zed_left_camera_frame_optical"))
def test_duplicate_work_area_corners_preserve_confirmed_scene(frame_id):
    points = np.array(
        [
            [0.8, -0.2, 0.6],
            [0.8, 0.2, 0.6],
            [0.8, 0.2, 1.0],
            [0.8, -0.2, 1.0],
        ],
        dtype=float,
    )
    message = _work_area_corners_message(points, frame_id=frame_id)
    executor = _work_area_scene_executor()

    moveit_executor.MoveItExecutor.on_work_area_corners(executor, message)
    assert executor.scene_confirmed is False
    assert executor.scene_initialized is False

    # Model a successful apply, then receive the projector's identical
    # periodic republish.  It must not schedule another PlanningScene apply.
    executor.scene_confirmed = True
    executor.scene_initialized = True
    accepted_geometry = executor.dynamic_work_area_corners.copy()
    moveit_executor.MoveItExecutor.on_work_area_corners(executor, message)

    assert executor.scene_confirmed is True
    assert executor.scene_initialized is True
    np.testing.assert_array_equal(
        executor.dynamic_work_area_corners, accepted_geometry
    )


def test_changed_work_area_corners_invalidate_confirmed_scene_once():
    initial = np.array(
        [
            [0.8, -0.2, 0.6],
            [0.8, 0.2, 0.6],
            [0.8, 0.2, 1.0],
            [0.8, -0.2, 1.0],
        ],
        dtype=float,
    )
    changed = initial.copy()
    changed[:, 2] += 0.01
    executor = _work_area_scene_executor()

    moveit_executor.MoveItExecutor.on_work_area_corners(
        executor, _work_area_corners_message(initial)
    )
    executor.scene_confirmed = True
    executor.scene_initialized = True
    moveit_executor.MoveItExecutor.on_work_area_corners(
        executor, _work_area_corners_message(changed)
    )

    assert executor.scene_confirmed is False
    assert executor.scene_initialized is False
    np.testing.assert_allclose(executor.dynamic_work_area_corners, changed)


def test_planning_scene_confirmation_requires_exact_geometry_revision():
    logger = _Logger()
    executor = SimpleNamespace(
        _scene_revision=2,
        _scene_confirmed_revision=-1,
        _scene_apply_inflight_revision=1,
        scene_confirmed=False,
        get_logger=lambda: logger,
    )
    success_future = SimpleNamespace(
        result=lambda: SimpleNamespace(success=True)
    )

    # Revision 1 was applied after geometry revision 2 became authoritative.
    # It must release the in-flight slot but may not confirm revision 2.
    moveit_executor.MoveItExecutor._apply_scene_done(
        executor, success_future, 1
    )
    assert executor.scene_confirmed is False
    assert executor._scene_apply_inflight_revision is None

    # An already-queued callback from an even older request must not clear the
    # slot now owned by the current revision.
    executor._scene_apply_inflight_revision = 2
    moveit_executor.MoveItExecutor._apply_scene_done(
        executor, success_future, 1
    )
    assert executor._scene_apply_inflight_revision == 2
    assert executor.scene_confirmed is False

    executor._scene_apply_inflight_revision = 2
    moveit_executor.MoveItExecutor._apply_scene_done(
        executor, success_future, 2
    )
    assert executor.scene_confirmed is True
    assert executor._scene_confirmed_revision == 2
    assert executor._scene_apply_inflight_revision is None


def _zero_distance_paint_executor(hold_s):
    calls = []
    node = SimpleNamespace(
        cartesian_client=_ReadyCartesianClient(),
        _process_row_tcp_poses={1: _pose(0.4, 0.0, 0.5)},
        _process_last_tcp_pose=_pose(0.4, 0.0, 0.5),
        stationary_paint_hold_s=float(hold_s),
        _start_stationary_paint_hold=lambda step, pose: calls.append(
            ("hold", step, pose)
        ),
        _process_motion_done=lambda pose: calls.append(("done", pose)),
        _complete_process_step=lambda: calls.append(("empty",)),
    )
    step = SimpleNamespace(
        mode="PAINT",
        rows=(SimpleNamespace(row_number=1),),
    )
    return node, step, calls


def test_real_execute_without_matching_v3_segment_sends_no_plan_or_fjt_goal():
    executor = _MissingSegmentExecutor()

    moveit_executor.MoveItExecutor.on_execute(
        executor, SimpleNamespace(data=True)
    )

    assert executor.plan_or_action_calls == 0
    assert executor.statuses == [
        ("PLAN_REJECTED", "MATCHING_V3_SEGMENT_REQUIRED")
    ]
    assert executor._logger.errors


def test_execution_abort_latch_rejects_duplicate_run_before_any_plan_call():
    executor = _MissingSegmentExecutor()
    executor._motion_abort_requested = True

    moveit_executor.MoveItExecutor.on_execute(
        executor, SimpleNamespace(data=True)
    )

    assert executor.plan_or_action_calls == 0
    assert executor.statuses == [("NOT_READY", "EXECUTION_ABORT_LATCHED")]


def test_execution_abort_reset_is_explicit_and_requires_safe_idle():
    executor = SimpleNamespace(
        executing=False,
        _active_trajectory_goal_token=None,
        _painting_command_enable=False,
        real_painting_enabled=False,
        _motion_abort_requested=True,
        _free_space_confirmed=True,
        _execution_free_space_confirmed=True,
        _robot_stationary_for_bias=lambda: True,
        _cancel_runtime_tare=lambda: None,
        _publish_free_space_confirmation=lambda _confirmed: None,
        _publish_painting_command=lambda *_args, **_kwargs: None,
        _publish_execution_status=lambda *_args, **_kwargs: None,
    )
    response = SimpleNamespace(success=False, message="")

    result = moveit_executor.MoveItExecutor.on_reset_execution_abort(
        executor, None, response
    )

    assert result.success is True
    assert executor._motion_abort_requested is False
    assert executor._free_space_confirmed is False


def test_execution_abort_reset_rejects_active_trajectory():
    executor = SimpleNamespace(
        executing=True,
        _active_trajectory_goal_token=object(),
        _painting_command_enable=False,
        real_painting_enabled=False,
        _motion_abort_requested=True,
        _free_space_confirmed=False,
        _execution_free_space_confirmed=False,
        _robot_stationary_for_bias=lambda: False,
    )
    response = SimpleNamespace(success=False, message="")

    result = moveit_executor.MoveItExecutor.on_reset_execution_abort(
        executor, None, response
    )

    assert result.success is False
    assert "TRAJECTORY_ACTIVE" in result.message
    assert executor._motion_abort_requested is True


def test_zero_distance_paint_uses_stationary_hold_only_when_enabled():
    executor, step, calls = _zero_distance_paint_executor(hold_s=2.0)

    moveit_executor.MoveItExecutor._plan_process_motion_step(executor, step)

    assert calls and calls[0][0] == "hold"


def test_zero_duration_preserves_legacy_zero_distance_completion():
    executor, step, calls = _zero_distance_paint_executor(hold_s=0.0)

    moveit_executor.MoveItExecutor._plan_process_motion_step(executor, step)

    assert calls and calls[0][0] == "done"


def test_stationary_paint_holds_after_required_motion_reaches_nominal_pose():
    calls = []
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _process_last_tcp_pose=None,
        _start_stationary_paint_hold=lambda step, pose: calls.append(
            ("hold", step, pose)
        ),
        _process_motion_done=lambda pose: calls.append(("done", pose)),
    )
    step = SimpleNamespace(mode="PAINT")
    final_pose = _pose(0.4, 0.0, 0.5)

    moveit_executor.MoveItExecutor._process_motion_target_reached(
        executor, step, final_pose, True
    )

    assert calls and calls[0][0] == "hold"
    assert executor._process_last_tcp_pose.position.x == pytest.approx(0.4)


def test_dry_run_central_dispatch_sends_no_physical_backend_command():
    calls = []
    logger = _Logger()
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["base"]
    point = JointTrajectoryPoint()
    point.positions = [0.0]
    trajectory.joint_trajectory.points = [point]
    executor = SimpleNamespace(
        dry_run=True,
        _motion_abort_requested=False,
        executing=True,
        execution_backend="follow_joint_trajectory",
        _trajectory_within_joint_limits=lambda *_args: True,
        _execute_trajectory_joint_command=lambda *_args, **_kwargs: calls.append(
            "joint_command"
        ),
        _execute_trajectory_follow_joint=lambda *_args, **_kwargs: calls.append(
            "follow_joint_trajectory"
        ),
        get_logger=lambda: logger,
    )

    result = moveit_executor.MoveItExecutor.execute_trajectory_direct(
        executor,
        trajectory,
        on_complete=lambda: calls.append("complete"),
        label="test trajectory",
    )

    assert result is True
    assert calls == ["complete"]
    assert logger.infos and "physical dispatch blocked" in logger.infos[-1]


def _stage1_joint_state(values):
    state = moveit_executor.JointState()
    state.name = list(moveit_executor.READY_POSE_JOINTS.keys())
    state.position = [float(value) for value in values]
    return state


def _stage1_candidate(name, quaternion):
    safety = _pose(0.45, 0.10, 0.80)
    safety.orientation.x = float(quaternion[0])
    safety.orientation.y = float(quaternion[1])
    safety.orientation.z = float(quaternion[2])
    safety.orientation.w = float(quaternion[3])
    return {
        "name": str(name),
        "safety_tcp_pose": safety,
        "retreat_tcp_pose": _pose(0.45, 0.10, 0.80),
        "motion_tip_poses": [_pose()],
        "motion_tcp_poses": [_pose()],
        "row_tcp_poses": {1: _pose()},
    }


def _stage1_ik_response(values=None, error_code=1):
    response = moveit_executor.GetPositionIK.Response()
    response.error_code.val = int(error_code)
    if values is not None:
        response.solution.joint_state = _stage1_joint_state(values)
    return response


def _dual_stage1_ik_executor():
    client = _QueuedIKClient()
    logger = _Logger()
    plans = []
    failures = []
    destroyed_timers = []
    token = object()
    node = SimpleNamespace(
        ik_client=client,
        current_joint_state=_stage1_joint_state([0.0] * 6),
        _stage1_ik_seed_state=None,
        _stage1_ik_candidate_generation=0,
        _stage1_ik_candidate_results={},
        _stage1_ik_candidate_timer=None,
        _stage1_attempt_token=token,
        _stage1_orientation_candidates=(
            _stage1_candidate("camera_up", (0.0, 0.0, 0.0, 1.0)),
            _stage1_candidate("camera_down", (0.0, 1.0, 0.0, 0.0)),
        ),
        _stage1_orientation_ranked=[],
        _stage1_orientation_rank_index=-1,
        _stage1_orientation_branch_frozen=False,
        _selected_segment_orientation_branch="",
        _stage1_retried=False,
        _motion_abort_requested=False,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time()
            )
        ),
        get_logger=lambda: logger,
        create_timer=lambda _period, callback: _FakeTimer(callback),
        destroy_timer=lambda timer: destroyed_timers.append(timer),
        _send_stage1_plan=lambda constraints, label, **kwargs: plans.append(
            (constraints, label, kwargs)
        ),
        _fail_stage1_before_motion=lambda reason: failures.append(str(reason)),
    )
    for method_name in (
        "_cancel_stage1_ik_candidate_timer",
        "_stage1_candidate_joint_metrics",
        "_request_stage1_orientation_candidate_iks",
        "_stage1_orientation_candidate_ik_done",
        "_finish_stage1_orientation_candidate_iks",
        "_activate_stage1_orientation_rank",
        "_apply_segment_orientation_candidate",
        "_make_joint_goal_constraints",
    ):
        setattr(
            node,
            method_name,
            MethodType(
                getattr(moveit_executor.MoveItExecutor, method_name), node
            ),
        )
    node._nearest_joint_equivalent = (
        moveit_executor.MoveItExecutor._nearest_joint_equivalent
    )
    return node, client, plans, failures


def test_dual_roller_ik_checks_both_collisions_with_same_seed_and_waits_for_both():
    node, client, plans, failures = _dual_stage1_ik_executor()

    node._request_stage1_orientation_candidate_iks()

    assert len(client.requests) == 2
    assert all(request.ik_request.avoid_collisions for request in client.requests)
    assert all(
        request.ik_request.ik_link_name == moveit_executor.EE_LINK
        and request.ik_request.pose_stamped.header.frame_id
        == moveit_executor.BASE_FRAME
        for request in client.requests
    )
    assert client.requests[0].ik_request.robot_state.joint_state.position == (
        client.requests[1].ik_request.robot_state.joint_state.position
    )
    q0 = client.requests[0].ik_request.pose_stamped.pose.orientation
    q1 = client.requests[1].ik_request.pose_stamped.pose.orientation
    assert (q0.x, q0.y, q0.z, q0.w) != (q1.x, q1.y, q1.z, q1.w)

    # Resolve the farther branch first. Selection must wait for the other
    # collision-aware result and remain independent of callback order.
    client.futures[1].resolve(_stage1_ik_response([0.8] * 6))
    assert plans == []
    client.futures[0].resolve(_stage1_ik_response([0.2] * 6))

    assert failures == []
    assert len(plans) == 1
    assert node._selected_segment_orientation_branch == "camera_up"
    assert plans[0][2]["candidate_rank_index"] == 0


def test_dual_roller_ik_rejects_collision_branch_and_uses_other_branch():
    node, client, plans, failures = _dual_stage1_ik_executor()
    node._request_stage1_orientation_candidate_iks()

    client.futures[0].resolve(_stage1_ik_response(error_code=-31))
    assert plans == []
    client.futures[1].resolve(
        _stage1_ik_response([1.6, 0.0, 0.0, 0.0, 0.0, 0.0])
    )

    assert failures == []
    assert len(plans) == 1
    assert node._selected_segment_orientation_branch == "camera_down"
    assert any(
        "large smooth rotation" in message
        for message in node.get_logger().warnings
    )


def test_dual_roller_ik_requires_both_collision_checks_to_complete():
    node, client, plans, failures = _dual_stage1_ik_executor()
    node._request_stage1_orientation_candidate_iks()

    # FRAME_TRANSFORM_FAILURE is not evidence that this candidate is collision
    # invalid.  Even if the other candidate succeeds, the two-check contract
    # requires a terminal rejection instead of silently selecting it.
    client.futures[0].resolve(_stage1_ik_response(error_code=-21))
    client.futures[1].resolve(_stage1_ik_response([0.2] * 6))

    assert plans == []
    assert len(failures) == 1
    assert failures[0].startswith(
        "ROLLER_ORIENTATION_COLLISION_CHECK_INCOMPLETE"
    )


@pytest.mark.parametrize(
    "malformed",
    ("missing", "nan", "duplicate"),
)
def test_dual_roller_ik_malformed_results_fail_closed_without_plan(malformed):
    node, client, plans, failures = _dual_stage1_ik_executor()
    node._request_stage1_orientation_candidate_iks()
    responses = []
    for _ in range(2):
        response = _stage1_ik_response([0.1] * 6)
        joint_state = response.solution.joint_state
        if malformed == "missing":
            joint_state.name.pop()
            joint_state.position.pop()
        elif malformed == "nan":
            joint_state.position[-1] = float("nan")
        else:
            joint_state.name[-1] = joint_state.name[0]
        responses.append(response)

    client.futures[1].resolve(responses[1])
    client.futures[0].resolve(responses[0])

    assert plans == []
    assert len(failures) == 1
    assert failures[0].startswith("NO_COLLISION_FREE_ROLLER_ORIENTATION")


def test_dual_roller_ik_timeout_invalidates_late_candidate_response():
    node, client, plans, failures = _dual_stage1_ik_executor()
    node._request_stage1_orientation_candidate_iks()
    timer = node._stage1_ik_candidate_timer

    client.futures[0].resolve(_stage1_ik_response([0.1] * 6))
    assert plans == []
    timer.callback()
    assert len(failures) == 1
    assert failures[0].startswith(
        "ROLLER_ORIENTATION_COLLISION_CHECK_INCOMPLETE"
    )
    assert plans == []

    # Model the terminal cleanup's generation invalidation before an old
    # service response eventually arrives.
    node._stage1_ik_candidate_generation += 1
    client.futures[1].resolve(_stage1_ik_response([0.1] * 6))
    assert plans == []
    assert len(failures) == 1


def test_nonfinite_trajectory_is_rejected_before_physical_dispatch():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(
        moveit_executor.READY_POSE_JOINTS.keys()
    )
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 5 + [float("nan")]
    trajectory.joint_trajectory.points = [point]
    backend_calls = []
    executor = SimpleNamespace(
        dry_run=False,
        active_target_name="wall",
        _acm_baseline_verified=True,
        _acm_baseline_verified_target_name="wall",
        _acm_baseline_verified_time=time.monotonic(),
        execution_backend="follow_joint_trajectory",
        _fjt_motion_state_unknown=False,
        _hardware_motion_inhibited=False,
        _motion_abort_requested=False,
        _trajectory_within_joint_limits=lambda jt, label: (
            moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
                executor, jt, label
            )
        ),
        _execute_trajectory_follow_joint=lambda traj, **_kwargs: (
            (backend_calls.append(True) or True)
            if moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
                executor, traj.joint_trajectory, "fake backend"
            )
            else False
        ),
        get_logger=lambda: _Logger(),
    )

    assert not moveit_executor.MoveItExecutor.execute_trajectory_direct(
        executor, trajectory, label="nonfinite"
    )
    assert backend_calls == []


def test_active_robot_model_bounds_and_discontinuity_guard_are_consistent():
    logger = _Logger()
    executor = SimpleNamespace(
        get_logger=lambda: logger,
        _max_joint_delta=moveit_executor.MoveItExecutor._max_joint_delta,
    )

    base_outside = _robot_trajectory([[3.2, 0.0, 0.0, 0.0, 0.0, 0.0]])
    wrist3_override = _robot_trajectory([[0.0, 0.0, 0.0, 0.0, 0.0, 3.2]])
    branch_jump = _robot_trajectory(
        [[0.0] * 6, [0.0, 1.50, 0.0, 0.0, 0.0, 0.0]]
    )
    nonmonotonic = _robot_trajectory(
        [[0.0] * 6, [0.01] * 6], times=[0.1, 0.1]
    )

    assert not moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
        executor, base_outside.joint_trajectory, "base-outside"
    )
    assert moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
        executor, wrist3_override.joint_trajectory, "wrist3-override"
    )
    assert not moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
        executor, branch_jump.joint_trajectory, "branch-jump"
    )
    assert not moveit_executor.MoveItExecutor._trajectory_within_joint_limits(
        executor, nonmonotonic.joint_trajectory, "nonmonotonic"
    )


def test_known_predispatch_failure_rejects_run_without_motion_abort():
    calls = []
    statuses = []
    executor = SimpleNamespace(
        _active_trajectory_goal_token=None,
        _joint_command_timer=None,
        _fjt_motion_state_unknown=False,
        _acm_state_unknown=False,
        _painting_command_enable=False,
        _motion_abort_requested=False,
        executing=True,
        get_logger=lambda: _Logger(),
        _reset_painting_process=lambda: calls.append("reset"),
        _request_motion_abort=lambda reason: calls.append(("abort", reason)),
        _publish_execution_status=lambda *args, **kwargs: statuses.append(
            (args, kwargs)
        ),
    )

    assert moveit_executor.MoveItExecutor._fail_workflow_known_safe(
        executor, "PLANNER_REJECTED"
    )

    assert calls == ["reset"]
    assert executor.executing is False
    assert statuses[0][0][:2] == ("PLAN_REJECTED", "PLANNER_REJECTED")


def test_fjt_goal_rejection_is_soft_but_accepted_execution_failure_is_hard():
    client = _QueuedActionClient()
    rejected = []
    failures = []
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _active_trajectory_goal_token=None,
        _active_trajectory_goal_handle=None,
        _active_trajectory_label="",
        _active_trajectory_cancel_requested=False,
        _active_trajectory_result_deadline=0.0,
        _active_trajectory_cancel_deadline=0.0,
        _fjt_motion_state_unknown=False,
        _last_trajectory_failure_phase="",
        _contact_search_cancel_on_contact=False,
        fjt_result_timeout_margin_s=2.0,
        traj_action_client=client,
        executing=True,
        get_logger=lambda: _Logger(),
        _trajectory_within_joint_limits=lambda *_args: True,
        _point_time_sec=lambda point: (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        ),
        _start_fjt_guard_timer=lambda *_args: None,
        _cancel_fjt_guard_timer=lambda: None,
        _latch_fjt_motion_unknown=lambda *_args: pytest.fail(
            "known action outcomes must not be classified as unknown"
        ),
        _complete_contact_search_after_early_contact=lambda: None,
    )
    executor._clear_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._clear_active_follow_joint_goal,
        executor,
    )
    trajectory = _robot_trajectory([[0.0] * 6])

    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        executor,
        trajectory,
        on_failure=lambda: failures.append("failure"),
        on_rejected=lambda: rejected.append("rejected"),
        label="test-rejected",
    )
    client.futures[0].resolve(SimpleNamespace(accepted=False))
    assert rejected == ["rejected"]
    assert failures == []
    assert executor._active_trajectory_goal_token is None

    executor.executing = True
    result_future = _ControlledFuture()
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: result_future,
    )
    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        executor,
        trajectory,
        on_failure=lambda: failures.append("failure"),
        on_rejected=lambda: rejected.append("rejected"),
        label="test-execution-failure",
    )
    client.futures[1].resolve(handle)
    result_future.resolve(
        SimpleNamespace(
            status=moveit_executor.GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(
                error_code=-1,
                error_string="controller rejected execution",
            ),
        )
    )
    assert rejected == ["rejected"]
    assert failures == ["failure"]


def test_repeated_abort_does_not_extend_pending_goal_cancel_deadline(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    executor = SimpleNamespace(
        _active_trajectory_goal_token=object(),
        _active_trajectory_goal_handle=None,
        _active_trajectory_cancel_requested=False,
        _active_trajectory_cancel_deadline=0.0,
        fjt_cancel_timeout_s=2.0,
        get_logger=lambda: _Logger(),
    )

    moveit_executor.MoveItExecutor._cancel_active_follow_joint_goal(
        executor, "first abort"
    )
    first_deadline = executor._active_trajectory_cancel_deadline
    clock["now"] = 100.0
    moveit_executor.MoveItExecutor._cancel_active_follow_joint_goal(
        executor, "repeated abort"
    )

    assert first_deadline == pytest.approx(12.0)
    assert executor._active_trajectory_cancel_deadline == first_deadline


def test_late_goal_acceptance_sends_queued_cancel_without_extending_deadline(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    client = _QueuedActionClient()
    cancel_future = _ControlledFuture()
    result_future = _ControlledFuture()
    cancel_requests = []
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=lambda: (
            cancel_requests.append(True) or cancel_future
        ),
        get_result_async=lambda: result_future,
    )
    executor = SimpleNamespace(
        _motion_abort_requested=True,
        _active_trajectory_goal_token=None,
        _active_trajectory_goal_handle=None,
        _active_trajectory_label="",
        _active_trajectory_cancel_requested=False,
        _active_trajectory_result_deadline=0.0,
        _active_trajectory_cancel_deadline=0.0,
        _fjt_motion_state_unknown=False,
        _last_trajectory_failure_phase="",
        _contact_search_cancel_on_contact=False,
        fjt_result_timeout_margin_s=2.0,
        fjt_cancel_timeout_s=2.0,
        traj_action_client=client,
        executing=True,
        get_logger=lambda: _Logger(),
        _trajectory_within_joint_limits=lambda *_args: True,
        _point_time_sec=lambda point: (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        ),
        _start_fjt_guard_timer=lambda *_args: None,
        _cancel_fjt_guard_timer=lambda: None,
        _latch_fjt_motion_unknown=lambda *_args: None,
        _complete_contact_search_after_early_contact=lambda: None,
    )
    executor._cancel_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._cancel_active_follow_joint_goal,
        executor,
    )
    executor._clear_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._clear_active_follow_joint_goal,
        executor,
    )
    trajectory = _robot_trajectory([[0.0] * 6])

    # Model a goal request sent immediately before an external abort arrives.
    executor._motion_abort_requested = False
    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        executor, trajectory, label="late-acceptance"
    )
    executor._motion_abort_requested = True
    executor._cancel_active_follow_joint_goal("abort while response pending")
    first_deadline = executor._active_trajectory_cancel_deadline
    assert first_deadline == pytest.approx(12.0)

    clock["now"] = 11.9
    client.futures[0].resolve(handle)

    assert cancel_requests == [True]
    assert executor._active_trajectory_cancel_deadline == first_deadline


@pytest.mark.parametrize(
    "callback_name",
    ("_stage2_done", "_cartesian_done", "_stage4_done"),
)
def test_legacy_cartesian_callbacks_never_dispatch_failed_moveit_result(
    callback_name,
):
    rejections = []
    dispatches = []
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=-1),
        fraction=1.0,
        solution=_robot_trajectory([[0.0] * 6]),
    )
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        get_logger=lambda: _Logger(),
        _fail_workflow_known_safe=lambda reason, **_kwargs: rejections.append(
            str(reason)
        ),
        execute_trajectory_direct=lambda *_args, **_kwargs: (
            dispatches.append(True) or True
        ),
    )

    getattr(moveit_executor.MoveItExecutor, callback_name)(
        executor, SimpleNamespace(result=lambda: response)
    )

    assert dispatches == []
    assert len(rejections) == 1
    assert "CARTESIAN_ERROR_CODE_-1" in rejections[0]


@pytest.mark.parametrize(
    ("fraction", "error_code", "expected"),
    (
        (float("nan"), 1, "CARTESIAN_FRACTION_INVALID"),
        (0.999, 1, "CARTESIAN_PATH_INCOMPLETE"),
        (1.0, -1, "CARTESIAN_ERROR_CODE_-1"),
    ),
)
def test_cartesian_response_requires_success_finite_and_full_endpoint(
    fraction,
    error_code,
    expected,
):
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=error_code),
        fraction=fraction,
        solution=_robot_trajectory([[0.0] * 6]),
    )

    assert expected in moveit_executor.MoveItExecutor._cartesian_response_error(
        response
    )


def test_stale_stage1_movegroup_callbacks_cannot_restart_old_run():
    logger = _Logger()
    current_token = object()
    side_effects = []
    executor = SimpleNamespace(
        _stage1_attempt_token=current_token,
        _stage1_orientation_rank_index=0,
        _motion_abort_requested=False,
        get_logger=lambda: logger,
    )
    stale_future = SimpleNamespace(
        result=lambda: side_effects.append("future-read")
    )

    moveit_executor.MoveItExecutor._stage1_goal_response(
        executor, stale_future, object(), 0
    )
    moveit_executor.MoveItExecutor._stage1_result(
        executor, stale_future, object(), 0
    )

    assert side_effects == []
    assert len(logger.warnings) == 2


def test_stale_stage1_scene_timer_cannot_start_or_cancel_new_run():
    old_token = object()
    new_token = object()
    old_timer = _FakeTimer(lambda: None)
    new_timer = _FakeTimer(lambda: None)
    starts = []
    cancels = []
    executor = SimpleNamespace(
        _stage1_attempt_token=new_token,
        _stage1_scene_wait_timer=new_timer,
        executing=True,
        _motion_abort_requested=False,
        scene_confirmed=True,
        _cancel_stage1_scene_wait_timer=lambda: cancels.append(True),
        _start_stage1_after_scene_confirmed=lambda: starts.append(True),
    )

    moveit_executor.MoveItExecutor._stage1_wait_for_scene_confirmed(
        executor, old_token, old_timer
    )

    assert starts == []
    assert cancels == []
    assert executor._stage1_scene_wait_timer is new_timer
    assert new_timer.cancelled is False


def test_stage1_branch_planner_failure_retries_then_switches_orientation():
    token = object()
    calls = []
    executor = SimpleNamespace(
        _stage1_attempt_token=token,
        _stage1_orientation_rank_index=0,
        _stage1_orientation_ranked=[{}],
        _stage1_ik_seed_state=object(),
        _motion_abort_requested=False,
        _stage1_retried=False,
        get_logger=lambda: _Logger(),
        _retry_stage1_with_default_planner=lambda *args: calls.append(
            ("default", args)
        ),
        _try_next_stage1_orientation_branch=lambda reason: calls.append(
            ("alternate", reason)
        ),
    )
    result = SimpleNamespace(error_code=SimpleNamespace(val=-1))
    future = SimpleNamespace(result=lambda: SimpleNamespace(result=result))

    moveit_executor.MoveItExecutor._stage1_result(
        executor, future, token, 0
    )
    assert calls[0][0] == "default"

    # Model the default-planner retry returning the same planning failure.
    moveit_executor.MoveItExecutor._stage1_result(
        executor, future, token, 0
    )
    assert calls[1] == ("alternate", "MOVEIT_ERROR_-1")


def test_stage1_planning_result_rechecks_joint_freshness_before_dispatch():
    token = object()
    dispatches = []
    failures = []
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(
        moveit_executor.READY_POSE_JOINTS.keys()
    )
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    trajectory.joint_trajectory.points = [point]
    result = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        planned_trajectory=trajectory,
    )
    executor = SimpleNamespace(
        _stage1_attempt_token=token,
        _stage1_orientation_rank_index=0,
        _stage1_orientation_ranked=[{}],
        _stage1_ik_seed_state=object(),
        _motion_abort_requested=False,
        _stage1_retried=False,
        _stage1_trajectory_is_safe=lambda *_args: True,
        _rescale_trajectory=lambda traj, scale: traj,
        _stage1_pre_motion_blockers=lambda: ("JOINT_STATE_STALE",),
        _d405_plan_endpoint_matches_ik=lambda *_args: (True, ""),
        _d405_plan_start_matches_measured=lambda *_args, **_kwargs: (
            True,
            "",
        ),
        _fail_stage1_before_motion=lambda reason: failures.append(str(reason)),
        execute_trajectory_direct=lambda *_args, **_kwargs: (
            dispatches.append(True) or True
        ),
        get_logger=lambda: _Logger(),
    )
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(result=result)
    )

    moveit_executor.MoveItExecutor._stage1_result(
        executor, future, token, 0
    )

    assert dispatches == []
    assert failures == ["STAGE1_DISPATCH_READINESS:JOINT_STATE_STALE"]


def test_stage1_planning_failure_cleans_snapshot_before_terminal_status():
    statuses = []
    calls = []
    executor = SimpleNamespace(
        _active_trajectory_goal_token=None,
        _joint_command_timer=None,
        _fjt_motion_state_unknown=False,
        _stage1_retried=True,
        _stage1_goal_constraints=object(),
        _stage1_on_complete=object(),
        _stage1_attempt_token=object(),
        _execution_snapshot={"path_id": "old"},
        _active_segment_path=object(),
        executing=True,
        get_logger=lambda: _Logger(),
        _cancel_stage1_scene_wait_timer=lambda: calls.append("cancel-scene"),
        _request_motion_abort=lambda reason: calls.append(("abort", reason)),
    )

    def reset_process():
        calls.append("reset")
        executor._execution_snapshot = None
        executor._active_segment_path = None
        executor._stage1_attempt_token = None

    def publish_status(state, reason):
        statuses.append(
            (
                state,
                reason,
                executor._execution_snapshot is not None,
                executor.executing,
            )
        )

    executor._reset_painting_process = reset_process
    executor._publish_execution_status = publish_status

    moveit_executor.MoveItExecutor._fail_stage1_before_motion(
        executor, "NO_SAFE_BRANCH"
    )

    assert calls == ["cancel-scene", "reset"]
    assert statuses == [
        ("PLAN_REJECTED", "NO_SAFE_BRANCH", False, False)
    ]
    assert executor._active_segment_path is None
    assert executor._stage1_goal_constraints is None
    assert executor._stage1_on_complete is None


def test_precontact_completion_stops_for_runtime_tare_before_contact_search():
    calls = []
    step = SimpleNamespace(mode="APPROACH_PRECONTACT")
    final_pose = _pose(0.4, 0.0, 0.5)
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _process_steps=[step],
        _process_step_index=0,
        _process_last_tcp_pose=None,
        painting_force_enabled=True,
        _begin_runtime_tare_before_contact=lambda received, planned: calls.append(
            ("tare", received, planned)
        ),
        _complete_process_step=lambda: calls.append(("complete",)),
    )

    moveit_executor.MoveItExecutor._process_motion_done(
        executor, final_pose
    )

    assert calls == [("tare", step, final_pose)]
    assert executor._process_last_tcp_pose.position.x == pytest.approx(0.4)


def test_runtime_tare_residual_requires_finite_low_filtered_and_raw_wrench():
    executor = SimpleNamespace(
        runtime_tare_max_force_norm_n=0.75,
        runtime_tare_max_torque_norm_nm=0.10,
    )
    healthy = {
        "filtered_wrench": [0.10, -0.20, 0.05, 0.01, -0.01, 0.0],
        "raw_wrench": [0.20, -0.10, 0.05, 0.02, 0.0, -0.01],
    }

    assert moveit_executor.MoveItExecutor._runtime_tare_residual_ok(
        executor, healthy
    )
    assert not moveit_executor.MoveItExecutor._runtime_tare_residual_ok(
        executor,
        {**healthy, "raw_wrench": [0.80, 0.0, 0.0, 0.0, 0.0, 0.0]},
    )
    assert not moveit_executor.MoveItExecutor._runtime_tare_residual_ok(
        executor,
        {**healthy, "filtered_wrench": [float("nan")] * 6},
    )


def test_workflow_executing_is_not_reported_as_physical_trajectory_active():
    executor = SimpleNamespace(
        executing=True,
        _active_trajectory_goal_token=None,
        _joint_command_timer=None,
    )
    assert not moveit_executor.MoveItExecutor._trajectory_command_active(executor)

    executor._active_trajectory_goal_token = object()
    assert moveit_executor.MoveItExecutor._trajectory_command_active(executor)


def test_precontact_geometry_gate_defers_only_expected_ft_startup_latch():
    now = time.monotonic()
    executor = SimpleNamespace(
        real_painting_enabled=True,
        active_target_name="wall",
        _acm_baseline_verified=True,
        _acm_baseline_verified_target_name="wall",
        _acm_baseline_verified_time=now,
        dry_run=False,
        painting_force_enabled=True,
        runtime_tare_enabled=True,
        _execution_tare_ready=False,
        _motion_abort_requested=False,
        current_joint_state=object(),
        current_joint_state_time=now,
        _safety_status_time=now,
        _safety_status={
            "ft_valid": False,
            "tf_valid": False,
            "bias_ready": False,
            "abort_latched": True,
            "reason": "FT_NONFINITE",
        },
        _guard_status_time=now,
        _guard_status_source_time=now,
        _guard_status_sequence=1,
        force_guard_status_timeout_s=0.5,
        _free_space_confirmed=True,
        runtime_ft_tare_client=_ReadyEndpoint(),
        force_safety_reset_client=_ReadyEndpoint(),
        wrench_guard_reset_client=_ReadyEndpoint(),
        _current_tcp_pose_np=lambda: object(),
        _controller_states={
            "joint_trajectory_controller": "active",
            "admittance_controller": "active",
        },
        execution_backend="follow_joint_trajectory",
        traj_action_client=_ReadyEndpoint(),
        ft_required_timeout_s=0.20,
        _controller_fault_time=now,
        _controller_fault=False,
    )
    executor._precontact_tare_pending = lambda: (
        moveit_executor.MoveItExecutor._precontact_tare_pending(executor)
    )
    executor._runtime_tare_services_ready = lambda: (
        moveit_executor.MoveItExecutor._runtime_tare_services_ready(executor)
    )
    executor._precontact_safety_fault_is_deferred = lambda: (
        moveit_executor.MoveItExecutor._precontact_safety_fault_is_deferred(
            executor
        )
    )

    geometry = moveit_executor.MoveItExecutor._real_geometry_runtime_blockers(
        executor
    )
    contact = moveit_executor.MoveItExecutor._real_force_runtime_blockers(
        executor
    )

    assert geometry == ()
    assert "EXECUTION_TARE_NOT_READY" in contact
    assert "FT_STALE" in contact
    assert "FT_BIAS_NOT_READY" in contact

    executor._acm_baseline_verified = False
    geometry = moveit_executor.MoveItExecutor._real_geometry_runtime_blockers(
        executor
    )
    assert "ACM_BASELINE_NOT_VERIFIED" in geometry
    executor._acm_baseline_verified = True

    executor._safety_status["reason"] = "NORMAL_OVERFORCE"
    geometry = moveit_executor.MoveItExecutor._real_geometry_runtime_blockers(
        executor
    )
    assert "ABORT_LATCHED:NORMAL_OVERFORCE" in geometry


def test_runtime_tare_waits_for_new_bias_and_continuous_residual_window(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    calls = []
    status = {
        "ft_valid": True,
        "tf_valid": True,
        "abort_latched": False,
        "bias_ready": True,
        "filtered_wrench": [0.1, 0.1, 0.1, 0.01, 0.01, 0.01],
        "raw_wrench": [0.1, 0.1, 0.1, 0.01, 0.01, 0.01],
    }
    executor = SimpleNamespace(
        _runtime_tare_token=object(),
        _motion_abort_requested=False,
        _runtime_tare_started_at=5.0,
        runtime_tare_timeout_s=20.0,
        _runtime_tare_phase="WAIT_MONITOR_BIAS",
        _runtime_tare_phase_started_at=9.0,
        _runtime_tare_verify_started_at=0.0,
        _runtime_tare_actual_metrics={},
        runtime_tare_verify_duration_s=0.40,
        runtime_tare_max_force_norm_n=0.75,
        runtime_tare_max_torque_norm_nm=0.10,
        _safety_status_time=10.0,
        _safety_status=status,
        ft_required_timeout_s=0.20,
        _guard_status_time=10.0,
        _guard_status={
            "compliance_active": False,
            "compliance_enabled": False,
            "forwarding": False,
        },
        wrench_guard_reset_client=object(),
        _cancel_runtime_tare_timer=lambda: calls.append("cancel"),
        _runtime_tare_fail=lambda reason: calls.append(("fail", reason)),
        _publish_painting_command=lambda *_args, **_kwargs: None,
        _trajectory_command_active=lambda: False,
        _robot_stationary_for_bias=lambda: True,
        _publish_free_space_confirmation=lambda value: calls.append(
            ("free", value)
        ),
        _runtime_tare_call=lambda client, phase, callback: calls.append(
            ("service", client, phase, callback)
        ),
        _runtime_guard_reset_done=lambda *_args: None,
        _runtime_tare_actual_pose_guard=lambda: (
            True,
            "",
            {"actual_clearance_m": 0.010},
        ),
    )
    executor._runtime_tare_residual_ok = lambda current: (
        moveit_executor.MoveItExecutor._runtime_tare_residual_ok(
            executor, current
        )
    )

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)
    assert executor._runtime_tare_phase == "VERIFY_RESIDUAL"
    assert executor._runtime_tare_verify_started_at == 0.0

    clock["now"] = 10.1
    executor._safety_status_time = 10.1
    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)
    assert executor._runtime_tare_verify_started_at == pytest.approx(10.1)
    assert not [call for call in calls if call[0] == "service"]

    clock["now"] = 10.51
    executor._safety_status_time = 10.51
    executor._guard_status_time = 10.51
    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)
    service_calls = [call for call in calls if call[0] == "service"]
    assert len(service_calls) == 1
    assert service_calls[0][2] == "GUARD_RESET_PENDING"


def test_runtime_tare_guard_ready_completes_and_starts_contact_search(
    monkeypatch,
):
    """The final tare metrics must not crash the transition to contact."""

    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    statuses = []
    scheduled = []
    contact_search_steps = []
    failures = []
    timer_cancellations = []
    metrics = {
        "actual_clearance_m": 0.010,
        "tcp_position_error_m": 0.0004,
        "tcp_orientation_error_deg": 0.2,
        "tool_axis_normal_error_deg": 0.3,
        "tcp_tf_age_s": 0.01,
    }
    precontact = SimpleNamespace(
        mode="APPROACH_PRECONTACT",
        rows=[SimpleNamespace(row_number=1)],
        force_n=0.0,
        speed_mps=0.005,
    )
    contact_search = SimpleNamespace(
        mode="CONTACT_SEARCH",
        rows=[SimpleNamespace(row_number=1)],
        force_n=0.0,
        speed_mps=0.002,
    )
    executor = SimpleNamespace(
        _runtime_tare_token=object(),
        _motion_abort_requested=False,
        _runtime_tare_started_at=5.0,
        runtime_tare_timeout_s=20.0,
        _runtime_tare_phase="WAIT_GUARD_STATUS",
        _runtime_tare_phase_started_at=9.5,
        _runtime_tare_actual_metrics=dict(metrics),
        _guard_status_time=10.0,
        _guard_status={
            "controller_fault": False,
            "compliance_active": False,
            "compliance_enabled": False,
            "forwarding": False,
        },
        _execution_tare_ready=False,
        _execution_state="PRECONTACT_TARE",
        _contact_search_distance_m=0.0,
        _process_step_index=0,
        _process_steps=[precontact, contact_search],
        real_painting_enabled=False,
        _active_segment_path=SimpleNamespace(version=3),
        _runtime_tare_actual_pose_guard=lambda: (True, "", dict(metrics)),
        _runtime_tare_fail=lambda reason: failures.append(str(reason)),
        _publish_painting_command=lambda *_args, **_kwargs: None,
        _trajectory_command_active=lambda: False,
        _robot_stationary_for_bias=lambda: True,
        _publish_free_space_confirmation=lambda _value: None,
        _real_force_runtime_blockers=lambda: (),
        _cancel_runtime_tare_timer=lambda: timer_cancellations.append(True),
        _publish_execution_status=lambda state, reason="", **fields: (
            statuses.append((state, reason, fields))
        ),
        _schedule_process_once=lambda delay, callback: scheduled.append(
            (delay, callback)
        ),
        _complete_process_step=lambda: None,
        _start_contact_search=lambda step: contact_search_steps.append(step),
        _request_motion_abort=lambda reason: failures.append(str(reason)),
        get_logger=lambda: _Logger(),
    )
    executor._transition_execution_state = MethodType(
        moveit_executor.MoveItExecutor._transition_execution_state, executor
    )
    executor._complete_process_step = MethodType(
        moveit_executor.MoveItExecutor._complete_process_step, executor
    )
    executor._execute_next_process_step = MethodType(
        moveit_executor.MoveItExecutor._execute_next_process_step, executor
    )

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert failures == []
    assert executor._runtime_tare_phase == "COMPLETE"
    assert executor._runtime_tare_token is None
    assert executor._execution_tare_ready is True
    assert timer_cancellations == [True]
    assert executor._process_step_index == 1
    assert len(scheduled) == 1
    assert scheduled[0][0] == pytest.approx(0.001)
    assert len(statuses) == 1
    state, reason, fields = statuses[0]
    assert state == "PRECONTACT_TARE_COMPLETE"
    assert reason == "fresh hardware/monitor bias and guard verified"
    assert fields["previous_state"] == "PRECONTACT_TARE"
    assert fields["next_state"] == "PRECONTACT_TARE_COMPLETE"
    assert fields["contact_search_distance_m"] == pytest.approx(0.0)
    for key, value in metrics.items():
        assert fields[key] == pytest.approx(value)

    scheduled[0][1]()

    assert contact_search_steps == [contact_search]
    assert failures == []


def test_execution_transition_preserves_canonical_status_fields():
    statuses = []
    executor = SimpleNamespace(
        _execution_state="PRECONTACT_TARE",
        _contact_search_distance_m=0.004,
        _publish_execution_status=lambda state, reason="", **fields: (
            statuses.append((state, reason, fields))
        ),
    )

    moveit_executor.MoveItExecutor._transition_execution_state(
        executor,
        "PRECONTACT_TARE_COMPLETE",
        "verified",
        actual_clearance_m=0.010,
        previous_state="caller-value",
        contact_search_distance_m=99.0,
        state="caller-state",
        timestamp_ns=1,
        plan_hash="caller-plan",
    )

    assert len(statuses) == 1
    state, reason, fields = statuses[0]
    assert state == "PRECONTACT_TARE_COMPLETE"
    assert reason == "verified"
    assert fields == {
        "actual_clearance_m": pytest.approx(0.010),
        "previous_state": "PRECONTACT_TARE",
        "next_state": "PRECONTACT_TARE_COMPLETE",
        "contact_search_distance_m": pytest.approx(0.004),
    }


def test_runtime_tare_pending_service_keeps_false_heartbeats_and_free_space(
    monkeypatch,
):
    """An async service wait must not starve the hardware tare interlocks."""

    clock = {"now": 20.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    commands = []
    free_space = []
    executor = SimpleNamespace(
        _runtime_tare_token=object(),
        _motion_abort_requested=False,
        _runtime_tare_started_at=19.0,
        runtime_tare_timeout_s=20.0,
        _runtime_tare_phase="HARDWARE_TARE_PENDING",
        _runtime_tare_quiet_started_at=18.0,
        _runtime_tare_actual_metrics={},
        _guard_status_time=20.0,
        _guard_status={
            "compliance_active": False,
            "compliance_enabled": False,
            "forwarding": False,
        },
        _cancel_runtime_tare_timer=lambda: pytest.fail(
            "pending async service must keep the tare timer alive"
        ),
        _runtime_tare_fail=lambda reason: pytest.fail(reason),
        _publish_painting_command=lambda mode, force, enable=False: commands.append(
            (mode, force, enable)
        ),
        _trajectory_command_active=lambda: False,
        _robot_stationary_for_bias=lambda: True,
        _publish_free_space_confirmation=lambda value: free_space.append(value),
        _runtime_tare_actual_pose_guard=lambda: (
            True,
            "",
            {"actual_clearance_m": 0.010},
        ),
    )

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)
    clock["now"] = 20.05
    executor._guard_status_time = 20.05
    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert commands == [("IDLE", 0.0, False), ("IDLE", 0.0, False)]
    assert free_space == [True, True]
    assert executor._runtime_tare_phase == "HARDWARE_TARE_PENDING"


def _runtime_tare_geometry_executor(
    *,
    actual_gap_m=0.010,
    tangent_position_error_m=0.0,
    orientation_error_deg=0.0,
    current_generation="generation-1",
):
    plane_normal = [1.0, 0.0, 0.0]
    contact_geometry_m = 0.026
    planned_gap_m = 0.010
    reach_m = moveit_executor.EOAT_TIP_OFFSET
    half_sqrt = 2.0 ** -0.5
    planned_q = [0.0, 0.0, -half_sqrt, half_sqrt]
    if orientation_error_deg:
        half_angle = moveit_executor.math.radians(orientation_error_deg) * 0.5
        delta_q = [moveit_executor.math.sin(half_angle), 0.0, 0.0,
                   moveit_executor.math.cos(half_angle)]
        actual_q = moveit_executor.quat_multiply(delta_q, planned_q)
    else:
        actual_q = planned_q
    planned_pose = _pose(
        contact_geometry_m + planned_gap_m + reach_m,
        0.0,
        0.0,
    )
    planned_pose.orientation.x = planned_q[0]
    planned_pose.orientation.y = planned_q[1]
    planned_pose.orientation.z = planned_q[2]
    planned_pose.orientation.w = planned_q[3]
    actual_position = [
        contact_geometry_m + actual_gap_m + reach_m,
        tangent_position_error_m,
        0.0,
    ]
    path = SimpleNamespace(
        version=3,
        path_id="path-1",
        plan_hash="a" * 64,
        work_area_id="area-1",
        plane_generation_id="generation-1",
    )
    service_calls = []
    failures = []
    executor = SimpleNamespace(
        _runtime_tare_token=object(),
        _motion_abort_requested=False,
        _runtime_tare_started_at=5.0,
        runtime_tare_timeout_s=20.0,
        _runtime_tare_phase="SETTLING",
        _runtime_tare_quiet_started_at=8.0,
        runtime_tare_quiet_s=0.75,
        _runtime_tare_actual_metrics={},
        _runtime_tare_context={
            "planned_tcp_pose": planned_pose,
            "plane_point": [0.0, 0.0, 0.0],
            "plane_normal": plane_normal,
            "contact_geometry_offset_m": contact_geometry_m,
            "path_id": "path-1",
            "plan_hash": "a" * 64,
            "work_area_id": "area-1",
            "plane_generation_id": "generation-1",
        },
        runtime_tare_min_actual_clearance_m=0.007,
        runtime_tare_max_tcp_position_error_m=0.003,
        runtime_tare_max_tcp_orientation_error_deg=3.0,
        runtime_tare_max_tcp_tf_age_s=0.20,
        _active_segment_path=path,
        _segment_path=path,
        _waypoints_path_id="path-1",
        _accepted_plan_path_id="path-1",
        _accepted_plan_hash="a" * 64,
        _current_work_area_id="area-1",
        _current_plane_generation_id=current_generation,
        _d405_plane_accepted=True,
        _runtime_tare_tcp_pose_sample=lambda: (
            (actual_position, actual_q, 0.01),
            "",
        ),
        _guard_status_time=10.0,
        _guard_status={
            "compliance_active": False,
            "compliance_enabled": False,
            "forwarding": False,
        },
        _cancel_runtime_tare_timer=lambda: pytest.fail(
            "geometry check must finish before timer cancellation"
        ),
        _runtime_tare_fail=lambda reason: failures.append(str(reason)),
        _publish_painting_command=lambda *_args, **_kwargs: None,
        _trajectory_command_active=lambda: False,
        _robot_stationary_for_bias=lambda: True,
        _publish_free_space_confirmation=lambda _value: None,
        runtime_ft_tare_client=object(),
        _runtime_hardware_tare_done=lambda *_args: None,
        _runtime_tare_call=lambda client, phase, callback: service_calls.append(
            (client, phase, callback)
        ),
    )
    executor._runtime_tare_identity_error = lambda: (
        moveit_executor.MoveItExecutor._runtime_tare_identity_error(executor)
    )
    executor._runtime_tare_actual_pose_guard = lambda: (
        moveit_executor.MoveItExecutor._runtime_tare_actual_pose_guard(executor)
    )
    return executor, service_calls, failures


@pytest.mark.parametrize(
    ("geometry", "reason_fragment"),
    (
        ({"actual_gap_m": 0.0}, "clearance"),
        ({"actual_gap_m": 0.006}, "clearance"),
        ({"tangent_position_error_m": 0.004}, "position error"),
        ({"orientation_error_deg": 5.0}, "orientation error"),
        ({"current_generation": "generation-2"}, "identity"),
    ),
)
def test_runtime_tare_actual_geometry_violation_sends_no_service(
    monkeypatch, geometry, reason_fragment
):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    executor, service_calls, failures = _runtime_tare_geometry_executor(
        **geometry
    )

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert service_calls == []
    assert len(failures) == 1
    assert reason_fragment in failures[0]


def test_runtime_tare_actual_ten_mm_pose_allows_hardware_service(monkeypatch):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    executor, service_calls, failures = _runtime_tare_geometry_executor()

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert failures == []
    assert len(service_calls) == 1
    assert service_calls[0][1] == "HARDWARE_TARE_PENDING"
    assert executor._runtime_tare_actual_metrics == {
        "actual_clearance_m": pytest.approx(0.010),
        "tcp_position_error_m": pytest.approx(0.0),
        "tcp_orientation_error_deg": pytest.approx(0.0),
        "tool_axis_normal_error_deg": pytest.approx(0.0),
        "tcp_tf_age_s": pytest.approx(0.01),
    }


def test_runtime_tare_post_service_drift_aborts_before_completion(monkeypatch):
    """The pose guard remains active after hardware tare has returned."""

    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    executor, service_calls, failures = _runtime_tare_geometry_executor(
        actual_gap_m=0.006
    )
    executor._runtime_tare_phase = "WAIT_MONITOR_BIAS"
    completed = []
    executor._complete_process_step = lambda: completed.append(True)

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert service_calls == []
    assert len(failures) == 1
    assert "clearance" in failures[0]
    assert completed == []


def test_runtime_tare_correct_pose_with_stale_tf_sends_no_service(monkeypatch):
    executor, service_calls, failures = _runtime_tare_geometry_executor()
    sample, _error = executor._runtime_tare_tcp_pose_sample()
    position, quaternion, _age = sample
    transform = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=9, nanosec=0),
        ),
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=position[0], y=position[1], z=position[2]
            ),
            rotation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
    )
    executor.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args, **_kwargs: transform
    )
    executor.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)
    )
    executor._runtime_tare_tcp_pose_sample = lambda: (
        moveit_executor.MoveItExecutor._runtime_tare_tcp_pose_sample(executor)
    )
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)

    moveit_executor.MoveItExecutor._runtime_tare_tick(executor)

    assert service_calls == []
    assert len(failures) == 1
    assert "TF stale" in failures[0]


def _contact_search_ack_executor(
    *,
    safety_mode="CONTACT_SEARCH",
    guard_mode="CONTACT_SEARCH",
    guard_forwarding=False,
):
    now = time.monotonic()
    command_context = {
        "generation": 1,
        "mode": "CONTACT_SEARCH",
        "force_n": 0.0,
        "enable": False,
        "published_at_s": now - 0.10,
        "guard_status_sequence": 40,
        "guard_source_timestamp_s": now - 0.20,
    }
    token = object()
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(
        moveit_executor.READY_POSE_JOINTS
    )
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points = [point]
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        fraction=1.0,
        solution=trajectory,
    )
    future = SimpleNamespace(result=lambda: response)
    dispatches = []
    hard_aborts = []
    rejections = []
    restores = []
    target = _pose(0.3, 0.0, 0.4)
    snapshot = object()
    executor = SimpleNamespace(
        real_painting_enabled=True,
        painting_force_enabled=True,
        _contact_search_mode_published_at=command_context["published_at_s"],
        _contact_search_command_context=command_context,
        _painting_command_context=command_context,
        _painting_command_generation=1,
        _safety_status_time=now,
        _safety_status={
            "mode": safety_mode,
            "ft_valid": True,
            "tf_valid": True,
            "bias_ready": True,
            "abort_latched": False,
            "force_enabled": False,
        },
        _guard_status_time=now,
        _guard_status_source_time=now,
        _guard_status_sequence=41,
        force_guard_status_timeout_s=0.5,
        _guard_status={
            "mode": guard_mode,
            "force_enable": False,
            "ft_valid": True,
            "tf_valid": True,
            "abort_latched": False,
            "controller_fault": False,
            "forwarding": guard_forwarding,
            "compliance_enabled": False,
            "compliance_active": False,
        },
        _current_real_plan_blockers=lambda: (),
        _real_force_runtime_blockers=lambda: (),
        _contact_search_context={
            "token": token,
            "target": target,
            "distance_m": 0.0005,
            "seed_state": object(),
            "scene_revision": 7,
            "execution_snapshot": snapshot,
        },
        _motion_abort_requested=False,
        _contact_search_step_active=True,
        _scene_revision=7,
        scene_confirmed=True,
        _scene_confirmed_revision=7,
        _execution_snapshot=snapshot,
        _process_last_tcp_pose=_pose(0.3005, 0.0, 0.4),
        contact_search_speed_mps=0.002,
        contact_search_mode_ack_timeout_s=1.0,
        _limit_trajectory_to_cartesian_speed=lambda solution, *_args: solution,
        _d405_plan_start_matches_measured=lambda *_args, **_kwargs: (
            True,
            "",
        ),
        execute_trajectory_direct=lambda *_args, **_kwargs: (
            dispatches.append(True) or True
        ),
        _request_motion_abort=lambda reason: hard_aborts.append(str(reason)),
        _fail_workflow_known_safe=lambda reason, **_kwargs: (
            rejections.append(str(reason)) or True
        ),
        _handle_known_dispatch_rejection=lambda reason, **_kwargs: (
            rejections.append(str(reason)) or True
        ),
        _handle_process_trajectory_failure=lambda reason: (
            hard_aborts.append(str(reason)) or True
        ),
        _set_contact_collision_allowed=lambda allowed: restores.append(allowed),
        _publish_execution_status=lambda *_args, **_kwargs: None,
        _contact_search_iteration=lambda _step: dispatches.append("iteration"),
        _schedule_process_once=lambda *_args, **_kwargs: pytest.fail(
            "test expected immediate ACK decision"
        ),
    )
    executor._contact_search_mode_ack_blockers = lambda: (
        moveit_executor.MoveItExecutor._contact_search_mode_ack_blockers(
            executor
        )
    )
    return (
        executor,
        future,
        token,
        dispatches,
        hard_aborts,
        rejections,
        restores,
    )


def test_contact_search_post_edge_old_idle_status_sends_no_fjt_goal():
    executor, future, token, dispatches, hard_aborts, rejections, _restores = (
        _contact_search_ack_executor(safety_mode="IDLE")
    )

    moveit_executor.MoveItExecutor._contact_search_plan_done(
        executor, future, token
    )

    assert dispatches == []
    assert hard_aborts == []
    assert len(rejections) == 1
    assert "SAFETY_MODE_ACK:IDLE" in rejections[0]


def test_contact_search_guard_forwarding_residue_sends_no_fjt_goal():
    executor, future, token, dispatches, hard_aborts, rejections, _restores = (
        _contact_search_ack_executor(guard_forwarding=True)
    )

    moveit_executor.MoveItExecutor._contact_search_plan_done(
        executor, future, token
    )

    assert dispatches == []
    assert hard_aborts == []
    assert len(rejections) == 1
    assert "GUARD_FORWARDING_NOT_OFF" in rejections[0]


def test_contact_search_fresh_healthy_mode_ack_dispatches_once():
    executor, future, token, dispatches, hard_aborts, rejections, _restores = (
        _contact_search_ack_executor()
    )

    moveit_executor.MoveItExecutor._contact_search_plan_done(
        executor, future, token
    )

    assert dispatches == [True]
    assert hard_aborts == []
    assert rejections == []


def test_early_contact_restores_acm_then_advances_to_ramp_up():
    scheduled = []
    transitions = []
    restores = []
    restore_callbacks = []
    ramps = []
    contact_step = SimpleNamespace(
        mode="CONTACT_SEARCH",
        force_n=0.0,
        speed_mps=0.002,
        rows=[SimpleNamespace(row_number=1)],
    )
    ramp_step = SimpleNamespace(
        mode="RAMP_UP",
        force_n=1.6,
        speed_mps=0.0,
        rows=[SimpleNamespace(row_number=2)],
    )
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _contact_search_cancel_on_contact=True,
        _contact_search_step_active=True,
        _contact_search_context=object(),
        _contact_search_confirmed=False,
        _process_step_index=0,
        _process_steps=[contact_step, ramp_step],
        real_painting_enabled=True,
        _current_real_plan_blockers=lambda: (),
        get_logger=lambda: _Logger(),
        _transition_execution_state=lambda state, reason: transitions.append(
            (state, reason)
        ),
        _set_contact_collision_allowed=lambda allowed, callback=None: (
            restores.append(bool(allowed))
            or (
                restore_callbacks.append(callback)
                if callback is not None
                else None
            )
        ),
        _schedule_process_once=lambda delay, callback: scheduled.append(
            (delay, callback)
        ),
        _start_process_ramp=lambda step: ramps.append(step),
        _request_motion_abort=lambda reason: pytest.fail(
            f"contact confirmation must not hard-abort: {reason}"
        ),
        _fail_workflow_known_safe=lambda reason: pytest.fail(
            f"contact confirmation must not reject the run: {reason}"
        ),
    )
    executor._complete_process_step = MethodType(
        moveit_executor.MoveItExecutor._complete_process_step,
        executor,
    )
    executor._execute_next_process_step = MethodType(
        moveit_executor.MoveItExecutor._execute_next_process_step,
        executor,
    )

    moveit_executor.MoveItExecutor._complete_contact_search_after_early_contact(
        executor
    )

    assert executor._contact_search_confirmed is True
    assert executor._contact_search_step_active is False
    assert restores == [False]
    assert transitions == [
        ("CONTACT_SEARCH_COMPLETE", "early contact canceled active trajectory")
    ]
    # CONTACT_SEARCH completion is not permission to enable force.  The step
    # remains frozen until the asynchronous full-ACM restore/readback callback
    # confirms the exact pre-contact baseline.
    assert executor._process_step_index == 0
    assert scheduled == []
    assert ramps == []
    assert len(restore_callbacks) == 1

    restore_callbacks[0](True)
    assert executor._process_step_index == 1
    assert len(scheduled) == 1

    delay, callback = scheduled[0]
    assert delay == pytest.approx(0.001)
    callback()
    assert ramps == [ramp_step]


def test_early_contact_restore_failure_blocks_ramp_up_and_aborts():
    restore_callbacks = []
    aborts = []
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _contact_search_cancel_on_contact=True,
        _contact_search_step_active=True,
        _contact_search_context=object(),
        _contact_search_confirmed=False,
        _transition_execution_state=lambda *_args: None,
        _set_contact_collision_allowed=lambda _allowed, callback=None: (
            restore_callbacks.append(callback)
            if callback is not None
            else None
        ),
        _complete_process_step=lambda: pytest.fail(
            "failed ACM restore must not advance to RAMP_UP"
        ),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
    )

    moveit_executor.MoveItExecutor._complete_contact_search_after_early_contact(
        executor
    )
    assert len(restore_callbacks) == 1
    restore_callbacks[0](False)

    assert aborts == ["CONTACT_SEARCH ACM restore failed"]


def test_contact_search_guard_requests_one_cancel_after_contact():
    token = object()
    cancel_future = _ControlledFuture()
    cancel_requests = []
    handle = SimpleNamespace(
        cancel_goal_async=lambda: (
            cancel_requests.append(True) or cancel_future
        )
    )
    executor = SimpleNamespace(
        _active_trajectory_goal_token=token,
        _active_trajectory_goal_handle=handle,
        _active_trajectory_label="CONTACT_SEARCH step",
        _active_trajectory_cancel_requested=False,
        _active_trajectory_result_deadline=time.monotonic() + 100.0,
        _active_trajectory_cancel_deadline=0.0,
        _motion_abort_requested=False,
        _contact_search_cancel_on_contact=False,
        _fjt_guard_timer=None,
        fjt_cancel_timeout_s=2.0,
        create_timer=lambda _period, callback: _FakeTimer(callback),
        destroy_timer=lambda _timer: None,
        get_logger=lambda: _Logger(),
        _post_tare_fjt_safety_blockers=lambda _now: (),
        _contact_search_contact_sensed=lambda: True,
        _ft_guard_triggered=lambda _label: False,
        _request_motion_abort=lambda reason: pytest.fail(
            f"confirmed contact must cancel, not abort: {reason}"
        ),
        _latch_fjt_motion_unknown=lambda *_args: pytest.fail(
            "confirmed contact cancel must remain a known trajectory state"
        ),
    )
    executor._cancel_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._cancel_active_follow_joint_goal,
        executor,
    )
    executor._cancel_fjt_guard_timer = MethodType(
        moveit_executor.MoveItExecutor._cancel_fjt_guard_timer,
        executor,
    )

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor,
        token,
        "CONTACT_SEARCH step",
        True,
    )
    timer = executor._fjt_guard_timer
    timer.callback()
    timer.callback()

    assert executor._contact_search_cancel_on_contact is True
    assert executor._active_trajectory_cancel_requested is True
    assert cancel_requests == [True]


def test_contact_search_canceled_result_completes_early_contact_without_abort():
    client = _QueuedActionClient()
    result_future = _ControlledFuture()
    completions = []
    failures = []
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: result_future,
    )
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _active_trajectory_goal_token=None,
        _active_trajectory_goal_handle=None,
        _active_trajectory_label="",
        _active_trajectory_cancel_requested=False,
        _active_trajectory_result_deadline=0.0,
        _active_trajectory_cancel_deadline=0.0,
        _fjt_motion_state_unknown=False,
        _last_trajectory_failure_phase="",
        _contact_search_cancel_on_contact=False,
        fjt_result_timeout_margin_s=2.0,
        traj_action_client=client,
        executing=True,
        get_logger=lambda: _Logger(),
        _trajectory_within_joint_limits=lambda *_args: True,
        _point_time_sec=lambda point: (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        ),
        _start_fjt_guard_timer=lambda *_args: None,
        _cancel_fjt_guard_timer=lambda: None,
        _latch_fjt_motion_unknown=lambda *_args: pytest.fail(
            "known canceled result must not become motion-unknown"
        ),
        _complete_contact_search_after_early_contact=lambda: completions.append(
            True
        ),
    )
    executor._clear_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._clear_active_follow_joint_goal,
        executor,
    )
    trajectory = _robot_trajectory([[0.0] * 6])

    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        executor,
        trajectory,
        on_failure=lambda: failures.append(True),
        force_guard=True,
        label="CONTACT_SEARCH step",
    )
    client.futures[0].resolve(handle)
    executor._contact_search_cancel_on_contact = True
    executor._contact_search_cancel_token = (
        executor._active_trajectory_goal_token
    )
    result_future.resolve(
        SimpleNamespace(
            status=moveit_executor.GoalStatus.STATUS_CANCELED,
            result=SimpleNamespace(
                error_code=0,
                error_string="contact cancellation",
            ),
        )
    )

    assert completions == [True]
    assert failures == []


def _contact_search_fjt_race_executor(
    result_future, *, handle_status=None, resolve_goal=True
):
    client = _QueuedActionClient()
    cancel_future = _ControlledFuture()
    cancel_requests = []
    completions = []
    normal_completions = []
    failures = []
    aborts = []
    timers = []
    contact = {"value": True}
    if handle_status is None:
        handle_status = moveit_executor.GoalStatus.STATUS_EXECUTING
    handle = SimpleNamespace(
        accepted=True,
        status=handle_status,
        get_result_async=lambda: result_future,
        cancel_goal_async=lambda: (
            cancel_requests.append(True) or cancel_future
        ),
    )
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _active_trajectory_goal_token=None,
        _active_trajectory_goal_handle=None,
        _active_trajectory_label="",
        _active_trajectory_result_future=None,
        _active_trajectory_result_consumer=None,
        _active_trajectory_terminal_committed=False,
        _active_trajectory_cancel_requested=False,
        _active_trajectory_cancel_reason="",
        _active_trajectory_result_deadline=0.0,
        _active_trajectory_cancel_deadline=0.0,
        _fjt_motion_state_unknown=False,
        _last_trajectory_failure_phase="",
        _contact_search_cancel_on_contact=False,
        _contact_search_cancel_token=None,
        _fjt_guard_timer=None,
        fjt_result_timeout_margin_s=2.0,
        fjt_cancel_timeout_s=2.0,
        traj_action_client=client,
        executing=True,
        get_logger=lambda: _Logger(),
        create_timer=lambda _period, callback: (
            timers.append(_FakeTimer(callback)) or timers[-1]
        ),
        destroy_timer=lambda _timer: None,
        _trajectory_within_joint_limits=lambda *_args: True,
        _point_time_sec=lambda point: (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        ),
        _post_tare_fjt_safety_blockers=lambda _now: (),
        _contact_search_contact_sensed=lambda: contact["value"],
        _ft_guard_triggered=lambda _label: False,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _latch_fjt_motion_unknown=lambda *_args: pytest.fail(
            "known terminal contact result must not become motion-unknown"
        ),
        _complete_contact_search_after_early_contact=lambda: completions.append(
            True
        ),
    )
    executor._cancel_fjt_guard_timer = MethodType(
        moveit_executor.MoveItExecutor._cancel_fjt_guard_timer,
        executor,
    )
    executor._start_fjt_guard_timer = MethodType(
        moveit_executor.MoveItExecutor._start_fjt_guard_timer,
        executor,
    )
    executor._cancel_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._cancel_active_follow_joint_goal,
        executor,
    )
    executor._clear_active_follow_joint_goal = MethodType(
        moveit_executor.MoveItExecutor._clear_active_follow_joint_goal,
        executor,
    )
    trajectory = _robot_trajectory([[0.0] * 6])
    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        executor,
        trajectory,
        on_complete=lambda: normal_completions.append(True),
        on_failure=lambda: failures.append(True),
        force_guard=True,
        label="CONTACT_SEARCH step",
    )
    if resolve_goal:
        client.futures[0].resolve(handle)
    return SimpleNamespace(
        executor=executor,
        client=client,
        handle=handle,
        result_future=result_future,
        cancel_future=cancel_future,
        cancel_requests=cancel_requests,
        completions=completions,
        normal_completions=normal_completions,
        failures=failures,
        aborts=aborts,
        timers=timers,
    )


def _successful_fjt_result():
    return SimpleNamespace(
        status=moveit_executor.GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(error_code=0, error_string=""),
    )


def test_contact_does_not_cancel_terminal_success_with_queued_result_callback():
    result_future = _DeferredCallbackFuture()
    race = _contact_search_fjt_race_executor(result_future)
    result_future.set_result_without_callbacks(_successful_fjt_result())

    race.timers[0].callback()

    assert race.cancel_requests == []
    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.failures == []
    assert race.aborts == []
    assert race.executor._active_trajectory_goal_token is None

    # The originally queued result callback is stale and cannot complete the
    # workflow a second time.
    result_future.fire_callbacks()
    assert race.completions == [True]


def test_contact_then_success_result_beats_late_cancel_response_exactly_once():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(result_future)

    race.timers[0].callback()
    assert race.cancel_requests == [True]

    result_future.resolve(_successful_fjt_result())
    race.cancel_future.resolve(SimpleNamespace(goals_canceling=[object()]))

    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.failures == []
    assert race.aborts == []
    assert race.executor._active_trajectory_goal_token is None


@pytest.mark.parametrize("cancel_response_first", (False, True))
def test_contact_terminal_cancel_rejection_and_success_converge_exactly_once(
    cancel_response_first,
):
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(result_future)
    race.timers[0].callback()
    assert race.cancel_requests == [True]

    if cancel_response_first:
        race.cancel_future.resolve(SimpleNamespace(goals_canceling=[]))
        result_future.resolve(_successful_fjt_result())
    else:
        result_future.resolve(_successful_fjt_result())
        race.cancel_future.resolve(SimpleNamespace(goals_canceling=[]))

    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.failures == []
    assert race.aborts == []


def test_contact_cancel_accepted_then_canceled_result_completes_once():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(result_future)
    race.timers[0].callback()
    race.cancel_future.resolve(SimpleNamespace(goals_canceling=[object()]))
    result_future.resolve(
        SimpleNamespace(
            status=moveit_executor.GoalStatus.STATUS_CANCELED,
            result=SimpleNamespace(error_code=0, error_string="contact"),
        )
    )

    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.failures == []
    assert race.aborts == []


def test_success_callback_before_contact_uses_normal_completion_without_cancel():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(result_future)
    old_timer = race.timers[0]

    result_future.resolve(_successful_fjt_result())
    old_timer.callback()

    assert race.normal_completions == [True]
    assert race.completions == []
    assert race.cancel_requests == []
    assert race.aborts == []


def test_stale_first_goal_callbacks_cannot_mutate_second_goal_state():
    first_result = _DeferredCallbackFuture()
    race = _contact_search_fjt_race_executor(first_result)
    first_timer = race.timers[0]
    first_result.set_result_without_callbacks(_successful_fjt_result())
    first_timer.callback()
    assert race.executor._active_trajectory_goal_token is None

    second_result = _ControlledFuture()
    second_cancel = _ControlledFuture()
    second_handle = SimpleNamespace(
        accepted=True,
        status=moveit_executor.GoalStatus.STATUS_EXECUTING,
        get_result_async=lambda: second_result,
        cancel_goal_async=lambda: second_cancel,
    )
    trajectory = _robot_trajectory([[0.0] * 6])
    assert moveit_executor.MoveItExecutor._execute_trajectory_follow_joint(
        race.executor,
        trajectory,
        label="CONTACT_SEARCH step 2",
        force_guard=True,
    )
    race.client.futures[1].resolve(second_handle)
    second_token = race.executor._active_trajectory_goal_token
    second_deadline = race.executor._active_trajectory_result_deadline
    second_timer = race.executor._fjt_guard_timer

    first_result.fire_callbacks()
    race.cancel_future.resolve(SimpleNamespace(goals_canceling=[]))
    first_timer.callback()

    assert race.executor._active_trajectory_goal_token is second_token
    assert race.executor._active_trajectory_goal_handle is second_handle
    assert race.executor._active_trajectory_result_deadline == second_deadline
    assert race.executor._fjt_guard_timer is second_timer


def test_contact_uses_terminal_success_status_without_sending_late_cancel():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(
        result_future,
        handle_status=moveit_executor.GoalStatus.STATUS_SUCCEEDED,
    )

    race.timers[0].callback()

    assert race.cancel_requests == []
    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.aborts == []


def test_contact_before_goal_response_sends_one_cancel_and_preserves_deadline():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(
        result_future,
        resolve_goal=False,
    )

    race.timers[0].callback()
    first_deadline = race.executor._active_trajectory_cancel_deadline
    assert race.executor._active_trajectory_cancel_requested is True
    assert first_deadline > 0.0
    assert race.cancel_requests == []

    race.client.futures[0].resolve(race.handle)
    assert race.cancel_requests == [True]
    assert race.executor._active_trajectory_cancel_deadline == first_deadline
    race.cancel_future.resolve(SimpleNamespace(goals_canceling=[]))
    result_future.resolve(_successful_fjt_result())

    assert race.completions == [True]
    assert race.normal_completions == []
    assert race.executor._active_trajectory_goal_token is None


def test_late_result_cannot_clear_unknown_motion_or_resume_workflow():
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(result_future)
    race.executor._fjt_motion_state_unknown = True
    race.executor._motion_abort_requested = True
    race.executor.executing = True

    result_future.resolve(_successful_fjt_result())

    assert race.executor._fjt_motion_state_unknown is True
    assert race.executor.executing is True
    assert race.completions == []
    assert race.normal_completions == []
    assert race.failures == []


def test_ready_result_exception_still_attempts_cancel_of_active_goal():
    class _FailingReadyFuture(_ControlledFuture):
        def result(self):
            raise RuntimeError("result transport failed")

    result_future = _FailingReadyFuture()
    race = _contact_search_fjt_race_executor(result_future)
    result_future._done = True
    race.executor._latch_fjt_motion_unknown = MethodType(
        moveit_executor.MoveItExecutor._latch_fjt_motion_unknown,
        race.executor,
    )
    race.executor._publish_execution_status = lambda *_args, **_kwargs: None
    race.executor._publish_motion_abort_latch = lambda: None
    race.executor._request_motion_abort = lambda _reason: setattr(
        race.executor, "_motion_abort_requested", True
    )

    race.executor._cancel_active_follow_joint_goal("contact result failed")

    assert race.executor._fjt_motion_state_unknown is True
    assert race.cancel_requests == [True]
    assert race.executor._active_trajectory_goal_token is not None


def test_get_result_request_failure_still_sends_queued_cancel(monkeypatch):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    result_future = _ControlledFuture()
    race = _contact_search_fjt_race_executor(
        result_future,
        resolve_goal=False,
    )
    race.executor._latch_fjt_motion_unknown = MethodType(
        moveit_executor.MoveItExecutor._latch_fjt_motion_unknown,
        race.executor,
    )
    race.executor._publish_execution_status = lambda *_args, **_kwargs: None
    race.executor._publish_motion_abort_latch = lambda: None
    race.executor._request_motion_abort = lambda _reason: setattr(
        race.executor, "_motion_abort_requested", True
    )
    race.handle.get_result_async = lambda: (_ for _ in ()).throw(
        RuntimeError("result request failed")
    )

    race.timers[0].callback()
    first_deadline = race.executor._active_trajectory_cancel_deadline
    clock["now"] = 11.9
    race.client.futures[0].resolve(race.handle)

    assert race.cancel_requests == [True]
    assert race.executor._active_trajectory_cancel_deadline == first_deadline
    assert race.executor._fjt_motion_state_unknown is True
    assert race.executor._active_trajectory_goal_token is not None


def test_contact_search_plan_pending_ft_invalidation_blocks_final_dispatch():
    executor, future, token, dispatches, hard_aborts, rejections, _restores = (
        _contact_search_ack_executor()
    )
    assert executor._contact_search_mode_ack_blockers() == ()
    executor._safety_status["ft_valid"] = False

    moveit_executor.MoveItExecutor._contact_search_plan_done(
        executor, future, token
    )

    assert dispatches == []
    assert hard_aborts == []
    assert len(rejections) == 1
    assert "SAFETY_ACK_FT_INVALID" in rejections[0]


@pytest.mark.parametrize(
    "plan_blocker",
    ("PLANE_GENERATION_CHANGED", "PATH_ID_CHANGED"),
)
def test_contact_search_post_tare_plan_identity_change_sends_no_fjt_goal(
    plan_blocker,
):
    executor, future, token, dispatches, hard_aborts, rejections, _restores = (
        _contact_search_ack_executor()
    )
    assert executor._contact_search_mode_ack_blockers() == ()
    executor._current_real_plan_blockers = lambda: (plan_blocker,)

    moveit_executor.MoveItExecutor._contact_search_plan_done(
        executor, future, token
    )

    assert dispatches == []
    assert hard_aborts == []
    assert len(rejections) == 1
    assert plan_blocker in rejections[0]


def test_contact_search_mode_ack_timeout_aborts_and_restores_acm(monkeypatch):
    (
        executor,
        _future,
        _token,
        dispatches,
        hard_aborts,
        rejections,
        restores,
    ) = (
        _contact_search_ack_executor(safety_mode="IDLE")
    )
    monkeypatch.setattr(
        moveit_executor.time,
        "monotonic",
        lambda: executor._contact_search_mode_published_at + 1.1,
    )

    moveit_executor.MoveItExecutor._wait_for_contact_search_mode_ack(
        executor, SimpleNamespace()
    )

    assert dispatches == []
    assert restores == [False]
    assert hard_aborts == []
    assert len(rejections) == 1
    assert "mode ACK timeout" in rejections[0]


def test_running_plan_status_change_keeps_initial_execution_snapshot():
    restores = []
    aborts = []
    logger = _Logger()
    executor = SimpleNamespace(
        real_painting_enabled=True,
        executing=True,
        _motion_abort_requested=False,
        _active_segment_path=SimpleNamespace(
            path_id="path-1", plan_hash="a" * 64
        ),
        _accepted_plan_hash="a" * 64,
        _accepted_plan_path_id="path-1",
        _execution_snapshot={
            "path_id": "path-1",
            "plan_hash": "a" * 64,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-1",
        },
        _plan_status_time=0.0,
        _set_contact_collision_allowed=lambda allowed: restores.append(allowed),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        get_logger=lambda: logger,
    )
    executor._abort_active_plan_invalidation = lambda reason: (
        moveit_executor.MoveItExecutor._abort_active_plan_invalidation(
            executor, reason
        )
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "state": "generated",
            "path_id": "path-2",
            "plan_hash": "b" * 64,
        }
    )

    moveit_executor.MoveItExecutor.on_plan_status(executor, message)

    assert executor._accepted_plan_hash == "a" * 64
    assert executor._accepted_plan_path_id == "path-1"
    assert restores == []
    assert aborts == []
    assert executor._execution_candidate_invalidated is True
    assert any("link0" in warning for warning in logger.warnings)

    moveit_executor.MoveItExecutor._clear_execution_snapshot(executor)
    assert executor._accepted_plan_hash == ""
    assert executor._accepted_plan_path_id == ""
    assert executor._d405_plane_accepted is False


@pytest.mark.parametrize(
    ("callback_name", "payload"),
    (
        (
            "on_d405_refinement_status",
            {
                "mode": "work_area",
                "accepted": True,
                "work_area_id": "work-area-2",
                "plane_generation_id": "plane-2",
            },
        ),
        (
            "on_work_area_state",
            {"selected": True, "work_area_id": "work-area-2"},
        ),
    ),
)
def test_running_d405_candidate_updates_do_not_mutate_initial_snapshot(
    callback_name, payload
):
    logger = _Logger()
    executor = SimpleNamespace(
        executing=True,
        _execution_snapshot={
            "path_id": "path-1",
            "plan_hash": "a" * 64,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-1",
        },
        _current_work_area_id="work-area-1",
        _current_plane_generation_id="plane-1",
        _d405_plane_accepted=True,
        _accepted_plan_hash="a" * 64,
        _accepted_plan_path_id="path-1",
        get_logger=lambda: logger,
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(payload)

    getattr(moveit_executor.MoveItExecutor, callback_name)(executor, message)

    assert executor._current_work_area_id == "work-area-1"
    assert executor._current_plane_generation_id == "plane-1"
    assert executor._d405_plane_accepted is True
    assert executor._accepted_plan_hash == "a" * 64
    assert executor._accepted_plan_path_id == "path-1"
    assert executor._execution_candidate_invalidated is True
    assert any("link0" in warning for warning in logger.warnings)


def test_ready_pose_handoff_gap_still_keeps_initial_d405_snapshot():
    logger = _Logger()
    executor = SimpleNamespace(
        # PRE_SKETCH_READY completion briefly clears `executing` before the
        # same accepted Run resumes.  The snapshot, not this transient flag,
        # owns the perception lock.
        executing=False,
        _execution_snapshot={
            "path_id": "path-1",
            "plan_hash": "a" * 64,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-1",
        },
        _current_work_area_id="work-area-1",
        _current_plane_generation_id="plane-1",
        _d405_plane_accepted=True,
        _accepted_plan_hash="a" * 64,
        _accepted_plan_path_id="path-1",
        get_logger=lambda: logger,
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "accepted": True,
            "work_area_id": "work-area-2",
            "plane_generation_id": "plane-2",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._current_work_area_id == "work-area-1"
    assert executor._current_plane_generation_id == "plane-1"
    assert executor._d405_plane_accepted is True


def test_same_d405_generation_status_does_not_invalidate_next_candidate():
    executor = SimpleNamespace(
        executing=True,
        _execution_snapshot={
            "path_id": "path-1",
            "plan_hash": "a" * 64,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-1",
        },
        _execution_candidate_invalidated=False,
        get_logger=lambda: _Logger(),
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "accepted": True,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-1",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._execution_candidate_invalidated is False


def _d405_prescan_status_executor(*, prescan_active=True):
    aborts = []
    lock_resets = []
    statuses = []
    executor = SimpleNamespace(
        real_painting_enabled=True,
        executing=True,
        _motion_abort_requested=False,
        _execution_snapshot=None,
        _d405_prescan_active=bool(prescan_active),
        _current_work_area_id="work-area-1",
        _current_plane_generation_id="plane-old",
        _d405_plane_accepted=True,
        _d405_status_time=0.0,
        _d405_refined_lock_active=True,
        _d405_refined_pose_armed=False,
        _d405_refined_pose_generation_id="",
        _accepted_plan_hash="a" * 64,
        _accepted_plan_path_id="path-1",
        _reset_d405_refined_lock=lambda reason, clear_surface=False: (
            lock_resets.append((str(reason), bool(clear_surface)))
        ),
        _publish_execution_status=lambda state, reason: statuses.append(
            (str(state), str(reason))
        ),
        _set_contact_collision_allowed=lambda _allowed: None,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        get_logger=lambda: _Logger(),
    )
    executor._abort_active_plan_invalidation = lambda reason: (
        moveit_executor.MoveItExecutor._abort_active_plan_invalidation(
            executor, reason
        )
    )
    return executor, aborts, lock_resets, statuses


@pytest.mark.parametrize(
    "state",
    (
        "capture_armed",
        "evaluating",
        "waiting_for_tf",
        "capture_rejected",
        "capture_ignored",
    ),
)
def test_d405_prescan_progress_states_do_not_invalidate_or_abort(state):
    executor, aborts, lock_resets, statuses = (
        _d405_prescan_status_executor()
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "state": state,
            "accepted": False,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-new",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._d405_plane_accepted is True
    assert executor._current_plane_generation_id == "plane-old"
    assert executor._accepted_plan_hash == "a" * 64
    assert executor._accepted_plan_path_id == "path-1"
    assert aborts == []
    assert lock_resets == []
    assert statuses == []


def test_d405_prescan_accepts_new_generation_without_hard_abort():
    executor, aborts, lock_resets, statuses = (
        _d405_prescan_status_executor()
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "state": "accepted",
            "accepted": True,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-new",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._d405_plane_accepted is True
    assert executor._current_plane_generation_id == "plane-new"
    assert executor._d405_refined_pose_armed is True
    assert executor._d405_refined_pose_generation_id == "plane-new"
    assert executor._accepted_plan_hash == ""
    assert executor._accepted_plan_path_id == ""
    assert aborts == []
    assert lock_resets == [("new accepted plane generation", True)]
    assert statuses == [("PLAN_INVALIDATED", "PLANE_GENERATION_CHANGED")]


def test_d405_prescan_rejection_clears_candidate_without_hard_abort():
    executor, aborts, _lock_resets, _statuses = (
        _d405_prescan_status_executor()
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "state": "rejected",
            "accepted": False,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-new",
            "rejection_reason": "inlier_ratio_rejected",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._d405_plane_accepted is False
    assert executor._current_plane_generation_id == "plane-new"
    assert executor._accepted_plan_hash == ""
    assert executor._accepted_plan_path_id == ""
    assert aborts == []


def test_d405_rejection_outside_prescan_still_aborts_active_motion():
    executor, aborts, _lock_resets, _statuses = (
        _d405_prescan_status_executor(prescan_active=False)
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "state": "rejected",
            "accepted": False,
            "work_area_id": "work-area-1",
            "plane_generation_id": "plane-new",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert aborts == ["PLAN_INVALIDATED:D405_PLANE_REJECTED"]


def test_stale_accepted_d405_status_does_not_reject_current_work_area():
    executor, aborts, lock_resets, statuses = (
        _d405_prescan_status_executor()
    )
    message = moveit_executor.String()
    message.data = moveit_executor.json.dumps(
        {
            "mode": "work_area",
            "state": "accepted",
            "accepted": True,
            "work_area_id": "work-area-old",
            "plane_generation_id": "plane-stale",
        }
    )

    moveit_executor.MoveItExecutor.on_d405_refinement_status(
        executor, message
    )

    assert executor._d405_plane_accepted is True
    assert executor._current_plane_generation_id == "plane-old"
    assert executor._accepted_plan_hash == "a" * 64
    assert executor._accepted_plan_path_id == "path-1"
    assert aborts == []
    assert lock_resets == []
    assert statuses == []


def test_execution_snapshot_deep_copies_link0_plane_corners_and_path():
    path = SimpleNamespace(
        version=3,
        frame_id="link0",
        path_id="path-1",
        plan_hash="a" * 64,
        work_area_id="work-area-1",
        plane_generation_id="plane-1",
        rows=[SimpleNamespace(position=[0.8, 0.0, 0.4])],
    )
    surface_point = np.array([0.8, 0.0, 0.4], dtype=float)
    surface_normal = np.array([-1.0, 0.0, 0.0], dtype=float)
    corners = np.array(
        [
            [0.8, -0.1, 0.5],
            [0.8, 0.1, 0.5],
            [0.8, 0.1, 0.3],
            [0.8, -0.1, 0.3],
        ],
        dtype=float,
    )
    executor = SimpleNamespace(
        dynamic_surface_point=surface_point,
        dynamic_surface_normal=surface_normal,
        dynamic_work_area_corners=corners,
        dynamic_surface_source="d405_refined",
        current_waypoints=[_pose(0.77, 0.0, 0.4)],
        _execution_snapshot=None,
        _active_segment_path=None,
    )

    error = moveit_executor.MoveItExecutor._capture_execution_snapshot(
        executor, path
    )
    assert error == ""

    surface_point[0] = 9.0
    surface_normal[:] = [0.0, 1.0, 0.0]
    corners[0, 1] = 9.0
    path.rows[0].position[0] = 9.0

    snapshot = executor._execution_snapshot
    assert snapshot["surface_point"].tolist() == [0.8, 0.0, 0.4]
    assert snapshot["surface_normal"].tolist() == [-1.0, 0.0, 0.0]
    assert snapshot["work_area_corners"][0].tolist() == [0.8, -0.1, 0.5]
    assert snapshot["segment_path"].rows[0].position[0] == 0.8


def test_fjt_cancel_timeout_keeps_active_unknown_token_and_republishes_abort(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    token = object()
    callbacks = []
    abort_publications = []
    statuses = []
    cancel_timer_calls = []
    executor = SimpleNamespace(
        _fjt_guard_timer=None,
        _active_trajectory_goal_token=token,
        _active_trajectory_result_deadline=100.0,
        _active_trajectory_cancel_requested=True,
        _active_trajectory_cancel_deadline=9.0,
        _motion_abort_requested=True,
        _fjt_motion_state_unknown=False,
        _execution_abort_reason="motion_abort",
        executing=False,
        _cancel_fjt_guard_timer=lambda: cancel_timer_calls.append(True),
        create_timer=lambda _period, callback: (
            callbacks.append(callback) or object()
        ),
        _publish_motion_abort_latch=lambda: abort_publications.append(True),
        _publish_execution_status=lambda *args, **kwargs: statuses.append(
            (args, kwargs)
        ),
        _request_motion_abort=lambda reason: pytest.fail(
            f"abort was already latched: {reason}"
        ),
        get_logger=lambda: _Logger(),
    )
    executor._latch_fjt_motion_unknown = lambda expected, name, reason: (
        moveit_executor.MoveItExecutor._latch_fjt_motion_unknown(
            executor, expected, name, reason
        )
    )

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor, token, "PAINT row", False
    )
    callbacks[0]()

    assert executor._active_trajectory_goal_token is token
    assert executor._fjt_motion_state_unknown is True
    assert executor.executing is True
    assert executor._execution_abort_reason == "FJT_CANCEL_TIMEOUT:PAINT row"
    assert abort_publications == [True]
    assert len(statuses) == 1
    assert statuses[0][0][0] == "FJT_MOTION_UNKNOWN"
    assert len(cancel_timer_calls) == 2


def test_unknown_fjt_keeps_trajectory_active_heartbeat_and_abort_republish():
    class _Publisher:
        def __init__(self):
            self.values = []

        def publish(self, message):
            self.values.append(bool(message.data))

    executor_heartbeat = _Publisher()
    trajectory_active = _Publisher()
    robot_stationary = _Publisher()
    motion_abort = _Publisher()
    mode_pub = _Publisher()
    force_pub = _Publisher()
    enable_pub = _Publisher()
    executor = SimpleNamespace(
        executing=True,
        _motion_abort_requested=True,
        _fjt_motion_state_unknown=True,
        _hardware_motion_inhibited=False,
        _active_trajectory_goal_token=object(),
        _joint_command_timer=None,
        _painting_command_mode="ABORT",
        _painting_command_force_n=0.0,
        _painting_command_enable=False,
        executor_heartbeat_pub=executor_heartbeat,
        trajectory_active_pub=trajectory_active,
        robot_stationary_pub=robot_stationary,
        motion_abort_pub=motion_abort,
        painting_mode_pub=mode_pub,
        painting_force_pub=force_pub,
        painting_enable_pub=enable_pub,
        _trajectory_command_active=lambda: True,
        _robot_stationary_for_bias=lambda: False,
        _publish_motion_abort_latch=lambda: (
            moveit_executor.MoveItExecutor._publish_motion_abort_latch(executor)
        ),
    )

    moveit_executor.MoveItExecutor._publish_executor_heartbeat(executor)

    assert trajectory_active.values == [True]
    assert robot_stationary.values == [False]
    assert motion_abort.values == [True]


@pytest.mark.parametrize(
    ("unknown", "hardware_inhibited", "expected"),
    (
        (True, False, "FJT_MOTION_STATE_UNKNOWN_RELAUNCH_REQUIRED"),
        (False, True, "HARDWARE_MOTION_INHIBITED_RELAUNCH_REQUIRED"),
    ),
)
def test_execution_abort_reset_rejects_unknown_or_hardware_inhibited_motion(
    unknown, hardware_inhibited, expected
):
    executor = SimpleNamespace(
        _fjt_motion_state_unknown=unknown,
        _hardware_motion_inhibited=hardware_inhibited,
        executing=False,
        _active_trajectory_goal_token=None,
        _painting_command_enable=False,
        _robot_stationary_for_bias=lambda: True,
        real_painting_enabled=False,
    )
    response = SimpleNamespace(success=None, message="")

    result = moveit_executor.MoveItExecutor.on_reset_execution_abort(
        executor, None, response
    )

    assert result.success is False
    assert expected in result.message


@pytest.mark.parametrize(
    ("unknown", "hardware_inhibited"),
    ((True, False), (False, True)),
)
def test_motion_dispatch_is_blocked_until_full_relaunch(
    unknown, hardware_inhibited
):
    backend_calls = []
    executor = SimpleNamespace(
        _fjt_motion_state_unknown=unknown,
        _hardware_motion_inhibited=hardware_inhibited,
        dry_run=False,
        execution_backend="follow_joint_trajectory",
        _execute_trajectory_follow_joint=lambda *_args, **_kwargs: (
            backend_calls.append(True) or True
        ),
        get_logger=lambda: _Logger(),
    )

    result = moveit_executor.MoveItExecutor.execute_trajectory_direct(
        executor, RobotTrajectory(), label="must not dispatch"
    )

    assert result is False
    assert backend_calls == []


def test_hardware_motion_inhibit_true_latches_until_relaunch(monkeypatch):
    clock = {"now": 1.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    aborts = []
    statuses = []
    executor = SimpleNamespace(
        _hardware_motion_inhibit_time=0.0,
        _hardware_motion_inhibited=False,
        _motion_abort_requested=False,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _publish_motion_abort_latch=lambda: pytest.fail(
            "first true ACK should latch the abort"
        ),
        _publish_execution_status=lambda *args, **kwargs: statuses.append(
            (args, kwargs)
        ),
        get_logger=lambda: _Logger(),
    )
    false_message = moveit_executor.Bool()
    false_message.data = False
    true_message = moveit_executor.Bool()
    true_message.data = True

    moveit_executor.MoveItExecutor.on_hardware_motion_inhibited(
        executor, false_message
    )
    moveit_executor.MoveItExecutor.on_hardware_motion_inhibited(
        executor, true_message
    )
    clock["now"] = 2.0
    moveit_executor.MoveItExecutor.on_hardware_motion_inhibited(
        executor, false_message
    )

    assert executor._hardware_motion_inhibited is True
    assert executor._hardware_motion_inhibit_time == pytest.approx(2.0)
    assert aborts == ["HARDWARE_MOTION_INHIBITED"]
    assert len(statuses) == 1


def _post_tare_fjt_guard_executor(*, tare_ready, status_time, status):
    token = object()
    callbacks = []
    aborts = []
    executor = SimpleNamespace(
        _fjt_guard_timer=None,
        _active_trajectory_goal_token=token,
        _active_trajectory_result_deadline=100.0,
        _active_trajectory_cancel_requested=False,
        _active_trajectory_cancel_deadline=0.0,
        _motion_abort_requested=False,
        _fjt_motion_state_unknown=False,
        real_painting_enabled=True,
        _execution_tare_ready=tare_ready,
        _safety_status_time=status_time,
        _safety_status=status,
        ft_required_timeout_s=0.20,
        _cancel_fjt_guard_timer=lambda: None,
        create_timer=lambda _period, callback: (
            callbacks.append(callback) or object()
        ),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        get_logger=lambda: _Logger(),
    )
    executor._post_tare_fjt_safety_blockers = lambda now: (
        moveit_executor.MoveItExecutor._post_tare_fjt_safety_blockers(
            executor, now
        )
    )
    return executor, token, callbacks, aborts


def test_pre_tare_approach_defers_invalid_ft_in_fjt_watchdog(monkeypatch):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    executor, token, callbacks, aborts = _post_tare_fjt_guard_executor(
        tare_ready=False,
        status_time=0.0,
        status={},
    )

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor, token, "APPROACH_PRECONTACT", False
    )
    callbacks[0]()

    assert aborts == []


@pytest.mark.parametrize("label", ("TRAVEL", "FINAL_RETRACT"))
def test_post_tare_fjt_stale_safety_aborts_all_physical_motion(
    monkeypatch, label
):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    executor, token, callbacks, aborts = _post_tare_fjt_guard_executor(
        tare_ready=True,
        status_time=9.5,
        status={
            "ft_valid": True,
            "tf_valid": True,
            "abort_latched": False,
        },
    )

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor, token, label, False
    )
    callbacks[0]()

    assert len(aborts) == 1
    assert f"POST_TARE_FJT_SAFETY:{label}" in aborts[0]
    assert "SAFETY_STATUS_STALE" in aborts[0]


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    (
        ("ft_valid", False, "FT_INVALID"),
        ("tf_valid", False, "FT_TF_INVALID"),
        ("abort_latched", True, "SAFETY_ABORT_LATCHED"),
    ),
)
def test_post_tare_fjt_requires_fresh_valid_clear_safety_status(
    monkeypatch, field, value, expected
):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.0)
    status = {
        "ft_valid": True,
        "tf_valid": True,
        "abort_latched": False,
        "reason": "TEST",
    }
    status[field] = value
    executor, token, callbacks, aborts = _post_tare_fjt_guard_executor(
        tare_ready=True,
        status_time=9.9,
        status=status,
    )

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor, token, "PAINT", False
    )
    callbacks[0]()

    assert len(aborts) == 1
    assert expected in aborts[0]


class _DiscardPublisher:
    def publish(self, _message):
        pass


def _guard_payload(mode, *, active, force_enable=None):
    if force_enable is None:
        force_enable = active
    return {
        "mode": mode,
        "force_enable": bool(force_enable),
        "forwarding": bool(active),
        "compliance_enabled": bool(active),
        "compliance_active": bool(active),
        "ft_valid": True,
        "tf_valid": True,
        "abort_latched": False,
        "controller_fault": False,
    }


def _command_context(generation, mode, *, enabled, published_at, sequence):
    return {
        "generation": generation,
        "mode": mode,
        "force_n": 1.6 if enabled else 0.0,
        "enable": enabled,
        "published_at_s": published_at,
        "guard_status_sequence": sequence,
        "guard_source_timestamp_s": published_at - 0.1,
    }


def _set_guard_sample(executor, *, now, sequence, mode, active):
    executor._guard_status_time = now
    executor._guard_status_source_time = now
    executor._guard_status_sequence = sequence
    executor._guard_status = _guard_payload(mode, active=active)


def test_source_stamped_guard_ack_rejects_queued_precommand_sample(monkeypatch):
    clock = {"now": 10.2}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    context = _command_context(
        2, "PAINT", enabled=True, published_at=10.0, sequence=20
    )
    executor = SimpleNamespace(
        painting_force_enabled=True,
        force_guard_status_timeout_s=0.5,
        _painting_command_generation=2,
        _painting_command_context=context,
        # Locally received after the command, but produced before it.
        _guard_status_time=10.1,
        _guard_status_source_time=9.99,
        _guard_status_sequence=21,
        _guard_status=_guard_payload("PAINT", active=True),
    )

    blockers = moveit_executor.MoveItExecutor._force_guard_status_blockers(
        executor,
        expected_mode="PAINT",
        active=True,
        command_context=context,
    )
    assert "GUARD_COMMAND_ACK_PENDING" in blockers

    _set_guard_sample(
        executor, now=10.21, sequence=22, mode="PAINT", active=True
    )
    clock["now"] = 10.21
    assert moveit_executor.MoveItExecutor._force_guard_status_blockers(
        executor,
        expected_mode="PAINT",
        active=True,
        command_context=context,
    ) == ()


def test_executor_accepts_only_new_source_stamped_ramp_status(monkeypatch):
    clock = {"now": 10.2}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    executor = SimpleNamespace(
        _painting_ramp_status={},
        _painting_ramp_status_time=0.0,
        _painting_ramp_status_source_time=0.0,
        _painting_ramp_status_sequence=0,
        _painting_ramp_complete=False,
        _painting_ramp_feedback_seq=0,
        _painting_ramp_feedback_time=0.0,
    )
    legacy = Bool(data=True)
    moveit_executor.MoveItExecutor.on_painting_ramp_complete(executor, legacy)
    assert executor._painting_ramp_feedback_seq == 0

    status = String()
    status.data = json.dumps(
        {
            "mode": "RAMP_UP",
            "force_enable": True,
            "ramp_complete": True,
            "published_monotonic_s": 10.1,
            "status_sequence": 7,
        }
    )
    moveit_executor.MoveItExecutor.on_painting_ramp_status(executor, status)
    assert executor._painting_ramp_feedback_seq == 1
    assert executor._painting_ramp_status_sequence == 7
    assert executor._painting_ramp_complete is True

    for malformed in ("[]", "null", "1", '"text"'):
        message = String(data=malformed)
        moveit_executor.MoveItExecutor.on_painting_ramp_status(
            executor, message
        )
        assert executor._painting_ramp_feedback_seq == 1
        assert executor._painting_ramp_status_sequence == 7
        assert executor._painting_ramp_complete is True

    # A later-delivered older source sample cannot roll the state back.
    stale = String()
    stale.data = json.dumps(
        {
            "mode": "RAMP_DOWN",
            "force_enable": True,
            "ramp_complete": False,
            "published_monotonic_s": 10.0,
            "status_sequence": 6,
        }
    )
    moveit_executor.MoveItExecutor.on_painting_ramp_status(executor, stale)
    assert executor._painting_ramp_feedback_seq == 1
    assert executor._painting_ramp_status_sequence == 7
    assert executor._painting_ramp_complete is True


def _ramp_executor(clock, step, initial_mode, *, initial_active):
    command = _command_context(
        1,
        initial_mode,
        enabled=initial_active,
        published_at=clock["now"] - 0.2,
        sequence=10,
    )
    scheduled = []
    aborts = []
    rejections = []
    execution_snapshot = object()
    executor = SimpleNamespace(
        painting_force_enabled=True,
        max_paint_force_n=15.0,
        _painting_command_generation=1,
        _painting_command_context=command,
        _execution_snapshot=execution_snapshot,
        _painting_command_mode=initial_mode,
        _painting_command_force_n=1.6 if initial_active else 0.0,
        _painting_command_enable=initial_active,
        _contact_search_command_context=(
            command if initial_mode == "CONTACT_SEARCH" else None
        ),
        _guard_status_time=clock["now"] - 0.1,
        _guard_status_source_time=clock["now"] - 0.1,
        _guard_status_sequence=11,
        _guard_status=_guard_payload(
            initial_mode, active=initial_active
        ),
        force_guard_status_timeout_s=0.5,
        force_guard_mode_ack_timeout_s=1.0,
        _contact_search_confirmed=True,
        _painting_ramp_feedback_seq=0,
        _painting_ramp_complete=False,
        _painting_ramp_status={},
        _painting_ramp_status_time=0.0,
        _painting_ramp_status_source_time=0.0,
        _painting_ramp_status_sequence=0,
        _painting_contact_confirmed=True,
        _painting_contact_feedback_time=clock["now"] - 0.1,
        require_contact_before_paint=True,
        painting_ramp_feedback_timeout_s=5.0,
        painting_ramp_settle_s=0.1,
        _process_force_ready=initial_active,
        _process_step_index=0,
        _process_steps=[step],
        _process_timer=None,
        _paint_entry_context=None,
        _force_phase_watchdog_timer=None,
        _force_phase_lease_context=(
            {
                "token": object(),
                "execution_snapshot": execution_snapshot,
                "mode": initial_mode,
                "active": True,
                "command_context": command,
                "step": step,
                "step_index": 0,
                "acknowledged": True,
                "zero_ack_observed": False,
            }
            if initial_active
            else None
        ),
        _motion_abort_requested=False,
        painting_force_pub=_DiscardPublisher(),
        painting_enable_pub=_DiscardPublisher(),
        painting_mode_pub=_DiscardPublisher(),
        create_timer=lambda _period, callback: _FakeTimer(callback),
        destroy_timer=lambda _timer: None,
        _transition_execution_state=lambda *_args, **_kwargs: None,
        _schedule_process_once=lambda delay, callback: scheduled.append(
            (delay, callback)
        ),
        _complete_process_step=lambda: None,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _fail_workflow_known_safe=lambda reason, **_kwargs: rejections.append(
            str(reason)
        ),
        get_logger=lambda: _Logger(),
    )
    executor._publish_painting_command = MethodType(
        moveit_executor.MoveItExecutor._publish_painting_command, executor
    )
    executor._cancel_process_timer = MethodType(
        moveit_executor.MoveItExecutor._cancel_process_timer, executor
    )
    return executor, scheduled, aborts, rejections


def test_ramp_up_waits_for_exact_postcommand_active_guard_ack(monkeypatch):
    clock = {"now": 20.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    step = SimpleNamespace(mode="RAMP_UP", force_n=1.6)
    executor, scheduled, aborts, rejections = _ramp_executor(
        clock, step, "CONTACT_SEARCH", initial_active=False
    )

    moveit_executor.MoveItExecutor._start_process_ramp(executor, step)
    timer = executor._process_timer
    # A legacy Bool TRUE received after the command cannot complete a ramp.
    executor._painting_ramp_complete = True
    timer.callback()
    assert scheduled == []

    # The first post-edge status may legitimately arrive before the
    # compliance controller reports active.  It is pending, not a fault.
    clock["now"] = 20.02
    _set_guard_sample(
        executor, now=20.02, sequence=12, mode="RAMP_UP", active=False
    )
    timer.callback()
    assert aborts == []
    assert scheduled == []

    clock["now"] = 20.05
    _set_guard_sample(
        executor, now=20.05, sequence=13, mode="RAMP_UP", active=True
    )
    executor._painting_ramp_status = {
        "mode": "RAMP_UP",
        "force_enable": True,
        "ramp_complete": True,
    }
    executor._painting_ramp_status_sequence = 1
    executor._painting_ramp_status_source_time = 20.05
    executor._painting_ramp_status_time = 20.05
    executor._painting_contact_feedback_time = 20.05
    timer.callback()

    assert aborts == []
    assert rejections == []
    assert len(scheduled) == 1
    assert executor._process_force_ready is True


def test_ramp_down_waits_for_postdisable_zero_ack_then_advances(monkeypatch):
    clock = {"now": 30.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    step = SimpleNamespace(mode="RAMP_DOWN", force_n=0.0)
    executor, scheduled, aborts, rejections = _ramp_executor(
        clock, step, "PAINT", initial_active=True
    )
    executor._contact_escape_context = {
        "ramp_step_index": 0,
        "execution_snapshot": executor._execution_snapshot,
        "phase": "RAMP_DOWN",
        "zero_ack_verified": False,
    }

    moveit_executor.MoveItExecutor._start_process_ramp(executor, step)
    timer = executor._process_timer
    clock["now"] = 30.05
    _set_guard_sample(
        executor, now=30.05, sequence=12, mode="RAMP_DOWN", active=True
    )
    executor._painting_ramp_status = {
        "mode": "RAMP_DOWN",
        "force_enable": True,
        "ramp_complete": True,
    }
    executor._painting_ramp_status_sequence = 1
    executor._painting_ramp_status_source_time = 30.05
    executor._painting_ramp_status_time = 30.05
    timer.callback()
    assert executor._painting_command_enable is False
    assert scheduled == []

    # A queued pre-disable sample cannot complete the barrier.
    timer.callback()
    assert scheduled == []

    clock["now"] = 30.10
    _set_guard_sample(
        executor, now=30.10, sequence=13, mode="RAMP_DOWN", active=False
    )
    timer.callback()

    assert aborts == []
    assert rejections == []
    assert len(scheduled) == 1
    assert executor._process_force_ready is False
    assert executor._force_phase_lease_context is None
    assert executor._contact_escape_context["zero_ack_verified"] is True
    assert executor._contact_escape_context["phase"] == "ZERO_ACK_VERIFIED"


def test_ramp_down_missing_zero_ack_is_explicit_hard_abort(monkeypatch):
    clock = {"now": 40.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    step = SimpleNamespace(mode="RAMP_DOWN", force_n=0.0)
    executor, scheduled, aborts, _rejections = _ramp_executor(
        clock, step, "PAINT", initial_active=True
    )
    moveit_executor.MoveItExecutor._start_process_ramp(executor, step)
    timer = executor._process_timer
    clock["now"] = 40.05
    _set_guard_sample(
        executor, now=40.05, sequence=12, mode="RAMP_DOWN", active=True
    )
    executor._painting_ramp_status = {
        "mode": "RAMP_DOWN",
        "force_enable": True,
        "ramp_complete": True,
    }
    executor._painting_ramp_status_sequence = 1
    executor._painting_ramp_status_source_time = 40.05
    executor._painting_ramp_status_time = 40.05
    timer.callback()

    clock["now"] = 41.10
    timer.callback()
    assert scheduled == []
    assert aborts == ["RAMP_DOWN_GUARD_ZERO_ACK_TIMEOUT"]


def test_paint_waits_for_mode_ack_and_monitors_guard_during_planning(
    monkeypatch,
):
    clock = {"now": 50.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    step = SimpleNamespace(mode="PAINT", force_n=1.6)
    executor, _scheduled, aborts, _rejections = _ramp_executor(
        clock, step, "RAMP_UP", initial_active=True
    )
    snapshot = object()
    acm_callbacks = []
    plans = []
    executor._execution_snapshot = snapshot
    executor._force_phase_lease_context["execution_snapshot"] = snapshot
    executor._set_contact_collision_allowed = (
        lambda _allowed, callback=None: acm_callbacks.append(callback)
    )
    executor._plan_process_motion_step = lambda planned: plans.append(planned)
    executor._paint_entry_is_current = MethodType(
        moveit_executor.MoveItExecutor._paint_entry_is_current, executor
    )
    executor._paint_entry_tick = MethodType(
        moveit_executor.MoveItExecutor._paint_entry_tick, executor
    )
    executor._paint_entry_acm_done = MethodType(
        moveit_executor.MoveItExecutor._paint_entry_acm_done, executor
    )

    moveit_executor.MoveItExecutor._start_paint_entry_barrier(executor, step)
    timer = executor._process_timer
    acm_callbacks[0](True)
    assert plans == []

    clock["now"] = 50.02
    _set_guard_sample(
        executor, now=50.02, sequence=12, mode="PAINT", active=False
    )
    timer.callback()
    assert plans == []
    assert aborts == []

    clock["now"] = 50.05
    _set_guard_sample(
        executor, now=50.05, sequence=13, mode="PAINT", active=True
    )
    timer.callback()
    assert plans == [step]

    # Planning is asynchronous.  Losing the guard before the Cartesian result
    # arrives must abort rather than dispatching from stale force state.
    clock["now"] = 50.60
    timer.callback()
    assert len(aborts) == 1
    assert "PAINT_GUARD_ACTIVE_ACK_FAILED" in aborts[0]


def test_active_paint_fjt_watchdog_aborts_when_guard_status_stales(monkeypatch):
    clock = {"now": 60.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    executor, token, callbacks, aborts = _post_tare_fjt_guard_executor(
        tare_ready=True,
        status_time=60.0,
        status={
            "ft_valid": True,
            "tf_valid": True,
            "abort_latched": False,
        },
    )
    command = _command_context(
        2, "PAINT", enabled=True, published_at=59.8, sequence=20
    )
    executor.painting_force_enabled = True
    executor.force_guard_status_timeout_s = 0.5
    executor._painting_command_generation = 2
    executor._painting_command_context = command
    executor._painting_command_mode = "PAINT"
    executor._painting_command_enable = True
    executor._contact_search_command_context = None
    _set_guard_sample(
        executor, now=60.0, sequence=21, mode="PAINT", active=True
    )
    executor._ft_guard_triggered = lambda _label: False

    moveit_executor.MoveItExecutor._start_fjt_guard_timer(
        executor, token, "PAINT PROCESS 4 PAINT", True
    )
    callbacks[0]()
    assert aborts == []

    clock["now"] = 60.51
    executor._safety_status_time = 60.51
    callbacks[0]()
    assert len(aborts) == 1
    assert "FJT_WRENCH_GUARD" in aborts[0]


def _contact_row(mode, *, offset=0.0):
    return SimpleNamespace(
        mode=mode,
        row_number=1,
        position=(0.0, 0.0, 0.0),
        normal=(0.0, 0.0, 1.0),
        tangent=(1.0, 0.0, 0.0),
        force_n=1.6 if mode == "PAINT" else 0.0,
        offset_m=float(offset),
        speed_mps=0.01,
    )


def _pose_at_z(value):
    pose = Pose()
    pose.position.z = float(value)
    pose.orientation.w = 1.0
    return pose


def test_paint_keeps_contact_acm_until_outward_retract_restore():
    paint = SimpleNamespace(mode="PAINT", rows=(_contact_row("PAINT"),))
    ramp = SimpleNamespace(
        mode="RAMP_DOWN", rows=(_contact_row("RAMP_DOWN"),)
    )
    retract = SimpleNamespace(
        mode="RETRACT", rows=(_contact_row("RETRACT", offset=0.01),)
    )
    travel = SimpleNamespace(mode="TRAVEL", rows=(_contact_row("TRAVEL"),))
    snapshot = object()
    advances = []
    restores = []
    aborts = []
    executor = SimpleNamespace(
        _motion_abort_requested=False,
        _process_steps=[paint, ramp, retract, travel],
        _process_step_index=0,
        _process_last_tcp_pose=None,
        _execution_snapshot=snapshot,
        active_target_name="wall",
        painting_force_enabled=True,
        _contact_collision_allowed=True,
        _contact_collision_baseline=object(),
        _contact_escape_context=None,
        _force_phase_lease_context={
            "mode": "PAINT",
            "active": True,
            "acknowledged": True,
        },
        _active_segment_path=SimpleNamespace(
            travel_clearance_m=0.01,
            final_retreat_offset_m=0.08,
        ),
        minimum_travel_clearance_m=0.01,
        _hardware_motion_inhibited=False,
        _fjt_motion_state_unknown=False,
        _acm_state_unknown=False,
        dry_run=False,
        _acm_baseline_verified=True,
        _acm_baseline_verified_target_name="wall",
        _acm_update_pending=None,
        _contact_collision_target_name="wall",
        executing=True,
        _execution_state="RETRACT",
        _painting_command_enable=True,
        _process_force_ready=True,
        _complete_process_step=lambda: advances.append(True),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _set_contact_collision_allowed=lambda allowed, callback=None: restores.append(
            (allowed, callback)
        ),
    )

    moveit_executor.MoveItExecutor._process_motion_done(
        executor, _pose_at_z(0.0)
    )
    assert advances == [True]
    assert restores == []
    context = executor._contact_escape_context
    assert context["phase"] == "PAINT_COMPLETE"

    context["zero_ack_verified"] = True
    context["phase"] = "ZERO_ACK_VERIFIED"
    executor._painting_command_enable = False
    executor._process_force_ready = False
    executor._process_step_index = 2
    assert moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(
        executor,
        requires_contact_acm=True,
        contact_acm_context=context,
    ) == ""
    target = _pose_at_z(0.01)
    assert moveit_executor.MoveItExecutor._contact_escape_step_error(
        executor, context, retract, [target]
    ) == ""

    moveit_executor.MoveItExecutor._process_motion_done(executor, target)
    assert advances == [True]
    assert len(restores) == 1
    assert restores[0][0] is False

    executor._contact_collision_allowed = False
    restores[0][1](True)
    assert advances == [True, True]
    assert executor._contact_escape_context is None
    restores[0][1](True)
    assert advances == [True, True]
    assert aborts == []


def test_contact_escape_rejects_tangent_motion_and_requires_zero_ack():
    step = SimpleNamespace(
        mode="RETRACT", rows=(_contact_row("RETRACT", offset=0.01),)
    )
    snapshot = object()
    context = {
        "phase": "PAINT_COMPLETE",
        "execution_snapshot": snapshot,
        "target": "wall",
        "escape_step_index": 0,
        "paint_final_pose": _pose_at_z(0.0),
        "surface_point": (0.0, 0.0, 0.0),
        "normal": (0.0, 0.0, 1.0),
        "tangent": (1.0, 0.0, 0.0),
        "zero_ack_verified": False,
    }
    executor = SimpleNamespace(
        _contact_escape_context=context,
        _motion_abort_requested=False,
        _execution_snapshot=snapshot,
        active_target_name="wall",
        _contact_collision_allowed=True,
        _contact_collision_baseline=object(),
        _process_steps=[step],
        _process_step_index=0,
        _painting_command_enable=False,
        _process_force_ready=False,
        _active_segment_path=SimpleNamespace(
            travel_clearance_m=0.01,
            final_retreat_offset_m=0.08,
        ),
        minimum_travel_clearance_m=0.01,
    )
    assert not moveit_executor.MoveItExecutor._contact_escape_is_current(
        executor, context, step=step, require_zero_ack=True
    )

    context["zero_ack_verified"] = True
    context["phase"] = "ZERO_ACK_VERIFIED"
    tangent_target = _pose_at_z(0.01)
    tangent_target.position.x = 0.01
    assert (
        moveit_executor.MoveItExecutor._contact_escape_step_error(
            executor, context, step, [tangent_target]
        )
        == "CONTACT_ESCAPE_NOT_BOUNDED_NORMAL_OUTWARD"
    )


def test_contact_escape_restore_failure_hard_aborts_before_travel():
    step = SimpleNamespace(mode="RETRACT")
    context = {"escape_step_index": 0}
    aborts = []
    executor = SimpleNamespace(
        _contact_escape_context=context,
        _motion_abort_requested=False,
        _process_step_index=0,
        _process_steps=[step],
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
    )
    moveit_executor.MoveItExecutor._contact_escape_restore_done(
        executor, context, step, False
    )
    assert aborts == ["CONTACT_ESCAPE_ACM_RESTORE_FAILED"]


def test_paint_cartesian_timeout_invalidates_late_result_without_dispatch():
    step = SimpleNamespace(mode="PAINT")
    token = object()
    context = {
        "token": token,
        "step": step,
        "timeout_timer": None,
    }
    aborts = []
    timers = []
    executor = SimpleNamespace(
        paint_cartesian_planning_timeout_s=0.2,
        _segment_cartesian_context=context,
        _segment_cartesian_timeout_timer=None,
        _motion_abort_requested=False,
        create_timer=lambda _period, callback: timers.append(
            _FakeTimer(callback)
        )
        or timers[-1],
        destroy_timer=lambda _timer: None,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _fail_workflow_known_safe=lambda reason: pytest.fail(str(reason)),
    )
    moveit_executor.MoveItExecutor._arm_segment_cartesian_timeout(
        executor, context
    )
    timers[0].callback()
    assert aborts == ["PAINT_CARTESIAN_TIMEOUT"]
    assert executor._segment_cartesian_context is None

    # The service may complete after the timeout, but the invalidated token
    # must make the callback inert before it can inspect or dispatch anything.
    moveit_executor.MoveItExecutor._process_cartesian_done(
        executor, _ControlledFuture(), token
    )
    assert aborts == ["PAINT_CARTESIAN_TIMEOUT"]


def test_paint_done_callback_cannot_beat_an_expired_deadline(monkeypatch):
    monkeypatch.setattr(moveit_executor.time, "monotonic", lambda: 10.01)
    step = SimpleNamespace(mode="PAINT")
    token = object()
    timer = _FakeTimer(lambda: None)
    context = {
        "token": token,
        "step": step,
        "step_index": 0,
        "planning_deadline_s": 10.0,
        "timeout_timer": timer,
        "contact_escape_context": None,
    }
    aborts = []
    executor = SimpleNamespace(
        _segment_cartesian_context=context,
        _segment_cartesian_timeout_timer=timer,
        _process_step_index=0,
        _motion_abort_requested=False,
        destroy_timer=lambda _timer: None,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
        _fail_workflow_known_safe=lambda reason: pytest.fail(str(reason)),
    )
    moveit_executor.MoveItExecutor._process_cartesian_done(
        executor, _ControlledFuture(), token
    )
    assert aborts == ["PAINT_CARTESIAN_DEADLINE_EXCEEDED"]
    assert executor._segment_cartesian_context is None


def test_force_lease_survives_paint_result_gap_and_aborts_on_stale_guard(
    monkeypatch,
):
    clock = {"now": 70.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    snapshot = object()
    command = _command_context(
        1, "RAMP_UP", enabled=True, published_at=69.9, sequence=10
    )
    timers = []
    aborts = []
    executor = SimpleNamespace(
        painting_force_enabled=True,
        force_guard_status_timeout_s=0.5,
        force_guard_mode_ack_timeout_s=1.0,
        _painting_command_generation=1,
        _painting_command_context=command,
        _execution_snapshot=snapshot,
        _process_step_index=0,
        _force_phase_lease_context=None,
        _force_phase_watchdog_timer=None,
        _motion_abort_requested=False,
        create_timer=lambda _period, callback: timers.append(
            _FakeTimer(callback)
        )
        or timers[-1],
        destroy_timer=lambda _timer: None,
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
    )
    _set_guard_sample(
        executor, now=70.0, sequence=11, mode="RAMP_UP", active=True
    )
    context = moveit_executor.MoveItExecutor._set_force_phase_lease_command(
        executor,
        command,
        mode="RAMP_UP",
        active=True,
        step=SimpleNamespace(mode="RAMP_UP"),
    )
    timers[0].callback()
    assert context["acknowledged"] is True

    # No FJT token is needed for this guard: it deliberately remains alive in
    # the result-to-RAMP_DOWN gap.
    clock["now"] = 70.51
    timers[0].callback()
    assert len(aborts) == 1
    assert "FORCE_LEASE_GUARD_LOST" in aborts[0]
