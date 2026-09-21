from types import MethodType, SimpleNamespace
import copy
import math
import time

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


moveit_executor = pytest.importorskip("sketch_control.moveit_executor")


class _Logger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message, *_args, **_kwargs):
        self.errors.append(str(message))

    def info(self, message, *_args, **_kwargs):
        self.infos.append(str(message))

    def warn(self, message, *_args, **_kwargs):
        self.warnings.append(str(message))


class _ControlledFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def resolve(self, result):
        self._result = result
        for callback in list(self._callbacks):
            callback(self)


class _QueuedServiceClient:
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


def _joint_state(values):
    state = moveit_executor.JointState()
    state.name = list(moveit_executor.READY_POSE_JOINTS)
    state.position = [float(value) for value in values]
    state.velocity = [0.0] * len(state.name)
    return state


def _trajectory(rows, *, names=None):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(
        names or moveit_executor.READY_POSE_JOINTS
    )
    for index, row in enumerate(rows):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in row]
        total_ns = int(index * 100_000_000)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.joint_trajectory.points.append(point)
    return trajectory


def _pose(x=0.0, y=0.0, z=0.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return pose


def _ik_response(values=None, error_code=1):
    response = moveit_executor.GetPositionIK.Response()
    response.error_code.val = int(error_code)
    if values is not None:
        response.solution.joint_state = _joint_state(values)
    return response


def _cartesian_response(fraction, trajectory=None, error_code=1):
    response = moveit_executor.GetCartesianPath.Response()
    response.error_code.val = int(error_code)
    response.fraction = fraction
    response.solution = trajectory or _trajectory([[0.0] * 6, [0.01] * 6])
    return response


def _bind(node, *method_names):
    for name in method_names:
        setattr(
            node,
            name,
            MethodType(getattr(moveit_executor.MoveItExecutor, name), node),
        )


def _dual_ik_executor():
    logger = _Logger()
    ik_client = _QueuedServiceClient()
    move_client = _QueuedActionClient()
    destroyed_timers = []
    finalized = []
    token = object()
    points = [
        np.array([0.80, -0.05, 0.60], dtype=float),
        np.array([0.80, 0.01, 0.60], dtype=float),
    ]
    normal = np.array([-1.0, 0.0, 0.0], dtype=float)
    quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    queue = [
        moveit_executor.MoveItExecutor._make_d405_scan_tcp_pose(
            SimpleNamespace(), point, normal, quaternion
        )
        for point in points
    ]
    node = SimpleNamespace(
        ik_client=ik_client,
        move_action_client=move_client,
        current_joint_state=_joint_state([0.0] * 6),
        current_joint_state_time=time.monotonic(),
        _d405_prescan_active=True,
        _d405_prescan_token=token,
        _d405_prescan_queue=queue,
        _d405_prescan_surface_points=points,
        _d405_prescan_surface_normal=normal,
        _d405_orientation_generation=0,
        _d405_orientation_context=None,
        _d405_orientation_timer=None,
        _d405_arrival_timer=None,
        _d405_selected_orientation_branch="",
        _motion_abort_requested=False,
        _scene_revision=7,
        _scene_confirmed_revision=7,
        scene_confirmed=True,
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        create_timer=lambda _period, callback: _FakeTimer(callback),
        destroy_timer=lambda timer: destroyed_timers.append(timer),
    )
    _bind(
        node,
        "_cancel_d405_orientation_timer",
        "_cancel_d405_arrival_timer",
        "_d405_prescan_callback_valid",
        "_d405_scene_revision_confirmed",
        "_make_d405_scan_tcp_pose",
        "_d405_scan_pose_branch",
        "_joint_state_within_limits",
        "_stage1_candidate_joint_metrics",
        "_request_d405_orientation_candidate_iks",
        "_d405_orientation_candidate_ik_done",
        "_finish_d405_orientation_candidate_iks",
        "_activate_d405_orientation_rank",
        "_try_next_d405_orientation_rank",
        "_send_d405_orientation_plan",
    )
    node._nearest_joint_equivalent = (
        moveit_executor.MoveItExecutor._nearest_joint_equivalent
    )
    return node, ik_client, move_client, finalized, logger


def test_d405_prescan_speed_is_only_seventy_percent_of_previous_value():
    assert moveit_executor.D405_PREFLIGHT_SPEED_SCALE == pytest.approx(0.0175)


def test_d405_orientation_branches_preserve_every_camera_center():
    node, _ik, _move, _finalized, _logger = _dual_ik_executor()

    branch_a = node._d405_scan_pose_branch(flipped=False)
    branch_b = node._d405_scan_pose_branch(flipped=True)

    assert len(branch_a) == len(branch_b) == 2
    for surface, pose_a, pose_b in zip(
        node._d405_prescan_surface_points, branch_a, branch_b
    ):
        q_a = np.array(
            [
                pose_a.orientation.x,
                pose_a.orientation.y,
                pose_a.orientation.z,
                pose_a.orientation.w,
            ]
        )
        q_b = np.array(
            [
                pose_b.orientation.x,
                pose_b.orientation.y,
                pose_b.orientation.z,
                pose_b.orientation.w,
            ]
        )
        position_a = np.array(
            [pose_a.position.x, pose_a.position.y, pose_a.position.z]
        )
        position_b = np.array(
            [pose_b.position.x, pose_b.position.y, pose_b.position.z]
        )
        camera_a = position_a + moveit_executor.quat_apply(
            q_a, moveit_executor.D405_COLLISION_CENTER
        )
        camera_b = position_b + moveit_executor.quat_apply(
            q_b, moveit_executor.D405_COLLISION_CENTER
        )
        expected = surface + (
            node._d405_prescan_surface_normal
            * moveit_executor.D405_PREFLIGHT_SCAN_STANDOFF
        )
        np.testing.assert_allclose(camera_a, expected, atol=1e-9)
        np.testing.assert_allclose(camera_b, expected, atol=1e-9)
        np.testing.assert_allclose(
            moveit_executor.quat_to_matrix(q_b),
            moveit_executor.quat_to_matrix(q_a) @ np.diag([-1.0, 1.0, -1.0]),
            atol=1e-9,
        )


def test_dual_d405_ik_uses_same_fresh_seed_and_exact_selected_joint_goal():
    node, ik_client, move_client, finalized, _logger = _dual_ik_executor()

    node._request_d405_orientation_candidate_iks(
        "D405_PRESCAN_1", node._d405_prescan_queue[0], finalized.append
    )

    assert len(ik_client.requests) == 2
    assert all(request.ik_request.avoid_collisions for request in ik_client.requests)
    assert all(
        request.ik_request.ik_link_name == moveit_executor.EE_LINK
        and request.ik_request.pose_stamped.header.frame_id
        == moveit_executor.BASE_FRAME
        for request in ik_client.requests
    )
    assert list(
        ik_client.requests[0].ik_request.robot_state.joint_state.position
    ) == list(
        ik_client.requests[1].ik_request.robot_state.joint_state.position
    ) == [0.0] * 6

    # Callback order must not influence deterministic nearest-branch ranking.
    ik_client.futures[1].resolve(_ik_response([0.40] * 6))
    assert move_client.goals == []
    ik_client.futures[0].resolve(_ik_response([0.10] * 6))

    assert finalized == []
    assert len(move_client.goals) == 1
    goal = move_client.goals[0]
    assert goal.planning_options.plan_only is True
    assert list(goal.request.start_state.joint_state.position) == [0.0] * 6
    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.0175)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.0175)
    constraints = goal.request.goal_constraints[0].joint_constraints
    assert [constraint.joint_name for constraint in constraints] == list(
        moveit_executor.READY_POSE_JOINTS
    )
    assert [constraint.position for constraint in constraints] == pytest.approx(
        [0.10] * 6
    )
    assert node._d405_selected_orientation_branch == "d405_near_current"


def test_dual_d405_ik_can_use_large_collision_free_symmetry_branch():
    node, ik_client, move_client, finalized, logger = _dual_ik_executor()

    node._request_d405_orientation_candidate_iks(
        "D405_PRESCAN_1", node._d405_prescan_queue[0], finalized.append
    )
    ik_client.futures[0].resolve(_ik_response(error_code=-31))
    ik_client.futures[1].resolve(
        _ik_response([1.60, 0.0, 0.0, 0.0, 0.0, 0.0])
    )

    assert finalized == []
    assert len(move_client.goals) == 1
    assert node._d405_selected_orientation_branch == (
        "d405_flipped_local_y_180"
    )
    assert any("large smooth rotation" in message for message in logger.warnings)


def test_stale_d405_ik_seed_fails_before_any_collision_request():
    node, ik_client, _move_client, finalized, _logger = _dual_ik_executor()
    node.current_joint_state_time = time.monotonic() - 1.0

    node._request_d405_orientation_candidate_iks(
        "D405_PRESCAN_1", node._d405_prescan_queue[0], finalized.append
    )

    assert ik_client.requests == []
    assert finalized == [False]


def _scene_wait_executor(monkeypatch):
    now = {"value": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    node, ik_client, move_client, finalized, logger = _dual_ik_executor()
    node.current_joint_state_time = now["value"]
    scan_poses = copy.deepcopy(node._d405_prescan_queue)
    published = []
    destroyed = []
    node._scene_revision = 11
    node._scene_confirmed_revision = -1
    node.scene_confirmed = False
    node._d405_scene_wait_timer = None
    node._d405_scene_wait_start = None
    node._d405_scene_wait_revision = None
    node._d405_prescan_timer = None
    node._d405_prescan_wait_start = None
    node._d405_prescan_capture_sent = False
    node._d405_prescan_capture_time = 0.0
    node._d405_prescan_index = 0
    node._d405_prescan_pose_done = lambda success: finalized.append(bool(success))
    node._build_d405_prescan_poses = lambda: copy.deepcopy(scan_poses)
    node.publish_scene_periodic = lambda: published.append(node._scene_revision)
    node.destroy_timer = lambda timer: destroyed.append(timer)
    _bind(
        node,
        "_cancel_d405_scene_wait_timer",
        "_invalidate_d405_prescan_callbacks",
        "_start_d405_scene_wait",
        "_d405_wait_for_scene_confirmed",
        "_begin_d405_prescan",
        "_cancel_d405_prescan_timer",
        "_plan_next_d405_prescan_pose",
    )
    return (
        node,
        ik_client,
        move_client,
        finalized,
        logger,
        published,
        destroyed,
        now,
    )


def test_dirty_scene_begin_waits_for_exact_revision_before_dual_ik(monkeypatch):
    node, ik_client, _move, _finalized, _logger, published, _destroyed, _now = (
        _scene_wait_executor(monkeypatch)
    )

    assert node._begin_d405_prescan(mode="work_area")
    timer = node._d405_scene_wait_timer

    assert published == [11]
    assert timer is not None
    assert ik_client.requests == []

    # Complete ApplyPlanningScene for the exact captured geometry revision.
    node._scene_apply_inflight_revision = 11
    moveit_executor.MoveItExecutor._apply_scene_done(
        node,
        SimpleNamespace(result=lambda: SimpleNamespace(success=True)),
        11,
    )
    assert node.scene_confirmed is True
    assert node._scene_confirmed_revision == 11
    timer.callback()

    assert node._d405_scene_wait_timer is None
    assert len(ik_client.requests) == 2
    assert all(request.ik_request.avoid_collisions for request in ik_client.requests)


def test_late_old_scene_wait_timer_cannot_start_new_prescan_ik(monkeypatch):
    node, ik_client, _move, _finalized, _logger, published, _destroyed, _now = (
        _scene_wait_executor(monkeypatch)
    )
    assert node._begin_d405_prescan(mode="work_area")
    old_timer = node._d405_scene_wait_timer

    node._scene_revision = 12
    node.scene_confirmed = False
    node._scene_confirmed_revision = -1
    assert node._begin_d405_prescan(mode="work_area")
    new_timer = node._d405_scene_wait_timer

    assert new_timer is not old_timer
    assert old_timer.cancelled is True
    assert published == [11, 12]
    old_timer.callback()
    assert ik_client.requests == []
    assert node._d405_scene_wait_timer is new_timer

    node.scene_confirmed = True
    node._scene_confirmed_revision = 12
    new_timer.callback()
    assert len(ik_client.requests) == 2


def _trajectory_guard_executor():
    logger = _Logger()
    node = SimpleNamespace(
        current_joint_state=_joint_state([0.0] * 6),
        get_logger=lambda: logger,
    )
    _bind(
        node,
        "_current_positions_for_joints",
        "_trajectory_joint_metrics",
        "_trajectory_within_joint_limits",
        "_d405_prescan_trajectory_is_safe",
    )
    node._max_joint_delta = moveit_executor.MoveItExecutor._max_joint_delta
    return node, logger


def test_d405_safety_allows_valid_dense_plan_but_keeps_meaningful_hard_gates():
    node, logger = _trajectory_guard_executor()
    goal = np.array(
        [1.22, math.sqrt(1.44**2 - 1.22**2), 0.0, 0.0, 0.0, 0.0]
    )
    dense = _trajectory(
        [goal * alpha for alpha in np.linspace(0.0, 1.0, 300)]
    )

    assert node._d405_prescan_trajectory_is_safe(
        dense, "D405_PRESCAN_VALID_DENSE"
    )
    assert any("diagnostic only" in message for message in logger.warnings)

    detour = _trajectory(
        [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0] * 6]
    )
    assert node._d405_prescan_trajectory_is_safe(
        detour, "D405_PRESCAN_PATH_TOO_LONG"
    )
    assert any("joint_path" in message for message in logger.warnings)

    large_but_smooth = _trajectory(
        [
            [math.radians(100.0) * alpha, 0.0, 0.0, 0.0, 0.0, 0.0]
            for alpha in np.linspace(0.0, 1.0, 100)
        ]
    )
    assert node._d405_prescan_trajectory_is_safe(
        large_but_smooth, "D405_PRESCAN_LARGE_SMOOTH_ROTATION"
    )
    assert any("start_goal_delta" in message for message in logger.warnings)

    discontinuous_jump = _trajectory(
        [[0.0] * 6, [math.radians(86.0), 0.0, 0.0, 0.0, 0.0, 0.0]]
    )
    assert not node._d405_prescan_trajectory_is_safe(
        discontinuous_jump, "D405_PRESCAN_DISCONTINUOUS_JUMP"
    )


def test_plan_start_must_still_match_fresh_measured_seed_and_first_point():
    trajectory = _trajectory([[0.0] * 6, [0.1] * 6])
    node = SimpleNamespace(
        current_joint_state=_joint_state([0.02] + [0.0] * 5),
        current_joint_state_time=time.monotonic(),
    )

    ok, reason = moveit_executor.MoveItExecutor._d405_plan_start_matches_measured(
        node, trajectory, _joint_state([0.0] * 6)
    )

    assert not ok
    assert reason.startswith("MEASURED_SEED_MISMATCH")


def _probe_executor(monkeypatch):
    now = {"value": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    logger = _Logger()
    client = _QueuedServiceClient()
    dispatches = []
    aborts = []
    finalized = []
    token = object()
    seed = _joint_state([0.0] * 6)
    node = SimpleNamespace(
        cartesian_client=client,
        current_joint_state=seed,
        current_joint_state_time=now["value"],
        _d405_prescan_token=token,
        _d405_prescan_active=True,
        _d405_cartesian_context=None,
        _motion_abort_requested=False,
        _scene_revision=5,
        _scene_confirmed_revision=5,
        scene_confirmed=True,
        _active_trajectory_goal_token=None,
        _fjt_motion_state_unknown=False,
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        _current_tcp_pose_np=lambda: (
            np.zeros(3, dtype=float),
            np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
        ),
        _joint_state_within_limits=lambda *_args: True,
        _d405_prescan_trajectory_is_safe=lambda *_args: True,
        _d405_plan_start_matches_measured=lambda *_args: (True, ""),
        _rescale_trajectory=lambda trajectory, **_kwargs: trajectory,
        _start_d405_post_fjt_verification=lambda *_args: None,
        execute_trajectory_direct=lambda trajectory, **kwargs: (
            dispatches.append((trajectory, kwargs)) or True
        ),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
    )
    _bind(
        node,
        "_d405_prescan_callback_valid",
        "_d405_scene_revision_confirmed",
        "_finish_d405_probe_context",
        "_plan_d405_probe_cartesian",
        "_d405_probe_cartesian_done",
    )
    node._d405_probe_pose_for_scale = (
        moveit_executor.MoveItExecutor._d405_probe_pose_for_scale
    )
    return node, client, dispatches, aborts, finalized, now


def test_cartesian_probe_discards_every_partial_path_then_uses_shorter_target(
    monkeypatch,
):
    node, client, dispatches, aborts, finalized, _now = _probe_executor(
        monkeypatch
    )
    quaternion = (0.1, 0.2, 0.3, 0.9)
    original = _pose(0.060, 0.0, 0.0, quaternion)
    partial_a = _trajectory([[0.0] * 6, [0.02] * 6])
    partial_b = _trajectory([[0.0] * 6, [0.03] * 6])
    complete = _trajectory([[0.0] * 6, [0.04] * 6])

    node._plan_d405_probe_cartesian(
        "D405_PRESCAN_2",
        original,
        finalized.append,
        token=node._d405_prescan_token,
    )
    assert len(client.requests) == 1
    frozen_seed = copy.deepcopy(client.requests[0].start_state.joint_state)
    node.current_joint_state.position[0] = 0.5

    client.futures[0].resolve(_cartesian_response(0.947, partial_a))
    assert dispatches == []
    assert len(client.requests) == 2
    client.futures[1].resolve(_cartesian_response(0.999, partial_b))
    assert dispatches == []
    assert len(client.requests) == 3
    client.futures[2].resolve(_cartesian_response(1.0, complete))

    targets = [request.waypoints[0] for request in client.requests]
    assert [target.position.x for target in targets] == pytest.approx(
        [0.060, 0.045, 0.030]
    )
    for target, request in zip(targets, client.requests):
        assert (
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ) == pytest.approx(quaternion)
        assert request.avoid_collisions is True
        assert request.max_velocity_scaling_factor == pytest.approx(0.0175)
        assert request.max_acceleration_scaling_factor == pytest.approx(0.0175)
        assert request.start_state.joint_state.position == frozen_seed.position
    assert [entry[0] for entry in dispatches] == [complete]
    assert partial_a not in [entry[0] for entry in dispatches]
    assert partial_b not in [entry[0] for entry in dispatches]
    assert aborts == []
    assert finalized == []


@pytest.mark.parametrize(
    ("fraction", "error_code"),
    ((float("nan"), 1), (1.0, -1)),
)
def test_cartesian_probe_invalid_or_low_result_never_dispatches(
    monkeypatch, fraction, error_code
):
    node, client, dispatches, aborts, finalized, _now = _probe_executor(
        monkeypatch
    )
    partial = _trajectory([[0.0] * 6, [0.02] * 6])
    node._plan_d405_probe_cartesian(
        "D405_PRESCAN_2",
        _pose(0.060),
        finalized.append,
        token=node._d405_prescan_token,
    )

    client.futures[0].resolve(
        _cartesian_response(fraction, partial, error_code=error_code)
    )

    assert dispatches == []
    assert aborts == []
    assert finalized == [False]
    assert len(client.requests) == 1


def test_cartesian_probe_any_finite_partial_uses_bounded_shorter_retries(
    monkeypatch,
):
    node, client, dispatches, aborts, finalized, _now = _probe_executor(
        monkeypatch
    )
    partial = _trajectory([[0.0] * 6, [0.02] * 6])
    node._plan_d405_probe_cartesian(
        "D405_PRESCAN_2",
        _pose(0.060),
        finalized.append,
        token=node._d405_prescan_token,
    )

    for request_index in range(3):
        client.futures[request_index].resolve(
            _cartesian_response(0.0, partial, error_code=1)
        )

    assert dispatches == []
    assert aborts == []
    assert finalized == [False]
    assert len(client.requests) == 3


def test_fjt_success_with_frozen_measured_joints_never_arms_capture(
    monkeypatch,
):
    now = {"value": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    token = object()
    aborts = []
    finalized = []
    destroyed = []
    trajectory = _trajectory([[0.0] * 6, [0.10] * 6])
    context = {
        "token": token,
        "scene_revision": 3,
        "finalize_cb": finalized.append,
    }
    node = SimpleNamespace(
        _d405_prescan_active=True,
        _d405_prescan_token=token,
        _motion_abort_requested=False,
        _d405_orientation_context=context,
        _d405_cartesian_context=None,
        _d405_arrival_timer=None,
        _scene_revision=3,
        _scene_confirmed_revision=3,
        scene_confirmed=True,
        current_joint_state=_joint_state([0.0] * 6),
        current_joint_state_time=now["value"],
        _robot_stationary_for_bias=lambda: True,
        get_logger=lambda: _Logger(),
        create_timer=lambda _period, callback: _FakeTimer(callback),
        destroy_timer=lambda timer: destroyed.append(timer),
        _request_motion_abort=lambda reason: aborts.append(str(reason)),
    )
    _bind(
        node,
        "_cancel_d405_arrival_timer",
        "_d405_prescan_callback_valid",
        "_d405_scene_revision_confirmed",
        "_d405_post_fjt_arrival_reason",
        "_start_d405_post_fjt_verification",
    )

    node._start_d405_post_fjt_verification(context, trajectory)
    timer = node._d405_arrival_timer
    assert finalized == []
    assert aborts == []

    now["value"] = 13.0
    node.current_joint_state_time = now["value"]
    timer.callback()

    assert finalized == []
    assert len(aborts) == 1
    assert aborts[0].startswith("D405_PRESCAN_ARRIVAL_UNVERIFIED")


def _surface_message(point, quaternion=(0.0, 0.0, 0.0, 1.0)):
    message = PoseStamped()
    message.header.frame_id = moveit_executor.BASE_FRAME
    message.pose = _pose(*point, quaternion=quaternion)
    return message


def _surface_executor():
    logger = _Logger()
    node = SimpleNamespace(
        dynamic_surface_point=None,
        dynamic_surface_normal=None,
        dynamic_surface_source="fallback",
        dynamic_surface_source_time=0.0,
        _pending_surface_msg=None,
        _pending_surface_source="zed",
        _scene_revision=0,
        scene_confirmed=True,
        scene_initialized=True,
        get_logger=lambda: logger,
        _canonical_world_frame=lambda frame: frame,
        _lookup_transform_to_base=lambda *_args, **_kwargs: None,
    )
    return node, logger


def test_identical_plane_replay_preserves_time_revision_and_confirmation(
    monkeypatch,
):
    now = {"value": 1.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    node, _logger = _surface_executor()

    assert moveit_executor.MoveItExecutor._set_active_surface(
        node, _surface_message((0.3, 0.2, 0.8)), "zed"
    )
    accepted_point = node.dynamic_surface_point.copy()
    accepted_normal = node.dynamic_surface_normal.copy()
    node.scene_confirmed = True
    node.scene_initialized = True
    now["value"] = 2.0
    node._pending_surface_msg = object()

    # Tangential point shift and q -> -q still describe the same signed plane.
    changed = moveit_executor.MoveItExecutor._set_active_surface(
        node,
        _surface_message((-0.5, 0.7, 0.8), quaternion=(0.0, 0.0, 0.0, -1.0)),
        "zed",
    )

    assert changed is False
    assert node.dynamic_surface_source_time == pytest.approx(1.0)
    assert node._scene_revision == 1
    assert node.scene_confirmed is True
    assert node.scene_initialized is True
    assert node._pending_surface_msg is None
    np.testing.assert_array_equal(node.dynamic_surface_point, accepted_point)
    np.testing.assert_array_equal(node.dynamic_surface_normal, accepted_normal)


def test_signed_surface_normal_flip_is_a_real_scene_change(monkeypatch):
    now = {"value": 1.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    node, _logger = _surface_executor()
    moveit_executor.MoveItExecutor._set_active_surface(
        node, _surface_message((0.3, 0.2, 0.8)), "zed"
    )
    node.scene_confirmed = True
    node.scene_initialized = True
    now["value"] = 2.0

    assert moveit_executor.MoveItExecutor._set_active_surface(
        node,
        _surface_message((0.3, 0.2, 0.8), quaternion=(1.0, 0.0, 0.0, 0.0)),
        "zed",
    )
    assert node._scene_revision == 2
    assert node.dynamic_surface_source_time == pytest.approx(2.0)
    np.testing.assert_allclose(node.dynamic_surface_normal, [0.0, 0.0, -1.0])
    assert node.scene_confirmed is False
    assert node.scene_initialized is False


def test_surface_offset_and_source_handoff_are_real_scene_changes(monkeypatch):
    now = {"value": 1.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: now["value"]
    )
    node, _logger = _surface_executor()
    moveit_executor.MoveItExecutor._set_active_surface(
        node, _surface_message((0.3, 0.2, 0.8)), "zed"
    )

    now["value"] = 2.0
    assert moveit_executor.MoveItExecutor._set_active_surface(
        node, _surface_message((0.3, 0.2, 0.802)), "zed"
    )
    assert node._scene_revision == 2
    assert node.dynamic_surface_source_time == pytest.approx(2.0)

    node.scene_confirmed = True
    node.scene_initialized = True
    now["value"] = 3.0
    assert moveit_executor.MoveItExecutor._set_active_surface(
        node, _surface_message((0.3, 0.2, 0.802)), "d405_refined"
    )
    assert node.dynamic_surface_source == "d405_refined"
    assert node.dynamic_surface_source_time == pytest.approx(3.0)
    assert node._scene_revision == 3
    assert node.scene_confirmed is False
    assert node.scene_initialized is False
