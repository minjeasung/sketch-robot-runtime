import copy
from types import MethodType, SimpleNamespace

import pytest
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    PlanningScene,
    PlanningSceneComponents,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene


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
    def __init__(self, ready=True):
        self.ready = bool(ready)
        self.requests = []
        self.futures = []
        self.wait_calls = []

    def service_is_ready(self):
        return self.ready

    def wait_for_service(self, timeout_sec):
        self.wait_calls.append(float(timeout_sec))
        return self.ready

    def call_async(self, request):
        future = _ControlledFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


class _FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False
        self.destroyed = False

    def cancel(self):
        self.cancelled = True


def _full_acm(required_pairs, *, target_pair=None, target_allowed=False):
    names = []
    for first, second in required_pairs:
        for name in (first, second):
            if name not in names:
                names.append(name)
    if target_pair is not None:
        for name in target_pair:
            if name not in names:
                names.append(name)
    matrix = AllowedCollisionMatrix()
    matrix.entry_names = list(names)
    matrix.entry_values = []
    for _ in names:
        row = AllowedCollisionEntry()
        row.enabled = [False] * len(names)
        matrix.entry_values.append(row)
    for first, second in required_pairs:
        i = names.index(first)
        j = names.index(second)
        matrix.entry_values[i].enabled[j] = True
        matrix.entry_values[j].enabled[i] = True
    if target_pair is not None:
        i = names.index(target_pair[0])
        j = names.index(target_pair[1])
        matrix.entry_values[i].enabled[j] = bool(target_allowed)
        matrix.entry_values[j].enabled[i] = bool(target_allowed)
    matrix.default_entry_names = ["octomap"]
    matrix.default_entry_values = [False]
    return matrix


def _get_response(matrix):
    response = GetPlanningScene.Response()
    response.scene.allowed_collision_matrix = copy.deepcopy(matrix)
    return response


def _apply_response(success=True):
    response = ApplyPlanningScene.Response()
    response.success = bool(success)
    return response


def _reordered_acm(matrix):
    names = list(reversed(matrix.entry_names))
    old_index = {
        name: index for index, name in enumerate(matrix.entry_names)
    }
    reordered = AllowedCollisionMatrix()
    reordered.entry_names = names
    for first in names:
        row = AllowedCollisionEntry()
        row.enabled = [
            bool(
                matrix.entry_values[old_index[first]].enabled[
                    old_index[second]
                ]
            )
            for second in names
        ]
        reordered.entry_values.append(row)
    reordered.default_entry_names = list(
        reversed(matrix.default_entry_names)
    )
    reordered.default_entry_values = list(
        reversed(matrix.default_entry_values)
    )
    return reordered


_ACM_METHODS = (
    "_notify_acm_callbacks",
    "_cancel_acm_health_query_timer",
    "_invalidate_acm_health_query",
    "_request_acm_baseline_health_check",
    "_acm_health_query_done",
    "_cancel_acm_phase_timeout",
    "_arm_acm_phase_timeout",
    "_clear_acm_transaction",
    "_mark_acm_state_unknown",
    "_acm_services_ready",
    "_acm_context_scene_is_current",
    "_request_acm_readback",
    "_acm_readback_done",
    "_apply_acm_matrix",
    "_acm_apply_done",
    "_start_acm_restore",
    "_acm_baseline_done",
    "_set_contact_collision_allowed",
)


def _executor():
    logger = _Logger()
    required_pairs = (
        ("link0", "link1"),
        ("link3", "link4"),
        ("aft200_link", "aft200_cable_guard_link"),
        (
            "paint_eoat_no_camera_link",
            moveit_executor.ROLLER_CONTACT_LINK,
        ),
    )
    timers = []
    node = SimpleNamespace(
        apply_scene_client=_QueuedServiceClient(),
        get_planning_scene_client=_QueuedServiceClient(),
        active_target_name="wall",
        scene_confirmed=True,
        _scene_revision=7,
        _scene_confirmed_revision=7,
        _scene_apply_inflight_revision=None,
        _contact_collision_allowed=False,
        _contact_collision_baseline=None,
        _contact_collision_target_name="",
        _acm_update_seq=0,
        _acm_update_pending=None,
        _acm_update_timer=None,
        _acm_state_unknown=False,
        _acm_baseline_verified=False,
        _acm_baseline_verified_target_name="",
        _acm_baseline_verified_time=0.0,
        _acm_health_query_seq=0,
        _acm_health_query_pending=None,
        _acm_health_query_timer=None,
        _latest_allowed_collision_matrix=None,
        _required_acm_allowed_pairs=required_pairs,
        executing=False,
        dry_run=True,
        _execution_state="IDLE",
        _motion_abort_requested=False,
        get_logger=lambda: logger,
        create_timer=lambda _duration, callback: (
            timers.append(_FakeTimer(callback)) or timers[-1]
        ),
        destroy_timer=lambda timer: setattr(timer, "destroyed", True),
        _timers=timers,
    )
    for name in _ACM_METHODS:
        setattr(
            node,
            name,
            MethodType(getattr(moveit_executor.MoveItExecutor, name), node),
        )
    return node, logger, required_pairs


def test_clean_startup_acm_is_verified_before_real_motion_is_unblocked():
    node, _logger, required_pairs = _executor()
    node.real_painting_enabled = True
    node.dry_run = False
    baseline = _full_acm(required_pairs)

    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_BASELINE_NOT_VERIFIED"
    )
    node._request_acm_baseline_health_check()
    assert len(node.get_planning_scene_client.requests) == 1
    assert (
        node.get_planning_scene_client.requests[0].components.components
        == PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert node._acm_baseline_verified is True
    assert node._acm_baseline_verified_target_name == "wall"
    assert node._acm_state_unknown is False
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == ""
    )


@pytest.mark.parametrize("use_default", (False, True))
def test_startup_preexisting_contact_allow_latches_relaunch_before_motion(
    use_default,
):
    node, logger, required_pairs = _executor()
    node.real_painting_enabled = True
    node.dry_run = False
    if use_default:
        baseline = _full_acm(required_pairs)
        baseline.default_entry_names.append("wall")
        baseline.default_entry_values.append(True)
    else:
        baseline = _full_acm(
            required_pairs,
            target_pair=(moveit_executor.ROLLER_CONTACT_LINK, "wall"),
            target_allowed=True,
        )

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert node._acm_baseline_verified is False
    assert node._acm_state_unknown is True
    assert node._contact_collision_allowed is True
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
    )
    assert any("startup baseline unsafe" in message for message in logger.errors)


def test_startup_acm_query_timeout_stays_blocked_and_retries_safely():
    node, _logger, required_pairs = _executor()
    node.real_painting_enabled = True
    node.dry_run = False
    baseline = _full_acm(required_pairs)

    node._request_acm_baseline_health_check()
    old_future = node.get_planning_scene_client.futures[0]
    node._acm_health_query_pending["timer"].callback()

    assert node._acm_health_query_pending is None
    assert node._acm_state_unknown is False
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_BASELINE_NOT_VERIFIED"
    )
    old_future.resolve(_get_response(baseline))
    assert node._acm_baseline_verified is False

    node._request_acm_baseline_health_check()
    assert len(node.get_planning_scene_client.requests) == 2
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(baseline)
    )
    assert node._acm_baseline_verified is True


def test_malformed_startup_acm_latches_relaunch_without_apply():
    node, _logger, required_pairs = _executor()
    node.real_painting_enabled = True
    node.dry_run = False
    malformed = _full_acm(required_pairs)
    malformed.entry_values.pop()

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(malformed)
    )

    assert node.apply_scene_client.requests == []
    assert node._acm_state_unknown is True
    assert node._contact_collision_allowed is True
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
    )


@pytest.mark.parametrize("contamination", ("extra_pair", "default_true"))
def test_startup_rejects_every_non_srdf_collision_allowance(contamination):
    node, logger, required_pairs = _executor()
    node.real_painting_enabled = True
    node.dry_run = False
    if contamination == "extra_pair":
        baseline = _full_acm(
            required_pairs,
            target_pair=("link0", "wall"),
            target_allowed=True,
        )
    else:
        baseline = _full_acm(required_pairs)
        baseline.default_entry_values[0] = True

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert node._acm_baseline_verified is False
    assert node._acm_state_unknown is True
    assert node._contact_collision_allowed is True
    assert node.apply_scene_client.requests == []
    assert any("startup baseline unsafe" in message for message in logger.errors)


@pytest.mark.parametrize("contamination", ("extra_pair", "default_true"))
def test_contact_transaction_rejects_non_srdf_fetched_baseline(contamination):
    node, _logger, required_pairs = _executor()
    if contamination == "extra_pair":
        baseline = _full_acm(
            required_pairs,
            target_pair=("link0", "wall"),
            target_allowed=True,
        )
    else:
        baseline = _full_acm(required_pairs)
        baseline.default_entry_values[0] = True
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert results == [False]
    assert node.apply_scene_client.requests == []
    assert node._acm_state_unknown is True
    assert node._contact_collision_allowed is True


def test_runtime_srdf_baseline_loader_matches_current_semantic_config():
    pairs = moveit_executor.MoveItExecutor._load_srdf_allowed_collision_pairs()

    assert len(pairs) == 23
    assert ("link3", "link4") in pairs
    assert (
        "paint_eoat_no_camera_link",
        moveit_executor.ROLLER_CONTACT_LINK,
    ) in pairs


def test_full_acm_transaction_preserves_srdf_and_restores_exact_baseline():
    node, _logger, required_pairs = _executor()
    target_pair = (moveit_executor.ROLLER_CONTACT_LINK, "wall")
    baseline = _full_acm(
        required_pairs,
    )
    allow_results = []
    restore_results = []

    node._set_contact_collision_allowed(True, allow_results.append)
    assert len(node.get_planning_scene_client.requests) == 1
    assert (
        node.get_planning_scene_client.requests[0].components.components
        == PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    assert node.apply_scene_client.requests == []
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert len(node.apply_scene_client.requests) == 1
    allow_request = node.apply_scene_client.requests[0]
    assert allow_request.scene.is_diff is True
    assert allow_request.scene.robot_state.is_diff is True
    allowed_matrix = allow_request.scene.allowed_collision_matrix
    assert moveit_executor.MoveItExecutor._acm_pair_value(
        allowed_matrix, *target_pair
    )
    for pair in required_pairs:
        assert moveit_executor.MoveItExecutor._acm_pair_value(
            allowed_matrix, *pair
        )
    assert allowed_matrix.default_entry_names == baseline.default_entry_names
    assert allowed_matrix.default_entry_values == baseline.default_entry_values
    assert allow_results == []

    node.apply_scene_client.futures[0].resolve(_apply_response())
    assert len(node.get_planning_scene_client.requests) == 2
    assert allow_results == []
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(allowed_matrix)
    )
    assert allow_results == [True]
    assert node._contact_collision_allowed is True

    node._set_contact_collision_allowed(False, restore_results.append)
    assert len(node.apply_scene_client.requests) == 2
    restored_request = node.apply_scene_client.requests[1]
    assert moveit_executor.MoveItExecutor._acm_equal(
        restored_request.scene.allowed_collision_matrix, baseline
    )
    node.apply_scene_client.futures[1].resolve(_apply_response())
    assert len(node.get_planning_scene_client.requests) == 3
    node.get_planning_scene_client.futures[2].resolve(
        _get_response(baseline)
    )

    assert restore_results == [True]
    assert node._contact_collision_allowed is False
    assert node._contact_collision_baseline is None
    assert node._contact_collision_target_name == ""
    assert moveit_executor.MoveItExecutor._acm_equal(
        node._latest_allowed_collision_matrix, baseline
    )


def test_restore_requested_while_allow_apply_pending_compensates_before_callback():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    allow_results = []
    restore_results = []

    node._set_contact_collision_allowed(True, allow_results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    assert len(node.apply_scene_client.requests) == 1

    node._set_contact_collision_allowed(False, restore_results.append)
    node.apply_scene_client.futures[0].resolve(_apply_response())
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(allowed_matrix)
    )

    # The allow completion is never exposed to CONTACT_SEARCH. Instead, the
    # frozen full baseline is immediately applied as a compensating restore.
    assert allow_results == [False]
    assert len(node.apply_scene_client.requests) == 2
    assert moveit_executor.MoveItExecutor._acm_equal(
        node.apply_scene_client.requests[1].scene.allowed_collision_matrix,
        baseline,
    )
    node.apply_scene_client.futures[1].resolve(_apply_response())
    node.get_planning_scene_client.futures[2].resolve(
        _get_response(baseline)
    )

    assert allow_results == [False]
    assert restore_results == [True]
    assert node._contact_collision_allowed is False


def test_acm_readback_verification_accepts_moveit_name_reordering():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(_reordered_acm(allowed_matrix))
    )

    assert results == [True]
    assert node._contact_collision_allowed is True


def test_restore_during_baseline_fetch_cancels_without_any_apply():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    allow_results = []
    restore_results = []

    node._set_contact_collision_allowed(True, allow_results.append)
    old_future = node.get_planning_scene_client.futures[0]
    node._set_contact_collision_allowed(False, restore_results.append)

    assert restore_results == [True]
    assert allow_results == [False]
    assert node.apply_scene_client.requests == []
    old_future.resolve(_get_response(baseline))
    assert node.apply_scene_client.requests == []
    assert node._contact_collision_allowed is False


def test_preexisting_roller_wall_allowance_is_never_adopted_as_baseline():
    node, logger, required_pairs = _executor()
    baseline = _full_acm(
        required_pairs,
        target_pair=(moveit_executor.ROLLER_CONTACT_LINK, "wall"),
        target_allowed=True,
    )
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert results == [False]
    assert node.apply_scene_client.requests == []
    assert node._contact_collision_allowed is True
    assert node._acm_state_unknown is True
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
    )
    assert any("already allowed" in message for message in logger.errors)


def test_default_entry_cannot_hide_preexisting_roller_wall_allowance():
    node, logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    baseline.default_entry_names.append("wall")
    baseline.default_entry_values.append(True)
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )

    assert results == [False]
    assert node.apply_scene_client.requests == []
    assert node._acm_state_unknown is True
    assert any("already allowed" in message for message in logger.errors)


def test_fetch_timeout_fails_known_safe_and_late_response_is_ignored():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    results = []

    node._set_contact_collision_allowed(True, results.append)
    context = node._acm_update_pending
    timer = context["timer"]
    old_future = node.get_planning_scene_client.futures[0]
    timer.callback()

    assert results == [False]
    assert node._acm_update_pending is None
    assert node._acm_state_unknown is False
    assert node._contact_collision_allowed is False
    old_future.resolve(_get_response(baseline))
    assert node.apply_scene_client.requests == []


def test_apply_timeout_latches_unknown_and_blocks_every_new_dispatch():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    context = node._acm_update_pending
    assert context["phase"] == "APPLY_ALLOW"
    context["timer"].callback()

    assert results == [False]
    assert node._acm_state_unknown is True
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
    )


def test_contact_acm_is_allowed_only_for_explicit_contact_dispatch_context():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)

    node._set_contact_collision_allowed(True)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(allowed_matrix)
    )
    node.executing = True
    node._execution_state = "CONTACT_SEARCH"

    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "CONTACT_ACM_ACTIVE"
    )
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(
            node, requires_contact_acm=True
        )
        == ""
    )
    node._execution_state = "SAFETY_APPROACH"
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(
            node, requires_contact_acm=True
        )
        == "CONTACT_ACM_CONTEXT_INVALID"
    )


def test_physical_wrapper_requires_explicit_contact_acm_permission():
    backend_calls = []
    node = SimpleNamespace(
        _hardware_motion_inhibited=False,
        _fjt_motion_state_unknown=False,
        _acm_state_unknown=False,
        _acm_update_pending=None,
        _contact_collision_allowed=True,
        _contact_collision_baseline=AllowedCollisionMatrix(),
        _contact_collision_target_name="wall",
        active_target_name="wall",
        _acm_baseline_verified=True,
        _acm_baseline_verified_target_name="wall",
        _acm_baseline_verified_time=moveit_executor.time.monotonic(),
        _execution_state="CONTACT_SEARCH",
        executing=True,
        _motion_abort_requested=False,
        dry_run=False,
        execution_backend="follow_joint_trajectory",
        get_logger=lambda: _Logger(),
        _execute_trajectory_follow_joint=lambda *args, **kwargs: (
            backend_calls.append((args, kwargs)) or True
        ),
    )
    trajectory = RobotTrajectory()

    assert not moveit_executor.MoveItExecutor.execute_trajectory_direct(
        node, trajectory, label="unprivileged"
    )
    assert backend_calls == []
    assert moveit_executor.MoveItExecutor.execute_trajectory_direct(
        node,
        trajectory,
        label="contact",
        requires_contact_acm=True,
    )
    assert len(backend_calls) == 1


def test_any_physical_wrapper_sends_zero_backend_calls_before_acm_preflight():
    backend_calls = []
    node = SimpleNamespace(
        real_painting_enabled=False,
        active_target_name="wall",
        _acm_baseline_verified=False,
        _acm_baseline_verified_target_name="",
        _acm_baseline_verified_time=0.0,
        _hardware_motion_inhibited=False,
        _fjt_motion_state_unknown=False,
        _acm_state_unknown=False,
        _acm_update_pending=None,
        _contact_collision_allowed=False,
        _motion_abort_requested=False,
        dry_run=False,
        execution_backend="follow_joint_trajectory",
        get_logger=lambda: _Logger(),
        _execute_trajectory_follow_joint=lambda *args, **kwargs: (
            backend_calls.append((args, kwargs)) or True
        ),
    )

    assert not moveit_executor.MoveItExecutor.execute_trajectory_direct(
        node, RobotTrajectory(), label="preflight_not_ready"
    )
    assert backend_calls == []


def test_acm_health_age_triggers_refresh_without_faking_collision_fault(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    node, _logger, required_pairs = _executor()
    node.real_painting_enabled = False
    node.dry_run = False
    baseline = _full_acm(required_pairs)

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == ""
    )

    clock["now"] = 10.6
    node._request_acm_baseline_health_check()
    assert len(node.get_planning_scene_client.requests) == 2
    clock["now"] = 11.6
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == ""
    )
    assert not moveit_executor.MoveItExecutor._acm_baseline_is_fresh(node)
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(baseline)
    )
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == ""
    )
    assert moveit_executor.MoveItExecutor._acm_baseline_is_fresh(node)


def test_periodic_health_detects_contamination_and_aborts_active_motion(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    node, _logger, required_pairs = _executor()
    node.dry_run = False
    baseline = _full_acm(required_pairs)
    aborts = []
    node._request_motion_abort = lambda reason: aborts.append(str(reason))

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    clock["now"] = 10.6
    node.executing = True
    contaminated = _full_acm(
        required_pairs,
        target_pair=("link0", "wall"),
        target_allowed=True,
    )
    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(contaminated)
    )

    assert aborts == ["ACM_BASELINE_UNSAFE_DURING_MOTION"]
    assert node._acm_state_unknown is True
    assert node._contact_collision_allowed is True


def test_stale_health_age_does_not_block_fresh_contact_transaction(
    monkeypatch,
):
    clock = {"now": 10.0}
    monkeypatch.setattr(
        moveit_executor.time, "monotonic", lambda: clock["now"]
    )
    node, _logger, required_pairs = _executor()
    node.dry_run = False
    baseline = _full_acm(required_pairs)

    node._request_acm_baseline_health_check()
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    clock["now"] = 10.6
    node._request_acm_baseline_health_check()
    old_health_future = node.get_planning_scene_client.futures[1]
    assert node._acm_health_query_pending is not None
    # Reproduce the live single-threaded callback delay: the last verified
    # clean snapshot is 1.522 s old, just beyond the 1.5 s telemetry lease.
    # CONTACT_SEARCH owns a new authoritative full-ACM GET, so age alone must
    # not reject the run before that transaction can execute.
    clock["now"] = 11.522
    assert not moveit_executor.MoveItExecutor._acm_baseline_is_fresh(node)

    allow_results = []
    restore_results = []
    node._set_contact_collision_allowed(True, allow_results.append)
    assert node._acm_health_query_pending is None
    assert allow_results == []
    transaction_baseline_future = node.get_planning_scene_client.futures[2]
    transaction_baseline_future.resolve(_get_response(baseline))
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[3].resolve(
        _get_response(allowed_matrix)
    )
    assert allow_results == [True]

    node._set_contact_collision_allowed(False, restore_results.append)
    node.apply_scene_client.futures[1].resolve(_apply_response())
    node.get_planning_scene_client.futures[4].resolve(
        _get_response(baseline)
    )
    assert restore_results == [True]
    assert node._contact_collision_allowed is False

    # The older health request can represent an out-of-order contact-allowed
    # snapshot. Its invalidated context must not contaminate the restored ACM.
    old_health_future.resolve(_get_response(allowed_matrix))
    assert node._acm_state_unknown is False
    assert node._contact_collision_allowed is False
    assert node._acm_baseline_verified is True


def test_restore_pending_blocks_reset_and_new_run_until_verified():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)

    node._set_contact_collision_allowed(True)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(allowed_matrix)
    )
    node._set_contact_collision_allowed(False)

    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_TRANSACTION_PENDING"
    )
    node.apply_scene_client.futures[1].resolve(_apply_response())
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_TRANSACTION_PENDING"
    )
    node.get_planning_scene_client.futures[2].resolve(
        _get_response(baseline)
    )
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == ""
    )


@pytest.mark.parametrize(
    "mutator, expected_reason",
    (
        (lambda matrix: matrix.entry_names.clear(), "entry_names empty"),
        (
            lambda matrix: matrix.entry_values.pop(),
            "row count",
        ),
        (
            lambda matrix: matrix.entry_values[0].enabled.pop(),
            "row 0 width",
        ),
        (
            lambda matrix: setattr(
                matrix.entry_values[0], "enabled",
                [False] + list(matrix.entry_values[0].enabled[1:]),
            ),
            "asymmetric",
        ),
        (
            lambda matrix: matrix.entry_names.__setitem__(
                1, matrix.entry_names[0]
            ),
            "duplicates",
        ),
        (
            lambda matrix: matrix.default_entry_values.clear(),
            "default entry names/values",
        ),
    ),
)
def test_invalid_or_partial_acm_is_rejected(mutator, expected_reason):
    _node, _logger, required_pairs = _executor()
    matrix = _full_acm(required_pairs)
    if expected_reason == "asymmetric":
        matrix.entry_values[0].enabled[1] = not bool(
            matrix.entry_values[1].enabled[0]
        )
    else:
        mutator(matrix)
    error = moveit_executor.MoveItExecutor._acm_validation_error(
        matrix, required_pairs
    )
    assert expected_reason in error


def test_missing_required_srdf_pair_never_reaches_apply_service():
    node, logger, required_pairs = _executor()
    incomplete = _full_acm(required_pairs)
    first, second = required_pairs[-1]
    i = incomplete.entry_names.index(first)
    j = incomplete.entry_names.index(second)
    incomplete.entry_values[i].enabled[j] = False
    incomplete.entry_values[j].enabled[i] = False
    results = []

    node._set_contact_collision_allowed(True, results.append)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(incomplete)
    )

    assert results == [False]
    assert node.apply_scene_client.requests == []
    assert node._contact_collision_allowed is True
    assert node._acm_state_unknown is True
    assert any("required SRDF pair" in message for message in logger.errors)


def test_empty_or_partial_monitored_diff_cannot_replace_full_acm_cache():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)
    node._latest_allowed_collision_matrix = copy.deepcopy(baseline)
    node._enabled_ids = set()
    node._monitor_logged = True

    empty = PlanningScene()
    moveit_executor.MoveItExecutor.on_scene_update(node, empty)
    assert moveit_executor.MoveItExecutor._acm_equal(
        node._latest_allowed_collision_matrix, baseline
    )

    partial = PlanningScene()
    partial.allowed_collision_matrix = _full_acm(
        ((moveit_executor.ROLLER_CONTACT_LINK, "wall"),)
    )
    moveit_executor.MoveItExecutor.on_scene_update(node, partial)
    assert moveit_executor.MoveItExecutor._acm_equal(
        node._latest_allowed_collision_matrix, baseline
    )


def test_restore_verification_failure_latches_acm_unknown_dispatch_blocker():
    node, _logger, required_pairs = _executor()
    baseline = _full_acm(required_pairs)

    node._set_contact_collision_allowed(True)
    node.get_planning_scene_client.futures[0].resolve(
        _get_response(baseline)
    )
    allowed_matrix = node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[1].resolve(
        _get_response(allowed_matrix)
    )
    assert node._contact_collision_allowed is True

    restore_results = []
    node._set_contact_collision_allowed(False, restore_results.append)
    node.apply_scene_client.futures[1].resolve(_apply_response())
    corrupted = copy.deepcopy(baseline)
    corrupted.entry_values[0].enabled[1] = not bool(
        corrupted.entry_values[1].enabled[0]
    )
    node.get_planning_scene_client.futures[2].resolve(
        _get_response(corrupted)
    )

    assert restore_results == [False]
    assert node._acm_state_unknown is True
    assert node._contact_collision_baseline is not None
    assert (
        moveit_executor.MoveItExecutor._motion_dispatch_inhibited_reason(node)
        == "ACM_STATE_UNKNOWN_RELAUNCH_REQUIRED"
    )
