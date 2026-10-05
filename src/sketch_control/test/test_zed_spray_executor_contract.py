"""Exercise live executor methods without requiring a ROS installation.

ROS hosts import the production module normally. Elsewhere, the fallback
compiles unchanged methods/constants from its AST into a private namespace;
only ROS messages, services, clocks and terminal notifications are doubled.
No doubles are installed in sys.modules or shared with other test files.
"""

import ast
import builtins
import copy
import importlib.util
import json
import math
import time
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from rbpodo_painting_control.segment_path import (
    attach_plan_hash,
    parse_segment_path,
    validate_segment_path_for_real_execution,
)
from rbpodo_painting_control.spray_path import rotation_from_spray_path
from rbpodo_painting_control.spray_eoat import load_spray_eoat_profile
from sketch_control.rotation_utils import quat_apply, quat_to_matrix
from sketch_control.d405_view_geometry import support_coordinates
from sketch_control.zed_spray_projection import validate_target_lock


_METHODS = (
    "_spray_eoat_load", "_spray_eoat_blocker", "_spray_endpoint_to_tcp",
    "_spray_eoat_reject", "_defer_candidate_invalidation",
    "_execution_snapshot_updates_locked", "_execution_surface_geometry",
    "on_eoat_segments", "_segment_tip_pose", "_validate_segment_path_geometry",
    "_build_segment_orientation_candidate", "_apply_segment_orientation_candidate",
    "_current_roller_axis_seed", "_prepare_segment_process", "_active_surface_plane",
    "_request_stage1_nearest_ik", "_request_stage1_orientation_candidate_iks",
    "_stage1_orientation_candidate_ik_done", "_finish_stage1_orientation_candidate_iks",
    "_stage1_candidate_joint_metrics", "_activate_stage1_orientation_rank",
    "_nearest_joint_equivalent", "_cancel_stage1_ik_candidate_timer",
    "_invalidate_stage1_orientation_candidates", "_send_stage1_plan",
    "_stage1_ik_done", "_stage1_joint_delta_summary", "_log_stage1_joint_delta",
    "_make_joint_goal_constraints",
    "_maybe_begin_d405_prescan", "_begin_d405_prescan",
    "_reset_d405_refined_lock", "_mark_scene_dirty",
    "_flip_pose_about_tcp_y", "_flip_segment_orientation_candidate",
    "_local_axis_in_world", "_brush_tip_to_tcp",
)
_CONSTANTS = {
    "PLANNING_GROUP", "EE_LINK", "BASE_FRAME", "ROLLER_RADIUS", "JOINT_LIMITS",
    "JOINT_LIMIT_MARGIN", "READY_POSE_JOINTS", "STAGE1_IK_TIMEOUT_S",
    "STAGE1_DUAL_IK_RESPONSE_TIMEOUT_S", "STAGE1_JOINT_GOAL_TOL",
    "STAGE1_LARGE_JOINT_DELTA_WARN_RAD", "D405_PREFLIGHT_SCAN_ENABLED",
    "D405_PREFLIGHT_SCAN_STANDOFF", "PLANNER_ID", "ALLOWED_PLANNING_TIME",
    "PLANNING_ATTEMPTS", "STAGE1_SPEED_SCALE",
    "TOOL_AXIS", "EOAT_TIP_OFFSET",
}


def _header():
    return NS(frame_id="", stamp=NS(sec=0, nanosec=0))


def _pose():
    return NS(position=NS(x=0., y=0., z=0.),
              orientation=NS(x=0., y=0., z=0., w=1.))


def _joint_state():
    return NS(header=_header(), name=[], position=[], velocity=[], effort=[])


def _robot_state():
    return NS(joint_state=_joint_state(), is_diff=False)


class _IK:
    @staticmethod
    def Request():
        return NS(ik_request=NS(
            group_name="", robot_state=_robot_state(), avoid_collisions=False,
            ik_link_name="", pose_stamped=NS(header=_header(), pose=_pose()),
            timeout=NS(sec=0, nanosec=0)))

    @staticmethod
    def Response():
        return NS(error_code=NS(val=0), solution=_robot_state())


class _MoveGroup:
    @staticmethod
    def Goal():
        return NS(request=NS(), planning_options=NS(
            plan_only=False, planning_scene_diff=NS(is_diff=False)))


class _Duration:
    def __init__(self, *, seconds):
        self.seconds = seconds

    def to_msg(self):
        return NS(sec=int(self.seconds),
                  nanosec=int((self.seconds % 1) * 1_000_000_000))


def _load_executor():
    if importlib.util.find_spec("rclpy") is not None:
        from sketch_control import moveit_executor
        return moveit_executor

    source = Path(__file__).parents[1] / "sketch_control" / "moveit_executor.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    production_class = next(n for n in tree.body
                            if isinstance(n, ast.ClassDef) and n.name == "MoveItExecutor")
    namespace = dict(
        copy=copy, replace=replace, json=json, math=math, time=time, np=np,
        Pose=_pose, JointState=_joint_state, RobotState=_robot_state,
        GetPositionIK=_IK, MoveGroup=_MoveGroup, Duration=_Duration,
        Constraints=lambda: NS(joint_constraints=[]), JointConstraint=NS,
        String=NS,
    )
    pure_modules = {
        "rbpodo_painting_control.spray_eoat",
        "rbpodo_painting_control.segment_path", "rbpodo_painting_control.spray_path",
        "sketch_control.rotation_utils", "sketch_control.work_area_geometry",
    }
    body = [ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
    for item in tree.body:
        if isinstance(item, ast.ImportFrom) and item.module in pure_modules:
            body.append(item)
        elif (isinstance(item, ast.Assign) and len(item.targets) == 1
              and isinstance(item.targets[0], ast.Name)
              and item.targets[0].id in _CONSTANTS):
            body.append(item)
    target_tree = ast.parse(source.with_name("targets.py").read_text(encoding="utf-8"))
    body.append(next(n for n in target_tree.body
                     if isinstance(n, ast.FunctionDef) and n.name == "get_target"))
    methods = [n for n in production_class.body
               if isinstance(n, ast.FunctionDef) and n.name in _METHODS]
    assert {n.name for n in methods} == set(_METHODS), "production harness methods changed"
    body.append(ast.ClassDef(name="MoveItExecutor", bases=[], keywords=[],
                             body=methods, decorator_list=[]))
    module = ast.fix_missing_locations(ast.Module(body=body, type_ignores=[]))
    exec(compile(module, str(source), "exec"), namespace)
    return NS(**namespace)


executor_module = _load_executor()


_MULTI_METHODS = (
    "_multi_busy", "_multi_status", "_multi_on_catalog", "_multi_on_select",
    "_multi_pose", "_multi_on_activate", "_multi_activate", "_multi_lock_zed_plane",
)
_ZED_LOCK_METHODS = ("_invalidate_zed_work_area", "_invalidate_zed_target")


def _load_multi_surface():
    if importlib.util.find_spec("rclpy") is not None:
        from geometry_msgs.msg import PoseStamped
        from sketch_control.multi_surface_execution import MultiSurfaceMixin
        from sketch_control.zed_spray_execution import ZedSprayExecutionMixin
        return NS(MultiSurfaceMixin=MultiSurfaceMixin,
                  ZedSprayExecutionMixin=ZedSprayExecutionMixin, PoseStamped=PoseStamped)

    # The invalidation method imports a message inside its body. Resolve only
    # that transport import locally, preserving its AST and global sys.modules.
    def transport_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "std_msgs.msg" and fromlist == ("String",) and level == 0:
            return NS(String=executor_module.String)
        return builtins.__import__(name, globals, locals, fromlist, level)

    namespace = dict(copy=copy, json=json, np=np, support_coordinates=support_coordinates,
                     String=executor_module.String,
                     PoseStamped=lambda: NS(header=_header(), pose=_pose()),
                     __builtins__=dict(vars(builtins), __import__=transport_import))
    directory = Path(__file__).parents[1] / "sketch_control"
    for filename, class_name, names in (
        ("multi_surface_execution.py", "MultiSurfaceMixin", _MULTI_METHODS),
        ("zed_spray_execution.py", "ZedSprayExecutionMixin", _ZED_LOCK_METHODS),
    ):
        source = directory / filename
        tree = ast.parse(source.read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        assert {n.name for n in methods} == set(names)
        module = ast.fix_missing_locations(ast.Module(body=[
            ast.ClassDef(name=class_name, bases=[], keywords=[], body=methods,
                         decorator_list=[])], type_ignores=[]))
        exec(compile(module, str(source), "exec"), namespace)
    source = directory / "target_selector_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == "_normal_to_quaternion")
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
    return NS(**namespace)


multi_module = _load_multi_surface()


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message, **_kwargs):
        self.errors.append(str(message))

    def info(self, *_args, **_kwargs):
        pass

    warn = info


class _Timer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _IKClient:
    def __init__(self):
        self.requests = []
        self.futures = []
        self.immediate_response = None
        self.raise_on_request = False

    def wait_for_service(self, **_kwargs):
        return True

    def call_async(self, request):
        self.requests.append(request)
        if self.raise_on_request:
            raise RuntimeError("IK transport disconnected")
        future = Future()
        self.futures.append(future)
        if self.immediate_response is not None:
            future.set_result(self.immediate_response)
        return future


class _ActionClient:
    def __init__(self):
        self.goals = []

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return Future()


def _row(mode, *, y=0., offset=0., force=0., speed=.02):
    return dict(mode=mode, x=0., y=y, z=0., nx=0., ny=0., nz=1.,
                tx=0., ty=1., tz=0., force_n=force, offset_m=offset, speed_mps=speed)


def _spray_payload(model="rb20_1900es", axis="+z", standoff=.65):
    metadata = dict(model_id=model, spray_tool_axis=axis,
                    spray_footprint_width_m=.35, spray_overlap=.30,
                    spray_spacing_m=.245, spray_speed_mps=.02,
                    spray_standoff_m=standoff,
                    **load_spray_eoat_profile(_TEST_PROFILES[model], model, axis).metadata())
    return attach_plan_hash(dict(
        version=3, process_mode="spray", frame_id="link0", path_id="123",
        work_area_id="area-1", plane_generation_id="zed:catalog:plane:1000000000",
        point_semantics="surface_point", contact_geometry_offset_m=0.,
        precontact_clearance_m=standoff, travel_clearance_m=standoff,
        safety_approach_offset_m=standoff, final_retreat_offset_m=standoff,
        tcp_normal_axis=axis, preserve_orientation_continuity=True,
        rows=[_row(mode, y=y, offset=standoff) for mode, y in (
            ("SPRAY_APPROACH", 0.), ("SPRAY", 0.),
            ("SPRAY", .4), ("SPRAY_FINISH", .4))],
        source=dict(plane="zed", view="wall_front", coverage="auto_fill",
                    selection_id="2000000000", work_area_id="area-1",
                    plane_generation_id="zed:catalog:plane:1000000000", **metadata),
        **metadata))


def _paint_payload():
    return attach_plan_hash(dict(
        version=3, process_mode="paint", frame_id="link0", path_id="123",
        work_area_id="area-1", plane_generation_id="d405:plane:1",
        point_semantics="surface_point", contact_geometry_offset_m=.026,
        precontact_clearance_m=.01, travel_clearance_m=.01,
        safety_approach_offset_m=.08, final_retreat_offset_m=.08,
        tcp_normal_axis="+y", preserve_orientation_continuity=True, source={},
        rows=[_row("APPROACH_PRECONTACT", offset=.01),
              _row("CONTACT_SEARCH", offset=.01, speed=.002),
              _row("RAMP_UP", force=2., speed=0.),
              _row("PAINT", force=2.), _row("RAMP_DOWN", speed=0.),
              _row("FINAL_RETRACT", offset=.08)]))


def _parse(payload):
    path = parse_segment_path(payload, default_contact_offset_m=.026,
                              max_force_n=20., minimum_clearance_m=.005,
                              allow_legacy=False)
    return validate_segment_path_for_real_execution(path)


_TEST_PROFILES = {}


@pytest.fixture(autouse=True)
def _verified_tools(tmp_path):
    from test_zed_spray_generation import write_profile
    for model, axis in (("rb20_1900es", "+z"), ("rb10_1300e_u", "-y")):
        directory = tmp_path / model
        directory.mkdir()
        _TEST_PROFILES[model] = str(write_profile(directory, model=model, axis=axis))
    yield
    _TEST_PROFILES.clear()


def _executor(mode="spray", model="rb20_1900es", axis="+z"):
    logger = _Logger()
    node = NS(
        model_id=model, spray_tool_axis=axis, spray_eoat_profile=_TEST_PROFILES[model],
        process_mode=mode, executing=False, real_painting_enabled=True, dry_run=False,
        painting_force_enabled=False, segment_contact_offset_m=.026,
        contact_geometry_offset_m=.026, minimum_travel_clearance_m=.005,
        precontact_clearance_m=.01, travel_clearance_m=.01,
        safety_approach_offset_m=.08, final_retreat_offset_m=.08,
        max_paint_force_n=20., _segment_path=None, _accepted_plan_hash="",
        _accepted_plan_path_id="", _execution_snapshot=None, _motion_abort_requested=False,
        dynamic_surface_point=np.array([0., 0., 0.]),
        dynamic_surface_normal=np.array([0., 0., 1.]),
        dynamic_work_area_corners=np.array([[-.2, -.2, 0.], [.2, -.2, 0.],
                                           [.2, .6, 0.], [-.2, .6, 0.]]),
        cfg={"objects": [{"name": "wall"}]}, active_target_name="wall",
        ik_client=_IKClient(), move_action_client=_ActionClient(),
        current_joint_state=executor_module.JointState(), _stage1_ik_seed_state=None,
        _stage1_ik_candidate_generation=0, _stage1_ik_candidate_results={},
        _stage1_ik_candidate_timer=None, _stage1_ik_candidates_finalized_generation=-1,
        _stage1_attempt_token=object(), _stage1_orientation_candidates=(),
        _stage1_orientation_ranked=[], _stage1_orientation_rank_index=-1,
        _stage1_orientation_branch_frozen=False, _selected_segment_orientation_branch="",
        _stage1_retried=False, _d405_prescan_active=False,
        statuses=[], failures=[], timers=[], get_logger=lambda: logger,
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: executor_module.JointState().header.stamp)),
        _current_tcp_pose_np=lambda: (np.zeros(3), np.array([0., 0., 0., 1.])),
    )
    node.current_joint_state.name = list(executor_module.READY_POSE_JOINTS)
    node.current_joint_state.position = [0.] * 6
    node._publish_execution_status = lambda *args, **kw: node.statuses.append((args, kw))
    node._fail_stage1_before_motion = node.failures.append
    node.destroy_timer = lambda _timer: None

    def create_timer(_period, callback):
        timer = _Timer(callback)
        node.timers.append(timer)
        return timer

    node.create_timer = create_timer
    for name in _METHODS:
        method = getattr(executor_module.MoveItExecutor, name)
        descriptor = vars(executor_module.MoveItExecutor)[name]
        setattr(node, name, method if isinstance(descriptor, staticmethod) else MethodType(method, node))
    return node


def _prepare(model="rb20_1900es", axis="+z"):
    node = _executor(model=model, axis=axis)
    node._active_segment_path = _parse(_spray_payload(model, axis))
    assert node._prepare_segment_process() is not None, node.get_logger().errors
    return node


def _ik_response(value=.1, error=1):
    response = executor_module.GetPositionIK.Response()
    response.error_code.val = error
    response.solution.joint_state.name = list(executor_module.READY_POSE_JOINTS)
    response.solution.joint_state.position = [float(value)] * 6
    return response


@pytest.mark.parametrize("model,axis", [("rb20_1900es", "+z"), ("rb10_1300e_u", "-y")])
@pytest.mark.parametrize("coverage", ["auto_fill", "manual_sketch"])
def test_real_spray_accepts_zero_geometry_and_custom_standoff(model, axis, coverage):
    node = _executor(model=model, axis=axis)
    payload = _spray_payload(model, axis)
    payload['source']['coverage'] = coverage
    payload = attach_plan_hash(payload)
    expected = _parse(payload)
    node.on_eoat_segments(executor_module.String(data=json.dumps(payload)))

    assert node._segment_path == expected, node.statuses
    assert node._segment_path.contact_geometry_offset_m == 0.
    assert node._segment_path.spray_standoff_m == .65
    assert node.statuses[-1][0][0] == "PATH_RECEIVED"


def test_real_paint_accepts_its_commissioned_geometry():
    node = _executor("paint")
    payload = _paint_payload()
    expected = _parse(payload)
    node.on_eoat_segments(executor_module.String(data=json.dumps(payload)))
    assert node._segment_path == expected, node.statuses


@pytest.mark.parametrize("model,axis", [("rb20_1900es", "+z"), ("rb10_1300e_u", "-y")])
def test_spray_preparation_preserves_preview_roll_when_current_tcp_x_is_opposite(model, axis):
    node = _executor(model=model, axis=axis)
    node._current_tcp_pose_np = lambda: (np.zeros(3), np.array([0., 0., 1., 0.]))
    node._active_segment_path = _parse(_spray_payload(model, axis))
    preview_rotation = rotation_from_spray_path([0., 0., 1.], [0., 1., 0.], axis)
    np.testing.assert_allclose(preview_rotation[:, 0], [1., 0., 0.], atol=1e-9)

    assert node._prepare_segment_process() is not None, node.get_logger().errors
    assert len(node._stage1_orientation_candidates) == 1
    candidate = node._stage1_orientation_candidates[0]
    for pose in candidate["row_tcp_poses"].values():
        q = pose.orientation
        actual_rotation = quat_to_matrix([q.x, q.y, q.z, q.w])
        np.testing.assert_allclose(
            actual_rotation, preview_rotation, atol=1e-9,
            err_msg="spray execution must preserve preview roll despite current TCP X")


def test_paint_preparation_preserves_current_axis_seed_and_two_candidates():
    node = _executor("paint")
    node._current_tcp_pose_np = lambda: (np.zeros(3), np.array([0., 0., 1., 0.]))
    node._active_segment_path = _parse(_paint_payload())
    assert node._prepare_segment_process() is not None, node.get_logger().errors
    assert len(node._stage1_orientation_candidates) == 2
    for candidate, expected_x in zip(node._stage1_orientation_candidates, [-1., 1.]):
        q = candidate["safety_tcp_pose"].orientation
        rotation = quat_to_matrix([q.x, q.y, q.z, q.w])
        np.testing.assert_allclose(rotation[:, 0], [expected_x, 0., 0.], atol=1e-9)
        np.testing.assert_allclose(rotation[:, 1], [0., 0., 1.], atol=1e-9)


@pytest.mark.parametrize("field", [
    "contact_geometry_offset_m", "precontact_clearance_m", "travel_clearance_m",
    "safety_approach_offset_m", "final_retreat_offset_m",
])
def test_real_paint_still_rejects_executor_geometry_mismatch(field):
    node = _executor("paint")
    payload = _paint_payload()
    _parse(payload)
    setattr(node, field, getattr(node, field) + .003)
    node.on_eoat_segments(executor_module.String(data=json.dumps(payload)))
    assert node._segment_path is None
    state, reason = node.statuses[-1][0]
    assert state == "PATH_REJECTED" and field in reason


@pytest.mark.parametrize("model,axis,nozzle", [
    ("rb20_1900es", "+z", [0., 0., 1.]),
    ("rb10_1300e_u", "-y", [0., -1., 0.]),
])
@pytest.mark.parametrize("immediate", [False, True])
def test_single_spray_candidate_installs_poses_on_ik_completion(model, axis, nozzle, immediate):
    node = _prepare(model, axis)
    assert len(node._stage1_orientation_candidates) == 1
    assert node._process_row_tcp_poses == {}
    if immediate:
        node.ik_client.immediate_response = _ik_response()
    node._request_stage1_nearest_ik()
    assert len(node.ik_client.requests) == 1
    request = node.ik_client.requests[0].ik_request
    assert request.avoid_collisions is True
    assert request.ik_link_name == "tcp" and request.pose_stamped.header.frame_id == "link0"
    if not immediate:
        assert node.move_action_client.goals == [] and node._process_row_tcp_poses == {}
        node.ik_client.futures[0].set_result(_ik_response())

    assert node.failures == []
    assert len(node.move_action_client.goals) == 1
    assert node.move_action_client.goals[0].planning_options.plan_only is True
    assert node._stage1_ik_candidate_timer is None
    assert node._selected_segment_orientation_branch == "spray_tool_axis"
    assert set(node._process_row_tcp_poses) == {1, 2, 3, 4}
    for number, expected_y in [(1, 0.), (2, 0.), (3, .4), (4, .4)]:
        pose = node._process_row_tcp_poses[number]
        np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z],
                                   [-.02, expected_y + .03, .83], atol=1e-8)
        q = pose.orientation
        np.testing.assert_allclose(quat_apply([q.x, q.y, q.z, q.w], nozzle),
                                   [0., 0., -1.], atol=1e-9)
    assert node._retreat_tcp_pose.position.z == pytest.approx(.83)
    assert node._process_last_tcp_pose.position.z == pytest.approx(.83)
    for timer in node.timers:
        timer.callback()
    assert len(node.move_action_client.goals) == 1 and node.failures == []


def test_two_candidates_still_wait_for_both_collision_checks():
    node = _prepare()
    first = node._stage1_orientation_candidates[0]
    second = copy.deepcopy(first)
    second["name"] = "other_candidate"
    node._stage1_orientation_candidates = (first, second)
    node._request_stage1_nearest_ik()
    assert len(node.ik_client.requests) == 2
    node.ik_client.futures[1].set_result(_ik_response(.4))
    assert node.move_action_client.goals == [] and node._process_row_tcp_poses == {}
    node.ik_client.futures[0].set_result(_ik_response(.1))
    assert node.failures == [] and len(node.move_action_client.goals) == 1
    assert node._selected_segment_orientation_branch == "spray_tool_axis"


@pytest.mark.parametrize("failure", ["request", "no_solution", "timeout"])
def test_single_candidate_failure_finishes_without_waiting_for_nonexistent_second(failure):
    node = _prepare()
    node.ik_client.raise_on_request = failure == "request"
    node._request_stage1_nearest_ik()
    if failure == "timeout":
        node._stage1_ik_candidate_timer.callback()
    elif failure == "no_solution":
        node.ik_client.futures[0].set_result(_ik_response(error=-31))
    assert len(node.failures) == 1
    assert node.move_action_client.goals == [] and node._process_row_tcp_poses == {}
    assert node._stage1_ik_candidate_timer is None
    if failure == "timeout":
        assert "IK_RESPONSE_TIMEOUT" in node.failures[0]
        assert set(node._stage1_ik_candidate_results) == {0}
        node.ik_client.futures[0].set_result(_ik_response())
        assert len(node.failures) == 1 and node.move_action_client.goals == []


def test_late_single_candidate_response_cannot_activate_a_new_attempt():
    node = _prepare()
    node._request_stage1_nearest_ik()
    old_future = node.ik_client.futures[0]
    node._invalidate_stage1_orientation_candidates()
    node._stage1_attempt_token = object()
    node._request_stage1_nearest_ik()
    old_future.set_result(_ik_response(.01))
    assert node.move_action_client.goals == [] and node._process_row_tcp_poses == {}
    node.ik_client.futures[1].set_result(_ik_response(.2))
    assert node.failures == [] and len(node.move_action_client.goals) == 1


@pytest.mark.parametrize("entry", ["auto", "sketch", "multi_target", "work_area"])
def test_spray_never_starts_a_d405_prescan(entry):
    node = _executor()

    def forbidden(*_args, **_kwargs):
        pytest.fail("spray entered the D405 capture/planning pipeline")

    node._d405_refined_surface_fresh = forbidden
    node._invalidate_d405_prescan_callbacks = forbidden
    node._build_d405_prescan_poses = forbidden
    node.publish_scene_periodic = forbidden
    node._start_d405_scene_wait = forbidden
    result = (node._maybe_begin_d405_prescan() if entry == "auto"
              else node._begin_d405_prescan(mode=entry))
    assert result is False
    assert not node.executing and not node._d405_prescan_active
    assert node.move_action_client.goals == []


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


def _catalog():
    plane = dict(id="wall-a", center=[0., 0., 1.], normal=[0., 0., -1.],
                 corners=[[-.5, -.5, 1.], [.5, -.5, 1.],
                          [.5, .5, 1.], [-.5, .5, 1.]],
                 inlier_count=120, rms_m=.004)
    other = copy.deepcopy(plane)
    other["id"] = "wall-b"
    return dict(generation="1000000000", frame_id="zed_left_camera_frame",
                planes=[plane, other])


def _json_message(payload):
    return executor_module.String(data=json.dumps(payload))


def _forbid_candidate_motion(*_args, **_kwargs):
    pytest.fail("candidate selection must not enter robot-state or D405 motion code")


def _multi_executor(mode="spray", catalog=None):
    node = _executor(mode)
    node._multi_catalog = dict(generation="", planes=[])
    node._multi_queue = []
    node._multi_selected = []
    node._multi_refined = {}
    node._multi_active_id = ""
    node._multi_current = None
    node._multi_state = "empty"
    node._active_trajectory_goal_token = None
    node._zed_selection_generation = "1000000000"
    node._zed_target_lock = None
    node._zed_plane_accepted = False
    node._zed_pending_area = None
    node._zed_accepted_area = None
    node._zed_work_area_selection_id = ""
    node._d405_refined_lock_active = False
    node.dynamic_surface_source = ""
    node._multi_status_pub = _Publisher()
    node._zed_target_lock_pub = _Publisher()
    node._multi_target_pub = _Publisher()
    node._multi_refined_pub = _Publisher()
    node._multi_target_capture_pub = _Publisher()
    node.current_joint_state = None
    node._current_tcp_pose_np = _forbid_candidate_motion
    node._multi_start_next = _forbid_candidate_motion
    node._begin_multi_order = _forbid_candidate_motion
    node._begin_d405_prescan = _forbid_candidate_motion
    stamp = multi_module.PoseStamped().header.stamp
    stamp.sec, stamp.nanosec = 7, 11
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: copy.deepcopy(stamp)))
    for cls, names in ((multi_module.MultiSurfaceMixin, _MULTI_METHODS),
                       (multi_module.ZedSprayExecutionMixin, _ZED_LOCK_METHODS)):
        for name in names:
            setattr(node, name, MethodType(getattr(cls, name), node))
    node._multi_on_catalog(_json_message(_catalog() if catalog is None else catalog))
    return node


def _select(node, ids=None, generation=None):
    node._multi_on_select(_json_message(dict(
        ids=["wall-a"] if ids is None else ids,
        generation=node._multi_catalog["generation"] if generation is None else generation)))


def _published_locks(node):
    return [payload for message in node._zed_target_lock_pub.messages
            if (payload := json.loads(message.data)).get("accepted") is True]


def _assert_no_candidate_motion(node):
    assert node.move_action_client.goals == [] and node.ik_client.requests == []
    assert not node.executing and not node._d405_prescan_active
    assert node._multi_queue == [] and node._multi_current is None
    assert node._multi_target_pub.messages == []
    assert node._multi_refined_pub.messages == []
    assert node._multi_target_capture_pub.messages == []


def test_spray_selects_current_catalog_without_robot_state_and_locks_metadata():
    node = _multi_executor()
    node._accepted_plan_hash = "previous-plan"
    node._accepted_plan_path_id = "previous-path"
    node._current_work_area_id = "previous-area"
    node._current_plane_generation_id = "previous-generation"
    _select(node, ["wall-a", "wall-b"])

    assert node.current_joint_state is None
    _assert_no_candidate_motion(node)
    assert node._multi_selected == ["wall-a", "wall-b"]
    assert node._multi_active_id == "wall-a"
    assert list(node._multi_refined) == ["wall-a", "wall-b"]
    assert not node._zed_plane_accepted  # Target lock is not work-area acceptance.
    assert not node._accepted_plan_hash and not node._accepted_plan_path_id
    assert not node._current_work_area_id and not node._current_plane_generation_id
    lock, = _published_locks(node)
    assert lock == node._zed_target_lock
    assert lock["source"] == "zed" and lock["state"] == "locked"
    assert lock["plane_generation_id"] == "zed:1000000000:wall-a:7000000011"
    assert lock["catalog_generation"] == "1000000000" and lock["plane_id"] == "wall-a"
    assert lock["frame_id"] == "zed_left_camera_frame"
    assert lock["stamp"] == {"sec": 7, "nanosec": 11}
    assert lock["center"] == [0., 0., 1.] and lock["normal"] == [0., 0., -1.]
    assert lock["corners"] == _catalog()["planes"][0]["corners"]
    assert lock["inlier_count"] == 120 and lock["rms_m"] == .004
    validate_target_lock(lock)
    status = json.loads(node._multi_status_pub.messages[-1].data)
    assert status["state"] == "ready" and status["source"] == "zed"
    assert status["running"] is False and status["error"] == ""


@pytest.mark.parametrize("stale", ["request", "catalog", "missing_selection"])
def test_spray_rejects_stale_catalog_or_selection_before_locking(stale):
    catalog = _catalog()
    if stale == "catalog":
        catalog["generation"] = "old-catalog"
    node = _multi_executor(catalog=catalog)
    if stale == "missing_selection":
        node._zed_selection_generation = ""
    _select(node, generation="old-request" if stale == "request" else None)
    assert _published_locks(node) == [] and node._zed_target_lock is None
    assert node._multi_selected == [] and node._multi_active_id == ""
    error = json.loads(node._multi_status_pub.messages[-1].data)["error"]
    assert error == ("INVALID_SELECTION" if stale == "request" else "STALE_ZED_SELECTION")
    _assert_no_candidate_motion(node)


@pytest.mark.parametrize("quality", [
    {"inlier_count": 79}, {"inlier_count": -1}, {"inlier_count": None},
    {"rms_m": .0151}, {"rms_m": -.001}, {"rms_m": float("nan")},
    {"rms_m": float("inf")},
])
def test_spray_rejects_low_quality_plane_before_lock_or_motion(quality):
    catalog = _catalog()
    catalog["planes"][0].update(quality)
    node = _multi_executor(catalog=catalog)
    _select(node)
    assert _published_locks(node) == [] and node._zed_target_lock is None
    assert node._multi_active_id == "" and not node._zed_plane_accepted
    assert json.loads(node._multi_status_pub.messages[-1].data)["error"] == "ZED_PLANE_QUALITY_REJECTED"
    _assert_no_candidate_motion(node)


def test_spray_activation_uses_current_selected_plane_without_d405_motion():
    node = _multi_executor()
    _select(node, ["wall-a", "wall-b"])
    node._multi_on_activate(_json_message(dict(generation="stale", id="wall-b")))
    assert node._multi_active_id == "wall-a" and len(_published_locks(node)) == 1
    node._multi_on_activate(_json_message(dict(generation="1000000000", id="wall-b")))
    assert node._multi_active_id == "wall-b"
    assert _published_locks(node)[-1]["plane_id"] == "wall-b"
    validate_target_lock(_published_locks(node)[-1])
    _assert_no_candidate_motion(node)


@pytest.mark.parametrize("missing", ["joint_state", "tcp"])
def test_paint_selection_still_requires_joint_state_and_tcp(missing):
    node = _multi_executor("paint")
    if missing == "tcp":
        node.current_joint_state = executor_module.JointState()
        node._current_tcp_pose_np = lambda: None
    _select(node)
    assert json.loads(node._multi_status_pub.messages[-1].data)["error"] == "ROBOT_STATE_UNAVAILABLE"
    assert node._multi_selected == [] and _published_locks(node) == []
    _assert_no_candidate_motion(node)


def test_rejected_spray_reselection_cannot_leave_active_target_outside_selected_ids():
    catalog = _catalog()
    catalog["planes"][1]["inlier_count"] = 79
    node = _multi_executor(catalog=catalog)
    _select(node)
    old_lock = copy.deepcopy(node._zed_target_lock)
    node._zed_plane_accepted = True
    node._accepted_plan_hash = "accepted-old-plan"
    node._accepted_plan_path_id = "accepted-old-path"
    _select(node, ["wall-b"])

    assert json.loads(node._multi_status_pub.messages[-1].data)["error"] == "ZED_PLANE_QUALITY_REJECTED"
    assert len(_published_locks(node)) == 1
    _assert_no_candidate_motion(node)
    if node._zed_target_lock is not None:
        # Reject transactionally, or invalidate the old lock and plan entirely.
        assert node._zed_target_lock == old_lock
        assert node._multi_selected == ["wall-a"], (
            "rejected selection replaced selected IDs but retained the previous accepted target/plan")
    else:
        assert not node._zed_plane_accepted
        assert not node._accepted_plan_hash and not node._accepted_plan_path_id
