"""Exercise actual generator methods with message-shaped values, without ROS."""

import ast
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from rbpodo_painting_control import segment_path, spray_path
from sketch_control import work_area_geometry
from sketch_control.rotation_utils import quat_from_matrix, quat_to_matrix
from sketch_control.zed_spray_projection import validate_target_lock
from test_zed_spray_geometry import atomic_status


class Message(SimpleNamespace):
    def __init__(self, **kwargs):
        super().__init__(data="", header=SimpleNamespace(frame_id="", stamp=SimpleNamespace(sec=0, nanosec=0)),
                         poses=[], pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=0.),
                         orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        self.__dict__.update(kwargs)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def generator():
    # Load the real class unchanged, omitting only imports and ROS startup.
    # This harness does not claim to exercise ROS transport or message typing.
    path = Path(__file__).parents[1] / "sketch_control" / "sketch_to_waypoints_node.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                      and item.name == "SketchToWaypointsNode")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
                             node_class], type_ignores=[])
    namespace = {}
    for dependency in (segment_path, spray_path, work_area_geometry):
        namespace.update({key: value for key, value in vars(dependency).items() if not key.startswith("__")})
    namespace.update(Node=object, np=np, math=math, time=time, json=json, replace=replace,
                     WORLD_FRAME="World", CAM_FRAME="camera", String=Message, PoseStamped=Message,
                     PoseArray=Message, Pose=lambda: Message().pose, quat_from_matrix=quat_from_matrix,
                     validate_target_lock=validate_target_lock,
                     _quat_to_rot=lambda *q: quat_to_matrix(q), TransformException=RuntimeError)
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    node = namespace["SketchToWaypointsNode"].__new__(namespace["SketchToWaypointsNode"])
    node.process_mode = "spray"
    node.model_id, node.spray_tool_axis = "rb10_1300e_u", "-y"
    node.spray_footprint_width_m, node.spray_overlap = .35, .30
    node.spray_speed_mps, node.spray_standoff_m = .02, .5
    node.real_painting_enabled, node.dry_run = True, False
    node.default_paint_force_n = 2.
    node.paint_speed_mps = .02
    node.contact_geometry_offset_m = .026
    node.travel_clearance_m = .01
    node.travel_speed_mps = .03
    node.contact_search_max_distance_m = .01
    node.contact_search_timeout_s = 10.
    node.publish_eoat_segments = True
    node.eoat_segment_frame = "link0"
    node.plan_status_pub, node.segment_pub = Publisher(), Publisher()
    node.pub = Publisher()
    node._publish_marker_clear = lambda *args, **kwargs: None
    node._publish_fill_preview = lambda *args, **kwargs: None
    node._publish_segment_markers = lambda *args, **kwargs: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=3, nanosec=0)))
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **kw: None, error=lambda *a, **kw: None,
                                              warn=lambda *a, **kw: None)
    node._segment_frame_transform = lambda: ("link0", np.eye(3), np.zeros(3))
    node.view_w, node.view_h = 801, 401
    node.work_area_pixel_tolerance_px = 1.
    node.work_area_containment_tolerance_m = .001
    node.work_area_plane_tolerance_m = .002
    node.zed_surface_status = atomic_status()
    node.zed_target_lock = dict(source="zed", state="locked", accepted=True, frame_id="World",
                               plane_generation_id=node.zed_surface_status["plane_generation_id"],
                               catalog_generation="catalog", plane_id="plane", stamp=dict(sec=1, nanosec=0),
                               center=[.5, .25, 0.], normal=[0., 0., 1.],
                               corners=copy.deepcopy(node.zed_surface_status["corners"]))
    node.latest_work_area_selection_id = "2000000000"
    node.latest_work_area_pixels = pixels()
    node.latest_work_area_rect_px = (0., 0., 800., 400.)
    node.current_work_area_id = "work-1"
    return node


def pixels(sec=2):
    return Message(header=SimpleNamespace(frame_id="wall_front", stamp=SimpleNamespace(sec=sec, nanosec=0)),
                   poses=[SimpleNamespace(position=SimpleNamespace(x=u, y=v, z=0.))
                          for u, v in ((0., 0.), (800., 0.), (800., 400.), (0., 400.))])


def send(callback, payload):
    callback(Message(data=json.dumps(payload)))


def generate(node, source=None):
    return node._publish_eoat_segments(
        [[np.array([999., 999., 999.]), np.array([1000., 999., 999.])]],
        np.array([1., 0., 0.]), np.array([0., 1., 0.]), path_id="path-1",
        source=source or dict(plane="zed", view="wall_front", coverage="auto_fill"))


def test_backend_rebuilds_coverage_and_discards_caller_strokes(generator):
    path = generate(generator)
    assert path is not None
    spray_rows = [row for row in path.rows if row.mode == "SPRAY"]
    assert len(spray_rows) == 8
    assert [row.position[0] for row in spray_rows[::2]] == pytest.approx(np.linspace(.175, .825, 4))
    assert [row.position[1] for row in spray_rows] == pytest.approx([0., .5, .5, 0., 0., .5, .5, 0.])
    assert all(row.position[2] == 0. for row in path.rows)
    assert path.contact_geometry_offset_m == 0.
    assert path.source["selection_id"] == "2000000000"
    segment_path.validate_segment_path_for_real_execution(path)


def test_auto_fill_callback_publishes_matching_hashed_path_and_preview(generator):
    previews = []
    generator._publish_fill_preview = lambda strokes, **kwargs: previews.append(strokes)
    generator._on_fill_work_area(None)
    assert len(generator.segment_pub.messages) == 1
    payload = json.loads(generator.segment_pub.messages[0].data)
    assert payload["plan_hash"] == segment_path.compute_plan_hash(payload)
    assert payload["source"]["coverage"] == "auto_fill"
    assert len(previews[-1]) == 4
    assert len(generator.pub.messages) == 1
    assert len(generator.pub.messages[0].poses) == len(payload["rows"])
    assert all(pose.position.z == pytest.approx(.5) for pose in generator.pub.messages[0].poses)


@pytest.mark.parametrize("source", [dict(plane="zed", view="wall_front"),
    dict(plane="d405_refined", view="wall_front", coverage="auto_fill"),
    dict(plane="zed", view="zed_raw", coverage="auto_fill")])
def test_backend_rejects_non_auto_spray_sources(generator, source):
    assert generate(generator, source) is None
    assert not generator.segment_pub.messages


def test_freehand_topic_cannot_trigger_spray(generator):
    assert generator._on_sketch(pixels()) is None
    assert json.loads(generator.plan_status_pub.messages[-1].data)["reason"] == "SPRAY_AUTO_COVERAGE_REQUIRED"


@pytest.mark.parametrize("missing", ["zed_surface_status", "zed_target_lock", "latest_work_area_pixels"])
def test_dry_run_still_requires_accepted_selected_zed(generator, missing):
    generator.dry_run, generator.real_painting_enabled = True, False
    setattr(generator, missing, None if missing == "latest_work_area_pixels" else {})
    assert generate(generator) is None


def test_status_before_pixels_is_buffered_until_matching_local_stamp(generator):
    status = generator.zed_surface_status
    generator.zed_surface_status = {}
    generator.latest_work_area_pixels = None
    generator.latest_work_area_selection_id = ""
    send(generator._on_zed_surface_status, status)
    assert generator._segment_context() is None
    generator._on_work_area_pixels(pixels())
    assert generator._segment_context() == (status["work_area_id"], status["plane_generation_id"])


def test_status_before_target_lock_is_buffered(generator):
    lock = generator.zed_target_lock
    generator.zed_target_lock = {}
    assert generator._segment_context() is None
    send(generator._on_zed_target_lock, lock)
    assert generator._segment_context() is not None


def test_projector_first_area_target_invalidation_does_not_retire_new_pixels(generator):
    status = copy.deepcopy(generator.zed_surface_status)
    generator.zed_surface_status = {}
    generator._on_work_area_pixels(pixels())
    send(generator._on_zed_surface_status, dict(
        source="zed", mode="target", accepted=False, state="invalidated",
        plane_generation_id=status["plane_generation_id"], selection_id="",
        reason="new work area selection"))
    send(generator._on_zed_surface_status, status)
    assert generator._segment_context() == (status["work_area_id"], status["plane_generation_id"])
    assert generate(generator) is not None


@pytest.mark.parametrize("selection", ["", "1000000000", "3000000000"])
def test_unrelated_area_invalidation_cannot_retire_active_selection(generator, selection):
    before = generator._segment_context()
    send(generator._on_zed_surface_status, dict(
        source="zed", mode="work_area", accepted=False, state="invalidated",
        plane_generation_id=generator.zed_target_lock["plane_generation_id"], selection_id=selection))
    assert generator._segment_context() == before


def test_matching_area_invalidation_revokes_area_and_stale_replay(generator):
    status = copy.deepcopy(generator.zed_surface_status)
    invalidation = dict(status, accepted=False, state="invalidated")
    send(generator._on_zed_surface_status, invalidation)
    send(generator._on_zed_surface_status, status)
    generator._on_work_area_pixels(pixels())
    assert generator._segment_context() is None


def test_wrong_selection_stamp_cannot_authorize(generator):
    generator.latest_work_area_pixels.header.stamp.sec = 3
    assert generate(generator) is None


def test_new_pixels_revoke_old_status_and_replay_cannot_reactivate(generator):
    status = copy.deepcopy(generator.zed_surface_status)
    generator._on_work_area_pixels(pixels(sec=3))
    send(generator._on_zed_surface_status, status)
    generator._on_work_area_pixels(pixels(sec=2))
    assert generator.latest_work_area_selection_id == "3000000000"
    assert generator._segment_context() is None


def test_mode_change_retires_lock_pixels_and_status(generator):
    status, lock = copy.deepcopy(generator.zed_surface_status), copy.deepcopy(generator.zed_target_lock)
    send(generator._on_process_mode, {"mode": "paint"})
    send(generator._on_process_mode, {"mode": "spray"})
    send(generator._on_zed_target_lock, lock)
    send(generator._on_zed_surface_status, status)
    generator._on_work_area_pixels(pixels())
    assert generator._segment_context() is None


def test_target_generation_change_revokes_existing_work_area(generator):
    lock = copy.deepcopy(generator.zed_target_lock)
    lock["stamp"]["sec"] = 3
    lock["plane_generation_id"] = "zed:catalog:plane:3000000000"
    send(generator._on_zed_target_lock, lock)
    assert generator._segment_context() is None
    assert generator.latest_work_area_pixels is None


def test_same_generation_changed_geometry_is_revoked(generator):
    lock = copy.deepcopy(generator.zed_target_lock)
    lock["center"][0] += .01
    send(generator._on_zed_target_lock, lock)
    assert generator._segment_context() is None


def test_generic_and_d405_callbacks_cannot_modify_spray_acceptance(generator):
    before = generator._segment_context()
    for callback in (generator._on_work_area, generator._on_refined_work_area,
                     generator._on_work_area_corners, generator._on_front_extent):
        callback(object())
    send(generator._on_work_area_state, dict(selected=False, state="invalidated"))
    send(generator._on_d405_refinement_status, dict(accepted=False, mode="work_area"))
    send(generator._on_zed_surface_status, dict(source="zed", mode="target", accepted=True, state="locked"))
    assert generator._segment_context() == before
    assert generate(generator) is not None


def test_opposite_normal_cannot_match_target_lock(generator):
    generator.zed_surface_status["orientation"] = [1., 0., 0., 0.]
    assert generate(generator) is None


def test_pixel_rectangle_must_match_atomic_geometry(generator):
    generator.latest_work_area_pixels.poses[1].position.x = 790.
    generator.latest_work_area_pixels.poses[2].position.x = 790.
    assert generate(generator) is None


def test_standoff_speed_and_axis_change_hash_and_execution_geometry(generator):
    before = generate(generator)
    generator.spray_standoff_m, generator.spray_speed_mps = .65, .025
    generator.model_id, generator.spray_tool_axis = "rb20_1900es", "+z"
    after = generate(generator)
    assert after.plan_hash != before.plan_hash
    assert after.spray_tool_axis == "+z"
    assert all(row.speed_mps == .025 for row in after.rows if row.mode == "SPRAY")
    assert all(segment_path.segment_waypoint_position(after, row)[2] == pytest.approx(.65)
               for row in after.rows)


def test_invalid_final_transform_rejects_spray(generator):
    generator._segment_frame_transform = lambda: ("link0", np.eye(3), np.array([np.nan, 0., 0.]))
    assert generate(generator) is None
    assert not generator.segment_pub.messages
