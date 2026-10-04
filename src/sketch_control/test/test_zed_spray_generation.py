"""Exercise actual generator methods with message-shaped values, without ROS."""

import ast
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import time
from itertools import groupby
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


class Marker(Message):
    DELETEALL, ADD, LINE_STRIP, SPHERE_LIST = 3, 0, 4, 7

    def __init__(self):
        super().__init__(scale=SimpleNamespace(x=0., y=0., z=0.), points=[])


@pytest.fixture
def generator(tmp_path):
    # Load the real class unchanged, omitting only imports and ROS startup.
    # This harness does not claim to exercise ROS transport or message typing.
    path = Path(__file__).parents[1] / "sketch_control" / "sketch_to_waypoints_node.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                      and item.name == "SketchToWaypointsNode")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
                             node_class], type_ignores=[])
    namespace = {}
    from rbpodo_painting_control import spray_eoat
    for dependency in (segment_path, spray_path, work_area_geometry, spray_eoat):
        namespace.update({key: value for key, value in vars(dependency).items() if not key.startswith("__")})
    namespace.update(Node=object, np=np, math=math, time=time, json=json, replace=replace,
                     groupby=groupby, Marker=Marker, Point=SimpleNamespace, ColorRGBA=SimpleNamespace,
                     MarkerArray=lambda: SimpleNamespace(markers=[]),
                     WORLD_FRAME="World", CAM_FRAME="camera", String=Message, PoseStamped=Message,
                     PoseArray=Message, Pose=lambda: Message().pose, quat_from_matrix=quat_from_matrix,
                     validate_target_lock=validate_target_lock,
                     _quat_to_rot=lambda *q: quat_to_matrix(q), TransformException=RuntimeError)
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    node = namespace["SketchToWaypointsNode"].__new__(namespace["SketchToWaypointsNode"])
    node.process_mode = "spray"
    node.model_id, node.spray_tool_axis = "rb10_1300e_u", "-y"
    node.spray_eoat_profile = str(write_profile(tmp_path))
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


def write_profile(tmp_path, *, model="rb10_1300e_u", axis="-y"):
    import trimesh
    mesh = trimesh.creation.box(extents=[.04, .06, .18])
    mesh.apply_translation([.02, .03, .09])
    (tmp_path / "tool.stl").write_bytes(mesh.export(file_type="stl"))
    data = dict(schema_version=1, model_id=model, spray_tool_axis=axis,
                mesh_file="tool.stl", mesh_scale_to_m=1., endpoint_confirmed=True,
                mesh_to_tcp=dict(translation_m=[0., 0., 0.], quaternion_xyzw=(
                    [2**-.5, 0., 0., 2**-.5] if axis == "-y" else [0., 0., 0., 1.])))
    path = tmp_path / "tool.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


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
    stamps = iter(range(3, 100))
    generator.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=next(stamps), nanosec=123456789)))
    generator._publish_fill_preview = lambda strokes, **kwargs: previews.append((strokes, kwargs))
    generator._on_fill_work_area(None)
    assert len(generator.segment_pub.messages) == 1
    payload = json.loads(generator.segment_pub.messages[0].data)
    assert payload["plan_hash"] == segment_path.compute_plan_hash(payload)
    assert payload["source"]["coverage"] == "auto_fill"
    assert len(previews[-1][0]) == 4
    preview_stamp = previews[-1][1]['stamp']
    assert generator._stamp_path_id(preview_stamp) == payload['path_id']
    assert len(generator.pub.messages) == 1
    assert len(generator.pub.messages[0].poses) == len(payload["rows"])
    assert all(pose.position.z == pytest.approx(.68) for pose in generator.pub.messages[0].poses)


def test_narrow_spray_publishes_center_preview_and_matching_world_path(generator):
    # A 200 mm area within the existing 1 m front view, with a 350 mm fan.
    generator.zed_surface_status.update(
        corners=[[.3, 0., 0.], [.5, 0., 0.], [.5, .5, 0.], [.3, .5, 0.]],
        position=[.4, .25, 0.])
    for pose, (u, v) in zip(generator.latest_work_area_pixels.poses,
                            ((240., 0.), (400., 0.), (400., 400.), (240., 400.))):
        pose.position.x, pose.position.y = u, v
    generator.latest_work_area_rect_px = (240., 0., 400., 400.)
    previews = []
    generator._publish_fill_preview = lambda strokes, **kwargs: previews.append(strokes)
    generator._on_fill_work_area(None)
    assert len(generator.segment_pub.messages) == 1
    payload = json.loads(generator.segment_pub.messages[0].data)
    assert payload["plan_hash"] == segment_path.compute_plan_hash(payload)
    assert payload["source"]["spray_footprint_width_m"] == .35
    assert previews[-1] == (((320., 0.), (320., 400.)),)
    poses = generator.pub.messages[0].poses
    # The mesh endpoint, rather than the TCP, follows the center spray pass.
    nozzle_positions = []
    for pose in poses:
        q = pose.orientation
        rotation = quat_to_matrix([q.x, q.y, q.z, q.w])
        tcp = np.array([pose.position.x, pose.position.y, pose.position.z])
        nozzle_positions.append(tcp + rotation @ np.array([.02, -.18, .03]))
    nozzle_positions = np.array(nozzle_positions)
    np.testing.assert_allclose(nozzle_positions[:, 0], .4, atol=1e-8)
    np.testing.assert_allclose(nozzle_positions[:, 2], .5, atol=1e-8)
    assert nozzle_positions[:, 1].min() == pytest.approx(0., abs=1e-8)
    assert nozzle_positions[:, 1].max() == pytest.approx(.5)


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


def test_standoff_speed_and_axis_change_hash_and_execution_geometry(generator, tmp_path):
    before = generate(generator)
    generator.spray_standoff_m, generator.spray_speed_mps = .65, .025
    generator.model_id, generator.spray_tool_axis = "rb20_1900es", "+z"
    write_profile(tmp_path, model=generator.model_id, axis=generator.spray_tool_axis)
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


def test_missing_eoat_profile_blocks_generation_and_clears_stale_output(generator):
    generator.spray_eoat_profile = ""
    cleared = []
    generator._publish_marker_clear = lambda *args: cleared.append("markers")
    generator._publish_fill_preview = lambda *args: cleared.append("fill")
    assert generate(generator) is None
    assert json.loads(generator.plan_status_pub.messages[-1].data)["state"] == "rejected"
    assert generator.segment_pub.messages[-1].data == ""
    assert generator.pub.messages[-1].poses == []
    assert set(cleared) == {"markers", "fill"}


def test_generation_rereads_profile_and_mesh_and_hashes_metadata(generator):
    path = generate(generator)
    assert path is not None
    payload = path.raw_payload
    assert len(payload["spray_eoat_profile_sha256"]) == 64
    assert payload["spray_endpoint_tcp_m"] == pytest.approx([.02, -.18, .03])
    for field in ("spray_eoat_profile_sha256", "spray_endpoint_tcp_m"):
        assert payload["source"][field] == payload[field]
    profile_path = Path(generator.spray_eoat_profile)
    data = json.loads(profile_path.read_text())
    data["endpoint_tcp_m"] = [.02, -.17, .03]
    profile_path.write_text(json.dumps(data))
    changed = generate(generator)
    assert changed.plan_hash != path.plan_hash
    assert changed.raw_payload["spray_endpoint_tcp_m"] == [.02, -.17, .03]
    import trimesh
    mesh_path = profile_path.parent / "tool.stl"
    mesh = trimesh.load_mesh(mesh_path)
    mesh.apply_scale(1.1)
    mesh_path.write_bytes(mesh.export(file_type="stl"))
    remeshed = generate(generator)
    assert remeshed.plan_hash != changed.plan_hash
    data["endpoint_confirmed"] = False
    profile_path.write_text(json.dumps(data))
    assert generate(generator) is None
    assert generator.segment_pub.messages[-1].data == ""


def test_deleted_mesh_invalidates_previous_generated_path(generator):
    assert generate(generator) is not None
    (Path(generator.spray_eoat_profile).parent / "tool.stl").unlink()
    assert generate(generator) is None
    assert generator.segment_pub.messages[-1].data == ""


def test_spray_markers_show_compensated_tcp_across_serpentine_strokes(generator):
    path = generate(generator)
    generator.marker_pub = Publisher()
    type(generator)._publish_segment_markers(generator, path, None)
    markers = generator.marker_pub.messages[-1].markers
    spheres = [marker for marker in markers if getattr(marker, "type", None) == Marker.SPHERE_LIST]
    points = [point for marker in spheres for point in marker.points]
    assert len(points) == len(path.rows)
    assert all(point.z == pytest.approx(.68) for point in points)
    poses = generator._compat_pose_array_from_segment(path, None).poses
    np.testing.assert_allclose([[p.x, p.y, p.z] for p in points],
                               [[p.position.x, p.position.y, p.position.z] for p in poses])
    safety = next(marker for marker in markers if getattr(marker, "ns", "").endswith("_safety_approach"))
    assert all(point.z == pytest.approx(.68) for point in safety.points)


def test_paint_generation_does_not_load_spray_profile(generator):
    generator.process_mode = "paint"
    generator.spray_eoat_profile = "missing-file.json"
    generator.real_painting_enabled, generator.dry_run = False, True
    generator._segment_context = lambda: ("work-1", "plane-1")
    generator.precontact_clearance_m = .005
    generator.safety_approach_offset_m = generator.final_retreat_offset_m = .08
    generator.contact_search_speed_mps = .002
    generator.retract_speed_mps = .01
    generator.approach_speed_mps = .005
    generator.roller_length_m = .175
    path = generate(generator, source={"plane": "d405_refined"})
    assert path is not None
    assert path.process_mode == "paint"
    assert "spray_endpoint_tcp_m" not in path.raw_payload
    assert path.contact_geometry_offset_m == .026


@pytest.mark.parametrize("axis,endpoint", [("-y", [.02, -.18, .03]), ("+z", [.02, .03, .18])])
@pytest.mark.parametrize("tilted", [False, True])
def test_preview_tcp_reconstructs_nozzle_half_meter_from_surface(generator, tmp_path, axis, endpoint, tilted):
    generator.model_id = "rb20_1900es" if axis == "+z" else "rb10_1300e_u"
    generator.spray_tool_axis = axis
    write_profile(tmp_path, model=generator.model_id, axis=axis)
    normal = np.array([.6, 0., .8] if tilted else [0., 0., 1.])
    if tilted:
        generator._segment_frame_transform = lambda: (
            "link0", np.array([[.8, 0., .6], [0., 1., 0.], [-.6, 0., .8]]),
            np.array([1., 2., 3.]))
    path = generate(generator)
    poses = generator._compat_pose_array_from_segment(path, None).poses
    for row, pose in zip(path.rows, poses):
        q = pose.orientation
        rotation = quat_to_matrix([q.x, q.y, q.z, q.w])
        tcp = np.array([pose.position.x, pose.position.y, pose.position.z])
        nozzle = tcp + rotation @ endpoint
        np.testing.assert_allclose(nozzle, np.asarray(row.position) + .5 * normal, atol=1e-8)
        assert np.dot(tcp - row.position, normal) == pytest.approx(.68)
