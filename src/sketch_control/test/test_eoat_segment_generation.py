import json

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker

from rbpodo_painting_control.segment_path import (
    SegmentPath,
    SegmentWaypoint,
    compute_plan_hash,
    parse_segment_path,
    rotation_from_surface_path,
    segment_waypoint_position,
    validate_segment_path_for_real_execution,
)
from sketch_control import moveit_executor
from sketch_control.sketch_to_waypoints_node import (
    ROLLER_LENGTH,
    ROLLER_RADIUS,
    SketchToWaypointsNode,
)


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _pose_rotation(pose):
    return moveit_executor.quat_to_matrix(
        np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
    )


def test_roller_symmetry_builds_two_exact_branches_and_freezes_each_raster():
    normal = (-1.0, 0.0, 0.0)
    rows = (
        SegmentWaypoint(
            mode="APPROACH_PRECONTACT",
            position=(0.80, -0.10, 0.45),
            normal=normal,
            tangent=(0.0, 0.0, 1.0),
            force_n=0.0,
            offset_m=0.010,
            speed_mps=0.005,
            row_number=1,
        ),
        SegmentWaypoint(
            mode="PAINT",
            position=(0.80, -0.10, 0.65),
            normal=normal,
            tangent=(0.0, 0.0, -1.0),
            force_n=1.6,
            offset_m=0.0,
            speed_mps=0.02,
            row_number=2,
        ),
        SegmentWaypoint(
            mode="PAINT",
            position=(0.80, 0.10, 0.45),
            normal=normal,
            tangent=(0.0, 1.0, 0.0),
            force_n=1.6,
            offset_m=0.0,
            speed_mps=0.02,
            row_number=3,
        ),
        SegmentWaypoint(
            mode="FINAL_RETRACT",
            position=(0.80, 0.15, 0.45),
            normal=normal,
            tangent=(0.0, 1.0, 0.0),
            force_n=0.0,
            offset_m=0.080,
            speed_mps=0.01,
            row_number=4,
        ),
    )
    path = SegmentPath(
        version=3,
        frame_id="link0",
        path_id="symmetry-test",
        contact_offset_m=0.026,
        contact_geometry_offset_m=0.026,
        safety_approach_offset_m=0.080,
        final_retreat_offset_m=0.080,
        rows=rows,
        source={},
    )
    node = type("PoseBuilder", (), {})()
    node._segment_tip_pose = lambda p, row, previous=None: (
        moveit_executor.MoveItExecutor._segment_tip_pose(
            node, p, row, previous
        )
    )
    node._brush_tip_to_tcp = moveit_executor.MoveItExecutor._brush_tip_to_tcp
    raw_x = rotation_from_surface_path(normal, rows[0].tangent)[:, 0]

    candidate_a = moveit_executor.MoveItExecutor._build_segment_orientation_candidate(
        node, path, raw_x, name="A"
    )
    candidate_b = moveit_executor.MoveItExecutor._flip_segment_orientation_candidate(
        node, candidate_a, name="B"
    )

    for key in ("safety_tcp_pose", "retreat_tcp_pose"):
        rotation_a = _pose_rotation(candidate_a[key])
        rotation_b = _pose_rotation(candidate_b[key])
        np.testing.assert_allclose(rotation_a[:, 1], rotation_b[:, 1], atol=1e-9)
        np.testing.assert_allclose(rotation_a[:, 0], -rotation_b[:, 0], atol=1e-9)
        np.testing.assert_allclose(rotation_a[:, 2], -rotation_b[:, 2], atol=1e-9)
        np.testing.assert_allclose(
            [
                candidate_a[key].position.x,
                candidate_a[key].position.y,
                candidate_a[key].position.z,
            ],
            [
                candidate_b[key].position.x,
                candidate_b[key].position.y,
                candidate_b[key].position.z,
            ],
            atol=1e-9,
        )

    rotations_a = [
        _pose_rotation(candidate_a["row_tcp_poses"][row.row_number])
        for row in rows
    ]
    rotations_b = [
        _pose_rotation(candidate_b["row_tcp_poses"][row.row_number])
        for row in rows
    ]
    # A serpentine direction reversal does not flip the roller.  The third
    # row then introduces an exact 90-degree sketch corner to prove the two
    # candidate bundles remain opposite even at the sign-test tie.
    assert float(np.dot(rotations_a[0][:, 0], rotations_a[1][:, 0])) > 0.999
    assert float(np.dot(rotations_b[0][:, 0], rotations_b[1][:, 0])) > 0.999
    assert all(
        float(np.dot(first[:, 0], second[:, 0])) < -0.999
        for first, second in zip(rotations_a, rotations_b)
    )


def _generator_stub(*, real=False, accepted=True):
    node = object.__new__(SketchToWaypointsNode)
    node.publish_eoat_segments = True
    node.real_painting_enabled = real
    node.dry_run = not real
    node.default_paint_force_n = 2.0
    node.paint_speed_mps = 0.020
    node.contact_geometry_offset_m = 0.026
    node.precontact_clearance_m = 0.005
    node.travel_clearance_m = 0.010
    node.safety_approach_offset_m = 0.080
    node.final_retreat_offset_m = 0.080
    node.contact_search_speed_mps = 0.002
    node.contact_search_max_distance_m = 0.010
    node.contact_search_timeout_s = 10.0
    node.retract_speed_mps = 0.010
    node.approach_speed_mps = 0.005
    node.travel_speed_mps = 0.030
    node.roller_length_m = 0.175
    node.latest_work_area_selection_id = "selection-1"
    node.current_work_area_id = "work-1"
    node.work_area_invalidation_seq = "1"
    node.d405_status = {
        "accepted": accepted,
        "work_area_id": "work-1" if accepted else "",
        "plane_generation_id": "plane-1" if accepted else "",
        "reason": "" if accepted else "not accepted",
    }
    node.d405_refined_pose_armed = False
    node.d405_refined_pose_generation_id = ""
    node.segment_pub = _CapturePublisher()
    node.marker_pub = _CapturePublisher()
    node.plan_status_pub = _CapturePublisher()
    node.fill_preview_pub = _CapturePublisher()
    node._segment_frame_transform = lambda: (
        "link0",
        np.eye(3),
        np.zeros(3),
    )
    node.get_logger = lambda: _Logger()
    return node


def _publish_two_strokes(node):
    strokes = [
        [np.array([0.0, 0.0, 0.0]), np.array([0.10, 0.05, 0.0])],
        [np.array([0.20, 0.20, 0.0]), np.array([0.15, 0.30, 0.0])],
    ]
    return node._publish_eoat_segments(
        strokes,
        normal_world=np.array([0.0, 0.0, 1.0]),
        fallback_tangent=np.array([1.0, 0.0, 0.0]),
        path_id="1234",
        source={"test": True},
    )


def test_separate_strokes_generate_hashed_v3_contact_search_process():
    node = _generator_stub()
    path = _publish_two_strokes(node)

    assert path is not None
    assert len(node.segment_pub.messages) == 1
    payload = json.loads(node.segment_pub.messages[0].data)
    assert payload["version"] == 3
    assert payload["path_id"] == "1234"
    assert payload["work_area_id"] == "work-1"
    assert payload["plane_generation_id"] == "plane-1"
    assert payload["point_semantics"] == "surface_point"
    assert payload["contact_geometry_offset_m"] == pytest.approx(ROLLER_RADIUS)
    assert payload["precontact_clearance_m"] == pytest.approx(0.005)
    assert payload["travel_clearance_m"] == pytest.approx(0.010)
    assert payload["safety_approach_offset_m"] == pytest.approx(0.080)
    assert payload["final_retreat_offset_m"] == pytest.approx(0.080)
    assert payload["source"]["roller_usable_length_m"] == pytest.approx(
        ROLLER_LENGTH
    )
    assert ROLLER_LENGTH == pytest.approx(0.175)
    assert payload["plan_hash"] == compute_plan_hash(payload)

    modes = [row["mode"] for row in payload["rows"]]
    assert modes == [
        "APPROACH_PRECONTACT",
        "CONTACT_SEARCH",
        "RAMP_UP",
        "PAINT",
        "PAINT",
        "RAMP_DOWN",
        "RETRACT",
        "TRAVEL",
        "APPROACH_PRECONTACT",
        "CONTACT_SEARCH",
        "RAMP_UP",
        "PAINT",
        "PAINT",
        "RAMP_DOWN",
        "FINAL_RETRACT",
    ]

    travel = next(row for row in payload["rows"] if row["mode"] == "TRAVEL")
    assert travel["force_n"] == 0.0
    assert travel["offset_m"] == pytest.approx(node.travel_clearance_m)
    assert travel["speed_mps"] == pytest.approx(node.travel_speed_mps)
    final_retract = next(
        row for row in payload["rows"] if row["mode"] == "FINAL_RETRACT"
    )
    assert final_retract["offset_m"] == pytest.approx(
        node.final_retreat_offset_m
    )

    parsed = parse_segment_path(
        payload,
        default_contact_offset_m=ROLLER_RADIUS,
        max_force_n=20.0,
        minimum_clearance_m=0.005,
        allow_legacy=False,
    )
    assert validate_segment_path_for_real_execution(parsed) is parsed
    assert len(parsed.rows) == len(payload["rows"])


def test_real_generation_rejects_missing_accepted_d405_ids():
    node = _generator_stub(real=True, accepted=False)

    assert _publish_two_strokes(node) is None
    assert node.segment_pub.messages == []
    statuses = [json.loads(message.data) for message in node.plan_status_pub.messages]
    assert statuses[-1]["state"] == "rejected"
    assert statuses[-1]["reason"] == "D405_REFINED_PLANE_REQUIRED"


def test_markers_come_from_final_segment_rows_and_include_hash_and_generation():
    node = _generator_stub()
    path = _publish_two_strokes(node)
    stamp = Time(sec=12, nanosec=34)

    node._publish_segment_markers(path, stamp)

    marker_array = node.marker_pub.messages[-1]
    assert marker_array.markers[0].action == Marker.DELETEALL
    assert all(
        path.plan_hash in marker.ns
        and path.plane_generation_id in marker.ns
        for marker in marker_array.markers[1:]
    )
    paint_spheres = next(
        marker
        for marker in marker_array.markers
        if marker.type == Marker.SPHERE_LIST and marker.ns.endswith("_paint")
    )
    first_paint = next(row for row in path.rows if row.mode == "PAINT")
    expected = segment_waypoint_position(path, first_paint)
    actual = paint_spheres.points[0]
    assert (actual.x, actual.y, actual.z) == pytest.approx(expected)
    assert actual.z == pytest.approx(0.026)

    safety_line = next(
        marker
        for marker in marker_array.markers
        if marker.type == Marker.LINE_STRIP
        and marker.ns.endswith("_safety_approach")
    )
    assert len(safety_line.points) == 2
    safety_start, precontact_end = safety_line.points
    assert safety_start.z == pytest.approx(
        node.contact_geometry_offset_m + node.safety_approach_offset_m
    )
    assert precontact_end.z == pytest.approx(
        node.contact_geometry_offset_m + node.precontact_clearance_m
    )

    search_arrow = next(
        marker
        for marker in marker_array.markers
        if marker.type == Marker.ARROW and len(marker.points) == 2
    )
    start, limit = search_arrow.points
    distance = np.linalg.norm(
        np.array([limit.x - start.x, limit.y - start.y, limit.z - start.z])
    )
    assert distance == pytest.approx(node.contact_search_max_distance_m)


def test_d405_target_mode_status_does_not_unlock_real_work_area_generation():
    node = _generator_stub(real=True, accepted=False)
    message = type("Message", (), {})()
    message.data = json.dumps(
        {
            "mode": "target",
            "accepted": True,
            "work_area_id": "wrong-work",
            "plane_generation_id": "wrong-plane",
        }
    )

    node._on_d405_refinement_status(message)

    assert node.d405_status["accepted"] is False
    assert _publish_two_strokes(node) is None


def test_d405_acceptance_requires_a_json_boolean():
    node = _generator_stub(real=True, accepted=False)
    message = type("Message", (), {})()
    message.data = json.dumps(
        {
            "mode": "work_area",
            "accepted": "false",
            "work_area_id": "work-1",
            "plane_generation_id": "plane-1",
        }
    )

    node._on_d405_refinement_status(message)

    assert node.d405_status["accepted"] is False
    assert _publish_two_strokes(node) is None


def test_stale_d405_status_for_previous_work_area_is_rejected():
    node = _generator_stub(real=True, accepted=False)
    node.current_work_area_id = "work-current"
    message = type("Message", (), {})()
    message.data = json.dumps(
        {
            "mode": "work_area",
            "accepted": True,
            "work_area_id": "work-previous",
            "plane_generation_id": "plane-old",
        }
    )

    node._on_d405_refinement_status(message)

    assert node.d405_status["accepted"] is False
    assert node.d405_status["reason"] == "accepted_status_work_area_mismatch"
    assert _publish_two_strokes(node) is None


def test_fill_preview_publishes_exact_backend_pixel_strokes():
    node = _generator_stub()
    strokes = (
        ((123.25, 45.5), (123.25, 300.75)),
        ((321.5, 300.75), (321.5, 45.5)),
    )

    node._publish_fill_preview(strokes, stamp=Time(sec=9, nanosec=8))

    message = node.fill_preview_pub.messages[-1]
    assert message.header.frame_id == "wall_front"
    assert message.header.stamp == Time(sec=9, nanosec=8)
    assert [
        (pose.position.x, pose.position.y, pose.position.z)
        for pose in message.poses
    ] == pytest.approx(
        [
            (123.25, 45.5, 0.0),
            (123.25, 300.75, 0.0),
            (321.5, 300.75, 1.0),
            (321.5, 45.5, 1.0),
        ]
    )


def test_fill_preview_is_committed_only_after_final_path_succeeds():
    node = _generator_stub()
    node.latest_work_area_rect_px = (100.0, 100.0, 500.0, 400.0)
    node.fill_overlap = 0.30
    node._current_work_area_size_m = lambda: (0.40, 0.30)
    node.get_clock = lambda: type(
        "Clock", (), {"now": lambda self: type(
            "Now", (), {"to_msg": lambda self: Time(sec=1, nanosec=2)}
        )()}
    )()
    node._on_sketch = lambda _message: None

    node._on_fill_work_area(object())

    assert node.fill_preview_pub.messages
    assert node.fill_preview_pub.messages[-1].poses == []


def test_fill_preview_matches_candidate_after_final_path_acceptance():
    node = _generator_stub()
    node.latest_work_area_rect_px = (100.0, 100.0, 500.0, 400.0)
    node.fill_overlap = 0.30
    node._current_work_area_size_m = lambda: (0.40, 0.30)
    node.get_clock = lambda: type(
        "Clock", (), {"now": lambda self: type(
            "Now", (), {"to_msg": lambda self: Time(sec=1, nanosec=2)}
        )()}
    )()
    node._on_sketch = lambda _message: object()

    node._on_fill_work_area(object())

    assert len(node.fill_preview_pub.messages[-1].poses) > 0


def test_new_work_area_pixels_invalidate_previous_refined_plane_and_markers():
    node = _generator_stub(real=True, accepted=True)
    node.view_w = 800
    node.view_h = 600
    node.work_area_pixel_tolerance_px = 1.0
    node.latest_refined_work_area = object()
    node.latest_refined_work_area_time = 123.0
    cleared = []
    node._publish_marker_clear = cleared.append

    message = PoseArray()
    message.header.frame_id = "wall_front"
    message.header.stamp = Time(sec=7, nanosec=11)
    for u, v in ((100.0, 120.0), (500.0, 420.0)):
        pose = Pose()
        pose.position.x = u
        pose.position.y = v
        message.poses.append(pose)

    node._on_work_area_pixels(message)

    assert node.latest_work_area_rect_px == pytest.approx(
        (100.0, 120.0, 500.0, 420.0)
    )
    assert node.latest_refined_work_area is None
    assert node.latest_refined_work_area_time == 0.0
    assert node.d405_status["accepted"] is False
    assert node.d405_refined_pose_armed is False
    assert cleared == ["World"]


def test_outside_free_sketch_is_rejected_with_json_point_count_not_clipped():
    node = _generator_stub()
    node.latest_work_area_rect_px = (100.0, 100.0, 400.0, 300.0)
    node.work_area_pixel_tolerance_px = 0.5
    cleared = []
    node._publish_marker_clear = cleared.append
    message = PoseArray()
    message.header.frame_id = "wall_front"
    for u, v in ((120.0, 120.0), (450.0, 200.0), (250.0, 350.0)):
        pose = Pose()
        pose.position.x = u
        pose.position.y = v
        message.poses.append(pose)

    node._on_sketch(message)

    status = json.loads(node.plan_status_pub.messages[-1].data)
    assert status["state"] == "rejected"
    assert status["reason"] == "PATH_OUTSIDE_WORK_AREA_2D"
    assert status["outside_point_count"] == 2
    assert node.segment_pub.messages == []
    assert cleared == ["World"]


def test_accepted_status_arms_exactly_the_next_refined_pose_generation():
    node = _generator_stub(real=True, accepted=False)
    old_pose = PoseStamped()
    old_pose.pose.position.x = -1.0
    node.latest_refined_work_area = old_pose
    node.latest_refined_work_area_time = 1.0
    status = type("Message", (), {})()
    status.data = json.dumps(
        {
            "mode": "work_area",
            "accepted": True,
            "work_area_id": "work-1",
            "plane_generation_id": "plane-2",
        }
    )

    node._on_d405_refinement_status(status)

    assert node.latest_refined_work_area is None
    assert node.d405_refined_pose_armed is True
    assert node.d405_refined_pose_generation_id == "plane-2"

    accepted_pose = PoseStamped()
    accepted_pose.pose.position.x = 2.0
    node._on_refined_work_area(accepted_pose)
    assert node.latest_refined_work_area is accepted_pose
    assert node.d405_refined_pose_armed is False

    extra_pose = PoseStamped()
    extra_pose.pose.position.x = 3.0
    node._on_refined_work_area(extra_pose)
    assert node.latest_refined_work_area is accepted_pose
