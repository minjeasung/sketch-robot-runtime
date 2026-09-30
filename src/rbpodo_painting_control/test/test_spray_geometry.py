import copy

import numpy as np
import pytest

from rbpodo_painting_control.segment_path import (
    SegmentPathError, attach_plan_hash, parse_segment_path, segment_waypoint_position,
    validate_segment_path_for_real_execution,
)
from rbpodo_painting_control.spray_path import (
    SPRAY_METADATA_FIELDS, resolve_spray_tool_axis, rotation_from_spray_path,
    spray_spacing_m,
)


def spray_payload():
    metadata = dict(model_id="rb10_1300e_u", spray_tool_axis="-y",
                    spray_footprint_width_m=.35, spray_overlap=.30,
                    spray_spacing_m=.245, spray_speed_mps=.02, spray_standoff_m=.5,
                    spray_eoat_profile_sha256="a" * 64,
                    spray_endpoint_tcp_m=[.02, -.18, .03])
    rows = []
    for mode, y in (("SPRAY_APPROACH", 0.), ("SPRAY", 0.),
                    ("SPRAY", .4), ("SPRAY_FINISH", .4)):
        rows.append(dict(mode=mode, x=0., y=y, z=0., nx=0., ny=0., nz=1.,
                         tx=0., ty=1., tz=0., force_n=0., offset_m=.5, speed_mps=.02))
    return attach_plan_hash(dict(
        version=3, process_mode="spray", frame_id="link0", path_id="path-1",
        work_area_id="area-1", plane_generation_id="zed:catalog:plane:1000000000",
        point_semantics="surface_point", contact_geometry_offset_m=0.,
        precontact_clearance_m=.5, travel_clearance_m=.5,
        safety_approach_offset_m=.5, final_retreat_offset_m=.5,
        tcp_normal_axis="-y", preserve_orientation_continuity=True, rows=rows,
        source=dict(plane="zed", view="wall_front", coverage="auto_fill", selection_id="2000000000",
                    work_area_id="area-1", plane_generation_id="zed:catalog:plane:1000000000", **metadata),
        **metadata,
    ))


def parse(payload):
    return parse_segment_path(payload, default_contact_offset_m=.026,
                              max_force_n=20., minimum_clearance_m=.005, allow_legacy=False)


@pytest.mark.parametrize("model,axis", [("rb20_1900es", "+z"), ("rb10_1300e_u", "-y")])
def test_model_axis_resolution(model, axis):
    assert resolve_spray_tool_axis(model, "") == axis
    assert resolve_spray_tool_axis(model, "+x") == "+x"


@pytest.mark.parametrize("model,override", [("unknown", ""), ("rb10_1300e_u", "z"),
                                           ("rb20_1900es", None)])
def test_unknown_or_invalid_axis_rejected(model, override):
    with pytest.raises(ValueError):
        resolve_spray_tool_axis(model, override)


@pytest.mark.parametrize("axis", ["+x", "-x", "+y", "-y", "+z", "-z"])
@pytest.mark.parametrize("normal", [(0., 0., 1.), (1., 2., 3.), (-1., 0., 0.)])
def test_signed_axis_points_inward_and_serpentine_does_not_flip(axis, normal):
    normal = np.asarray(normal) / np.linalg.norm(normal)
    tangent = np.cross(normal, [0., 1., 0.])
    first = rotation_from_spray_path(normal, tangent, axis)
    second = rotation_from_spray_path(normal, -tangent, axis, first[:, 0])
    axis_index = "xyz".index(axis[1])
    sign = 1. if axis[0] == "+" else -1.
    np.testing.assert_allclose(first[:, axis_index] * sign, -normal, atol=1e-12)
    np.testing.assert_allclose(first.T @ first, np.eye(3), atol=1e-12)
    assert np.linalg.det(first) == pytest.approx(1.)
    np.testing.assert_allclose(first, second, atol=1e-12)


@pytest.mark.parametrize("normal,tangent", [([0, 0, 0], [1, 0, 0]),
    ([0, 0, 1], [0, 0, 2]), ([0, 0, float("nan")], [1, 0, 0])])
def test_bad_orientation_vectors_rejected(normal, tangent):
    with pytest.raises(ValueError):
        rotation_from_spray_path(normal, tangent, "+z")


def test_spray_tcp_standoff_has_no_roller_offset():
    path = parse(spray_payload())
    validate_segment_path_for_real_execution(path)
    assert path.spray_tool_axis == "-y"
    assert path.spray_standoff_m == .5
    for row in path.rows:
        np.testing.assert_allclose(segment_waypoint_position(path, row),
                                   np.asarray(row.position) + np.asarray(row.normal) * .5)


@pytest.mark.parametrize("field", SPRAY_METADATA_FIELDS)
def test_missing_spray_metadata_fails_closed_even_with_new_hash(field):
    payload = spray_payload()
    del payload[field]
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("field,value", [
    ("spray_tool_axis", ""), ("spray_tool_axis", "forward"),
    ("spray_footprint_width_m", 0.), ("spray_overlap", -0.1), ("spray_overlap", 1.),
    ("spray_spacing_m", 0.), ("spray_spacing_m", .1),
    ("spray_standoff_m", -1.), ("spray_speed_mps", 0.), ("spray_speed_mps", True),
])
def test_invalid_spray_metadata_rejected_after_rehash(field, value):
    payload = spray_payload()
    payload[field] = payload["source"][field] = value
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("field,value", [("plane", "d405_refined"), ("view", "zed_raw"),
    ("selection_id", ""), ("selection_id", "selection-1"), ("selection_id", "0"),
    ("coverage", "freehand"), ("work_area_id", "stale"),
    ("plane_generation_id", "old"), ("spray_standoff_m", .7)])
def test_wrong_source_identity_or_metadata_rejected(field, value):
    payload = spray_payload()
    payload["source"][field] = value
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("field", SPRAY_METADATA_FIELDS)
def test_spray_metadata_changes_are_bound_to_hash(field):
    payload = copy.deepcopy(spray_payload())
    value = payload[field]
    payload[field] = "changed" if isinstance(value, str) else (
        [value[0] + .01, *value[1:]] if isinstance(value, list) else value + .01)
    with pytest.raises(SegmentPathError, match="plan_hash mismatch"):
        parse(payload)


def test_nondefault_standoff_is_exact_and_roller_geometry_rejected():
    payload = spray_payload()
    payload["spray_standoff_m"] = payload["source"]["spray_standoff_m"] = .65
    for field in ("precontact_clearance_m", "travel_clearance_m", "safety_approach_offset_m",
                  "final_retreat_offset_m"):
        payload[field] = .65
    for row in payload["rows"]:
        row["offset_m"] = .65
    path = parse(attach_plan_hash(payload))
    assert segment_waypoint_position(path, path.rows[0])[2] == .65
    payload["contact_geometry_offset_m"] = .026
    with pytest.raises(SegmentPathError, match="roller"):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("width,overlap", [(0., .3), (.35, 1.), (float("nan"), .3),
                                          (.35, float("inf"))])
def test_invalid_spacing_rejected(width, overlap):
    with pytest.raises(ValueError):
        spray_spacing_m(width, overlap)


@pytest.mark.parametrize("field", ["spray_eoat_profile_sha256", "spray_endpoint_tcp_m"])
def test_eoat_contract_rejects_old_plans_and_source_mismatch(field):
    payload = spray_payload()
    del payload[field]
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))
    payload = spray_payload()
    del payload["source"][field]
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("field,value", [
    ("spray_eoat_profile_sha256", ""), ("spray_eoat_profile_sha256", "g" * 64),
    ("spray_eoat_profile_sha256", 123),
    ("spray_endpoint_tcp_m", [0., 0.]), ("spray_endpoint_tcp_m", [0., 0., 0., 0.]),
    ("spray_endpoint_tcp_m", [False, 0., 0.]),
    ("spray_endpoint_tcp_m", ["0", 0., 0.]), ("spray_endpoint_tcp_m", None),
])
def test_invalid_eoat_metadata_cannot_be_authorized_by_rehash(field, value):
    payload = spray_payload()
    payload[field] = payload["source"][field] = value
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_endpoint_rejected_even_without_hash_verification(bad):
    payload = spray_payload()
    payload["spray_endpoint_tcp_m"] = payload["source"]["spray_endpoint_tcp_m"] = [0., bad, 0.]
    with pytest.raises(SegmentPathError, match="finite"):
        parse_segment_path(payload, default_contact_offset_m=.026, max_force_n=20.,
                           minimum_clearance_m=.005, allow_legacy=False, verify_plan_hash=False)


def test_source_endpoint_bool_cannot_equal_a_numeric_zero():
    payload = spray_payload()
    payload["spray_endpoint_tcp_m"] = [0., -.18, .03]
    payload["source"]["spray_endpoint_tcp_m"] = [False, -.18, .03]
    with pytest.raises(SegmentPathError):
        parse(attach_plan_hash(payload))
