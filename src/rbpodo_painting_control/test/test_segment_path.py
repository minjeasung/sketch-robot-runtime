import json
import math
from dataclasses import replace

import pytest

from rbpodo_painting_control.segment_path import (
    SegmentPathError,
    attach_plan_hash,
    build_execution_steps,
    canonical_segment_json,
    compute_plan_hash,
    parse_segment_path,
    rotation_from_surface_path,
    segment_waypoint_position,
    transform_segment_path,
    validate_segment_path_for_real_execution,
)


def row(mode, *, x=0.0, force=0.0, offset=0.0, speed=0.01, tx=1.0):
    return {
        "mode": mode,
        "x": float(x),
        "y": 0.0,
        "z": 0.0,
        "nx": 0.0,
        "ny": 0.0,
        "nz": 1.0,
        "tx": float(tx),
        "ty": 0.0,
        "tz": 0.0,
        "force_n": float(force),
        "offset_m": float(offset),
        "speed_mps": float(speed),
    }


def legacy_parse(rows, warnings=None):
    return parse_segment_path(
        {
            "version": 2,
            "frame_id": "link0",
            "path_id": "42",
            "point_semantics": "surface_point",
            "contact_offset_m": 0.031,
            "rows": rows,
        },
        default_contact_offset_m=0.031,
        max_force_n=20.0,
        minimum_clearance_m=0.005,
        warn=None if warnings is None else warnings.append,
    )


def v3_payload(rows=None, **overrides):
    if rows is None:
        rows = [
            row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
            row("CONTACT_SEARCH", offset=0.005, speed=0.002),
            row("RAMP_UP", force=2.0, speed=0.0),
            row("PAINT", force=2.0, speed=0.02),
            row("RAMP_DOWN", speed=0.0),
            row("FINAL_RETRACT", offset=0.080, speed=0.01),
        ]
    payload = {
        "version": 3,
        "frame_id": "link0",
        "path_id": "path-42",
        "work_area_id": "work-7",
        "plane_generation_id": "plane-9",
        "point_semantics": "surface_point",
        "contact_geometry_offset_m": 0.026,
        "precontact_clearance_m": 0.005,
        "travel_clearance_m": 0.010,
        "safety_approach_offset_m": 0.080,
        "final_retreat_offset_m": 0.080,
        "tcp_normal_axis": "+y",
        "preserve_orientation_continuity": True,
        "rows": rows,
        "source": {"test": True},
    }
    payload.update(overrides)
    return attach_plan_hash(payload)


def parse_v3(payload=None):
    return parse_segment_path(
        payload or v3_payload(),
        default_contact_offset_m=0.026,
        max_force_n=20.0,
        minimum_clearance_m=0.005,
        allow_legacy=False,
    )


def test_v3_parse_and_strict_real_validation_succeeds():
    payload = v3_payload()
    path = parse_v3(payload)

    assert path.version == 3
    assert path.contact_geometry_offset_m == pytest.approx(0.026)
    assert path.precontact_clearance_m == pytest.approx(0.005)
    assert path.travel_clearance_m == pytest.approx(0.010)
    assert path.safety_approach_offset_m == pytest.approx(0.080)
    assert path.final_retreat_offset_m == pytest.approx(0.080)
    assert path.plan_hash == compute_plan_hash(payload)
    assert validate_segment_path_for_real_execution(
        path,
        expected_path_id="path-42",
        expected_plan_hash=payload["plan_hash"],
        expected_work_area_id="work-7",
        expected_plane_generation_id="plane-9",
    ) is path


@pytest.mark.parametrize(
    "rows,reason",
    (
        (
            [
                row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
                row("CONTACT_SEARCH", offset=0.005, speed=0.002),
                row("RAMP_UP", force=2.0, speed=0.0),
                row("PAINT", force=2.0, speed=0.02),
                row("RAMP_DOWN", speed=0.0),
                row("TRAVEL", x=0.1, offset=0.010),
                row("FINAL_RETRACT", x=0.1, offset=0.080),
            ],
            "followed immediately",
        ),
        (
            [
                row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
                row("CONTACT_SEARCH", offset=0.005, speed=0.002),
                row("RAMP_UP", force=2.0, speed=0.0),
                row("PAINT", force=2.0, speed=0.02),
                row("RAMP_DOWN", speed=0.0),
                row("FINAL_RETRACT", x=0.01, offset=0.080),
            ],
            "last PAINT surface point",
        ),
        (
            [
                row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
                row("CONTACT_SEARCH", offset=0.005, speed=0.002),
                row("RAMP_UP", force=2.0, speed=0.0),
                row("PAINT", force=2.0, speed=0.02),
                row("RAMP_DOWN", speed=0.0),
                row("FINAL_RETRACT", offset=0.081),
            ],
            "commissioned clearance",
        ),
    ),
)
def test_v3_contact_escape_must_be_immediate_normal_outward(rows, reason):
    path = parse_v3(v3_payload(rows))
    with pytest.raises(SegmentPathError, match=reason):
        validate_segment_path_for_real_execution(path)


def test_v3_contact_escape_rejects_groupable_second_retract_row():
    rows = [
        row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
        row("CONTACT_SEARCH", offset=0.005, speed=0.002),
        row("RAMP_UP", force=2.0, speed=0.0),
        row("PAINT", force=2.0, speed=0.02),
        row("RAMP_DOWN", speed=0.0),
        row("RETRACT", offset=0.010, speed=0.01),
        row("RETRACT", offset=0.010, speed=0.01),
        row("TRAVEL", x=0.1, offset=0.010, speed=0.01),
        row("APPROACH_PRECONTACT", x=0.1, offset=0.005, speed=0.005),
        row("CONTACT_SEARCH", x=0.1, offset=0.005, speed=0.002),
        row("RAMP_UP", x=0.1, force=2.0, speed=0.0),
        row("PAINT", x=0.1, force=2.0, speed=0.02),
        row("RAMP_DOWN", x=0.1, speed=0.0),
        row("FINAL_RETRACT", x=0.1, offset=0.080, speed=0.01),
    ]
    path = parse_v3(v3_payload(rows))
    with pytest.raises(SegmentPathError, match="followed by TRAVEL"):
        validate_segment_path_for_real_execution(path)


def test_v3_contact_escape_checks_actual_roller_center_tangent_motion():
    tilt_rad = math.radians(2.0)
    escape = row("FINAL_RETRACT", offset=0.080, speed=0.01)
    escape.update(
        nx=math.sin(tilt_rad),
        ny=0.0,
        nz=math.cos(tilt_rad),
    )
    rows = [
        row("APPROACH_PRECONTACT", offset=0.005, speed=0.005),
        row("CONTACT_SEARCH", offset=0.005, speed=0.002),
        row("RAMP_UP", force=2.0, speed=0.0),
        row("PAINT", force=2.0, speed=0.02),
        row("RAMP_DOWN", speed=0.0),
        escape,
    ]
    path = parse_v3(v3_payload(rows))
    with pytest.raises(SegmentPathError, match="roller-center motion"):
        validate_segment_path_for_real_execution(path)


def test_canonical_hash_sorts_keys_excludes_hash_and_fixes_float_precision():
    first = v3_payload()
    reordered = json.loads(json.dumps(first))
    reordered = {key: reordered[key] for key in reversed(list(reordered))}
    reordered["plan_hash"] = "f" * 64

    assert compute_plan_hash(first) == compute_plan_hash(reordered)
    canonical = canonical_segment_json(first)
    assert '"contact_geometry_offset_m":0.026000000' in canonical
    assert '"plan_hash"' not in canonical


def test_hash_rounds_beyond_contract_precision_deterministically():
    first = v3_payload(contact_geometry_offset_m=0.02600000001)
    second = v3_payload(contact_geometry_offset_m=0.02600000002)
    assert compute_plan_hash(first) == compute_plan_hash(second)


def test_v2_is_parseable_for_legacy_but_rejected_for_real_execution():
    path = legacy_parse([row("TRAVEL", offset=0.01)])
    assert path.version == 2
    with pytest.raises(SegmentPathError, match="legacy/dry-run only"):
        validate_segment_path_for_real_execution(path)
    with pytest.raises(SegmentPathError, match="legacy/dry-run only"):
        parse_segment_path(
            path.raw_payload,
            default_contact_offset_m=0.031,
            max_force_n=20.0,
            minimum_clearance_m=0.005,
            allow_legacy=False,
        )


def test_v3_missing_path_id_is_rejected():
    payload = v3_payload(path_id="")
    with pytest.raises(SegmentPathError, match="path_id is required"):
        parse_v3(payload)


@pytest.mark.parametrize("field", ["point_semantics", "tcp_normal_axis"])
def test_v3_requires_explicit_coordinate_contract_fields(field):
    payload = v3_payload()
    payload.pop(field)
    payload = attach_plan_hash(payload)
    with pytest.raises(SegmentPathError, match=field):
        parse_v3(payload)


def test_v3_rejects_ambiguous_legacy_contact_offset():
    payload = v3_payload(contact_offset_m=0.031)
    with pytest.raises(SegmentPathError, match="legacy-only"):
        parse_v3(payload)


def test_mismatched_plan_hash_is_rejected():
    payload = v3_payload()
    payload["rows"][3]["x"] = 0.2
    with pytest.raises(SegmentPathError, match="plan_hash mismatch"):
        parse_v3(payload)


def test_mismatched_plane_generation_is_rejected_for_real_execution():
    path = parse_v3()
    with pytest.raises(SegmentPathError, match="plane_generation_id mismatch"):
        validate_segment_path_for_real_execution(
            path, expected_plane_generation_id="plane-other"
        )


def test_strict_validator_rejects_object_mutated_away_from_hashed_payload():
    path = parse_v3()
    tampered = replace(path, plane_generation_id="plane-tampered")
    with pytest.raises(SegmentPathError, match="does not match"):
        validate_segment_path_for_real_execution(
            tampered, expected_plane_generation_id="plane-tampered"
        )


def test_invalid_normal_and_parallel_tangent_are_rejected():
    bad_normal = v3_payload()
    bad_normal["rows"][3].update(nx=0.0, ny=0.0, nz=0.0)
    bad_normal = attach_plan_hash(bad_normal)
    with pytest.raises(SegmentPathError, match="normal vector is zero"):
        parse_v3(bad_normal)

    bad_tangent = v3_payload()
    bad_tangent["rows"][3].update(tx=0.0, ty=0.0, tz=1.0)
    bad_tangent = attach_plan_hash(bad_tangent)
    with pytest.raises(SegmentPathError, match="parallel"):
        parse_v3(bad_tangent)


def test_v3_requires_contact_search_before_ramp_up():
    rows = [
        row("APPROACH_PRECONTACT", offset=0.005),
        row("RAMP_UP", force=2.0),
        row("PAINT", force=2.0),
        row("RAMP_DOWN"),
        row("FINAL_RETRACT", offset=0.080),
    ]
    with pytest.raises(SegmentPathError, match="requires CONTACT_SEARCH"):
        parse_v3(v3_payload(rows))


def test_strict_real_rejects_partial_path_without_final_retract():
    rows = [
        row("APPROACH_PRECONTACT", offset=0.005),
        row("CONTACT_SEARCH", offset=0.005),
        row("RAMP_UP", force=2.0),
        row("PAINT", force=2.0),
        row("RAMP_DOWN"),
        row("TRAVEL", offset=0.010),
    ]
    path = parse_v3(v3_payload(rows))
    with pytest.raises(SegmentPathError, match="end with FINAL_RETRACT"):
        validate_segment_path_for_real_execution(path)


def test_builds_motion_steps_from_safe_v3_process():
    path = parse_v3()
    steps = build_execution_steps(path.rows)
    assert [step.mode for step in steps] == [
        "APPROACH_PRECONTACT",
        "CONTACT_SEARCH",
        "RAMP_UP",
        "PAINT",
        "RAMP_DOWN",
        "FINAL_RETRACT",
    ]


def test_rejects_paint_without_ramp_up():
    with pytest.raises(SegmentPathError, match="requires a completed RAMP_UP"):
        legacy_parse([row("PAINT", force=2.0), row("RAMP_DOWN")])


def test_rejects_retract_before_ramp_down():
    with pytest.raises(SegmentPathError, match="requires RAMP_DOWN"):
        legacy_parse(
            [
                row("RAMP_UP", force=2.0),
                row("PAINT", force=2.0),
                row("RETRACT", offset=0.01),
            ]
        )


def test_forces_noncontact_wrench_to_zero_and_warns():
    warnings = []
    path = legacy_parse([row("TRAVEL", force=4.0, offset=0.01)], warnings)
    assert path.rows[0].force_n == 0.0
    assert any("forcing non-contact wrench to zero" in item for item in warnings)


def test_warns_for_small_legacy_noncontact_clearance():
    warnings = []
    legacy_parse([row("TRAVEL", offset=0.001)], warnings)
    assert any("below minimum" in item for item in warnings)


def test_segment_waypoint_position_separates_geometry_and_clearance():
    path = parse_v3()
    paint = next(item for item in path.rows if item.mode == "PAINT")
    approach = next(
        item for item in path.rows if item.mode == "APPROACH_PRECONTACT"
    )
    assert segment_waypoint_position(path, paint) == pytest.approx((0.0, 0.0, 0.026))
    assert segment_waypoint_position(path, approach) == pytest.approx(
        (0.0, 0.0, 0.031)
    )


def test_hashed_v3_path_cannot_be_transformed_after_generation():
    with pytest.raises(SegmentPathError, match="cannot be transformed"):
        transform_segment_path(
            parse_v3(),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            translation_xyz=(0.0, 0.0, 0.0),
            target_frame="World",
        )


def test_reversing_stroke_keeps_tcp_orientation_continuous():
    first = rotation_from_surface_path([0, 0, 1], [1, 0, 0])
    second = rotation_from_surface_path(
        [0, 0, 1], [-1, 0, 0], previous_tcp_x=first[:, 0]
    )
    assert math.isclose(float(first[:, 0] @ second[:, 0]), 1.0, abs_tol=1e-9)
    assert math.isclose(float(second[:, 1] @ [0, 0, 1]), 1.0, abs_tol=1e-9)
