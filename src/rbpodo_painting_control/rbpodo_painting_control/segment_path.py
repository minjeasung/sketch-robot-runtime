"""Shared schema and safety rules for segment-driven painting paths."""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import math
import re
from typing import Any, Callable, Iterable

import numpy as np


from rbpodo_painting_control.spray_path import SPRAY_MODES, validate_spray_path

SUPPORTED_MODES = SPRAY_MODES | {
    "IDLE",
    "APPROACH",
    "APPROACH_PRECONTACT",
    "CONTACT_SEARCH",
    "RAMP_UP",
    "CONTACT",
    "PAINT",
    "RAMP_DOWN",
    "RETRACT",
    "TRAVEL",
    "NONCONTACT",
    "DWELL",
    "FINISH_RETRACT",
    "FINAL_RETRACT",
    "ABORT",
}
CONTACT_MOTION_MODES = {"CONTACT", "PAINT"}
ZERO_FORCE_MOTION_MODES = {
    "APPROACH",
    "APPROACH_PRECONTACT",
    "CONTACT_SEARCH",
    "RETRACT",
    "TRAVEL",
    "NONCONTACT",
    "FINISH_RETRACT",
    "FINAL_RETRACT",
}
MOTION_MODES = CONTACT_MOTION_MODES | ZERO_FORCE_MOTION_MODES | SPRAY_MODES
ZERO_FORCE_MODES = ZERO_FORCE_MOTION_MODES | {"IDLE", "ABORT"}
CLEARANCE_MODES = {
    "RETRACT",
    "TRAVEL",
    "NONCONTACT",
    "FINISH_RETRACT",
    "FINAL_RETRACT",
}

SEGMENT_SCHEMA_VERSION = 3
CANONICAL_FLOAT_PRECISION = 9
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class SegmentPathError(ValueError):
    """Raised when a process path cannot be executed safely."""


@dataclass(frozen=True)
class SegmentWaypoint:
    mode: str
    position: tuple[float, float, float]
    normal: tuple[float, float, float]
    tangent: tuple[float, float, float]
    force_n: float
    offset_m: float
    speed_mps: float
    row_number: int


@dataclass(frozen=True)
class SegmentPath:
    version: int
    frame_id: str
    path_id: str
    contact_offset_m: float
    rows: tuple[SegmentWaypoint, ...]
    source: dict[str, Any]
    point_semantics: str = "surface_point"
    plan_hash: str = ""
    work_area_id: str = ""
    plane_generation_id: str = ""
    contact_geometry_offset_m: float = 0.0
    precontact_clearance_m: float = 0.0
    travel_clearance_m: float = 0.0
    safety_approach_offset_m: float = 0.0
    final_retreat_offset_m: float = 0.0
    tcp_normal_axis: str = "+y"
    preserve_orientation_continuity: bool = True
    raw_payload: dict[str, Any] | None = None
    process_mode: str = "paint"


@dataclass(frozen=True)
class ExecutionStep:
    mode: str
    rows: tuple[SegmentWaypoint, ...]
    force_n: float
    speed_mps: float

    @property
    def is_motion(self) -> bool:
        return self.mode in MOTION_MODES


def _canonical_json_value(value: Any, float_precision: int) -> str:
    """Serialize JSON deterministically while keeping floats as JSON numbers.

    Python's default JSON encoder shortens floats (for example, ``1.0``), so it
    does not provide the fixed numeric representation required by the painting
    plan contract. Integers remain integers; every finite float is emitted with
    exactly ``float_precision`` digits after the decimal point. Negative zero is
    normalized to positive zero.
    """

    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    if isinstance(value, (int, np.integer)) and not isinstance(value, bool):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if not math.isfinite(number):
            raise SegmentPathError("canonical segment JSON contains a non-finite float")
        threshold = 0.5 * (10.0 ** (-float_precision))
        if abs(number) < threshold:
            number = 0.0
        return format(number, f".{float_precision}f")
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(
            _canonical_json_value(item, float_precision) for item in value
        ) + "]"
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise SegmentPathError("canonical segment JSON object keys must be strings")
        parts = []
        for key in sorted(value):
            encoded_key = json.dumps(key, ensure_ascii=False, separators=(",", ":"))
            parts.append(
                encoded_key + ":" + _canonical_json_value(value[key], float_precision)
            )
        return "{" + ",".join(parts) + "}"
    raise SegmentPathError(
        f"canonical segment JSON contains unsupported type: {type(value).__name__}"
    )


def canonical_segment_json(
    payload: str | dict[str, Any],
    *,
    exclude_plan_hash: bool = True,
    float_precision: int = CANONICAL_FLOAT_PRECISION,
) -> str:
    """Return the canonical JSON representation used for the SHA-256 plan hash.

    The root ``plan_hash`` member is intentionally excluded from its own digest.
    This exclusion is part of the schema contract and avoids a circular hash.
    """

    if isinstance(payload, str):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise SegmentPathError(f"invalid segment JSON: {exc}") from exc
    elif isinstance(payload, dict):
        data = payload
    else:
        raise SegmentPathError("segment payload must be JSON text or an object")
    if not isinstance(data, dict):
        raise SegmentPathError("segment payload root must be an object")
    if not isinstance(float_precision, int) or float_precision < 0:
        raise SegmentPathError("float_precision must be a non-negative integer")
    canonical_data = dict(data)
    if exclude_plan_hash:
        canonical_data.pop("plan_hash", None)
    return _canonical_json_value(canonical_data, float_precision)


def compute_plan_hash(payload: str | dict[str, Any]) -> str:
    """Compute the v3 plan SHA-256 over canonical JSON without ``plan_hash``."""

    canonical = canonical_segment_json(payload, exclude_plan_hash=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def attach_plan_hash(payload: dict[str, Any]) -> dict[str, Any]:
    """Return a shallow copy of ``payload`` with its canonical plan hash set."""

    result = dict(payload)
    result.pop("plan_hash", None)
    result["plan_hash"] = compute_plan_hash(result)
    return result


def _finite_float(value: Any, field: str, row_number: int) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SegmentPathError(
            f"row {row_number}: {field} must be numeric"
        ) from exc
    if not math.isfinite(result):
        raise SegmentPathError(f"row {row_number}: {field} must be finite")
    return result


def _normalized_vector(
    values: Iterable[Any], field: str, row_number: int
) -> np.ndarray:
    vector = np.asarray(
        [_finite_float(v, field, row_number) for v in values], dtype=float
    )
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise SegmentPathError(f"row {row_number}: {field} vector is zero")
    return vector / norm


def parse_segment_path(
    payload: str | dict[str, Any],
    *,
    default_contact_offset_m: float,
    max_force_n: float,
    minimum_clearance_m: float,
    warn: Callable[[str], None] | None = None,
    allow_legacy: bool = True,
    verify_plan_hash: bool = True,
) -> SegmentPath:
    """Parse and validate the versioned JSON process-path contract.

    Non-contact force is forced to zero after warning. Structural errors and
    unsafe force-mode ordering are rejected before any robot goal is sent.
    """

    if isinstance(payload, str):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise SegmentPathError(f"invalid segment JSON: {exc}") from exc
    elif isinstance(payload, dict):
        data = payload
    else:
        raise SegmentPathError("segment payload must be JSON text or an object")

    if not isinstance(data, dict):
        raise SegmentPathError("segment payload root must be an object")
    try:
        version = int(data.get("version", 1))
    except (TypeError, ValueError) as exc:
        raise SegmentPathError("version must be an integer") from exc
    if version not in (1, 2, SEGMENT_SCHEMA_VERSION):
        raise SegmentPathError(f"unsupported segment path version: {version}")
    if version < SEGMENT_SCHEMA_VERSION and not allow_legacy:
        raise SegmentPathError(
            f"segment schema version {version} is legacy/dry-run only; "
            f"real execution requires version {SEGMENT_SCHEMA_VERSION}"
        )

    frame_id = str(data.get("frame_id", "")).strip()
    if not frame_id:
        raise SegmentPathError("frame_id is required")
    path_id = str(data.get("path_id", "")).strip()
    point_semantics = str(
        data.get("point_semantics", "surface_point")
    ).strip().lower()
    if point_semantics != "surface_point":
        raise SegmentPathError(
            "only point_semantics='surface_point' is supported"
        )

    plan_hash = ""
    work_area_id = ""
    plane_generation_id = ""
    precontact_clearance_m = 0.0
    travel_clearance_m = max(0.0, float(minimum_clearance_m))
    safety_approach_offset_m = travel_clearance_m
    final_retreat_offset_m = travel_clearance_m
    tcp_normal_axis = str(data.get("tcp_normal_axis", "+y")).strip().lower()
    preserve_orientation_continuity = bool(
        data.get("preserve_orientation_continuity", True)
    )

    if version == SEGMENT_SCHEMA_VERSION:
        for field in ("point_semantics", "tcp_normal_axis"):
            if field not in data:
                raise SegmentPathError(
                    f"{field} is required for segment schema version 3"
                )
        if "contact_offset_m" in data:
            raise SegmentPathError(
                "contact_offset_m is legacy-only; version 3 requires "
                "contact_geometry_offset_m and separate clearances"
            )
        if not path_id:
            raise SegmentPathError("path_id is required for segment schema version 3")
        work_area_id = str(data.get("work_area_id", "")).strip()
        if not work_area_id:
            raise SegmentPathError("work_area_id is required for segment schema version 3")
        plane_generation_id = str(data.get("plane_generation_id", "")).strip()
        if not plane_generation_id:
            raise SegmentPathError(
                "plane_generation_id is required for segment schema version 3"
            )
        contact_geometry_offset_m = _finite_float(
            data.get("contact_geometry_offset_m"),
            "contact_geometry_offset_m",
            0,
        )
        precontact_clearance_m = _finite_float(
            data.get("precontact_clearance_m"),
            "precontact_clearance_m",
            0,
        )
        travel_clearance_m = _finite_float(
            data.get("travel_clearance_m"),
            "travel_clearance_m",
            0,
        )
        safety_approach_offset_m = _finite_float(
            data.get("safety_approach_offset_m"),
            "safety_approach_offset_m",
            0,
        )
        final_retreat_offset_m = _finite_float(
            data.get("final_retreat_offset_m", travel_clearance_m),
            "final_retreat_offset_m",
            0,
        )
        for field, value in (
            ("contact_geometry_offset_m", contact_geometry_offset_m),
            ("precontact_clearance_m", precontact_clearance_m),
            ("travel_clearance_m", travel_clearance_m),
            ("safety_approach_offset_m", safety_approach_offset_m),
            ("final_retreat_offset_m", final_retreat_offset_m),
        ):
            if value < 0.0:
                raise SegmentPathError(f"{field} must be non-negative")
        if tcp_normal_axis != "+y":
            raise SegmentPathError("tcp_normal_axis must be '+y'")
        if data.get("preserve_orientation_continuity") is not True:
            raise SegmentPathError(
                "preserve_orientation_continuity must be true for version 3"
            )
        preserve_orientation_continuity = True
        plan_hash = str(data.get("plan_hash", "")).strip().lower()
        if not _SHA256_RE.fullmatch(plan_hash):
            raise SegmentPathError("plan_hash must be a lowercase SHA-256 hex digest")
        if verify_plan_hash:
            calculated = compute_plan_hash(data)
            if plan_hash != calculated:
                raise SegmentPathError(
                    f"plan_hash mismatch: payload={plan_hash}, calculated={calculated}"
                )
        contact_offset_m = contact_geometry_offset_m
    else:
        contact_offset_m = _finite_float(
            data.get("contact_offset_m", default_contact_offset_m),
            "contact_offset_m",
            0,
        )
        if contact_offset_m < 0.0:
            raise SegmentPathError("contact_offset_m must be non-negative")
        contact_geometry_offset_m = contact_offset_m

    raw_rows = data.get("rows")
    if not isinstance(raw_rows, list) or not raw_rows:
        raise SegmentPathError("rows must be a non-empty list")

    rows: list[SegmentWaypoint] = []
    max_force_n = max(0.0, float(max_force_n))
    minimum_clearance_m = max(
        0.0,
        float(minimum_clearance_m),
        travel_clearance_m if version == SEGMENT_SCHEMA_VERSION else 0.0,
    )
    for row_number, raw in enumerate(raw_rows, start=1):
        if not isinstance(raw, dict):
            raise SegmentPathError(f"row {row_number}: expected an object")
        mode = str(raw.get("mode", "")).strip().upper()
        if mode not in SUPPORTED_MODES:
            raise SegmentPathError(f"row {row_number}: unsupported mode {mode!r}")

        position = tuple(
            _finite_float(raw.get(key), key, row_number)
            for key in ("x", "y", "z")
        )
        normal = _normalized_vector(
            (raw.get("nx"), raw.get("ny"), raw.get("nz")),
            "normal",
            row_number,
        )
        tangent = _normalized_vector(
            (raw.get("tx"), raw.get("ty"), raw.get("tz")),
            "tangent",
            row_number,
        )
        tangent = tangent - normal * float(np.dot(tangent, normal))
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm < 1e-8:
            raise SegmentPathError(
                f"row {row_number}: tangent is parallel to surface normal"
            )
        tangent /= tangent_norm

        force_n = _finite_float(raw.get("force_n", 0.0), "force_n", row_number)
        offset_m = _finite_float(raw.get("offset_m", 0.0), "offset_m", row_number)
        speed_mps = _finite_float(
            raw.get("speed_mps", 0.0), "speed_mps", row_number
        )
        if force_n < 0.0:
            raise SegmentPathError(
                f"row {row_number}: force_n is a positive magnitude"
            )
        if force_n > max_force_n + 1e-9:
            raise SegmentPathError(
                f"row {row_number}: force_n={force_n:.3f} exceeds "
                f"max_force_n={max_force_n:.3f}"
            )
        if speed_mps < 0.0:
            raise SegmentPathError(f"row {row_number}: speed_mps must be non-negative")
        if mode in ZERO_FORCE_MODES and force_n > 1e-9:
            if warn is not None:
                warn(
                    f"row {row_number} {mode} requested {force_n:.3f} N; "
                    "forcing non-contact wrench to zero"
                )
            force_n = 0.0
        if mode in CLEARANCE_MODES and offset_m < minimum_clearance_m:
            if warn is not None:
                warn(
                    f"row {row_number} {mode} clearance {offset_m:.4f} m is "
                    f"below minimum {minimum_clearance_m:.4f} m"
                )

        rows.append(
            SegmentWaypoint(
                mode=mode,
                position=position,
                normal=tuple(float(v) for v in normal),
                tangent=tuple(float(v) for v in tangent),
                force_n=force_n,
                offset_m=offset_m,
                speed_mps=speed_mps,
                row_number=row_number,
            )
        )

    process_mode = str(data.get("process_mode", "paint"))
    if process_mode not in {"paint", "spray"}:
        raise SegmentPathError("invalid process_mode")
    if process_mode == "paint" and any(r.mode in SPRAY_MODES for r in rows):
        raise SegmentPathError("spray rows require process_mode=spray")
    if process_mode == "spray" and any(float(r.get("force_n", 0)) != 0 for r in raw_rows):
        raise SegmentPathError("spray force must be zero")
    _validate_force_sequence(
        rows,
        require_contact_search=version == SEGMENT_SCHEMA_VERSION,
    )
    source = data.get("source", {})
    if not isinstance(source, dict):
        source = {"value": source}
    path = SegmentPath(
        version=version,
        frame_id=frame_id,
        path_id=path_id,
        contact_offset_m=contact_offset_m,
        rows=tuple(rows),
        source=source,
        point_semantics=point_semantics,
        plan_hash=plan_hash,
        work_area_id=work_area_id,
        plane_generation_id=plane_generation_id,
        contact_geometry_offset_m=contact_geometry_offset_m,
        precontact_clearance_m=precontact_clearance_m,
        travel_clearance_m=travel_clearance_m,
        safety_approach_offset_m=safety_approach_offset_m,
        final_retreat_offset_m=final_retreat_offset_m,
        tcp_normal_axis=tcp_normal_axis,
        preserve_orientation_continuity=preserve_orientation_continuity,
        raw_payload=dict(data),
        process_mode=process_mode,
    )
    if process_mode == "spray":
        validate_spray_path(path)
    return path


def _validate_force_sequence(
    rows: Iterable[SegmentWaypoint], *, require_contact_search: bool = False
) -> None:
    force_ready = False
    precontact_ready = False
    contact_search_complete = False
    for row in rows:
        if row.mode == "APPROACH_PRECONTACT":
            if force_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: APPROACH_PRECONTACT requires RAMP_DOWN"
                )
            precontact_ready = True
            contact_search_complete = False
            continue
        if row.mode == "CONTACT_SEARCH":
            if force_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: CONTACT_SEARCH requires RAMP_DOWN"
                )
            if not precontact_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: CONTACT_SEARCH requires "
                    "APPROACH_PRECONTACT"
                )
            contact_search_complete = True
            continue
        if row.mode == "RAMP_UP":
            if force_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: duplicate RAMP_UP before RAMP_DOWN"
                )
            if require_contact_search and not contact_search_complete:
                raise SegmentPathError(
                    f"row {row.row_number}: RAMP_UP requires CONTACT_SEARCH"
                )
            force_ready = True
            continue
        if row.mode in CONTACT_MOTION_MODES:
            if not force_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: {row.mode} requires a completed "
                    "RAMP_UP"
                )
            continue
        if row.mode == "RAMP_DOWN":
            if not force_ready:
                raise SegmentPathError(
                    f"row {row.row_number}: RAMP_DOWN requires an active RAMP_UP"
                )
            force_ready = False
            continue
        if row.mode in ZERO_FORCE_MOTION_MODES and force_ready:
            raise SegmentPathError(
                f"row {row.row_number}: {row.mode} requires RAMP_DOWN before "
                "non-contact motion"
            )
        if row.mode in {"RETRACT", "TRAVEL", "NONCONTACT", "FINAL_RETRACT"}:
            precontact_ready = False
            contact_search_complete = False
        if row.mode == "ABORT":
            force_ready = False
            precontact_ready = False
            contact_search_complete = False
    if force_ready:
        raise SegmentPathError("path ends before RAMP_DOWN completes")


def validate_segment_path_for_real_execution(
    path: SegmentPath,
    *,
    expected_path_id: str | None = None,
    expected_plan_hash: str | None = None,
    expected_work_area_id: str | None = None,
    expected_plane_generation_id: str | None = None,
) -> SegmentPath:
    """Validate all fail-closed invariants required before real robot motion.

    Parsing legacy payloads remains available for visualization and dry-run, but
    this validator deliberately accepts only a complete, self-consistent v3
    process path.
    """

    if path.version != SEGMENT_SCHEMA_VERSION:
        raise SegmentPathError(
            f"segment schema version {path.version} is legacy/dry-run only; "
            f"real execution requires version {SEGMENT_SCHEMA_VERSION}"
        )
    for field, value in (
        ("path_id", path.path_id),
        ("plan_hash", path.plan_hash),
        ("work_area_id", path.work_area_id),
        ("plane_generation_id", path.plane_generation_id),
    ):
        if not str(value).strip():
            raise SegmentPathError(f"{field} is required for real execution")
    if path.raw_payload is None:
        raise SegmentPathError("raw payload is required to verify plan_hash")
    calculated = compute_plan_hash(path.raw_payload)
    if calculated != path.plan_hash:
        raise SegmentPathError(
            f"plan_hash mismatch: payload={path.plan_hash}, calculated={calculated}"
        )
    raw_force_cap = max(
        [0.0]
        + [
            float(raw.get("force_n", 0.0))
            for raw in path.raw_payload.get("rows", [])
            if isinstance(raw, dict)
            and isinstance(raw.get("force_n", 0.0), (int, float))
            and math.isfinite(float(raw.get("force_n", 0.0)))
        ]
    )
    reparsed = parse_segment_path(
        path.raw_payload,
        default_contact_offset_m=path.contact_geometry_offset_m,
        max_force_n=raw_force_cap,
        minimum_clearance_m=path.travel_clearance_m,
        allow_legacy=False,
        verify_plan_hash=True,
    )
    if reparsed != path:
        raise SegmentPathError(
            "segment path object does not match its hashed raw payload"
        )

    expected_values = (
        ("path_id", expected_path_id, path.path_id),
        ("plan_hash", expected_plan_hash, path.plan_hash),
        ("work_area_id", expected_work_area_id, path.work_area_id),
        (
            "plane_generation_id",
            expected_plane_generation_id,
            path.plane_generation_id,
        ),
    )
    for field, expected, actual in expected_values:
        if expected is not None and str(expected).strip() != str(actual).strip():
            raise SegmentPathError(
                f"{field} mismatch: expected={expected!r}, payload={actual!r}"
            )

    if path.process_mode == "spray":
        return validate_spray_path(path)

    modes = [row.mode for row in path.rows]
    if not modes or modes[0] != "APPROACH_PRECONTACT":
        raise SegmentPathError(
            "real path must begin with APPROACH_PRECONTACT"
        )
    if modes[-1] != "FINAL_RETRACT":
        raise SegmentPathError("real path must end with FINAL_RETRACT")
    if "CONTACT_SEARCH" not in modes:
        raise SegmentPathError("real path requires CONTACT_SEARCH")
    if "PAINT" not in modes:
        raise SegmentPathError("real path requires at least one PAINT row")
    if path.contact_geometry_offset_m <= 0.0:
        raise SegmentPathError(
            "contact_geometry_offset_m must be positive for real execution"
        )
    if path.precontact_clearance_m <= 0.0:
        raise SegmentPathError(
            "precontact_clearance_m must be positive for real execution"
        )
    if path.travel_clearance_m + 1e-9 < path.precontact_clearance_m:
        raise SegmentPathError(
            "travel_clearance_m must be at least precontact_clearance_m"
        )
    if path.safety_approach_offset_m + 1e-9 < path.precontact_clearance_m:
        raise SegmentPathError(
            "safety_approach_offset_m must be at least precontact_clearance_m"
        )
    if path.final_retreat_offset_m + 1e-9 < path.travel_clearance_m:
        raise SegmentPathError(
            "final_retreat_offset_m must be at least travel_clearance_m"
        )

    for row in path.rows:
        if row.offset_m < -1e-9:
            raise SegmentPathError(
                f"row {row.row_number}: offset_m must be non-negative"
            )
        if row.mode == "PAINT" and abs(row.offset_m) > 1e-9:
            raise SegmentPathError(
                f"row {row.row_number}: PAINT offset_m must be zero"
            )
        if row.mode in {"APPROACH_PRECONTACT", "CONTACT_SEARCH"} and (
            row.offset_m + 1e-9 < path.precontact_clearance_m
        ):
            raise SegmentPathError(
                f"row {row.row_number}: {row.mode} offset_m is below "
                "precontact_clearance_m"
            )
        if row.mode in {"RETRACT", "TRAVEL", "FINAL_RETRACT"} and (
            row.offset_m + 1e-9 < path.travel_clearance_m
        ):
            raise SegmentPathError(
                f"row {row.row_number}: {row.mode} offset_m is below "
                "travel_clearance_m"
            )
        if row.mode == "FINAL_RETRACT" and (
            row.offset_m + 1e-9 < path.final_retreat_offset_m
        ):
            raise SegmentPathError(
                f"row {row.row_number}: FINAL_RETRACT offset_m is below "
                "final_retreat_offset_m"
            )

    # A collision exception for the roller/target pair is intentionally open
    # during PAINT.  The only motion permitted before that exception is
    # restored is a pure normal-outward escape at the same surface point.
    # Enforce that contract in the immutable v3 payload rather than trusting a
    # mode label supplied by a caller.
    rows = tuple(path.rows)
    last_paint: SegmentWaypoint | None = None

    def _unit(values: tuple[float, float, float]) -> np.ndarray:
        vector = np.asarray(values, dtype=float)
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm <= 1e-9:
            raise SegmentPathError("contact escape vector is invalid")
        return vector / norm

    for index, row in enumerate(rows):
        if row.mode == "PAINT":
            last_paint = row
            continue
        if row.mode != "RAMP_DOWN":
            continue
        if last_paint is None:
            raise SegmentPathError(
                f"row {row.row_number}: RAMP_DOWN has no preceding PAINT"
            )
        if index == 0 or rows[index - 1].mode != "PAINT":
            raise SegmentPathError(
                f"row {row.row_number}: RAMP_DOWN must immediately follow PAINT"
            )
        if index + 1 >= len(rows):
            raise SegmentPathError(
                f"row {row.row_number}: RAMP_DOWN must be followed by RETRACT"
            )
        escape = rows[index + 1]
        if escape.mode not in {"RETRACT", "FINAL_RETRACT"}:
            raise SegmentPathError(
                f"row {row.row_number}: RAMP_DOWN must be followed immediately "
                "by RETRACT or FINAL_RETRACT"
            )

        paint_point = np.asarray(last_paint.position, dtype=float)
        ramp_point = np.asarray(row.position, dtype=float)
        escape_point = np.asarray(escape.position, dtype=float)
        if float(np.linalg.norm(ramp_point - paint_point)) > 0.001:
            raise SegmentPathError(
                f"row {row.row_number}: RAMP_DOWN moved away from the PAINT "
                "surface point"
            )
        if float(np.linalg.norm(escape_point - paint_point)) > 0.001:
            raise SegmentPathError(
                f"row {escape.row_number}: contact RETRACT must stay at the "
                "last PAINT surface point"
            )
        paint_normal = _unit(last_paint.normal)
        paint_tangent = _unit(last_paint.tangent)
        for candidate in (row, escape):
            if float(np.dot(_unit(candidate.normal), paint_normal)) < 0.999:
                raise SegmentPathError(
                    f"row {candidate.row_number}: contact escape normal changed"
                )
            if float(np.dot(_unit(candidate.tangent), paint_tangent)) < 0.999:
                raise SegmentPathError(
                    f"row {candidate.row_number}: contact escape tangent changed"
                )

        expected_offset = (
            path.final_retreat_offset_m
            if escape.mode == "FINAL_RETRACT"
            else path.travel_clearance_m
        )
        if abs(float(escape.offset_m) - float(expected_offset)) > 1e-6:
            raise SegmentPathError(
                f"row {escape.row_number}: {escape.mode} offset must equal "
                f"the commissioned clearance {expected_offset:.6f}m"
            )

        # Match the runtime contact-escape gate before any physical motion.
        # Small, individually tolerated changes to the surface point/normal can
        # otherwise combine over the roller-center offset into a tangential
        # motion larger than the 2 mm escape envelope.  Validate the actual
        # planned roller-center displacement here, not just the raw row fields.
        escape_normal = _unit(escape.normal)
        paint_pose = paint_point + paint_normal * (
            float(path.contact_geometry_offset_m) + float(last_paint.offset_m)
        )
        escape_pose = escape_point + escape_normal * (
            float(path.contact_geometry_offset_m) + float(escape.offset_m)
        )
        escape_delta = escape_pose - paint_pose
        outward = float(np.dot(escape_delta, paint_normal))
        tangent_delta = float(
            np.linalg.norm(escape_delta - paint_normal * outward)
        )
        if (
            not math.isfinite(outward)
            or not math.isfinite(tangent_delta)
            or outward < float(path.travel_clearance_m) - 0.002
            or abs(outward - float(expected_offset)) > 0.003
            or tangent_delta > 0.002
        ):
            raise SegmentPathError(
                f"row {escape.row_number}: contact escape must be a bounded "
                "normal-outward roller-center motion"
            )
        if escape.mode == "FINAL_RETRACT" and index + 1 != len(rows) - 1:
            raise SegmentPathError(
                f"row {escape.row_number}: FINAL_RETRACT must end the path"
            )
        if escape.mode == "RETRACT" and (
            index + 2 >= len(rows) or rows[index + 2].mode != "TRAVEL"
        ):
            raise SegmentPathError(
                f"row {escape.row_number}: RETRACT must be followed by TRAVEL"
            )
        last_paint = None
    return path


def build_execution_steps(rows: Iterable[SegmentWaypoint]) -> tuple[ExecutionStep, ...]:
    """Group compatible waypoint rows without losing per-row force/speed changes."""

    steps: list[ExecutionStep] = []
    current: list[SegmentWaypoint] = []
    key: tuple[str, float, float] | None = None

    def flush() -> None:
        nonlocal current, key
        if not current:
            return
        first = current[0]
        positive_speeds = [row.speed_mps for row in current if row.speed_mps > 0.0]
        speed = min(positive_speeds) if positive_speeds else 0.0
        steps.append(
            ExecutionStep(first.mode, tuple(current), first.force_n, speed)
        )
        current = []
        key = None

    for row in rows:
        row_key = (row.mode, round(row.force_n, 9), round(row.speed_mps, 9))
        if row.mode not in MOTION_MODES:
            flush()
            steps.append(ExecutionStep(row.mode, (row,), row.force_n, row.speed_mps))
            continue
        if key is not None and row_key != key:
            flush()
        key = row_key
        current.append(row)
    flush()
    return tuple(steps)


def segment_waypoint_position(
    path: SegmentPath, row: SegmentWaypoint
) -> tuple[float, float, float]:
    """Return the roller-center pose used by planning and visualization."""

    normal = np.asarray(row.normal, dtype=float)
    point = np.asarray(row.position, dtype=float)
    position = point + normal * (
        float(path.contact_geometry_offset_m) + float(row.offset_m)
    )
    return tuple(float(value) for value in position)


def transform_segment_path(
    path: SegmentPath,
    rotation_xyzw: Iterable[float],
    translation_xyz: Iterable[float],
    target_frame: str,
) -> SegmentPath:
    """Rigidly transform surface points and direction vectors into another frame."""

    if path.version >= SEGMENT_SCHEMA_VERSION:
        raise SegmentPathError(
            "version 3 is a hashed final-frame path and cannot be transformed"
        )

    q = np.asarray(list(rotation_xyzw), dtype=float)
    q_norm = float(np.linalg.norm(q))
    if q_norm < 1e-9:
        raise SegmentPathError("frame transform quaternion is zero")
    q /= q_norm
    x, y, z, w = q
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    translation = np.asarray(list(translation_xyz), dtype=float)
    transformed = []
    for row in path.rows:
        position = rotation @ np.asarray(row.position) + translation
        normal = rotation @ np.asarray(row.normal)
        tangent = rotation @ np.asarray(row.tangent)
        transformed.append(
            replace(
                row,
                position=tuple(float(v) for v in position),
                normal=tuple(float(v) for v in normal),
                tangent=tuple(float(v) for v in tangent),
            )
        )
    return replace(path, frame_id=target_frame, rows=tuple(transformed))


def rotation_from_surface_path(
    normal: Iterable[float],
    motion_tangent: Iterable[float],
    previous_tcp_x: Iterable[float] | None = None,
) -> np.ndarray:
    """Return TCP rotation with +Y normal and a continuous roller +X axis.

    ``motion_tangent`` is the intended rolling direction on the surface. The
    roller's long TCP +X axis is perpendicular to it. Reversing a raster stroke
    therefore does not cause a 180-degree TCP flip.
    """

    tcp_y = np.asarray(normal, dtype=float)
    tcp_y /= np.linalg.norm(tcp_y) + 1e-12
    tangent = np.asarray(motion_tangent, dtype=float)
    tangent = tangent - tcp_y * float(np.dot(tangent, tcp_y))
    if float(np.linalg.norm(tangent)) < 1e-8:
        raise SegmentPathError("motion tangent is parallel to surface normal")
    tangent /= np.linalg.norm(tangent)

    tcp_x = np.cross(tcp_y, tangent)
    tcp_x /= np.linalg.norm(tcp_x) + 1e-12
    if previous_tcp_x is not None:
        previous = np.asarray(previous_tcp_x, dtype=float)
        previous = previous - tcp_y * float(np.dot(previous, tcp_y))
        if float(np.linalg.norm(previous)) > 1e-8:
            previous /= np.linalg.norm(previous)
            if float(np.dot(tcp_x, previous)) < 0.0:
                tcp_x = -tcp_x

    tcp_z = np.cross(tcp_x, tcp_y)
    tcp_z /= np.linalg.norm(tcp_z) + 1e-12
    tcp_x = np.cross(tcp_y, tcp_z)
    tcp_x /= np.linalg.norm(tcp_x) + 1e-12
    return np.column_stack([tcp_x, tcp_y, tcp_z])
