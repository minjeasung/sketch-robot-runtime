"""Non-contact spraying contract. Positions are measured from the surface."""
import math
import re
import numpy as np

from rbpodo_painting_control.spray_geometry import (
    resolve_spray_tool_axis,
    rotation_from_spray_path,
    spray_spacing_m,
)

SPRAY_MODES = {"SPRAY_APPROACH", "SPRAY", "SPRAY_TRAVEL", "SPRAY_FINISH"}
STANDOFF_M = 0.5
SPRAY_METADATA_FIELDS = (
    "model_id", "spray_tool_axis", "spray_footprint_width_m", "spray_overlap",
    "spray_spacing_m", "spray_speed_mps", "spray_standoff_m",
    "spray_eoat_profile_sha256", "spray_endpoint_tcp_m",
)


def make_spray_rows(strokes, normal, fallback, transform_point, transform_tangent,
                    row_factory, speed, travel_speed, standoff_m=STANDOFF_M):
    if not strokes:
        raise ValueError("spraying requires nonempty strokes")
    if any(not math.isfinite(float(v)) or float(v) <= 0.0
           for v in (speed, travel_speed, standoff_m)):
        raise ValueError("spray speeds and standoff must be finite and positive")
    rows = []
    for index, stroke in enumerate(strokes):
        tangent = np.asarray(stroke[-1]) - np.asarray(stroke[0])
        if np.linalg.norm(tangent) < 1e-8:
            raise ValueError("spraying requires a nonzero stroke")
        tangent = transform_tangent(tangent)
        start = transform_point(stroke[0])
        rows.append(row_factory("SPRAY_APPROACH" if index == 0 else "SPRAY_TRAVEL",
                                start, normal, tangent, 0., standoff_m, travel_speed))
        for point in stroke:
            rows.append(row_factory("SPRAY", transform_point(point), normal,
                                    tangent, 0., standoff_m, speed))
    rows.append(row_factory("SPRAY_FINISH", transform_point(strokes[-1][-1]), normal,
                            tangent, 0., standoff_m, travel_speed))
    return rows


def validate_spray_path(path):
    from rbpodo_painting_control.segment_path import SegmentPathError
    def reject(reason):
        raise SegmentPathError(reason)
    if path.version != 3 or path.process_mode != "spray":
        reject("spray requires a version 3 spray plan")
    payload = path.raw_payload or {}
    if any(field not in payload for field in SPRAY_METADATA_FIELDS):
        reject("spray plan requires complete spray metadata")
    if not isinstance(payload["model_id"], str) or not payload["model_id"].strip():
        reject("spray plan requires model_id")
    try:
        axis = resolve_spray_tool_axis(payload["model_id"], payload["spray_tool_axis"])
        if axis != payload["spray_tool_axis"]:
            reject("spray_tool_axis must be an explicit resolved signed TCP axis")
        digest = payload["spray_eoat_profile_sha256"]
        if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            reject("spray_eoat_profile_sha256 must be a SHA-256 digest")
        endpoint = payload["spray_endpoint_tcp_m"]
        if (not isinstance(endpoint, list) or len(endpoint) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in endpoint)):
            reject("spray_endpoint_tcp_m must contain three finite numbers")
        numeric = SPRAY_METADATA_FIELDS[2:7]
        if any(isinstance(payload[field], bool) or not isinstance(payload[field], (int, float))
               or not math.isfinite(payload[field]) for field in numeric):
            reject("spray metadata must contain finite numbers")
        spacing = spray_spacing_m(payload["spray_footprint_width_m"], payload["spray_overlap"])
        standoff = float(payload["spray_standoff_m"])
        speed = float(payload["spray_speed_mps"])
        if standoff <= 0.0 or speed <= 0.0 or payload["spray_spacing_m"] <= 0.0:
            reject("spray spacing, speed and standoff must be positive")
        if not math.isclose(spacing, payload["spray_spacing_m"], rel_tol=1e-8, abs_tol=1e-9):
            reject("spray spacing does not match footprint and overlap")
    except (TypeError, ValueError) as exc:
        reject(str(exc))
    if path.spray_tool_axis != axis or path.spray_standoff_m != standoff:
        reject("spray path metadata does not match raw payload")
    if path.tcp_normal_axis != axis:
        reject("spray tcp_normal_axis must match spray_tool_axis")
    if path.contact_geometry_offset_m != 0.0 or path.contact_offset_m != 0.0:
        reject("spray must not apply roller or brush geometry offsets")
    if (path.source.get("plane") != "zed" or path.source.get("view") != "wall_front"
            or path.source.get("coverage") != "auto_fill"
            or not path.plane_generation_id.startswith("zed:")
            or path.source.get("plane_generation_id") != path.plane_generation_id
            or path.source.get("work_area_id") != path.work_area_id
            or not isinstance(path.source.get("selection_id"), str)
            or not path.source["selection_id"].strip()):
        reject("spray requires matching selected ZED plane and work-area identities")
    selection_id = path.source["selection_id"]
    if not selection_id.isdecimal() or int(selection_id) <= 0 or str(int(selection_id)) != selection_id:
        reject("spray selection_id must be a positive pixel timestamp in nanoseconds")
    if any(path.source.get(field) != payload[field] for field in SPRAY_METADATA_FIELDS):
        reject("spray source metadata does not match the plan")
    source_endpoint = path.source["spray_endpoint_tcp_m"]
    if (not isinstance(source_endpoint, list)
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in source_endpoint)):
        reject("spray source endpoint must contain finite numbers")
    if not path.rows or path.rows[0].mode != "SPRAY_APPROACH" or path.rows[-1].mode != "SPRAY_FINISH":
        reject("spray plan must begin OFF and finish OFF")
    if any(not math.isfinite(v) or abs(v-standoff) > 1e-9 for v in
           (path.precontact_clearance_m, path.travel_clearance_m,
            path.safety_approach_offset_m, path.final_retreat_offset_m)):
        reject("spray clearances must match spray_standoff_m")
    previous = None
    strokes = 0
    for index, row in enumerate(path.rows):
        if any(not np.all(np.isfinite(vector)) or np.asarray(vector).shape != (3,)
               for vector in (row.position, row.normal, row.tangent)):
            reject("spray geometry must be finite three-vectors")
        if (abs(np.linalg.norm(row.normal) - 1.0) > 1e-6
                or abs(np.linalg.norm(row.tangent) - 1.0) > 1e-6
                or abs(np.dot(row.normal, row.tangent)) > 1e-6):
            reject("spray normal and tangent must be unit orthogonal vectors")
        if row.mode == "SPRAY_APPROACH" and index != 0:
            reject("spray approach is only valid at the beginning")
        if row.mode == "SPRAY_FINISH" and index != len(path.rows)-1:
            reject("spray finish is only valid at the end")
        if np.linalg.norm(np.asarray(row.normal)-path.rows[0].normal) > 1e-6:
            reject("one spray plan must use one measured surface normal")
        if row.mode not in SPRAY_MODES or row.force_n != 0.:
            reject("spray must not contain contact/force commands")
        if not math.isfinite(row.offset_m) or abs(row.offset_m-standoff) > 1e-9 or not math.isfinite(row.speed_mps) or row.speed_mps <= 0:
            reject("spray requires positive speed and the configured standoff")
        if row.mode == "SPRAY" and abs(row.speed_mps-speed) > 1e-9:
            reject("spray row speed must match spray_speed_mps")
        if row.mode == "SPRAY" and (previous is None or previous.mode != "SPRAY"):
            if previous is None or previous.mode not in {"SPRAY_APPROACH", "SPRAY_TRAVEL"}:
                reject("spray stroke requires an OFF approach")
            if np.linalg.norm(np.asarray(row.position)-previous.position) > 1e-8:
                reject("spray stroke must start at the OFF travel endpoint")
            strokes += 1
        if row.mode in {"SPRAY_TRAVEL", "SPRAY_FINISH"} and (previous is None or previous.mode != "SPRAY"):
            reject("spray travel/finish must follow a stroke")
        if row.mode == "SPRAY_FINISH" and np.linalg.norm(np.asarray(row.position)-previous.position) > 1e-8:
            reject("spray finish must remain at the final stroke endpoint")
        previous = row
    if not strokes:
        reject("empty spray path")
    return path
