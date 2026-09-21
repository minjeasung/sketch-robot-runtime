"""Non-contact spraying contract. Geometry is measured from the roller surface."""
import math
import numpy as np

SPRAY_MODES = {"SPRAY_APPROACH", "SPRAY", "SPRAY_TRAVEL", "SPRAY_FINISH"}
STANDOFF_M = 0.5


def make_spray_rows(strokes, normal, fallback, transform_point, transform_tangent,
                    row_factory, speed, travel_speed):
    rows = []
    for index, stroke in enumerate(strokes):
        tangent = np.asarray(stroke[-1]) - np.asarray(stroke[0])
        if np.linalg.norm(tangent) < 1e-8:
            raise ValueError("spraying requires a nonzero stroke")
        tangent = transform_tangent(tangent)
        start = transform_point(stroke[0])
        rows.append(row_factory("SPRAY_APPROACH" if index == 0 else "SPRAY_TRAVEL",
                                start, normal, tangent, 0., STANDOFF_M, travel_speed))
        for point in stroke:
            rows.append(row_factory("SPRAY", transform_point(point), normal,
                                    tangent, 0., STANDOFF_M, speed))
    rows.append(row_factory("SPRAY_FINISH", transform_point(strokes[-1][-1]), normal,
                            tangent, 0., STANDOFF_M, travel_speed))
    return rows


def validate_spray_path(path):
    from rbpodo_painting_control.segment_path import SegmentPathError
    def reject(reason):
        raise SegmentPathError(reason)
    if path.version != 3 or path.process_mode != "spray":
        reject("spray requires a version 3 spray plan")
    if not path.rows or path.rows[0].mode != "SPRAY_APPROACH" or path.rows[-1].mode != "SPRAY_FINISH":
        reject("spray plan must begin OFF and finish OFF")
    if any(abs(v-STANDOFF_M) > 1e-9 for v in
           (path.precontact_clearance_m, path.travel_clearance_m,
            path.safety_approach_offset_m, path.final_retreat_offset_m)):
        reject("spray clearances must be exactly 0.5 m")
    previous = None
    strokes = 0
    for index, row in enumerate(path.rows):
        if row.mode == "SPRAY_APPROACH" and index != 0:
            reject("spray approach is only valid at the beginning")
        if row.mode == "SPRAY_FINISH" and index != len(path.rows)-1:
            reject("spray finish is only valid at the end")
        if np.linalg.norm(np.asarray(row.normal)-path.rows[0].normal) > 1e-6:
            reject("one spray plan must use one measured surface normal")
        if row.mode not in SPRAY_MODES or row.force_n != 0.:
            reject("spray must not contain contact/force commands")
        if abs(row.offset_m-STANDOFF_M) > 1e-9 or not math.isfinite(row.speed_mps) or row.speed_mps <= 0:
            reject("spray requires positive speed and a 0.5 m standoff")
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
