"""Pure work-area geometry shared by path generation and tests.

The functions in this module deliberately have no ROS dependencies. Pixel
coordinates use the wall-front convention: ``u`` grows right and ``v`` grows
down. 3-D quadrilateral corners are ordered TL, TR, BR, BL.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np


class WorkAreaGeometryError(ValueError):
    """Raised when a selected work area cannot produce a safe path."""


PixelPoint = tuple[float, float]
PixelRect = tuple[float, float, float, float]
PixelStroke = tuple[PixelPoint, PixelPoint]


def _cross2(a, b):
    return a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]


def _edge_cuts(a, b, starts, ends):
    """Parameters where segment a->b meets any polygon edge, including overlap."""
    direction, edges = b-a, ends-starts
    length2 = float(direction @ direction)
    if length2 < 1e-16:
        return []
    delta = starts-a
    denominator = _cross2(direction, edges)
    nonparallel = abs(denominator) > 1e-10
    t = np.divide(_cross2(delta, edges), denominator,
                  out=np.zeros(len(edges)), where=nonparallel)
    u = np.divide(_cross2(delta, direction), denominator,
                  out=np.zeros(len(edges)), where=nonparallel)
    hits = nonparallel & (t >= -1e-9) & (t <= 1+1e-9) & (u >= -1e-9) & (u <= 1+1e-9)
    cuts = np.clip(t[hits], 0., 1.).tolist()
    collinear = ~nonparallel & (abs(_cross2(delta, direction)) < 1e-8)
    if np.any(collinear):
        first = (starts[collinear]-a) @ direction / length2
        last = (ends[collinear]-a) @ direction / length2
        lo, hi = np.minimum(first, last), np.maximum(first, last)
        overlaps = (hi >= -1e-9) & (lo <= 1+1e-9)
        cuts.extend(np.clip(np.r_[lo[overlaps], hi[overlaps]], 0., 1.).tolist())
    return cuts


def work_area_polygon(points, width, height):
    """Validate one simple boundary; legacy two-corner boxes remain supported."""
    polygon = np.asarray(points, dtype=float)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or not 2 <= len(polygon) <= 1024
            or not np.isfinite(polygon).all()):
        raise WorkAreaGeometryError("work area requires 2 to 1024 finite pixel vertices")
    if len(polygon) == 2:
        lo, hi = polygon.min(axis=0), polygon.max(axis=0)
        polygon = np.array([lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]])
    polygon = polygon[np.r_[True, np.linalg.norm(np.diff(polygon, axis=0), axis=1) > 1e-8]]
    if len(polygon) > 1 and np.linalg.norm(polygon[0]-polygon[-1]) < 1e-8:
        polygon = polygon[:-1]
    if (len(polygon) < 3 or width <= 1 or height <= 1 or np.any(polygon < 0)
            or np.any(polygon[:, 0] > width-1) or np.any(polygon[:, 1] > height-1)):
        raise WorkAreaGeometryError("work area must enclose a region inside Wall Front")
    ends = np.roll(polygon, -1, axis=0)
    if abs(float(np.sum(_cross2(polygon, ends)))) < 2.:
        raise WorkAreaGeometryError("work area has no usable enclosed area")
    for i, (a, b) in enumerate(zip(polygon, ends)):
        others = [j for j in range(len(polygon)) if j not in {i, (i-1) % len(polygon), (i+1) % len(polygon)}]
        if _edge_cuts(a, b, polygon[others], ends[others]):
            raise WorkAreaGeometryError("work area boundary crosses or touches itself")
        previous = polygon[i-1]-a
        if abs(_cross2(previous, b-a)) < 1e-8 and previous @ (b-a) > 0:
            raise WorkAreaGeometryError("work area boundary doubles back")
    return polygon


def _inside_polygon(point, polygon):
    ends = np.roll(polygon, -1, axis=0)
    edges, delta = ends-polygon, point-polygon
    length2 = np.sum(edges*edges, axis=1)
    t = np.clip(np.sum(delta*edges, axis=1)/length2, 0., 1.)
    if np.min(np.linalg.norm(delta-t[:, None]*edges, axis=1)) <= 1e-7:
        return True
    crossing = (polygon[:, 1] > point[1]) != (ends[:, 1] > point[1])
    p, q = polygon[crossing], ends[crossing]
    intersections = p[:, 0]+(point[1]-p[:, 1])*(q[:, 0]-p[:, 0])/(q[:, 1]-p[:, 1])
    return bool(np.count_nonzero(intersections > point[0]) % 2)


def _polygon_intervals(a, b, polygon):
    cuts = sorted(set([0., 1., *_edge_cuts(a, b, polygon, np.roll(polygon, -1, axis=0))]))
    return [(lo, hi) for lo, hi in zip(cuts, cuts[1:]) if hi-lo > 1e-10
            and _inside_polygon(a+((lo+hi)/2)*(b-a), polygon)]


def strokes_inside_polygon(strokes, polygon):
    """Check complete segments, including excursions through a concave gap."""
    boundary = np.asarray(polygon, dtype=float)
    for stroke in strokes:
        points = np.asarray(stroke, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2
                or not np.isfinite(points).all()
                or not all(_inside_polygon(p, boundary) for p in points)):
            return False
        for a, b in zip(points, points[1:]):
            if np.linalg.norm(b-a) < 1e-8:
                continue
            if sum(hi-lo for lo, hi in _polygon_intervals(a, b, boundary)) < 1-1e-8:
                return False
    return bool(strokes)


def clip_strokes_to_polygon(strokes, polygon):
    """Clip each fill pass; disconnected pieces become separate OFF-linked strokes."""
    boundary = np.asarray(polygon, dtype=float)
    result = []
    for stroke in strokes:
        a, b = np.asarray(stroke[0], dtype=float), np.asarray(stroke[-1], dtype=float)
        for lo, hi in _polygon_intervals(a, b, boundary):
            result.append((tuple(a+lo*(b-a)), tuple(a+hi*(b-a))))
    return result


def validate_zed_surface_status(payload):
    """Validate an atomic work-area geometry message before it can be cached."""
    if not isinstance(payload, dict) or (
        payload.get("source") != "zed" or payload.get("mode") != "work_area"
        or payload.get("accepted") is not True or payload.get("state") != "locked"
    ):
        raise WorkAreaGeometryError("accepted locked ZED work-area status is required")
    for field in ("plane_generation_id", "work_area_id", "selection_id", "frame_id"):
        if not isinstance(payload.get(field), str) or not payload[field].strip():
            raise WorkAreaGeometryError(f"ZED status requires {field}")
    if not payload["plane_generation_id"].startswith("zed:"):
        raise WorkAreaGeometryError("ZED status has a non-ZED plane generation")
    selection = payload["selection_id"]
    if not selection.isdecimal() or int(selection) <= 0 or str(int(selection)) != selection:
        raise WorkAreaGeometryError("selection_id must be a positive pixel timestamp in nanoseconds")
    stamp = payload.get("target_stamp")
    if (not isinstance(stamp, dict)
            or any(type(stamp.get(key)) is not int for key in ("sec", "nanosec"))
            or stamp["sec"] < 0 or not 0 <= stamp["nanosec"] < 1_000_000_000
            or stamp["sec"] * 1_000_000_000 + stamp["nanosec"] <= 0):
        raise WorkAreaGeometryError("ZED target_stamp is invalid")
    for field in ("view_width", "view_height"):
        if type(payload.get(field)) is not int or payload[field] <= 1:
            raise WorkAreaGeometryError(f"ZED {field} must be an integer greater than one")
    try:
        position = np.asarray(payload.get("position"), dtype=float)
        orientation = np.asarray(payload.get("orientation"), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise WorkAreaGeometryError("ZED position must be a finite three-vector")
        if (orientation.shape != (4,) or not np.all(np.isfinite(orientation))
                or abs(float(np.linalg.norm(orientation)) - 1.0) > 1e-5):
            raise WorkAreaGeometryError("ZED orientation must be a unit quaternion")
        x, y, z, w = orientation
        normal = np.array([2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)])
        for field in ("corners", "front_extent"):
            quad = np.asarray(payload.get(field), dtype=float)
            quad_size_m(quad)
            _quad_projection(quad)
            edges = np.roll(quad, -1, axis=0) - quad
            turns = np.cross(edges, np.roll(edges, -1, axis=0)) @ normal
            if not (np.all(turns > 1e-8) or np.all(turns < -1e-8)):
                raise WorkAreaGeometryError(f"ZED {field} must be convex and nondegenerate")
            if np.max(np.abs((quad - position) @ normal)) > 0.002:
                raise WorkAreaGeometryError(f"ZED {field} does not lie on the accepted plane")
        if np.linalg.norm(np.mean(payload["corners"], axis=0) - position) > 0.002:
            raise WorkAreaGeometryError("ZED position must be the work-area center")
        if outside_quad_3d_indices(payload["corners"], payload["front_extent"]):
            raise WorkAreaGeometryError("ZED work area is outside its front extent")
        if "boundary_pixels" in payload:
            boundary = work_area_polygon(payload["boundary_pixels"],
                                         payload["view_width"], payload["view_height"])
            lo, hi = boundary.min(axis=0), boundary.max(axis=0)
            envelope = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]]
            expected = [bilinear_quad_point(payload["front_extent"],
                u/(payload["view_width"]-1), v/(payload["view_height"]-1)) for u, v in envelope]
            if not np.allclose(expected, payload["corners"], atol=1e-6, rtol=0.):
                raise WorkAreaGeometryError("ZED boundary does not match its work-area envelope")
    except (TypeError, ValueError) as exc:
        raise WorkAreaGeometryError(str(exc)) from exc
    return payload


def generate_spray_fill_strokes(rect, *, work_area_width_m, work_area_height_m,
                               footprint_width_m, overlap):
    """Generate spray coverage, with one center stroke for a narrow area.

    The actual fan width is unchanged: a narrow area's spray footprint extends
    beyond its sides, while the nozzle center path stays inside the selection.
    """
    return generate_fill_strokes(
        rect, work_area_width_m=work_area_width_m,
        work_area_height_m=work_area_height_m,
        roller_length_m=footprint_width_m, overlap=overlap,
        allow_footprint_overhang=True,
    )


def _finite_point2(point: Sequence[float], label: str) -> PixelPoint:
    if len(point) < 2:
        raise WorkAreaGeometryError(f"{label} must have two coordinates")
    u, v = float(point[0]), float(point[1])
    if not math.isfinite(u) or not math.isfinite(v):
        raise WorkAreaGeometryError(f"{label} must be finite")
    return u, v


def pixel_rect_from_points(
    points: Iterable[Sequence[float]],
    image_width: int,
    image_height: int,
    *,
    boundary_tolerance_px: float = 1.0,
    minimum_side_px: float = 1.0,
) -> PixelRect:
    """Return an axis-aligned selected rectangle after validating image bounds."""

    width = int(image_width)
    height = int(image_height)
    if width <= 1 or height <= 1:
        raise WorkAreaGeometryError("wall-front image size is unavailable")
    parsed = [_finite_point2(point, f"point {index}") for index, point in enumerate(points)]
    if len(parsed) < 2:
        raise WorkAreaGeometryError("work area requires at least two pixel points")
    tolerance = max(0.0, float(boundary_tolerance_px))
    u_limit = float(width - 1)
    v_limit = float(height - 1)
    outside = [
        index
        for index, (u, v) in enumerate(parsed)
        if u < -tolerance
        or u > u_limit + tolerance
        or v < -tolerance
        or v > v_limit + tolerance
    ]
    if outside:
        raise WorkAreaGeometryError(
            f"work-area pixels outside wall-front image: count={len(outside)}"
        )
    u0 = max(0.0, min(point[0] for point in parsed))
    v0 = max(0.0, min(point[1] for point in parsed))
    u1 = min(u_limit, max(point[0] for point in parsed))
    v1 = min(v_limit, max(point[1] for point in parsed))
    minimum = max(0.0, float(minimum_side_px))
    if u1 - u0 < minimum or v1 - v0 < minimum:
        raise WorkAreaGeometryError(
            f"work area is too small in pixels: {u1-u0:.3f}x{v1-v0:.3f}"
        )
    return float(u0), float(v0), float(u1), float(v1)


def outside_pixel_rect_indices(
    points: Iterable[Sequence[float]],
    rect: PixelRect,
    *,
    tolerance_px: float = 1.0,
) -> list[int]:
    """Return indexes outside ``rect``; coordinates are never clipped."""

    u0, v0, u1, v1 = [float(value) for value in rect]
    tolerance = max(0.0, float(tolerance_px))
    outside = []
    for index, point in enumerate(points):
        u, v = _finite_point2(point, f"point {index}")
        if (
            u < u0 - tolerance
            or u > u1 + tolerance
            or v < v0 - tolerance
            or v > v1 + tolerance
        ):
            outside.append(index)
    return outside


def bilinear_quad_point(
    corners: Sequence[Sequence[float]], su: float, sv: float
) -> np.ndarray:
    """Map normalized wall-front coordinates to a TL/TR/BR/BL 3-D quad."""

    pts = np.asarray(corners, dtype=float)
    if pts.shape != (4, 3) or not np.all(np.isfinite(pts)):
        raise WorkAreaGeometryError("quad corners must be finite shape (4, 3)")
    su = float(su)
    sv = float(sv)
    if not math.isfinite(su) or not math.isfinite(sv):
        raise WorkAreaGeometryError("normalized quad coordinates must be finite")
    tl, tr, br, bl = pts
    top = tl + (tr - tl) * su
    bottom = bl + (br - bl) * su
    return top + (bottom - top) * sv


def quad_size_m(corners: Sequence[Sequence[float]]) -> tuple[float, float]:
    """Return mean opposing-edge width and height for a TL/TR/BR/BL quad."""

    pts = np.asarray(corners, dtype=float)
    if pts.shape != (4, 3) or not np.all(np.isfinite(pts)):
        raise WorkAreaGeometryError("quad corners must be finite shape (4, 3)")
    tl, tr, br, bl = pts
    width = 0.5 * (float(np.linalg.norm(tr - tl)) + float(np.linalg.norm(br - bl)))
    height = 0.5 * (float(np.linalg.norm(bl - tl)) + float(np.linalg.norm(br - tr)))
    if width <= 1e-9 or height <= 1e-9:
        raise WorkAreaGeometryError("quad has zero physical extent")
    return width, height


def _quad_projection(corners: Sequence[Sequence[float]]):
    pts = np.asarray(corners, dtype=float)
    if pts.shape != (4, 3) or not np.all(np.isfinite(pts)):
        raise WorkAreaGeometryError("quad corners must be finite shape (4, 3)")
    origin = pts[0]
    u_axis = pts[1] - origin
    u_norm = float(np.linalg.norm(u_axis))
    if u_norm < 1e-9:
        raise WorkAreaGeometryError("quad top edge is degenerate")
    u_axis /= u_norm
    normal = np.zeros(3, dtype=float)
    for index in range(4):
        normal += np.cross(pts[index] - origin, pts[(index + 1) % 4] - origin)
    n_norm = float(np.linalg.norm(normal))
    if n_norm < 1e-9:
        raise WorkAreaGeometryError("quad normal is degenerate")
    normal /= n_norm
    v_axis = np.cross(normal, u_axis)
    v_axis /= np.linalg.norm(v_axis) + 1e-12
    if float(np.dot(v_axis, pts[3] - origin)) < 0.0:
        v_axis = -v_axis
    polygon = np.column_stack(((pts - origin) @ u_axis, (pts - origin) @ v_axis))
    signed_area = 0.5 * float(
        sum(
            polygon[index, 0] * polygon[(index + 1) % 4, 1]
            - polygon[(index + 1) % 4, 0] * polygon[index, 1]
            for index in range(4)
        )
    )
    if abs(signed_area) < 1e-12:
        raise WorkAreaGeometryError("quad projected area is zero")
    orientation = 1.0 if signed_area > 0.0 else -1.0
    return pts, origin, u_axis, v_axis, normal, polygon, orientation


def point_in_quad_3d(
    point: Sequence[float],
    corners: Sequence[Sequence[float]],
    *,
    boundary_tolerance_m: float = 0.001,
    plane_tolerance_m: float = 0.002,
) -> bool:
    """Return whether a point lies on and inside the convex 3-D work-area quad."""

    (
        _pts,
        origin,
        u_axis,
        v_axis,
        normal,
        polygon,
        orientation,
    ) = _quad_projection(corners)
    p = np.asarray(point, dtype=float)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        return False
    if abs(float(np.dot(p - origin, normal))) > max(0.0, float(plane_tolerance_m)):
        return False
    projected = np.array([float(np.dot(p - origin, u_axis)), float(np.dot(p - origin, v_axis))])
    tolerance = max(0.0, float(boundary_tolerance_m))
    for index in range(4):
        a = polygon[index]
        b = polygon[(index + 1) % 4]
        edge = b - a
        edge_length = float(np.linalg.norm(edge))
        if edge_length < 1e-12:
            return False
        cross = edge[0] * (projected[1] - a[1]) - edge[1] * (projected[0] - a[0])
        if orientation * cross < -tolerance * edge_length:
            return False
    return True


def outside_quad_3d_indices(
    points: Iterable[Sequence[float]],
    corners: Sequence[Sequence[float]],
    *,
    boundary_tolerance_m: float = 0.001,
    plane_tolerance_m: float = 0.002,
) -> list[int]:
    """Return indexes outside the selected 3-D work-area quadrilateral."""

    return [
        index
        for index, point in enumerate(points)
        if not point_in_quad_3d(
            point,
            corners,
            boundary_tolerance_m=boundary_tolerance_m,
            plane_tolerance_m=plane_tolerance_m,
        )
    ]


def generate_fill_strokes(
    rect: PixelRect,
    *,
    work_area_width_m: float,
    work_area_height_m: float,
    roller_length_m: float,
    overlap: float,
    minimum_stroke_length_m: float = 0.005,
    allow_footprint_overhang: bool = False,
) -> tuple[PixelStroke, ...]:
    """Generate a vertical serpentine fill wholly inside a selected rectangle.

    The roller's long axis is horizontal, so each stroke center keeps a
    half-roller margin from the left and right work-area boundaries. Spray
    explicitly permits footprint overhang for areas narrower than its fan.
    """

    u0, v0, u1, v1 = [float(value) for value in rect]
    width_px = u1 - u0
    height_px = v1 - v0
    physical_width = float(work_area_width_m)
    physical_height = float(work_area_height_m)
    roller_length = float(roller_length_m)
    overlap = float(overlap)
    values = [
        width_px,
        height_px,
        physical_width,
        physical_height,
        roller_length,
        overlap,
    ]
    if not all(math.isfinite(value) for value in values):
        raise WorkAreaGeometryError("fill geometry values must be finite")
    if width_px <= 0.0 or height_px <= 0.0:
        raise WorkAreaGeometryError("selected pixel rectangle has no area")
    if physical_width <= 0.0 or physical_height <= 0.0:
        raise WorkAreaGeometryError("selected work area has no physical area")
    if roller_length <= 0.0:
        raise WorkAreaGeometryError("roller_length_m must be positive")
    if not 0.0 <= overlap < 1.0:
        raise WorkAreaGeometryError("overlap must be in [0, 1)")
    if physical_width + 1e-9 < roller_length and not allow_footprint_overhang:
        raise WorkAreaGeometryError(
            f"work area width {physical_width:.4f}m is smaller than roller "
            f"length {roller_length:.4f}m"
        )
    if physical_height + 1e-9 < max(0.0, float(minimum_stroke_length_m)):
        raise WorkAreaGeometryError(
            f"work area height {physical_height:.4f}m cannot form a valid stroke"
        )

    usable_m = max(0.0, physical_width - roller_length)
    step_m = roller_length * (1.0 - overlap)
    if usable_m <= 1e-9:
        centers = [(u0 + u1) * 0.5]
    else:
        roller_px = roller_length / physical_width * width_px
        left = u0 + roller_px * 0.5
        right = u1 - roller_px * 0.5
        count = max(2, int(math.ceil(usable_m / step_m)) + 1)
        centers = list(np.linspace(left, right, count))

    strokes = []
    for index, u in enumerate(centers):
        start_v, end_v = (v0, v1) if index % 2 == 0 else (v1, v0)
        strokes.append(((float(u), float(start_v)), (float(u), float(end_v))))
    return tuple(strokes)
