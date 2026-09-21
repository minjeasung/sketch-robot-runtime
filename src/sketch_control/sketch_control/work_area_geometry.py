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
) -> tuple[PixelStroke, ...]:
    """Generate a vertical serpentine fill wholly inside a selected rectangle.

    The roller's long axis is horizontal, so each stroke center keeps a
    half-roller margin from the left and right work-area boundaries.
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
    if physical_width + 1e-9 < roller_length:
        raise WorkAreaGeometryError(
            f"work area width {physical_width:.4f}m is smaller than roller "
            f"length {roller_length:.4f}m"
        )
    if physical_height + 1e-9 < max(0.0, float(minimum_stroke_length_m)):
        raise WorkAreaGeometryError(
            f"work area height {physical_height:.4f}m cannot form a valid stroke"
        )

    roller_px = roller_length / physical_width * width_px
    left = u0 + roller_px * 0.5
    right = u1 - roller_px * 0.5
    usable_m = max(0.0, physical_width - roller_length)
    step_m = roller_length * (1.0 - overlap)
    if usable_m <= 1e-9:
        centers = [(left + right) * 0.5]
    else:
        count = max(2, int(math.ceil(usable_m / step_m)) + 1)
        centers = list(np.linspace(left, right, count))

    strokes = []
    for index, u in enumerate(centers):
        start_v, end_v = (v0, v1) if index % 2 == 0 else (v1, v0)
        strokes.append(((float(u), float(start_v)), (float(u), float(end_v))))
    return tuple(strokes)
