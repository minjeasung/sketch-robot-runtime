"""Pure non-contact spray geometry, independent of roller/brush transforms."""

import math

import numpy as np


SPRAY_TOOL_AXES = frozenset({"+x", "-x", "+y", "-y", "+z", "-z"})
MODEL_SPRAY_AXES = {"rb20_1900es": "+z", "rb10_1300e_u": "-y"}


def resolve_spray_tool_axis(model_id, override=""):
    """Resolve a signed nozzle axis; unknown models require an explicit axis."""
    if not isinstance(override, str) or not isinstance(model_id, str):
        raise ValueError("model_id and spray_tool_axis must be strings")
    axis = override.strip().lower()
    if not axis:
        axis = MODEL_SPRAY_AXES.get(model_id.strip().lower(), "")
    if axis not in SPRAY_TOOL_AXES:
        raise ValueError("spray_tool_axis requires a signed TCP axis or a known model_id")
    return axis


def spray_spacing_m(footprint_width_m, overlap):
    width, overlap = float(footprint_width_m), float(overlap)
    if not math.isfinite(width) or width <= 0.0:
        raise ValueError("spray_footprint_width_m must be finite and positive")
    if not math.isfinite(overlap) or not 0.0 <= overlap < 1.0:
        raise ValueError("spray_overlap must be finite and in [0, 1)")
    spacing = width * (1.0 - overlap)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spray spacing must be finite and positive")
    return spacing


def _unit(vector, label):
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must be a finite three-vector")
    length = float(np.linalg.norm(vector))
    if not math.isfinite(length) or length < 1e-9:
        raise ValueError(f"{label} must be nonzero")
    return vector / length


def rotation_from_spray_path(normal, tangent, tool_axis, previous_tcp_x=None):
    """Return right-handed TCP axes with the signed nozzle axis aimed inward.

    Surface normals point out toward the camera. The tangential TCP axis keeps
    its sign on serpentine reversals. For an X nozzle, TCP X is constrained by
    the normal, so a canonical tangential sign supplies the remaining roll.
    """
    axis = resolve_spray_tool_axis("", tool_axis)
    normal = _unit(normal, "normal")
    tangent = _unit(tangent, "tangent")
    tangent = _unit(tangent - normal * np.dot(tangent, normal), "surface tangent")
    cross_axis = _unit(np.cross(normal, tangent), "transverse axis")
    previous = None if previous_tcp_x is None else _unit(previous_tcp_x, "previous_tcp_x")
    # A deterministic initial sign also handles exact 90-degree ties.
    if cross_axis[int(np.argmax(np.abs(cross_axis)))] < 0.0:
        cross_axis = -cross_axis
    if axis[1] != "x" and previous is not None and np.dot(cross_axis, previous) < -1e-9:
        cross_axis = -cross_axis
    inward_axis = (-1.0 if axis[0] == "+" else 1.0) * normal
    if axis[1] == "z":
        x, z = cross_axis, inward_axis
        y = np.cross(z, x)
    elif axis[1] == "y":
        x, y = cross_axis, inward_axis
        z = np.cross(x, y)
    else:
        x, y = inward_axis, cross_axis
        z = np.cross(x, y)
    return np.column_stack((x, y, z))
