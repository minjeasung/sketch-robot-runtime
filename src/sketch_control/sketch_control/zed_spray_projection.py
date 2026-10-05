"""ROS-independent validation and metric rectification of selected ZED planes."""
import numpy as np
import cv2

from sketch_control.rotation_utils import quat_from_matrix
from sketch_control.work_area_geometry import bilinear_quad_point, work_area_polygon


def stamp_ns(stamp):
    """Validate a JSON ROS stamp without silently truncating malformed values."""
    if not isinstance(stamp, dict):
        raise ValueError("missing stamp")
    sec, nanosec = stamp.get("sec"), stamp.get("nanosec")
    if (type(sec) is not int or type(nanosec) is not int
            or sec < 0 or not 0 <= nanosec < 1_000_000_000):
        raise ValueError("invalid stamp")
    result = sec * 1_000_000_000 + nanosec
    if result <= 0:
        raise ValueError("zero stamp")
    return result


def _array(value, shape, label):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"invalid {label}")
    return result


def validate_work_area_request(payload, generation):
    """Validate a generation-bound request; its stamp is a browser event ID."""
    if (not isinstance(payload, dict) or payload.get("source") != "zed"
            or not generation or payload.get("plane_generation_id") != generation):
        raise ValueError("work request does not match the active ZED generation")
    header = payload.get("header")
    if not isinstance(header, dict) or header.get("frame_id") != "wall_front":
        raise ValueError("work request must use wall_front")
    stamp = header.get("stamp")
    stamp_ns(stamp)
    if stamp["sec"] > 2_147_483_647:
        raise ValueError("work request stamp cannot be represented by ROS Time")
    pixels = payload.get("pixels")
    if (not isinstance(pixels, list) or len(pixels) > 1024
            or any(not isinstance(point, list) or len(point) != 2
                   or any(type(value) not in (int, float) for value in point)
                   for point in pixels)):
        raise ValueError("work request pixels must be one numeric boundary")
    points = np.asarray(pixels, dtype=float).reshape(-1, 2)
    if not np.isfinite(points).all():
        raise ValueError("work request pixels must be finite")
    return dict(sec=stamp["sec"], nanosec=stamp["nanosec"]), points


def _rectangle(corners):
    points = _array(corners, (4, 3), "plane corners")
    right, down = points[1] - points[0], points[3] - points[0]
    width, height = float(np.linalg.norm(right)), float(np.linalg.norm(down))
    if min(width, height) < 0.01 or max(width, height) > 10.0:
        raise ValueError("unsupported plane size")
    if (abs(float(right @ down)) > 1e-6 * width * height
            or np.linalg.norm(points[2] - points[0] - right - down) > 1e-6):
        raise ValueError("catalog support must be a planar rectangle")
    return points, right / width, down / height, width, height


def validate_target_lock(payload):
    """Accept only an explicit, generation-bound ZED lock with bounded support."""
    if (not isinstance(payload, dict) or payload.get("accepted") is not True
            or payload.get("source") != "zed" or payload.get("state") != "locked"):
        raise ValueError("target is not an accepted ZED lock")
    # The producer performs catalog quality gating. If it also supplies the
    # measurements, reject incomplete or contradictory evidence here.
    if "inlier_count" in payload or "rms_m" in payload:
        count, rms = payload.get("inlier_count"), payload.get("rms_m")
        if (type(count) is not int or count < 80 or type(rms) not in (int, float)
                or not np.isfinite(rms) or not 0 <= rms <= 0.015):
            raise ValueError("insufficient ZED plane quality")
    for name in ("plane_generation_id", "catalog_generation", "plane_id", "frame_id"):
        if not isinstance(payload.get(name), str) or not payload[name].strip():
            raise ValueError(f"missing {name}")
    activation = stamp_ns(payload.get("stamp"))
    expected = f"zed:{payload['catalog_generation']}:{payload['plane_id']}:{activation}"
    if payload["plane_generation_id"] != expected:
        raise ValueError("target generation does not match its identity")
    center = _array(payload.get("center"), (3,), "center")
    normal = _array(payload.get("normal"), (3,), "normal")
    if abs(float(np.linalg.norm(normal)) - 1.0) > 0.001:
        raise ValueError("target normal is not unit length")
    normal = normal / np.linalg.norm(normal)
    points, right, down, width, height = _rectangle(payload.get("corners"))
    if np.max(np.abs((points - center) @ normal)) > 1e-5:
        raise ValueError("support is not on the target plane")
    offset = center - points[0]
    if not (0 <= offset @ right <= width and 0 <= offset @ down <= height):
        raise ValueError("target center is outside catalog support")
    result = dict(payload)
    result.update(center=center, normal=normal, corners=points, stamp_ns=activation)
    return result


def plane_orientation(corners, normal):
    """Pose +Z follows the selected normal; +X follows the frontal right edge."""
    _points, right, _down, _width, _height = _rectangle(corners)
    normal = _array(normal, (3,), "normal")
    return quat_from_matrix(np.column_stack((right, np.cross(normal, right), normal)))


def project_target_rectangle(target, intrinsics, image_size, *, rotation=None, translation=None):
    """Project bounded plane support into ZED RGB, keeping a true metric rectangle.

    The transform maps the target frame into the RGB optical frame. A metric
    envelope can extend beyond the image even when every selected pixel is
    visible. Retain its geometry and render missing image regions as borders;
    validate_visible_work_area rejects selections touching those unseen areas.
    """
    K = _array(intrinsics, (3, 3), "camera intrinsics")
    if (K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1])
            or abs(K[0, 1]) > 1e-9 or abs(K[1, 0]) > 1e-9):
        raise ValueError("invalid rectified camera intrinsics")
    image_width, image_height = image_size
    if image_width <= 1 or image_height <= 1:
        raise ValueError("invalid image size")
    R = _array(np.eye(3) if rotation is None else rotation, (3, 3), "camera rotation")
    t = _array(np.zeros(3) if translation is None else translation, (3,), "camera translation")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0.999999:
        raise ValueError("invalid camera rotation")
    points, _right, _down, width, height = _rectangle(target["corners"])
    points = points.copy()
    camera = points @ R.T + t
    center = camera.mean(axis=0)
    facing = float((R @ target["normal"]) @ (-center)) / max(np.linalg.norm(center), 1e-12)
    if np.any(camera[:, 2] <= 0.05) or facing < 0.1:
        raise ValueError("plane is behind, back-facing or grazing the camera")
    homogeneous = camera @ K.T
    pixels = homogeneous[:, :2] / homogeneous[:, 2:]
    if not np.isfinite(pixels).all():
        raise ValueError("invalid projected plane support")
    image_bounds = np.float32([[0, 0], [image_width - 1, 0],
                               [image_width - 1, image_height - 1], [0, image_height - 1]])
    visible_area, _ = cv2.intersectConvexConvex(pixels.astype(np.float32), image_bounds)
    if visible_area < 100.0:
        raise ValueError("catalog support is outside ZED image")
    # Catalog axes may have either sign. Keep right/down aligned with the RGB
    # camera without replacing the metric support by a raw-image bounding box.
    if (pixels[1, 0] + pixels[2, 0]) < (pixels[0, 0] + pixels[3, 0]):
        points, pixels = points[[1, 0, 3, 2]], pixels[[1, 0, 3, 2]]
    if (pixels[3, 1] + pixels[2, 1]) < (pixels[0, 1] + pixels[1, 1]):
        points, pixels = points[[3, 2, 1, 0]], pixels[[3, 2, 1, 0]]
    edges = np.roll(pixels, -1, axis=0) - pixels
    twice_area = abs(float(np.sum(pixels[:, 0] * np.roll(pixels[:, 1], -1)
                                  - pixels[:, 1] * np.roll(pixels[:, 0], -1))))
    if np.min(np.linalg.norm(edges, axis=1)) < 4.0 or twice_area < 200.0:
        raise ValueError("insufficient projected plane support")
    scale = 899.0 / max(width, height)
    size = (int(round(width * scale)) + 1, int(round(height * scale)) + 1)
    if min(size) < 32:
        raise ValueError("plane aspect ratio is too extreme")
    return points, pixels.astype(np.float32), size


def validate_visible_work_area(corners, intrinsics, rotation, translation, image_size):
    """Require the whole selected quad to lie in the observed camera image.

    Perspective projection of a planar convex quad with positive depth remains
    convex, so testing all four vertices also bounds every generated interior
    point. Use the exact camera geometry that produced the displayed front view.
    """
    camera = np.asarray(corners) @ rotation.T + translation
    if not np.isfinite(camera).all() or np.any(camera[:, 2] <= 0.05):
        raise ValueError("work area is outside ZED image")
    homogeneous = camera @ intrinsics.T
    pixels = homogeneous[:, :2] / homogeneous[:, 2:]
    width, height = image_size
    if (not np.isfinite(pixels).all() or np.any(pixels < -1e-6)
            or np.any(pixels[:, 0] > width - 1 + 1e-6)
            or np.any(pixels[:, 1] > height - 1 + 1e-6)):
        raise ValueError("work area is outside ZED image; select within the visible region")


def select_work_area(extent, view_size, pixels):
    """Return the metric envelope of a validated Wall Front boundary.

    The producer also publishes the boundary so path generation can enforce
    its exact shape; this envelope retains the existing rectangular frame.
    """
    width, height = view_size
    points = work_area_polygon(pixels, width, height)
    if width <= 1 or height <= 1:
        raise ValueError("front view is unavailable")
    lo, hi = points.min(axis=0), points.max(axis=0)
    if np.any(lo < 0) or hi[0] > width - 1 or hi[1] > height - 1:
        raise ValueError("work area is outside wall_front")
    if np.any(hi - lo < 20.0):
        raise ValueError("work area is too small")
    rectangle = np.array([lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]])
    support, _right, _down, _w, _h = _rectangle(extent)
    return np.asarray([bilinear_quad_point(support, u / (width - 1), v / (height - 1))
                       for u, v in rectangle])
