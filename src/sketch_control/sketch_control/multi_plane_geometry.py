"""ROS-independent, bounded multi-plane extraction inside sketch polygons."""
import numpy as np
from sketch_control.pointcloud_utils import ransac_plane


def polygon_mask(pixels, polygons):
    """Union of closed sketch contours; a two-point stroke defines a rectangle."""
    pixels = np.asarray(pixels, dtype=float)
    mask = np.zeros(len(pixels), dtype=bool)
    for contour in polygons:
        p = np.asarray(contour, dtype=float)
        if len(p) == 2:
            lo, hi = p.min(axis=0), p.max(axis=0)
            p = np.array([lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]])
        if len(p) < 3 or not np.isfinite(p).all():
            continue
        inside = np.zeros(len(pixels), dtype=bool)
        x, y = pixels.T
        for a, b in zip(p, np.roll(p, -1, axis=0)):
            if abs(b[1] - a[1]) < 1e-12:
                continue
            cross = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            inside ^= ((a[1] > y) != (b[1] > y)) & (x < cross)
        mask |= inside
    return mask


def convex_hull(points):
    pts = sorted(set(map(tuple, np.asarray(points, dtype=float))))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    return [list(p) for p in half(pts)[:-1] + half(pts[::-1])[:-1]]


def extract_planes(points, pixels, *, max_planes=8, min_points=80,
                   threshold=0.015, iterations=2000):
    points, pixels = np.asarray(points, float), np.asarray(pixels, float)
    if points.shape != (len(pixels), 3) or pixels.shape[1:] != (2,):
        raise ValueError("point/pixel shape mismatch")
    remaining = np.flatnonzero(np.isfinite(points).all(axis=1))
    planes = []
    for _ in range(max_planes):
        if len(remaining) < min_points:
            break
        model, indices = ransac_plane(points[remaining], threshold, iterations)
        indices = np.asarray(indices, dtype=int)
        if len(indices) < min_points:
            break
        cloud = points[remaining[indices]]
        normal = np.asarray(model[:3], float)
        normal /= np.linalg.norm(normal)
        center = cloud.mean(axis=0)
        if normal @ center > 0:
            normal = -normal
        seed = np.array([0., 1., 0.])
        if abs(seed @ normal) > .95:
            seed = np.array([1., 0., 0.])
        right = np.cross(seed, normal)
        right /= np.linalg.norm(right)
        up = np.cross(normal, right)
        uv = (cloud-center) @ np.column_stack((right, up))
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        corners = [center + u*right + v*up for u, v in
                   [(lo[0], hi[1]), (hi[0], hi[1]), (hi[0], lo[1]), (lo[0], lo[1])]]
        support_hull = np.asarray(convex_hull(uv))
        if len(support_hull) > 64:
            # Inscribed simplification keeps ROI payloads bounded without expanding support.
            support_hull = support_hull[np.linspace(0,len(support_hull)-1,64,dtype=int)]
        planes.append(dict(center=center.tolist(), normal=normal.tolist(),
                           corners=np.asarray(corners).tolist(),
                           support_polygon=(center + support_hull @ np.column_stack((right, up)).T).tolist(),
                           polygon_px=convex_hull(pixels[remaining[indices]]),
                           inlier_count=len(indices),
                           rms_m=float(np.sqrt(np.mean(((cloud-center) @ normal)**2)))))
        keep = np.ones(len(remaining), dtype=bool)
        keep[indices] = False
        remaining = remaining[keep]
    return planes
