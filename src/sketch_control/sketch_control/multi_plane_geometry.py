"""ROS-independent, bounded multi-plane extraction inside sketch polygons."""
import numpy as np
from sketch_control.pointcloud_utils import segment_planes_iterative_ransac


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


def extract_planes(
        points, pixels, *, max_planes=8, min_points=80,
        threshold=0.015, iterations=2000, voxel_size=0.01,
        sor_mean_k=12, sor_std_ratio=1.0, sor_max_points=3500,
        max_fit_points=7000, removal_threshold_scale=1.25,
        min_global_inlier_ratio=0.02, seed=7, pixel_stride=None, rgb=None,
        structure="generic"):
    """Extract robust plane candidates while preserving sketch pixel support.

    Organized depth (pixel_stride supplied) uses connected RGB/depth regions
    before fitting, and splits disconnected inliers before constructing hulls.
    Unorganized clouds retain iterative RANSAC. Both preserve original support.
    """
    if structure not in ("generic", "hbeam"):
        raise ValueError("structure must be generic or hbeam")
    points = np.asarray(points, dtype=float)
    pixels = np.asarray(pixels, dtype=float)
    if points.shape != (len(pixels), 3) or pixels.shape[1:] != (2,):
        raise ValueError("point/pixel shape mismatch")

    fit_options = dict(
        max_planes=max_planes,
        max_iterations=iterations,
        distance_threshold=threshold,
        min_inliers=min_points,
        voxel_size=voxel_size,
        sor_mean_k=sor_mean_k,
        sor_std_ratio=sor_std_ratio,
        sor_max_points=sor_max_points,
        max_fit_points=max_fit_points,
        seed=seed,
        removal_threshold_scale=removal_threshold_scale,
        min_global_inlier_ratio=min_global_inlier_ratio,
    )
    if pixel_stride is None:
        segments = segment_planes_iterative_ransac(points, **fit_options)
    else:
        from sketch_control.organized_planes import spatial_plane_segments
        segments = spatial_plane_segments(points, pixels, pixel_stride=pixel_stride,
                                          rgb=rgb, structure=structure, **fit_options)

    planes = []
    for segment in segments:
        indices = np.asarray(segment["inlier_indices"], dtype=int)
        if indices.shape[0] < int(min_points):
            continue

        cloud = points[indices]
        normal = np.asarray(segment["model"][:3], dtype=float)
        normal /= np.linalg.norm(normal)
        center = cloud.mean(axis=0)
        if normal @ center > 0:
            normal = -normal

        axis_seed = np.array([0., 1., 0.])
        if abs(axis_seed @ normal) > .95:
            axis_seed = np.array([1., 0., 0.])
        right = np.cross(axis_seed, normal)
        right /= np.linalg.norm(right)
        up = np.cross(normal, right)

        basis = np.column_stack((right, up))
        uv = (cloud-center) @ basis
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        corners = [
            center + u*right + v*up
            for u, v in [
                (lo[0], hi[1]),
                (hi[0], hi[1]),
                (hi[0], lo[1]),
                (lo[0], lo[1]),
            ]
        ]

        support_hull = np.asarray(convex_hull(uv))
        if len(support_hull) > 64:
            # Inscribed simplification keeps ROI payloads bounded without
            # expanding support beyond observed inliers.
            support_hull = support_hull[
                np.linspace(0, len(support_hull)-1, 64, dtype=int)
            ]

        planes.append(dict(
            center=center.tolist(),
            normal=normal.tolist(),
            corners=np.asarray(corners).tolist(),
            support_polygon=(center + support_hull @ basis.T).tolist(),
            polygon_px=convex_hull(pixels[indices]),
            inlier_count=int(segment["inlier_count"]),
            rms_m=float(segment["rms_m"]),
            segment_inlier_ratio=float(segment["segment_inlier_ratio"]),
            global_inlier_ratio=float(segment["global_inlier_ratio"]),
            removed_count=int(segment["removed_count"]),
            remaining_after_removal=int(segment["remaining_after_removal"]),
            region_rms_m=float(segment.get("region_rms_m", segment["rms_m"])),
            partial_support=bool(segment.get("region_rms_m", 0.) > threshold
                                 and segment["segment_inlier_ratio"] < .8),
        ))
    return planes
