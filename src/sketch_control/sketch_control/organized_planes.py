"""Spatial support for plane fitting on an RGB/depth pixel lattice.

Long image edges propose boundaries; depth geometry decides whether adjacent
regions really differ. This prevents a global RANSAC hypothesis from consuming
pieces of several narrow faces before the recessed face can be fitted.
"""
import cv2
import numpy as np

from sketch_control.pointcloud_utils import (
    _refine_plane_svd, segment_planes_iterative_ransac,
)


def _lattice(pixels, stride):
    origin = pixels.min(axis=0)
    coordinates = (pixels - origin) / stride
    if not np.allclose(coordinates, np.rint(coordinates)):
        raise ValueError("plane pixels must lie on the sampling lattice")
    xy = np.rint(coordinates).astype(int)
    width, height = xy.max(axis=0) + 1
    if width * height > 2_000_000:
        raise ValueError("plane pixel lattice too large")
    grid = np.full((height, width), -1, dtype=int)
    grid[xy[:, 1], xy[:, 0]] = np.arange(len(xy))
    a = np.r_[grid[:, :-1].ravel(), grid[:-1].ravel()]
    b = np.r_[grid[:, 1:].ravel(), grid[1:].ravel()]
    valid = (a >= 0) & (b >= 0)
    return xy, a[valid], b[valid]


def _components(xy, a, b, active):
    # Insert graph edges between pixel vertices on a doubled raster. Ordinary
    # 4-connectivity then implements the graph without an additional dependency.
    width, height = xy.max(axis=0) + 1
    mask = np.zeros((2*height+1, 2*width+1), np.uint8)
    q = 2*xy + 1
    mask[q[active, 1], q[active, 0]] = 1
    keep = active[a] & active[b]
    mid = xy[a[keep]] + xy[b[keep]] + 1
    mask[mid[:, 1], mid[:, 0]] = 1
    _, labels = cv2.connectedComponents(mask, connectivity=4)
    return labels[q[:, 1], q[:, 0]]


def _image_boundaries(rgb, pixels, stride):
    if rgb is None:
        return []
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("plane RGB image must be uint8 HxWx3")
    lo = np.maximum(np.floor(pixels.min(axis=0)).astype(int)-8, 0)
    hi = np.minimum(np.ceil(pixels.max(axis=0)).astype(int)+9,
                    [rgb.shape[1], rgb.shape[0]])
    if np.any(hi <= lo) or np.any(pixels < 0) or np.any(
            pixels >= [rgb.shape[1], rgb.shape[0]]):
        raise ValueError("plane pixels outside RGB image")
    gray = cv2.cvtColor(rgb[lo[1]:hi[1], lo[0]:hi[0]], cv2.COLOR_RGB2GRAY)
    lines = cv2.createLineSegmentDetector().detect(gray)[0]
    if lines is None:
        return []
    lines = lines[:, 0].astype(float) + np.tile(lo, 2)
    lengths = np.linalg.norm(lines[:, 2:]-lines[:, :2], axis=1)
    minimum = max(12*stride, .25*float(np.ptp(pixels, axis=0).max()))
    # Bound both processing time and over-segmentation in textured images.
    order = np.argsort(-lengths)
    return lines[order[lengths[order] >= minimum][:32]]


def spatial_plane_segments(points, pixels, *, pixel_stride, rgb, **fit_options):
    """Fit connected faces; returned indices refer to the original cloud.

    RGB lines are extended only within the selected ROI. Coplanar neighbours
    are rejoined, so a painted stripe alone cannot manufacture two work faces.
    Only fitted inliers are returned, never the whole image region or a line's
    extrapolated extent. Missing RGB still allows depth/connectivity splitting.
    """
    if len(points) == 0:
        return []
    stride = int(pixel_stride)
    if stride < 1 or not np.isfinite(points).all() or not np.isfinite(pixels).all():
        raise ValueError("invalid organized plane samples")
    xy, a, b = _lattice(pixels, stride)
    threshold = fit_options['distance_threshold']
    linked = np.abs(points[a, 2]-points[b, 2]) <= max(.025, 1.5*threshold)
    da, db = a[linked], b[linked]
    boundaries = np.ones(len(da), dtype=bool)
    for x1, y1, x2, y2 in _image_boundaries(rgb, pixels, stride):
        side = (pixels[:, 0]-x1)*(y2-y1) - (pixels[:, 1]-y1)*(x2-x1)
        boundaries &= side[da]*side[db] >= 0
    labels = _components(xy, da[boundaries], db[boundaries],
                         np.ones(len(points), dtype=bool))
    minimum = fit_options['min_inliers']
    # All regions compete for the same bounded candidate budget.
    counts = np.bincount(labels)
    order = np.argsort(-counts)
    order = order[(order != 0) & (counts[order] >= minimum)]
    groups = [np.flatnonzero(labels == label)
              for label in order[:max(16, 2*fit_options['max_planes'])]]
    proposals = []
    for group in groups:
        # Connectivity does not imply planarity: without an RGB boundary a
        # flange can meet the web continuously in depth within one region.
        options = dict(fit_options, min_global_inlier_ratio=0.)
        for segment in segment_planes_iterative_ransac(points[group], **options):
            proposals.append((group, segment))

    # Merge only neighbours separated by an image edge, and only if both
    # complete regions agree with the same plane. Depth gaps remain separate.
    owner = np.full(len(points), -1, dtype=int)
    for i, (group, segment) in enumerate(proposals):
        owner[group[segment['inlier_indices']]] = i
    pairs = np.unique(np.sort(np.c_[owner[da[~boundaries]],
                                    owner[db[~boundaries]]], axis=1), axis=0)
    parents = np.arange(len(proposals))

    def root(index):
        while parents[index] != index:
            index = parents[index]
        return index

    for left, right in pairs:
        if left < 0:
            continue
        left, right = root(left), root(right)
        if left == right:
            continue
        ga, sa = proposals[left]
        gb, sb = proposals[right]
        ma, mb = np.asarray(sa['model']), np.asarray(sb['model'])
        if abs(ma[:3] @ mb[:3]) < .985:
            continue
        if (np.percentile(abs(points[ga] @ mb[:3]+mb[3]), 95) > threshold
                or np.percentile(abs(points[gb] @ ma[:3]+ma[3]), 95) > threshold):
            continue
        group = np.union1d(ga, gb)
        normal, offset, rms = _refine_plane_svd(points[group])
        residual = abs(points[group] @ normal+offset)
        inside = np.flatnonzero(residual < threshold)
        merged = dict(sa, model=np.r_[normal, offset], inlier_indices=inside,
                      inlier_count=len(inside), rms_m=rms)
        proposals[right] = (group, merged)
        proposals[left] = None
        parents[left] = right

    segments = []
    for proposal in proposals:
        if proposal is None:
            continue
        group, segment = proposal
        indices = group[segment['inlier_indices']]
        active = np.zeros(len(points), dtype=bool)
        active[indices] = True
        connected = _components(xy, da, db, active)
        counts = np.bincount(connected[indices])
        for label in np.flatnonzero(counts >= minimum):
            support = indices[connected[indices] == label]
            if len(support) < minimum:
                continue
            # A few columns of edge noise can have a tiny RMS yet an arbitrary
            # normal. Require an observable area, not just a long thin line.
            sides = cv2.minAreaRect(pixels[support].astype(np.float32))[1]
            if min(sides) < 3*stride or len(support)*stride**2/max(sides) < 3*stride:
                continue
            normal, offset, _ = _refine_plane_svd(points[support])
            support = support[abs(points[support] @ normal+offset) < threshold]
            if len(support) < minimum:
                continue
            residual = points[support] @ normal+offset
            region_residual = points[group] @ normal+offset
            segments.append(dict(segment, model=np.r_[normal, offset],
                                 inlier_indices=support, inlier_count=len(support),
                                 rms_m=float(np.sqrt(np.mean(residual**2))),
                                 segment_inlier_ratio=len(support)/len(group),
                                 global_inlier_ratio=len(support)/len(points),
                                 region_rms_m=float(np.sqrt(np.mean(region_residual**2)))))
    return sorted(segments, key=lambda s: -s['inlier_count'])[:fit_options['max_planes']]
