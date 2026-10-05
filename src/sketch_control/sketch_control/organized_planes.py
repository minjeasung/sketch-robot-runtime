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


def _depth_links(points, xy, a, b, threshold):
    """Keep steep planes connected without bridging actual depth steps.

    Inverse depth is affine in image coordinates on a pinhole-viewed plane.
    A large depth change is accepted only if neighbouring changes continue
    that slope; a discontinuity between parallel faces does not pass.
    """
    z = points[:, 2]
    linked = abs(z[a]-z[b]) <= max(.025, 1.5*threshold)
    pending = np.flatnonzero(~linked & (z[a] > 0) & (z[b] > 0))
    if not len(pending):
        return linked
    width, height = xy.max(axis=0)+1
    grid = np.full((height, width), -1, dtype=int)
    grid[xy[:, 1], xy[:, 0]] = np.arange(len(xy))

    def lookup(coordinates):
        valid = ((coordinates >= 0) & (coordinates < [width, height])).all(axis=1)
        indices = np.full(len(coordinates), -1, dtype=int)
        indices[valid] = grid[coordinates[valid, 1], coordinates[valid, 0]]
        return indices

    left, right = a[pending], b[pending]
    delta = xy[right]-xy[left]
    before, after = lookup(xy[left]-delta), lookup(xy[right]+delta)
    inv = np.divide(1., z, out=np.zeros_like(z), where=z > 0)
    slope = inv[right]-inv[left]
    tolerance = .5*threshold/np.maximum(z[left], z[right])**2
    smooth, observed = np.ones(len(pending), bool), np.zeros(len(pending), bool)
    for outer, inner, sign in ((before, left, 1), (after, right, -1)):
        valid = (outer >= 0) & (z[outer] > 0)
        observed |= valid
        smooth[valid] &= abs(sign*(inv[inner[valid]]-inv[outer[valid]])-slope[valid]) <= tolerance[valid]
    linked[pending] = observed & smooth
    return linked


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


def _longitudinal_support(region_pixels, support_pixels):
    """Whether support spans most of an elongated region, at any rotation."""
    box = cv2.boxPoints(cv2.minAreaRect(region_pixels.astype(np.float32)))
    axes = np.array([box[1]-box[0], box[2]-box[1]])
    lengths = np.linalg.norm(axes, axis=1)
    if max(lengths) < 2*max(min(lengths), 1.):
        return True
    axis = axes[np.argmax(lengths)]/max(lengths)
    return np.ptp(support_pixels @ axis) >= .5*np.ptp(region_pixels @ axis)


def _residual_shard(points, pixels, group, segment, proposals, threshold):
    model = np.asarray(segment['model'])
    support = group[segment['inlier_indices']]
    if _longitudinal_support(pixels[group], pixels[support]):
        return False
    for other_group, other in proposals:
        if other is segment or other_group is not group:
            continue
        reference = np.asarray(other['model'])
        if abs(model[:3] @ reference[:3]) < .866:
            continue
        other_support = group[other['inlier_indices']]
        if not _longitudinal_support(pixels[group], pixels[other_support]):
            continue
        if np.percentile(abs(points[support] @ reference[:3]+reference[3]), 95) > 4*threshold:
            continue
        # Parallel but offset faces can both belong to a steel section.
        # Keep a consistent depth step; suppress the tilted residual patches
        # created when noisy depth on one long face is fitted repeatedly.
        alignment = abs(model[:3] @ reference[:3])
        separation = abs(points[support].mean(axis=0) @ reference[:3]+reference[3])
        if alignment > .99985 and separation > threshold:
            continue
        return True
    return False


def spatial_plane_segments(points, pixels, *, pixel_stride, rgb,
                           structure="generic", **fit_options):
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
    linked = _depth_links(points, xy, a, b, threshold)
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

    # Local normal seeds preserve narrow folds whose full depth variation is
    # smaller than the RANSAC tolerance. They still need measured 2D support.
    from sketch_control.plane_normals import normal_seed_models
    proposals.extend(normal_seed_models(points, xy, da[boundaries], db[boundaries],
                                         groups, _components, threshold, minimum))

    # Multiple scales often propose the same face. Keep its most precise
    # measured model before point competition; duplicates would fragment it.
    distinct = []
    for group, segment in sorted(proposals, key=lambda p: p[1]['rms_m']):
        model = np.asarray(segment['model'])
        seed_indices = group[segment['inlier_indices']]
        seed_pixels = pixels[seed_indices]
        if min(cv2.minAreaRect(seed_pixels.astype(np.float32))[1]) < stride:
            continue
        # Scattered leftovers must not become a competing model and take
        # support away from a connected face just because their total is large.
        active = np.zeros(len(points), dtype=bool)
        active[seed_indices] = True
        connected = _components(xy, da, db, active)
        if np.bincount(connected[seed_indices]).max() < minimum:
            continue
        support = points[seed_indices]
        duplicate = False
        for other_group, other in distinct:
            if other_group is not group:
                continue
            other_model = np.asarray(other['model'])
            other_support = points[other_group[other['inlier_indices']]]
            if (abs(model[:3] @ other_model[:3]) > .996
                    and np.percentile(abs(support @ other_model[:3]+other_model[3]), 80) < threshold*.5
                    and np.percentile(abs(other_support @ model[:3]+model[3]), 95) < threshold*.5):
                duplicate = True
                break
        if not duplicate:
            distinct.append((group, segment))
    proposals = distinct
    if structure == 'hbeam':
        # A short but distinct or partially occluded face remains eligible.
        # Suppress only similarly oriented, nearby shards of an observed long face.
        proposals = [(group, segment) for group, segment in distinct
                     if not _residual_shard(points, pixels, group, segment, distinct, threshold)]

    # Merge only neighbours separated by an image edge, and only if both
    # complete regions agree with the same plane. Depth gaps remain separate.
    owner = np.full(len(points), -1, dtype=int)
    best = np.full(len(points), threshold)
    for i, (group, segment) in enumerate(proposals):
        model = np.asarray(segment['model'])
        residual = abs(points[group] @ model[:3]+model[3])
        take = residual < best[group]
        owner[group[take]] = i
        best[group[take]] = residual[take]
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

    # Sequential RANSAC removes a tolerance band before the next face is
    # fitted. Reassign the original points jointly to competing models so a
    # flange cannot permanently consume the edge of a narrow recessed web.
    proposals = [proposal for proposal in proposals if proposal is not None]
    best = np.full(len(points), np.inf, dtype=float)
    owner = np.full(len(points), -1, dtype=int)
    for i, (group, segment) in enumerate(proposals):
        model = np.asarray(segment['model'])
        residual = abs(points[group] @ model[:3] + model[3])
        take = residual < best[group]
        owner[group[take]] = i
        best[group[take]] = residual[take]

    segments = []
    for proposal_index, proposal in enumerate(proposals):
        if proposal is None:
            continue
        group, segment = proposal
        face_region = group[owner[group] == proposal_index]
        indices = face_region[best[face_region] < threshold]
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
            region_residual = points[face_region] @ normal+offset
            segments.append(dict(segment, model=np.r_[normal, offset],
                                 inlier_indices=support, inlier_count=len(support),
                                 rms_m=float(np.sqrt(np.mean(residual**2))),
                                 segment_inlier_ratio=len(support)/len(face_region),
                                 global_inlier_ratio=len(support)/len(points),
                                 region_rms_m=float(np.sqrt(np.mean(region_residual**2)))))
    return sorted(segments, key=lambda s: -s['inlier_count'])[:fit_options['max_planes']]
