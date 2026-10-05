"""Bounded RGB refinement of planes already established by depth geometry."""
import cv2
import numpy as np


def refine_image_boundaries(points, pixels, segments, lines, a, b, stride,
                            threshold, minimum):
    """Move only ambiguous measured points between two adjacent fitted faces.

    No plane is created, removed or refitted here. A line inside one face has
    no authority to split it. Even a line near a real crease can only affect
    points within two sampling strides that agree with both plane models.
    """
    if len(segments) < 2 or not len(lines):
        return segments
    owner = np.full(len(points), -1, dtype=int)
    for i, segment in enumerate(segments):
        owner[segment['inlier_indices']] = i
    original = owner.copy()
    models = np.array([s['model'] for s in segments])
    residual = abs(points @ models[:, :3].T+models[:, 3])
    margin = min(.002, .2*threshold)
    for x1, y1, x2, y2 in lines:
        direction = np.array([x2-x1, y2-y1])
        length = np.linalg.norm(direction)
        if length < stride:
            continue
        direction /= length
        relative = pixels-[x1, y1]
        along = relative @ direction
        side = relative @ np.array([-direction[1], direction[0]])
        extent = (along >= 0) & (along <= length)
        near = extent & (abs(side) <= 2*stride)
        crossing = ((side[a]*side[b] < 0) & extent[a] & extent[b]
                    & (original[a] >= 0) & (original[b] >= 0)
                    & (original[a] != original[b]))
        pairs, counts = np.unique(np.sort(np.c_[original[a[crossing]],
                                               original[b[crossing]]], axis=1),
                                  axis=0, return_counts=True)
        if not len(counts) or counts.max() < 6:
            continue
        first, second = pairs[np.argmax(counts)]
        if abs(models[first, :3] @ models[second, :3]) > .985:
            continue
        # Stable geometry on each side must identify distinct existing faces.
        context = extent & (abs(side) <= 4*stride)
        decisive = abs(residual[:, first]-residual[:, second]) > margin
        side_owners = []
        for half in (side < 0, side >= 0):
            votes = original[context & half & decisive
                             & np.isin(original, [first, second])]
            if len(votes) < 6:
                break
            winner = first if np.count_nonzero(votes == first) > len(votes)/2 else second
            if np.mean(votes == winner) < .8:
                break
            side_owners.append(winner)
        if len(side_owners) != 2 or side_owners[0] == side_owners[1]:
            continue
        ambiguous = (near & np.isin(original, [first, second])
                     & (residual[:, first] < threshold)
                     & (residual[:, second] < threshold)
                     & (abs(residual[:, first]-residual[:, second]) <= margin))
        owner[ambiguous & (side < 0)] = side_owners[0]
        owner[ambiguous & (side >= 0)] = side_owners[1]

    refined = []
    # If a tiny face would lose observability, keep the geometric assignment.
    if any(np.count_nonzero(owner == i) < minimum for i in range(len(segments))):
        return segments
    # Use the same valid depth links as the geometry stage. RGB must not cut
    # a connecting strip or make a narrow face unobservable.
    xy = np.rint((pixels-pixels.min(axis=0))/stride).astype(int)
    width, height = xy.max(axis=0)+1
    for i in range(len(segments)):
        indices = np.flatnonzero(owner == i)
        active = owner == i
        mask = np.zeros((2*height+1, 2*width+1), np.uint8)
        vertices = 2*xy[indices]+1
        mask[vertices[:, 1], vertices[:, 0]] = 1
        links = active[a] & active[b]
        mid = xy[a[links]]+xy[b[links]]+1
        mask[mid[:, 1], mid[:, 0]] = 1
        count, _ = cv2.connectedComponents(mask, connectivity=4)
        sides = cv2.minAreaRect(pixels[indices].astype(np.float32))[1]
        if (count != 2 or min(sides) < 3*stride
                or len(indices)*stride**2/max(sides) < 3*stride):
            return segments
    for i, segment in enumerate(segments):
        old = np.asarray(segment['inlier_indices'])
        indices = np.flatnonzero(owner == i)
        region_count = len(old)/segment['segment_inlier_ratio']
        new_region_count = region_count+len(indices)-len(old)
        old_sum = np.sum(residual[old, i]**2)
        new_sum = np.sum(residual[indices, i]**2)
        region_sum = region_count*segment['region_rms_m']**2+new_sum-old_sum
        refined.append(dict(segment, inlier_indices=indices, inlier_count=len(indices),
                            rms_m=float(np.sqrt(new_sum/len(indices))),
                            segment_inlier_ratio=len(indices)/new_region_count,
                            global_inlier_ratio=len(indices)/len(points),
                            region_rms_m=float(np.sqrt(max(0., region_sum)/new_region_count))))
    return refined
