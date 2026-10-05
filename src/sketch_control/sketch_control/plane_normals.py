"""Local measured plane seeds for folds with weak or absent RGB boundaries."""
import cv2
import numpy as np

from sketch_control.pointcloud_utils import _refine_plane_svd


def local_normals(points, xy, window=5):
    """Estimate normals and reliability using covariance on the pixel lattice.

    Missing depth contributes neither coordinates nor weight. Eigenvalue ratios
    reject windows spanning a crease or an almost one-dimensional edge strip.
    """
    width, height = xy.max(axis=0) + 1
    grid = np.zeros((height, width, 3), dtype=float)
    weight = np.zeros((height, width), dtype=float)
    grid[xy[:, 1], xy[:, 0]] = points
    weight[xy[:, 1], xy[:, 0]] = 1.

    def total(values):
        return cv2.boxFilter(values, -1, (window, window), normalize=False,
                             borderType=cv2.BORDER_CONSTANT)[xy[:, 1], xy[:, 0]]

    count = total(weight)
    means = total(grid) / np.maximum(count[:, None], 1.)
    covariance = np.empty((len(points), 3, 3))
    for i in range(3):
        for j in range(i, 3):
            value = total(grid[:, :, i]*grid[:, :, j])/np.maximum(count, 1.)
            covariance[:, i, j] = covariance[:, j, i] = value-means[:, i]*means[:, j]
    values, vectors = np.linalg.eigh(covariance)
    reliable = ((count >= max(6, window*window*.6)) & (values[:, 1] > 1e-8)
                & (values[:, 0] < .08*values[:, 1]))
    return vectors[:, :, 0], reliable


def normal_seed_models(points, xy, a, b, groups, components, threshold, minimum):
    """Return measured seed models, associated with their original region.

    Seeds supplement RANSAC rather than replacing measured depth or imposing
    nominal H-section angles. A whole seed must support its fitted model.
    """
    out = []
    for window in (3, 5):
        normals, reliable = local_normals(points, xy, window)
        aligned = abs(np.einsum('ij,ij->i', normals[a], normals[b])) > .94
        for group in groups:
            active = np.zeros(len(points), bool)
            active[group] = reliable[group]
            labels = components(xy, a[aligned], b[aligned], active)
            counts = np.bincount(labels[group][active[group]])
            order = np.argsort(-counts)
            for label in order[:16]:
                if label == 0 or counts[label] < minimum:
                    continue
                support = group[(labels[group] == label) & active[group]]
                normal, offset, rms = _refine_plane_svd(points[support])
                if np.percentile(abs(points[support] @ normal+offset), 95) > threshold*.5:
                    continue
                out.append((group, dict(model=np.r_[normal, offset],
                                       inlier_indices=np.flatnonzero(np.isin(group, support)),
                                       inlier_count=len(support), rms_m=rms,
                                       removed_count=0, remaining_after_removal=len(group)-len(support))))
    return out
