"""Numpy-only point cloud helpers for runtime perception nodes."""
import numpy as np


def voxel_downsample(points, voxel_size):
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape[0] == 0:
        return pts.reshape(0, 3)
    idx = np.floor(pts / float(voxel_size)).astype(np.int64)
    _uniq, inverse = np.unique(idx, axis=0, return_inverse=True)
    sums = np.zeros((_uniq.shape[0], 3), dtype=np.float64)
    counts = np.bincount(inverse).astype(np.float64)
    np.add.at(sums, inverse, pts)
    return (sums / counts[:, None]).astype(np.float32)


def voxel_representative_indices(points, voxel_size):
    """Return one original point index nearest each voxel centroid.

    Unlike voxel_downsample(), this preserves a link back to the original cloud.
    That is required by the sketch workflow so plane inliers can still be mapped
    to source image pixels after robust fitting.
    """
    pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if pts.shape[0] == 0 or float(voxel_size) <= 0.0:
        return np.arange(pts.shape[0], dtype=int)

    keys = np.floor(pts / float(voxel_size)).astype(np.int64)
    _uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse).astype(np.float64)
    sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
    np.add.at(sums, inverse, pts)
    centroids = sums / counts[:, None]

    delta = pts - centroids[inverse]
    distance2 = np.einsum("ij,ij->i", delta, delta)
    order = np.lexsort((distance2, inverse))
    ordered_groups = inverse[order]
    first = np.r_[True, ordered_groups[1:] != ordered_groups[:-1]]
    return order[first]


def statistical_outlier_indices(points, mean_k=12, std_ratio=1.0,
                                max_points=3500, seed=7, chunk_size=256):
    """Return indices kept by bounded statistical outlier removal.

    The exact neighbour calculation is intentionally bounded.  When the input
    is larger than max_points, this function returns kept indices from a
    deterministic random sample.  The multi-plane fitter later re-evaluates the
    refined plane against the full remaining cloud, so this sampling only
    affects hypothesis generation, not the final support polygon.
    """
    pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    n_points = int(pts.shape[0])
    if n_points <= 2:
        return np.arange(n_points, dtype=int)

    source_indices = np.arange(n_points, dtype=int)
    limit = int(max_points)
    if limit > 0 and n_points > limit:
        rng = np.random.default_rng(int(seed))
        source_indices = np.sort(
            rng.choice(n_points, size=limit, replace=False)
        )
        work = pts[source_indices]
    else:
        work = pts

    n_work = int(work.shape[0])
    k = min(max(1, int(mean_k)), n_work - 1)
    means = np.empty(n_work, dtype=np.float64)
    chunk = max(1, int(chunk_size))

    for start in range(0, n_work, chunk):
        stop = min(n_work, start + chunk)
        diff = work[start:stop, None, :] - work[None, :, :]
        distance2 = np.einsum("ijk,ijk->ij", diff, diff)
        nearest = np.partition(distance2, kth=k, axis=1)[:, 1:k + 1]
        means[start:stop] = np.sqrt(nearest).mean(axis=1)

    mean = float(np.mean(means))
    std = float(np.std(means))
    threshold = mean + float(std_ratio) * std
    return source_indices[means <= threshold]


def _refine_plane_svd(points):
    """Least-squares plane refinement. Return normalized (normal, d, rms)."""
    pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if pts.shape[0] < 3:
        raise ValueError("at least 3 points are required")
    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        raise ValueError("degenerate plane points")
    normal = normal / norm
    d = -float(np.dot(normal, centroid))
    residual = pts @ normal + d
    rms = float(np.sqrt(np.mean(residual * residual)))
    return normal, d, rms


def ransac_plane(points, distance_threshold, num_iterations, sample_limit=50000,
                 seed=7):
    """Return (a,b,c,d), inlier_indices for a normalized refined plane."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape[0] < 3:
        raise ValueError("at least 3 points are required")

    rng = np.random.default_rng(seed)
    if pts.shape[0] > sample_limit:
        sample_idx = rng.choice(pts.shape[0], size=sample_limit, replace=False)
        sample = pts[sample_idx]
    else:
        sample = pts

    best_model = None
    best_count = -1
    best_error = float("inf")
    n_sample = sample.shape[0]

    for _ in range(int(num_iterations)):
        i0, i1, i2 = rng.choice(n_sample, size=3, replace=False)
        p0, p1, p2 = sample[i0], sample[i1], sample[i2]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -float(np.dot(normal, p0))
        distances = np.abs(sample @ normal + d)
        inlier_mask = distances < float(distance_threshold)
        count = int(np.count_nonzero(inlier_mask))
        error = float(np.mean(distances[inlier_mask])) if count else float("inf")
        if count < best_count or (count == best_count and error >= best_error):
            continue
        best_model = (normal, d)
        best_count = count
        best_error = error

    if best_model is None:
        raise RuntimeError("RANSAC failed to find a plane")

    normal, d = best_model
    threshold = float(distance_threshold)
    full_distances = np.abs(pts @ normal + d)
    inliers = np.flatnonzero(full_distances < threshold)

    # Two-stage least-squares refinement on full inliers.  This mirrors the
    # runtime wall estimator: RANSAC proposes a plane, SVD refines it, then the
    # refined model gets one final support/re-fit pass.
    for _ in range(2):
        if inliers.shape[0] < 3:
            break
        normal, d, _ = _refine_plane_svd(pts[inliers])
        full_distances = np.abs(pts @ normal + d)
        refined = np.flatnonzero(full_distances < threshold)
        if refined.shape[0] < 3:
            break
        inliers = refined

    return (
        [float(normal[0]), float(normal[1]), float(normal[2]), float(d)],
        inliers.tolist(),
    )


def segment_planes_iterative_ransac(
        points, *, max_planes=8, max_iterations=2000,
        distance_threshold=0.015, min_inliers=80, voxel_size=0.01,
        sor_mean_k=12, sor_std_ratio=1.0, sor_max_points=3500,
        max_fit_points=7000, seed=7, removal_threshold_scale=1.25,
        min_global_inlier_ratio=0.03):
    """Extract multiple planes with robust sequential RANSAC.

    Pipeline for each candidate:
      full remaining cloud
        -> voxel representatives
        -> bounded statistical outlier removal
        -> RANSAC hypothesis
        -> full-cloud inlier re-evaluation
        -> SVD refinement
        -> slightly expanded plane removal
        -> next candidate

    Returned inlier_indices always reference the original input array so callers
    can preserve image-pixel support and sketch geometry.
    """
    pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    finite = np.all(np.isfinite(pts), axis=1)
    remaining = np.flatnonzero(finite)
    total_count = int(remaining.shape[0])
    minimum = max(3, int(min_inliers))
    if total_count < minimum:
        return []

    threshold = float(distance_threshold)
    remove_scale = max(1.0, float(removal_threshold_scale))
    min_ratio = max(0.0, float(min_global_inlier_ratio))
    planes = []

    for segment_id in range(max(1, int(max_planes))):
        remaining_count = int(remaining.shape[0])
        if remaining_count < minimum:
            break

        local = pts[remaining]
        representative_local = voxel_representative_indices(
            local, float(voxel_size)
        )
        fit_indices = remaining[representative_local]

        sor_local = statistical_outlier_indices(
            pts[fit_indices],
            mean_k=sor_mean_k,
            std_ratio=sor_std_ratio,
            max_points=sor_max_points,
            seed=int(seed) + 101 * segment_id,
        )
        sor_fit_indices = fit_indices[sor_local]
        if sor_fit_indices.shape[0] >= minimum:
            fit_indices = sor_fit_indices

        if fit_indices.shape[0] < minimum:
            break

        try:
            model, _ = ransac_plane(
                pts[fit_indices],
                threshold,
                max_iterations,
                sample_limit=max(3, int(max_fit_points)),
                seed=int(seed) + 9973 * segment_id,
            )
        except (ValueError, RuntimeError):
            break

        normal = np.asarray(model[:3], dtype=np.float64)
        d = float(model[3])

        # Re-evaluate on every remaining original point, then refine twice.
        inlier_local = np.flatnonzero(
            np.abs(pts[remaining] @ normal + d) <= threshold
        )
        if inlier_local.shape[0] < minimum:
            break

        rms = float("inf")
        for _ in range(2):
            normal, d, rms = _refine_plane_svd(
                pts[remaining[inlier_local]]
            )
            refined_local = np.flatnonzero(
                np.abs(pts[remaining] @ normal + d) <= threshold
            )
            inlier_local = refined_local
            if inlier_local.shape[0] < minimum:
                break

        if inlier_local.shape[0] < minimum:
            break

        inlier_indices = remaining[inlier_local]
        segment_ratio = (
            float(inlier_indices.shape[0]) / float(remaining_count)
        )
        global_ratio = (
            float(inlier_indices.shape[0]) / float(total_count)
        )
        if planes and global_ratio < min_ratio:
            break

        remove_mask = (
            np.abs(pts[remaining] @ normal + d)
            <= threshold * remove_scale
        )
        removed_count = int(np.count_nonzero(remove_mask))
        if removed_count < int(inlier_indices.shape[0]):
            remove_mask = np.zeros(remaining_count, dtype=bool)
            remove_mask[inlier_local] = True
            removed_count = int(inlier_indices.shape[0])

        planes.append({
            "model": [
                float(normal[0]), float(normal[1]), float(normal[2]), float(d)
            ],
            "inlier_indices": inlier_indices.tolist(),
            "inlier_count": int(inlier_indices.shape[0]),
            "rms_m": float(rms),
            "segment_id": int(segment_id),
            "segment_remaining_count": int(remaining_count),
            "segment_inlier_ratio": float(segment_ratio),
            "global_inlier_ratio": float(global_ratio),
            "removed_count": int(removed_count),
            "remaining_after_removal": int(remaining_count - removed_count),
        })
        remaining = remaining[~remove_mask]

    return planes
