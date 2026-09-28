import numpy as np

from sketch_control.multi_plane_geometry import extract_planes
from sketch_control.pointcloud_utils import segment_planes_iterative_ransac


def _best_alignment(segments, expected):
    expected = np.asarray(expected, dtype=float)
    return max(
        abs(float(np.dot(np.asarray(s["model"][:3], dtype=float), expected)))
        for s in segments
    )


def test_robust_iterative_ransac_recovers_two_planes_with_outliers():
    rng = np.random.default_rng(22)

    front_x = rng.uniform(-0.55, 0.55, size=2800)
    front_y = rng.uniform(-0.35, 0.35, size=2800)
    front_z = 1.20 + rng.normal(0.0, 0.004, size=2800)
    front = np.column_stack((front_x, front_y, front_z))

    side_x = 0.45 + rng.normal(0.0, 0.004, size=1700)
    side_y = rng.uniform(-0.35, 0.35, size=1700)
    side_z = rng.uniform(0.55, 1.35, size=1700)
    side = np.column_stack((side_x, side_y, side_z))

    outliers = rng.uniform(
        [-0.75, -0.55, 0.35],
        [0.85, 0.55, 1.65],
        size=(500, 3),
    )
    points = np.vstack((front, side, outliers))

    segments = segment_planes_iterative_ransac(
        points,
        max_planes=3,
        max_iterations=400,
        distance_threshold=0.012,
        min_inliers=300,
        voxel_size=0.01,
        sor_max_points=2200,
        max_fit_points=3000,
        seed=12,
        min_global_inlier_ratio=0.05,
    )

    assert len(segments) >= 2
    assert _best_alignment(segments, [0.0, 0.0, 1.0]) > 0.995
    assert _best_alignment(segments, [1.0, 0.0, 0.0]) > 0.995
    assert all(s["rms_m"] < 0.01 for s in segments[:2])
    assert segments[0]["remaining_after_removal"] < len(points)


def test_sketch_planes_keep_original_pixel_support_after_prefiltering():
    rng = np.random.default_rng(31)

    y1, z1 = np.meshgrid(
        np.linspace(-0.30, 0.30, 24),
        np.linspace(0.60, 1.20, 24),
    )
    wall_a = np.column_stack((
        0.55 + rng.normal(0.0, 0.0015, size=y1.size),
        y1.ravel(),
        z1.ravel(),
    ))

    x2, y2 = np.meshgrid(
        np.linspace(-0.35, 0.35, 22),
        np.linspace(-0.30, 0.30, 22),
    )
    wall_b = np.column_stack((
        x2.ravel(),
        y2.ravel(),
        1.05 + rng.normal(0.0, 0.0015, size=x2.size),
    ))

    points = np.vstack((wall_a, wall_b))
    pixels = np.column_stack((
        np.arange(points.shape[0], dtype=float),
        np.arange(points.shape[0], dtype=float) % 40,
    ))

    planes = extract_planes(
        points,
        pixels,
        max_planes=4,
        min_points=120,
        threshold=0.006,
        iterations=300,
        voxel_size=0.008,
        sor_max_points=1800,
        max_fit_points=2200,
        min_global_inlier_ratio=0.05,
    )

    assert len(planes) == 2
    assert all(p["inlier_count"] >= 450 for p in planes)
    assert all(len(p["polygon_px"]) >= 3 for p in planes)
    assert all(len(p["support_polygon"]) >= 3 for p in planes)
    assert abs(float(np.dot(planes[0]["normal"], planes[1]["normal"]))) < 0.02
