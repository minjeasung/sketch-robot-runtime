from dataclasses import replace

import numpy as np

from sketch_control.plane_lifecycle import (
    SinglePlaneFitConfig,
    capture_once_and_fit_plane,
    validate_single_plane_result,
)


def _config(**overrides):
    config = SinglePlaneFitConfig(
        voxel_size_m=0.0,
        ransac_distance_threshold_m=0.004,
        ransac_iterations=400,
        min_roi_points=150,
        min_inliers=100,
        min_inlier_ratio=0.65,
        max_rms_residual_m=0.004,
        max_residual_m=0.012,
        max_normal_delta_deg=12.0,
        # D405 is authoritative for depth.  The ZED seed may therefore be
        # displaced along the surface normal by substantially more than the
        # D405 fit noise.
        max_plane_shift_m=0.25,
        max_tf_age_s=0.25,
    )
    return replace(config, **overrides)


def _horizontal_plane(rng, count, z, x_bounds, y_bounds, noise=0.0015):
    x = rng.uniform(*x_bounds, size=count)
    y = rng.uniform(*y_bounds, size=count)
    return np.column_stack(
        [x, y, z + rng.normal(0.0, noise, size=count)]
    )


def _fit(points, config=None, reference_normal=(0.0, 0.0, 1.0)):
    config = config or _config()
    result = capture_once_and_fit_plane(
        points,
        reference_point=(0.0, 0.0, 0.0),
        reference_normal=reference_normal,
        config=config,
    )
    decision = validate_single_plane_result(result, config, 0.01)
    return result, decision


def test_broad_clutter_accepts_large_d405_wall_at_14cm_zed_offset():
    rng = np.random.default_rng(91)
    wall = _horizontal_plane(
        rng, 3800, 0.14, (-0.32, 0.32), (-0.26, 0.26)
    )
    # Exactly 10,000 non-planar clutter points occupy the broad ROI while
    # staying outside the candidate-centered narrow support band.
    lower_clutter = np.column_stack(
        [
            rng.uniform(-0.35, 0.35, 5000),
            rng.uniform(-0.30, 0.30, 5000),
            rng.uniform(-0.25, 0.10, 5000),
        ]
    )
    upper_clutter = np.column_stack(
        [
            rng.uniform(-0.35, 0.35, 5000),
            rng.uniform(-0.30, 0.30, 5000),
            rng.uniform(0.18, 0.40, 5000),
        ]
    )

    result, decision = _fit(
        np.vstack([wall, lower_clutter, upper_clutter])
    )

    assert decision.accepted
    assert result.roi_point_count == 13800
    assert 0.25 < result.broad_inlier_ratio < 0.35
    assert result.inlier_ratio > 0.95
    assert result.support_point_count >= 3750
    assert result.support_span_major_m > 0.50
    assert result.support_span_minor_m > 0.40
    assert abs(abs(result.plane_shift_m_from_zed) - 0.14) < 0.003


def test_two_comparable_planes_are_rejected_as_ambiguous():
    rng = np.random.default_rng(92)
    first = _horizontal_plane(
        rng, 3800, 0.14, (-0.32, 0.32), (-0.26, 0.26)
    )
    second = _horizontal_plane(
        rng, 3300, -0.11, (-0.30, 0.30), (-0.25, 0.25)
    )

    result, decision = _fit(np.vstack([first, second]))

    assert not decision.accepted
    assert decision.rejection_reason == "ambiguous_multiple_planes"
    assert result.secondary_plane_inlier_count >= 3200
    assert result.secondary_plane_relative_inliers > 0.80
    assert result.secondary_plane_separation_m > 0.20


def test_intersecting_competing_plane_is_rejected_by_normal_difference():
    rng = np.random.default_rng(98)
    wall = _horizontal_plane(
        rng, 3800, 0.14, (-0.32, 0.32), (-0.26, 0.26), noise=0.0012
    )
    xy = rng.uniform((-0.30, -0.25), (0.30, 0.25), size=(3300, 2))
    competing = np.column_stack(
        [
            xy,
            0.14 + 0.30 * xy[:, 0] + rng.normal(0.0, 0.0012, 3300),
        ]
    )

    result, decision = _fit(np.vstack([wall, competing]))

    assert not decision.accepted
    assert decision.rejection_reason == "ambiguous_multiple_planes"
    assert result.secondary_plane_relative_inliers > 0.70
    assert result.secondary_plane_separation_m < 0.01
    assert result.secondary_plane_normal_delta_deg > 15.0


def test_unstructured_scattered_cloud_is_rejected_by_broad_ratio():
    rng = np.random.default_rng(93)
    scattered = rng.uniform(
        low=(-0.35, -0.30, -0.25),
        high=(0.35, 0.30, 0.40),
        size=(12000, 3),
    )

    result, decision = _fit(scattered)

    assert not decision.accepted
    assert decision.rejection_reason == "broad_inlier_ratio_rejected"
    assert result.broad_inlier_ratio < 0.05


def test_candidate_with_wrong_normal_is_rejected():
    rng = np.random.default_rng(94)
    xy = rng.uniform((-0.25, -0.25), (0.25, 0.25), size=(2500, 2))
    tilted = np.column_stack(
        [
            xy,
            0.12 + 0.45 * xy[:, 0] + rng.normal(0.0, 0.001, 2500),
        ]
    )

    result, decision = _fit(tilted)

    assert not decision.accepted
    assert decision.rejection_reason == "normal_delta_rejected"
    assert result.normal_delta_deg_from_zed > 20.0


def test_numerous_points_on_small_parallel_object_lack_wall_extent():
    rng = np.random.default_rng(95)
    small_plate = _horizontal_plane(
        rng, 2000, 0.14, (-0.04, 0.04), (-0.04, 0.04), noise=0.001
    )

    result, decision = _fit(small_plate)

    assert not decision.accepted
    assert decision.rejection_reason == "insufficient_support_extent"
    assert result.inlier_ratio > 0.98
    assert result.support_span_major_m < 0.10
    assert result.support_span_minor_m < 0.10


def test_large_parallel_patch_away_from_target_ray_is_rejected():
    rng = np.random.default_rng(96)
    off_target_patch = _horizontal_plane(
        rng, 2500, 0.14, (0.25, 0.65), (-0.20, 0.20), noise=0.001
    )

    result, decision = _fit(off_target_patch)

    assert not decision.accepted
    assert decision.rejection_reason == "target_support_offset_rejected"
    assert result.target_support_offset_m > 0.40


def test_candidate_with_too_little_support_is_rejected():
    rng = np.random.default_rng(97)
    sparse_wall = _horizontal_plane(
        rng, 120, 0.14, (-0.30, 0.30), (-0.25, 0.25), noise=0.001
    )
    config = _config(min_roi_points=100, min_support_points=150)

    result, decision = _fit(sparse_wall, config=config)

    assert not decision.accepted
    assert decision.rejection_reason == "too_few_support_points"
    assert result.support_point_count == 120
