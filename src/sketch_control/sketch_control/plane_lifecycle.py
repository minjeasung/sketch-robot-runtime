"""Pure D405 single-capture fitting and plane lifecycle helpers.

This module deliberately has no ROS imports.  The D405 node is responsible for
turning a ``PointCloud2`` into points in the reference frame; the functions and
state machine below make the safety-critical decisions deterministic and easy
to unit test.
"""

from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from sketch_control.pointcloud_utils import ransac_plane, voxel_downsample


def _digest(prefix: str, payload: Dict[str, Any], length: int = 20) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return f"{prefix}_{hashlib.sha256(encoded).hexdigest()[:length]}"


def canonical_work_area_id(
    frame_id: str,
    corners: Sequence[Sequence[float]],
    precision: int = 5,
) -> str:
    """Return a stable fallback ID for a work-area quadrilateral.

    An explicit upstream ID is preferable because selecting the exact same
    rectangle twice is still a new operator event.  This hash is only the
    deterministic compatibility fallback for legacy ``PoseArray`` input.
    """

    values = [
        [round(float(value), precision) for value in point[:3]]
        for point in corners[:4]
    ]
    return _digest(
        "work_area",
        {"frame_id": str(frame_id), "corners": values},
    )


def canonical_target_id(
    frame_id: str,
    position: Sequence[float],
    orientation_xyzw: Sequence[float],
    precision: int = 5,
) -> str:
    """Return a stable ID for the legacy bare target ``PoseStamped``."""

    return _digest(
        "target",
        {
            "frame_id": str(frame_id),
            "position": [round(float(v), precision) for v in position[:3]],
            "orientation_xyzw": [
                round(float(v), precision) for v in orientation_xyzw[:4]
            ],
        },
    )


def calibration_file_sha256(path: str) -> str:
    """Hash a calibration file, returning a deterministic state for absence."""

    if not str(path).strip():
        return "disabled"
    calibration_path = Path(path).expanduser()
    if not calibration_path.is_file():
        return "missing"
    digest = hashlib.sha256()
    with calibration_path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class SinglePlaneFitConfig:
    """Parameters used by the pure single-cloud fit and validation layers."""

    voxel_size_m: float = 0.003
    ransac_distance_threshold_m: float = 0.004
    ransac_iterations: int = 180
    min_roi_points: int = 150
    min_inliers: int = 100
    min_inlier_ratio: float = 0.65
    max_rms_residual_m: float = 0.004
    max_residual_m: float = 0.012
    max_normal_delta_deg: float = 12.0
    max_plane_shift_m: float = 0.08
    max_tf_age_s: float = 0.25

    # Two-stage fit controls.  The first RANSAC runs over the broad D405 ROI
    # and finds a candidate.  Quality is then evaluated inside a narrow band
    # around that candidate instead of treating every piece of foreground
    # clutter in the broad ROI as a wall-plane outlier.
    support_band_half_width_m: float = 0.006
    min_broad_inlier_ratio: float = 0.15
    min_support_points: int = 100
    support_span_quantile: float = 0.02
    min_support_span_m: float = 0.15
    max_target_support_offset_m: float = 0.32
    min_secondary_plane_inliers: int = 100
    max_secondary_plane_relative_inliers: float = 0.60
    min_secondary_plane_separation_m: float = 0.020
    min_secondary_plane_normal_delta_deg: float = 5.0


@dataclass(frozen=True)
class SinglePlaneFitResult:
    """Plane candidate and all quality values derived from exactly one cloud."""

    fit_succeeded: bool
    fit_error: str
    center: Optional[Tuple[float, float, float]]
    normal: Optional[Tuple[float, float, float]]
    roi_point_count: int
    voxel_point_count: int
    inlier_count: int
    inlier_ratio: float
    rms_residual_m: float
    max_residual_m: float
    normal_delta_deg_from_zed: float
    plane_shift_m_from_zed: float
    broad_inlier_count: int = 0
    broad_inlier_ratio: float = 0.0
    support_point_count: int = 0
    support_span_major_m: float = 0.0
    support_span_minor_m: float = 0.0
    target_support_offset_m: float = math.inf
    secondary_plane_inlier_count: int = 0
    secondary_plane_relative_inliers: float = 0.0
    secondary_plane_separation_m: float = 0.0
    secondary_plane_normal_delta_deg: float = 0.0

    @classmethod
    def failed(
        cls,
        reason: str,
        roi_point_count: int = 0,
        voxel_point_count: int = 0,
    ) -> "SinglePlaneFitResult":
        return cls(
            fit_succeeded=False,
            fit_error=str(reason),
            center=None,
            normal=None,
            roi_point_count=int(roi_point_count),
            voxel_point_count=int(voxel_point_count),
            inlier_count=0,
            inlier_ratio=0.0,
            rms_residual_m=math.inf,
            max_residual_m=math.inf,
            normal_delta_deg_from_zed=math.inf,
            plane_shift_m_from_zed=math.inf,
        )

    def metrics(self) -> Dict[str, Any]:
        return {
            "roi_point_count": int(self.roi_point_count),
            "voxel_point_count": int(self.voxel_point_count),
            "inlier_count": int(self.inlier_count),
            "inlier_ratio": float(self.inlier_ratio),
            "rms_residual_m": float(self.rms_residual_m),
            "max_residual_m": float(self.max_residual_m),
            "normal_delta_deg_from_zed": float(
                self.normal_delta_deg_from_zed
            ),
            "plane_shift_m_from_zed": float(self.plane_shift_m_from_zed),
            "broad_inlier_count": int(self.broad_inlier_count),
            "broad_inlier_ratio": float(self.broad_inlier_ratio),
            "support_point_count": int(self.support_point_count),
            "support_span_major_m": float(self.support_span_major_m),
            "support_span_minor_m": float(self.support_span_minor_m),
            "target_support_offset_m": float(self.target_support_offset_m),
            "secondary_plane_inlier_count": int(
                self.secondary_plane_inlier_count
            ),
            "secondary_plane_relative_inliers": float(
                self.secondary_plane_relative_inliers
            ),
            "secondary_plane_separation_m": float(
                self.secondary_plane_separation_m
            ),
            "secondary_plane_normal_delta_deg": float(
                self.secondary_plane_normal_delta_deg
            ),
        }


@dataclass(frozen=True)
class PlaneValidationDecision:
    accepted: bool
    rejection_reason: str = ""


def capture_once_and_fit_plane(
    roi_points: Sequence[Sequence[float]],
    reference_point: Sequence[float],
    reference_normal: Sequence[float],
    config: SinglePlaneFitConfig,
) -> SinglePlaneFitResult:
    """Fit one plane from one already-transformed ROI point set.

    This function never accumulates points from another capture.  A failed fit
    is represented as data so validation and status reporting remain uniform.
    """

    points = np.asarray(roi_points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,):
        return SinglePlaneFitResult.failed("invalid_roi_shape")
    points = points[np.isfinite(points).all(axis=1)]
    roi_count = int(points.shape[0])

    reference_point_np = np.asarray(reference_point, dtype=float)
    reference = np.asarray(reference_normal, dtype=float)
    if (
        reference_point_np.shape != (3,)
        or reference.shape != (3,)
        or not np.all(np.isfinite(reference_point_np))
        or not np.all(np.isfinite(reference))
    ):
        return SinglePlaneFitResult.failed(
            "invalid_reference_plane",
            roi_point_count=roi_count,
            voxel_point_count=roi_count,
        )

    if roi_count < 3:
        return SinglePlaneFitResult.failed(
            "too_few_roi_points",
            roi_point_count=roi_count,
            voxel_point_count=roi_count,
        )

    fit_points = points
    if float(config.voxel_size_m) > 0.0:
        fit_points = voxel_downsample(fit_points, config.voxel_size_m)
    fit_count = int(fit_points.shape[0])
    if fit_count < 3:
        return SinglePlaneFitResult.failed(
            "too_few_voxel_points",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )

    support_band = float(config.support_band_half_width_m)
    span_quantile = float(config.support_span_quantile)
    min_broad_ratio = float(config.min_broad_inlier_ratio)
    min_support_span = float(config.min_support_span_m)
    max_target_offset = float(config.max_target_support_offset_m)
    max_secondary_relative = float(
        config.max_secondary_plane_relative_inliers
    )
    min_secondary_separation = float(
        config.min_secondary_plane_separation_m
    )
    min_secondary_normal_delta = float(
        config.min_secondary_plane_normal_delta_deg
    )
    if (
        not math.isfinite(support_band)
        or support_band <= float(config.ransac_distance_threshold_m)
        or not math.isfinite(span_quantile)
        or not 0.0 <= span_quantile < 0.5
        or not math.isfinite(min_broad_ratio)
        or not 0.0 <= min_broad_ratio <= 1.0
        or int(config.min_support_points) < 3
        or not math.isfinite(min_support_span)
        or min_support_span < 0.0
        or not math.isfinite(max_target_offset)
        or max_target_offset < 0.0
        or int(config.min_secondary_plane_inliers) < 3
        or not math.isfinite(max_secondary_relative)
        or not 0.0 <= max_secondary_relative <= 1.0
        or not math.isfinite(min_secondary_separation)
        or min_secondary_separation < 0.0
        or not math.isfinite(min_secondary_normal_delta)
        or not 0.0 <= min_secondary_normal_delta <= 90.0
    ):
        return SinglePlaneFitResult.failed(
            "invalid_fit_config",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )

    # Stage 1: locate the strongest plane in the broad D405 ROI.  The broad
    # ratio remains an explicit safety metric, but it is not reused as the
    # final purity ratio because the broad ROI intentionally contains clutter.
    try:
        broad_model, broad_inliers = ransac_plane(
            fit_points,
            config.ransac_distance_threshold_m,
            config.ransac_iterations,
        )
    except Exception:
        return SinglePlaneFitResult.failed(
            "ransac_failed",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )

    broad_indices = np.asarray(broad_inliers, dtype=int)
    if (
        broad_indices.size == 0
        or np.any(broad_indices < 0)
        or np.any(broad_indices >= fit_count)
    ):
        return SinglePlaneFitResult.failed(
            "too_few_inliers",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )

    broad_normal = np.asarray(broad_model[:3], dtype=float)
    broad_normal_norm = float(np.linalg.norm(broad_normal))
    reference_norm = float(np.linalg.norm(reference))
    if (
        broad_normal.shape != (3,)
        or not np.all(np.isfinite(broad_normal))
        or not math.isfinite(broad_normal_norm)
        or not math.isfinite(reference_norm)
        or broad_normal_norm < 1e-12
        or reference_norm < 1e-12
    ):
        return SinglePlaneFitResult.failed(
            "invalid_plane_normal",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
    broad_normal /= broad_normal_norm
    reference /= reference_norm
    if float(np.dot(broad_normal, reference)) < 0.0:
        broad_normal = -broad_normal
    broad_center = np.asarray(fit_points[broad_indices], dtype=float).mean(axis=0)
    broad_d = -float(np.dot(broad_normal, broad_center))
    broad_count = int(broad_indices.size)
    broad_ratio = float(broad_count / max(fit_count, 1))

    # Stage 2: refit only the geometrically coherent band around the broad
    # candidate.  This is the population used for the configured final inlier
    # ratio, so unrelated foreground/background points cannot dilute it.
    distance_from_broad = np.abs(fit_points @ broad_normal + broad_d)
    support_mask = distance_from_broad <= support_band
    support_points = np.asarray(fit_points[support_mask], dtype=float)
    support_count = int(support_points.shape[0])
    if support_count < 3:
        failed = SinglePlaneFitResult.failed(
            "too_few_support_points",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
        return replace(
            failed,
            broad_inlier_count=broad_count,
            broad_inlier_ratio=broad_ratio,
            support_point_count=support_count,
        )

    try:
        refined_model, refined_inliers = ransac_plane(
            support_points,
            config.ransac_distance_threshold_m,
            config.ransac_iterations,
        )
    except Exception:
        failed = SinglePlaneFitResult.failed(
            "support_refit_failed",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
        return replace(
            failed,
            broad_inlier_count=broad_count,
            broad_inlier_ratio=broad_ratio,
            support_point_count=support_count,
        )

    inlier_indices = np.asarray(refined_inliers, dtype=int)
    if (
        inlier_indices.size == 0
        or np.any(inlier_indices < 0)
        or np.any(inlier_indices >= support_count)
    ):
        failed = SinglePlaneFitResult.failed(
            "too_few_inliers",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
        return replace(
            failed,
            broad_inlier_count=broad_count,
            broad_inlier_ratio=broad_ratio,
            support_point_count=support_count,
        )

    normal = np.asarray(refined_model[:3], dtype=float)
    normal_norm = float(np.linalg.norm(normal))
    if (
        normal.shape != (3,)
        or not np.all(np.isfinite(normal))
        or not math.isfinite(normal_norm)
        or normal_norm < 1e-12
    ):
        return SinglePlaneFitResult.failed(
            "invalid_plane_normal",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
    normal /= normal_norm
    if float(np.dot(normal, reference)) < 0.0:
        normal = -normal

    inlier_points = np.asarray(support_points[inlier_indices], dtype=float)
    inlier_center = inlier_points.mean(axis=0)
    plane_d = -float(np.dot(normal, inlier_center))
    residuals = np.abs(inlier_points @ normal + plane_d)
    rms_residual = float(np.sqrt(np.mean(np.square(residuals))))
    max_residual = float(np.max(residuals))

    dot = float(np.clip(np.dot(normal, reference), -1.0, 1.0))
    normal_delta_deg = math.degrees(math.acos(dot))
    signed_shift = float(np.dot(reference_point_np, normal) + plane_d)
    refined_center = reference_point_np - signed_shift * normal
    inlier_count = int(inlier_indices.size)

    # Robust two-dimensional support extent.  Using the central quantiles in
    # the PCA-aligned plane axes prevents a few distant depth artifacts from
    # making a small parallel object look like a wall-sized surface.
    centered_inliers = inlier_points - inlier_center
    try:
        _u, _s, axes = np.linalg.svd(centered_inliers, full_matrices=False)
        in_plane_axes = np.asarray(axes[:2], dtype=float)
        projected = centered_inliers @ in_plane_axes.T
        lower = np.quantile(projected, span_quantile, axis=0)
        upper = np.quantile(projected, 1.0 - span_quantile, axis=0)
        spans = np.sort(np.asarray(upper - lower, dtype=float))[::-1]
        support_span_major = float(spans[0])
        support_span_minor = float(spans[1])
    except Exception:
        return SinglePlaneFitResult.failed(
            "support_geometry_failed",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )
    target_offset_vector = inlier_center - refined_center
    target_offset_vector -= float(np.dot(target_offset_vector, normal)) * normal
    target_support_offset = float(np.linalg.norm(target_offset_vector))

    # Search the population outside the primary support band for a second
    # structured plane.  A comparable second plane makes the selection
    # geometrically ambiguous and is rejected by validation below.
    secondary_count = 0
    secondary_separation = 0.0
    secondary_normal_delta = 0.0
    residual_population = np.asarray(fit_points[~support_mask], dtype=float)
    if residual_population.shape[0] >= 3:
        try:
            secondary_model, secondary_inliers = ransac_plane(
                residual_population,
                config.ransac_distance_threshold_m,
                config.ransac_iterations,
            )
            secondary_indices = np.asarray(secondary_inliers, dtype=int)
            if (
                secondary_indices.size > 0
                and np.all(secondary_indices >= 0)
                and np.all(secondary_indices < residual_population.shape[0])
            ):
                secondary_normal = np.asarray(
                    secondary_model[:3], dtype=float
                )
                secondary_normal_norm = float(
                    np.linalg.norm(secondary_normal)
                )
                if (
                    np.all(np.isfinite(secondary_normal))
                    and math.isfinite(secondary_normal_norm)
                    and secondary_normal_norm >= 1e-12
                ):
                    secondary_normal /= secondary_normal_norm
                    secondary_points = residual_population[secondary_indices]
                    secondary_center = secondary_points.mean(axis=0)
                    secondary_count = int(secondary_indices.size)
                    secondary_separation = abs(
                        float(np.dot(secondary_center, normal) + plane_d)
                    )
                    secondary_dot = float(
                        np.clip(abs(np.dot(secondary_normal, normal)), -1.0, 1.0)
                    )
                    secondary_normal_delta = math.degrees(
                        math.acos(secondary_dot)
                    )
        except Exception:
            secondary_count = 0
            secondary_separation = 0.0
            secondary_normal_delta = 0.0
    secondary_relative = float(secondary_count / max(broad_count, 1))

    derived_values = np.concatenate(
        [
            refined_center,
            normal,
            np.asarray(
                [
                    rms_residual,
                    max_residual,
                    normal_delta_deg,
                    signed_shift,
                    broad_ratio,
                    support_span_major,
                    support_span_minor,
                    target_support_offset,
                    secondary_relative,
                    secondary_separation,
                    secondary_normal_delta,
                ],
                dtype=float,
            ),
        ]
    )
    if not np.all(np.isfinite(derived_values)):
        return SinglePlaneFitResult.failed(
            "invalid_fit_metrics",
            roi_point_count=roi_count,
            voxel_point_count=fit_count,
        )

    return SinglePlaneFitResult(
        fit_succeeded=True,
        fit_error="",
        center=tuple(float(v) for v in refined_center),
        normal=tuple(float(v) for v in normal),
        roi_point_count=roi_count,
        voxel_point_count=fit_count,
        inlier_count=inlier_count,
        inlier_ratio=float(inlier_count / max(support_count, 1)),
        rms_residual_m=rms_residual,
        max_residual_m=max_residual,
        normal_delta_deg_from_zed=float(normal_delta_deg),
        plane_shift_m_from_zed=signed_shift,
        broad_inlier_count=broad_count,
        broad_inlier_ratio=broad_ratio,
        support_point_count=support_count,
        support_span_major_m=support_span_major,
        support_span_minor_m=support_span_minor,
        target_support_offset_m=target_support_offset,
        secondary_plane_inlier_count=secondary_count,
        secondary_plane_relative_inliers=secondary_relative,
        secondary_plane_separation_m=secondary_separation,
        secondary_plane_normal_delta_deg=secondary_normal_delta,
    )


def validate_single_plane_result(
    result: SinglePlaneFitResult,
    config: SinglePlaneFitConfig,
    transform_age_s: Optional[float],
) -> PlaneValidationDecision:
    """Apply all configured quality limits to one fitted plane candidate."""

    if not result.fit_succeeded:
        return PlaneValidationDecision(False, result.fit_error or "fit_failed")
    quality_values = (
        result.inlier_ratio,
        result.rms_residual_m,
        result.max_residual_m,
        result.normal_delta_deg_from_zed,
        result.plane_shift_m_from_zed,
        result.broad_inlier_ratio,
        result.support_span_major_m,
        result.support_span_minor_m,
        result.target_support_offset_m,
        result.secondary_plane_relative_inliers,
        result.secondary_plane_separation_m,
        result.secondary_plane_normal_delta_deg,
    )
    if (
        result.center is None
        or result.normal is None
        or not np.all(np.isfinite(np.asarray(result.center, dtype=float)))
        or not np.all(np.isfinite(np.asarray(result.normal, dtype=float)))
        or not all(math.isfinite(float(value)) for value in quality_values)
        or result.roi_point_count < 0
        or result.voxel_point_count < 0
        or result.inlier_count < 0
        or result.broad_inlier_count < 0
        or result.support_point_count < 0
        or result.secondary_plane_inlier_count < 0
        or result.inlier_count > result.voxel_point_count
        or result.inlier_count > result.support_point_count
        or result.broad_inlier_count > result.voxel_point_count
        or result.support_point_count > result.voxel_point_count
        or not 0.0 <= result.inlier_ratio <= 1.0
        or not 0.0 <= result.broad_inlier_ratio <= 1.0
        or result.support_span_major_m < result.support_span_minor_m
        or result.support_span_minor_m < 0.0
        or result.target_support_offset_m < 0.0
        or result.secondary_plane_relative_inliers < 0.0
        or result.secondary_plane_separation_m < 0.0
        or not 0.0 <= result.secondary_plane_normal_delta_deg <= 90.0
    ):
        return PlaneValidationDecision(False, "invalid_fit_metrics")
    if result.roi_point_count < int(config.min_roi_points):
        return PlaneValidationDecision(False, "too_few_roi_points")
    if result.broad_inlier_ratio < float(config.min_broad_inlier_ratio):
        return PlaneValidationDecision(False, "broad_inlier_ratio_rejected")
    if result.support_point_count < int(config.min_support_points):
        return PlaneValidationDecision(False, "too_few_support_points")
    if result.inlier_count < int(config.min_inliers):
        return PlaneValidationDecision(False, "too_few_inliers")
    if result.inlier_ratio < float(config.min_inlier_ratio):
        return PlaneValidationDecision(False, "inlier_ratio_rejected")
    if result.rms_residual_m > float(config.max_rms_residual_m):
        return PlaneValidationDecision(False, "rms_residual_rejected")
    if result.max_residual_m > float(config.max_residual_m):
        return PlaneValidationDecision(False, "max_residual_rejected")
    if result.normal_delta_deg_from_zed > float(config.max_normal_delta_deg):
        return PlaneValidationDecision(False, "normal_delta_rejected")
    if abs(result.plane_shift_m_from_zed) > float(config.max_plane_shift_m):
        return PlaneValidationDecision(False, "plane_shift_rejected")
    if (
        result.secondary_plane_inlier_count
        >= int(config.min_secondary_plane_inliers)
        and result.secondary_plane_relative_inliers
        >= float(config.max_secondary_plane_relative_inliers)
        and (
            result.secondary_plane_separation_m
            >= float(config.min_secondary_plane_separation_m)
            or result.secondary_plane_normal_delta_deg
            >= float(config.min_secondary_plane_normal_delta_deg)
        )
    ):
        return PlaneValidationDecision(False, "ambiguous_multiple_planes")
    if (
        result.support_span_major_m < float(config.min_support_span_m)
        or result.support_span_minor_m < float(config.min_support_span_m)
    ):
        return PlaneValidationDecision(False, "insufficient_support_extent")
    if result.target_support_offset_m > float(
        config.max_target_support_offset_m
    ):
        return PlaneValidationDecision(False, "target_support_offset_rejected")
    if transform_age_s is None or not math.isfinite(float(transform_age_s)):
        return PlaneValidationDecision(False, "transform_age_unavailable")
    if float(transform_age_s) < 0.0:
        return PlaneValidationDecision(False, "transform_age_invalid")
    if float(transform_age_s) > float(config.max_tf_age_s):
        return PlaneValidationDecision(False, "transform_too_old")
    return PlaneValidationDecision(True, "")


class PlaneLifecycle:
    """Pure first-accepted lock and explicit-invalidation state machine."""

    def __init__(self, scope: str, work_area_id: str = ""):
        self.scope = str(scope)
        self.work_area_id = str(work_area_id)
        self.invalidation_seq = 0
        self.plane_generation_id = self._generation_id("initial", "initial")
        self.state = "invalidated"
        self.accepted = False
        self.rejection_reason = "not_refined"
        self.capture_armed = False
        self.capture_deadline = 0.0
        self.paint_active = False
        self._pending_invalidation = None

    def _generation_id(self, reason: str, token: Any) -> str:
        return _digest(
            "plane",
            {
                "scope": self.scope,
                "work_area_id": self.work_area_id,
                "invalidation_seq": int(self.invalidation_seq),
                "reason": str(reason),
                "token": str(token),
            },
        )

    def invalidate(
        self,
        reason: str,
        *,
        work_area_id: Optional[str] = None,
        generation_id: Optional[str] = None,
        token: Any = "",
    ) -> bool:
        """Invalidate now, or defer without changing the PAINT snapshot."""

        request = {
            "reason": str(reason),
            "work_area_id": work_area_id,
            "generation_id": generation_id,
            "token": token,
        }
        if self.paint_active:
            if (
                request["work_area_id"] is None
                and self._pending_invalidation is not None
            ):
                request["work_area_id"] = self._pending_invalidation[
                    "work_area_id"
                ]
            self._pending_invalidation = request
            self.state = "paint_locked"
            return False
        self._apply_invalidation(**request)
        return True

    def _apply_invalidation(
        self,
        reason: str,
        work_area_id: Optional[str],
        generation_id: Optional[str],
        token: Any,
    ) -> None:
        if work_area_id is not None:
            self.work_area_id = str(work_area_id)
        self.invalidation_seq += 1
        self.plane_generation_id = (
            str(generation_id)
            if generation_id
            else self._generation_id(reason, token)
        )
        self.state = "invalidated"
        self.accepted = False
        self.rejection_reason = str(reason)
        self.capture_armed = False
        self.capture_deadline = 0.0

    def set_paint_active(self, active: bool) -> bool:
        """Return True when leaving PAINT applied a deferred invalidation."""

        active = bool(active)
        if active:
            self.paint_active = True
            self.capture_armed = False
            self.capture_deadline = 0.0
            self.state = "paint_locked"
            return False

        was_active = self.paint_active
        self.paint_active = False
        if was_active and self._pending_invalidation is not None:
            pending = self._pending_invalidation
            self._pending_invalidation = None
            self._apply_invalidation(**pending)
            return True
        if was_active:
            self.state = "accepted" if self.accepted else "invalidated"
        return False

    def arm_capture(self, now: float, timeout_s: float) -> PlaneValidationDecision:
        if self.paint_active:
            return PlaneValidationDecision(False, "paint_locked")
        if self.accepted:
            return PlaneValidationDecision(False, "first_accepted_locked")
        if self.capture_armed:
            return PlaneValidationDecision(False, "capture_already_armed")
        self.capture_armed = True
        self.capture_deadline = float(now) + max(0.0, float(timeout_s))
        self.state = "capture_armed"
        self.rejection_reason = ""
        return PlaneValidationDecision(True, "")

    def consume_capture(self, now: float) -> PlaneValidationDecision:
        """Atomically consume the one cloud owned by the current trigger."""

        if not self.capture_armed:
            return PlaneValidationDecision(False, "capture_not_armed")
        deadline = self.capture_deadline
        self.capture_armed = False
        self.capture_deadline = 0.0
        if float(now) > deadline:
            self.reject("capture_timeout")
            return PlaneValidationDecision(False, "capture_timeout")
        self.state = "evaluating"
        return PlaneValidationDecision(True, "")

    def expire_capture(self, now: float) -> bool:
        if not self.capture_armed or float(now) <= self.capture_deadline:
            return False
        self.capture_armed = False
        self.capture_deadline = 0.0
        self.reject("capture_timeout")
        return True

    def accept(self) -> None:
        self.capture_armed = False
        self.capture_deadline = 0.0
        self.accepted = True
        self.rejection_reason = ""
        self.state = "paint_locked" if self.paint_active else "accepted"

    def reject(self, reason: str) -> None:
        self.capture_armed = False
        self.capture_deadline = 0.0
        self.accepted = False
        self.rejection_reason = str(reason)
        self.state = "rejected"

    def snapshot(self) -> Dict[str, Any]:
        return {
            "state": self.state,
            "accepted": bool(self.accepted),
            "rejection_reason": self.rejection_reason,
            "work_area_id": self.work_area_id,
            "plane_generation_id": self.plane_generation_id,
            "paint_active": bool(self.paint_active),
            "capture_armed": bool(self.capture_armed),
            "invalidation_seq": int(self.invalidation_seq),
        }
