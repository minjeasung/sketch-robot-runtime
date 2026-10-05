import numpy as np
import cv2

from sketch_control.plane_boundaries import refine_image_boundaries
from sketch_control.organized_planes import _lattice


def _segment(points, indices, model):
    residual = points[indices] @ model[:3]+model[3]
    rms = float(np.sqrt(np.mean(residual**2)))
    return dict(model=model, inlier_indices=indices, inlier_count=len(indices),
                segment_inlier_ratio=1., global_inlier_ratio=len(indices)/len(points),
                rms_m=rms, region_rms_m=rms)


def test_rgb_refinement_cannot_disconnect_a_measured_face():
    y, x = np.mgrid[:48, -10:11]
    valid = ~((x <= 0) & (y >= 20) & (y <= 27))
    x, y = x[valid], y[valid]
    first = (x <= 0) | ((x == 1) & (y >= 19) & (y <= 28))
    z = np.where(first, 1., 1/(1-x/1000))
    points = np.c_[x*z/1000, y*z/1000, z]
    pixels = np.c_[x*4, y*4]
    models = [np.array([0., 0., 1., -1.]), np.array([-1., 0., 1., -1.])/np.sqrt(2)]
    segments = [_segment(points, np.flatnonzero(mask), model)
                for mask, model in zip([first, ~first], models)]
    _, a, b = _lattice(pixels, 4)
    refined = refine_image_boundaries(points, pixels, segments,
                                      [[2., 0., 2., 188.]], a, b, 4, .015, 80)
    assert len(refined) == 2
    for segment in refined:
        mask = np.zeros((48, 21), np.uint8)
        idx = segment['inlier_indices']; mask[y[idx], x[idx]+10] = 1
        count, _ = cv2.connectedComponents(mask, connectivity=4)
        assert count == 2  # background and exactly one measured connected face


def test_rgb_refinement_moves_only_ambiguous_points_without_refitting_planes():
    y, x = np.mgrid[0:160:4, 0:200:4]
    px = np.c_[x.ravel(), y.ravel()]
    rays = np.c_[(px[:, 0]-100)/1000, px[:, 1]/1000, np.ones(len(px))]
    first = px[:, 0] <= 100
    z = np.where(first, 1., 1/(1+.25*rays[:, 0]))
    points = rays*z[:, None]
    labels = np.where(first, 0, 1)
    labels[(px[:, 0] == 96) & (px[:, 1] % 8 == 0)] = 1
    labels[(px[:, 0] == 104) & (px[:, 1] % 8 == 4)] = 0
    models = [np.array([0., 0., 1., -1.]), np.array([.25, 0., 1., -1.])/np.sqrt(1.0625)]
    segments = [_segment(points, np.flatnonzero(labels == i), model) for i, model in enumerate(models)]
    _, a, b = _lattice(px, 4)
    refined = refine_image_boundaries(points, px, segments,
                                      [[100.5, 0., 100.5, 156.]], a, b, 4, .015, 80)
    new_labels = np.full(len(px), -1)
    for i, segment in enumerate(refined):
        np.testing.assert_array_equal(segment['model'], models[i])
        new_labels[segment['inlier_indices']] = i
    assert np.all(new_labels[px[:, 0] == 96] == 0)
    assert np.all(new_labels[px[:, 0] == 104] == 1)
    moved = new_labels != labels
    assert np.any(moved)
    assert np.max(abs(px[moved, 0]-100.5)) <= 8
    assert len(refined) == 2


def test_exported_plane_keeps_fitted_offset_after_boundary_points_change(monkeypatch):
    from sketch_control.multi_plane_geometry import extract_planes
    y, x = np.mgrid[:10, :10]
    pixels = np.c_[x.ravel()*4, y.ravel()*4]
    points = np.c_[x.ravel()*.01, y.ravel()*.01, np.full(x.size, 1.002)]
    # Geometry fitting established z=1. RGB reassignment changed which noisy
    # observations represent the boundary, not the established plane itself.
    segment = _segment(points, np.arange(len(points)), np.array([0., 0., 1., -1.]))
    segment.update(removed_count=0, remaining_after_removal=0)
    monkeypatch.setattr('sketch_control.organized_planes.spatial_plane_segments',
                        lambda *args, **kwargs: [segment])
    plane = extract_planes(points, pixels, pixel_stride=4)[0]
    assert abs(plane['center'][2]-1.) < 1e-12
    np.testing.assert_allclose(np.asarray(plane['corners'])[:, 2], 1., atol=1e-12)
    np.testing.assert_allclose(np.asarray(plane['support_polygon'])[:, 2], 1., atol=1e-12)
