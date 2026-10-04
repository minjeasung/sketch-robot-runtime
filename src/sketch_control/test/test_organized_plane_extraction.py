"""Image topology must keep web/flange support and disconnected faces apart."""
import numpy as np

from sketch_control.multi_plane_geometry import extract_planes


def _cloud(depth, stride=4):
    y, x = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[y, x].ravel()
    pixels = np.column_stack((x.ravel(), y.ravel()))
    valid = np.isfinite(z)
    pixels, z = pixels[valid], z[valid]
    points = np.column_stack(((pixels[:, 0]-120)*z/500,
                              (pixels[:, 1]-200)*z/500, z))
    return points, pixels


def _extract(depth, rgb=None):
    points, pixels = _cloud(depth)
    return extract_planes(points, pixels, pixel_stride=4, rgb=rgb,
                          min_points=80, iterations=300, threshold=.006)


def test_disconnected_coplanar_patches_never_bridge_the_gap():
    depth = np.ones((240, 240))
    depth[:, 80:160] = np.nan
    planes = _extract(depth)
    assert len(planes) == 2
    for plane in planes:
        x = np.array(plane['polygon_px'])[:, 0]
        assert x.max() <= 76 or x.min() >= 160


def test_folded_surface_keeps_both_faces_beside_a_disconnected_patch():
    _, x = np.mgrid[:240, :240]
    depth = np.where(x < 80, 1., 1 / (1 + .8 * (x - 80) / 500))
    depth[:, 160:200] = np.nan
    depth[:, 200:] = 1.5
    planes = _extract(depth)
    assert len(planes) == 3
    tilted_normal = np.array([.8, 0., 1.064])
    tilted_normal /= np.linalg.norm(tilted_normal)
    assert any(abs(np.dot(plane['normal'], tilted_normal)) > .99
               for plane in planes)


def test_web_keeps_its_pixel_support_when_it_touches_a_flange():
    y, x = np.mgrid[:400, :240]
    # A recessed web between two flanges. Right flange meets the web
    # continuously in depth, but has a different normal and a visible edge.
    depth = np.where(x < 80, 1.0, 1.25)
    depth = np.where(x >= 160, 1/(.8 + (x-160)/500), depth)
    depth += np.random.default_rng(4).normal(0, .001, depth.shape)
    rgb = np.zeros((400, 240, 3), dtype=np.uint8)
    rgb[:, :80] = 180
    rgb[:, 80:160] = 80
    rgb[:, 160:] = 140
    planes = _extract(depth, rgb)
    assert len(planes) == 3
    web = min(planes, key=lambda p: abs(p['center'][2]-1.25))
    polygon = np.array(web['polygon_px'])
    assert polygon[:, 0].min() >= 80
    assert polygon[:, 0].max() <= 160
    assert np.ptp(polygon[:, 1]) >= 380
    assert abs(np.dot(web['normal'], [0, 0, 1])) > .995
    assert web['inlier_count'] > 1700


def test_color_stripe_does_not_split_one_geometric_wall():
    depth = np.ones((400, 240))
    rgb = np.full((400, 240, 3), 180, np.uint8)
    rgb[:, 80:160] = 60
    planes = _extract(depth, rgb)
    assert len(planes) == 1
    assert planes[0]['inlier_count'] == 6000


def test_thin_boundary_strip_is_not_offered_as_a_work_plane():
    depth = np.full((400, 240), np.nan)
    depth[:, 100:108] = 1.
    assert _extract(depth) == []


def test_empty_organized_selection_has_no_candidates():
    assert _extract(np.full((40, 40), np.nan)) == []


def test_bent_thin_edge_is_not_a_wide_work_plane():
    y, x = np.mgrid[:400, :240]
    depth = np.full((400, 240), np.nan)
    left = 60 + (y/400)**2*80
    depth[(x >= left) & (x < left+8)] = 1.
    assert _extract(depth) == []


def test_adjacent_color_regions_merge_across_multiple_boundaries():
    depth = np.ones((400, 240))
    rgb = np.full((400, 240, 3), 180, np.uint8)
    rgb[200:, :120] = 60
    rgb[200:, 120:] = 110
    assert len(_extract(depth, rgb)) == 1
