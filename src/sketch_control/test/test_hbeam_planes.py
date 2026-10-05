"""Visible H-section faces must retain measured, separate pixel support."""
import numpy as np
import pytest

from sketch_control.multi_plane_geometry import extract_planes


def _folded_beam(width=240, height=320, narrow=24, angle=35., rotate=False, distance=1.2):
    """Ray/plane intersections: recessed web meets an orthogonal flange.

    The third face is parallel to the web, with a real depth discontinuity.
    All RGB pixels have the same value: geometry must find the fold.
    """
    y, x = np.mgrid[:height, :width]
    t = np.deg2rad(angle)
    web = np.array([np.sin(t), 0., np.cos(t)])
    flange = np.array([np.cos(t), 0., -np.sin(t)])
    ray = np.stack(((x-width/2)/500, (y-height/2)/500, np.ones_like(x)), axis=-1)
    boundary = width//2 + narrow//2
    web_d = distance * web[2]
    join_ray = np.array([(boundary-width/2)/500, 0., 1.])
    join = join_ray * (web_d / (join_ray @ web))
    side_d = join @ flange
    depth = web_d / (ray @ web)
    labels = np.zeros_like(x)
    side = x >= boundary
    depth[side] = side_d / (ray[side] @ flange)
    labels[side] = 1
    front = x < width//2 - narrow//2
    depth[front] = (web_d-.15) / (ray[front] @ web)
    labels[front] = 2
    depth += np.random.default_rng(73).normal(0, .0006, depth.shape)
    if rotate:
        depth, labels = depth.T.copy(), labels.T.copy()
        web, flange = web[[1, 0, 2]], flange[[1, 0, 2]]
    yy, xx = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
    z = depth[yy, xx].ravel()
    pixels = np.c_[xx.ravel(), yy.ravel()]
    points = np.c_[(pixels[:, 0]-depth.shape[1]/2)*z/500,
                   (pixels[:, 1]-depth.shape[0]/2)*z/500, z]
    truth = labels[yy, xx].ravel()
    return points, pixels, truth, [web, flange, web], depth.shape


@pytest.mark.parametrize('rotate', [False, True])
@pytest.mark.parametrize('narrow', [16, 24])
@pytest.mark.parametrize('structure', ['generic', 'hbeam'])
def test_narrow_hbeam_web_survives_without_color_edges(rotate, narrow, structure):
    points, pixels, truth, normals, shape = _folded_beam(rotate=rotate, narrow=narrow)
    planes = extract_planes(points, pixels, pixel_stride=4,
                            rgb=np.full((*shape, 3), 100, np.uint8),
                            structure=structure, iterations=300)
    assert len(planes) == 3
    for label, expected in enumerate(normals):
        target = pixels[truth == label]
        center = target.mean(axis=0)
        plane = min(planes, key=lambda p: np.linalg.norm(np.mean(p['polygon_px'], axis=0)-center))
        assert abs(np.dot(plane['normal'], expected)) > .995
        assert plane['inlier_count'] >= .85*len(target)
        polygon = np.array(plane['polygon_px'])
        direction = 1 if rotate else 0
        assert polygon[:, direction].min() >= target[:, direction].min()-4
        assert polygon[:, direction].max() <= target[:, direction].max()+4


def test_small_nearby_face_keeps_dense_support_after_voxel_sampling():
    # 100 measured points but fewer than 80 one-centimetre voxels. A small
    # flange still has enough 2D support to fit a well-conditioned plane.
    y, x = np.mgrid[:10, :10]
    pixels = np.c_[x.ravel()*4, y.ravel()*4]
    points = np.c_[x.ravel()*.003, y.ravel()*.003, np.ones(x.size)*.4]
    planes = extract_planes(points, pixels, pixel_stride=4, iterations=100)
    assert len(planes) == 1
    assert planes[0]['inlier_count'] == 100
    assert abs(planes[0]['normal'][2]) > .999


def test_close_hbeam_fold_is_not_averaged_into_one_plane():
    points, pixels, truth, normals, shape = _folded_beam(distance=.4, narrow=24)
    planes = extract_planes(points, pixels, pixel_stride=4, iterations=300)
    assert len(planes) == 3
    web = min(planes, key=lambda p: abs(p['center'][0]))
    assert abs(np.dot(web['normal'], normals[0])) > .995
    assert web['inlier_count'] >= .85*np.count_nonzero(truth == 0)
    assert np.max(np.array(web['polygon_px'])[:, 0]) <= 132


@pytest.mark.parametrize('frame', range(12))
def test_recorded_depth_models_are_invariant_to_rgb_shadows(frame):
    from pathlib import Path
    scene = np.load(Path(__file__).parent/'fixtures/hbeam_depth.npz')
    depth = scene['depths'][frame]
    y, x = np.mgrid[:depth.shape[0], :depth.shape[1]]
    z = depth.ravel(); pixels = np.c_[x.ravel()*4, y.ravel()*4]
    valid = np.isfinite(z) & (z > .15) & (z < 5.)
    pixels, z = pixels[valid], z[valid]; K = scene['K']
    points = np.c_[(pixels[:, 0]-K[0, 2])*z/K[0, 0],
                   (pixels[:, 1]-K[1, 2])*z/K[1, 1], z]
    # This recording has biased depth, not calibrated plane ground truth.
    # The former count==4 / fixed web annotation encoded an image-based split
    # the user identified as wrong. It must not force that split back in.
    baseline = extract_planes(points, pixels, pixel_stride=4, structure='hbeam')
    assert baseline
    rgb = scene['rgb'].copy()
    rgb[:, 24+frame*8:] = rgb[:, 24+frame*8:]//4
    planes = extract_planes(points, pixels, pixel_stride=4, rgb=rgb, structure='hbeam')
    assert len(planes) == len(baseline)
    for before, after in zip(baseline, planes):
        np.testing.assert_allclose(after['normal'], before['normal'], atol=1e-12)
        before_offset = np.dot(before['normal'], before['center'])
        after_offset = np.dot(after['normal'], after['center'])
        assert after_offset == pytest.approx(before_offset, abs=1e-12)
        assert after['inlier_count'] >= 80


def test_hbeam_profile_preserves_short_transverse_face():
    # A short transverse surface at the end of an elongated member is a real
    # different orientation, not a residual shard of its longitudinal face.
    y, x = np.mgrid[:320, :80]
    depth = np.where(y < 280, 1., 1/(1+(y-280)/500))
    yy, xx = np.mgrid[0:320:4, 0:80:4]
    z = depth[yy, xx].ravel(); pixels = np.c_[xx.ravel(), yy.ravel()]
    points = np.c_[(pixels[:, 0]-40)*z/500, (pixels[:, 1]-160)*z/500, z]
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 2
    transverse = max(planes, key=lambda p: abs(p['normal'][1]))
    expected = np.array([0., 1., .76]); expected /= np.linalg.norm(expected)
    assert abs(np.dot(transverse['normal'], expected)) > .995
    assert transverse['inlier_count'] >= 160


def test_support_ratio_describes_each_face_not_the_whole_connected_beam():
    points, pixels, _, _, _ = _folded_beam(narrow=24)
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 3
    assert all(p['segment_inlier_ratio'] > .95 for p in planes)
    assert not any(p['partial_support'] for p in planes)


def test_partly_occluded_web_is_not_suppressed_as_a_short_shard():
    points, pixels, truth, normals, shape = _folded_beam(width=80, narrow=24)
    occluded = (truth == 0) & (pixels[:, 1] < 160)
    flange_normal = normals[1]
    flange_d = np.median(points[truth == 1] @ flange_normal)
    rays = points[occluded]/points[occluded, 2, None]
    points[occluded] = rays*(flange_d/(rays @ flange_normal))[:, None]
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 3
    web = min(planes, key=lambda p: abs(p['center'][0]))
    assert abs(np.dot(web['normal'], normals[0])) > .995
    assert web['inlier_count'] >= 190
    assert np.min(np.array(web['polygon_px'])[:, 1]) >= 156


def test_grazing_flange_is_connected_despite_large_pixel_depth_steps():
    points, pixels, truth, normals, _ = _folded_beam(width=200, narrow=40, angle=20)
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 3
    flange = max(planes, key=lambda p: abs(np.dot(p['normal'], normals[1])))
    assert abs(np.dot(flange['normal'], normals[1])) > .995
    assert flange['inlier_count'] >= .9*np.count_nonzero(truth == 1)


def test_short_parallel_offset_face_is_not_a_residual_shard():
    y, x = np.mgrid[0:320:4, 0:80:4]
    z = np.where(y < 240, 1., 1.020).ravel()
    pixels = np.c_[x.ravel(), y.ravel()]
    points = np.c_[(pixels[:, 0]-40)*z/500, (pixels[:, 1]-160)*z/500, z]
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 2
    assert sorted(round(p['center'][2], 3) for p in planes) == [1., 1.02]


@pytest.mark.parametrize('shadow_x', [110, 118, 150])
@pytest.mark.parametrize('rotate', [False, True])
def test_shadow_edge_cannot_cut_or_duplicate_a_measured_beam_face(shadow_x, rotate):
    points, pixels, truth, normals, shape = _folded_beam(rotate=rotate)
    rgb = np.full((*shape, 3), 180, np.uint8)
    if rotate:
        rgb[shadow_x:, :] = 35
    else:
        rgb[:, shadow_x:] = 35
    planes = extract_planes(points, pixels, pixel_stride=4, rgb=rgb,
                            structure='hbeam', iterations=300)
    assert len(planes) == 3
    for label, expected in enumerate(normals):
        target = pixels[truth == label]
        plane = min(planes, key=lambda p: np.linalg.norm(np.mean(p['polygon_px'], axis=0)-target.mean(axis=0)))
        assert abs(np.dot(plane['normal'], expected)) > .995
        assert plane['inlier_count'] >= .95*len(target)
        polygon = np.array(plane['polygon_px'])
        direction = 1 if rotate else 0
        assert polygon[:, direction].min() <= target[:, direction].min()+4
        assert polygon[:, direction].max() >= target[:, direction].max()-4


@pytest.mark.parametrize('frame', [0, 1, 2])
def test_shadow_recording_preserves_models_and_reports_raw_depth_disagreement(frame):
    # User annotation: the former pink "plane 5" is part of the cyan "plane 1",
    # not an additional steel face. Its depth is curved/bias-corrupted.
    # RGB must not change the 3D hypotheses; raw-depth diagnostics must expose
    # disagreement rather than claim that RANSAC recovered the physical faces.
    from pathlib import Path
    scene = np.load(Path(__file__).parent/'fixtures/shadow_beam_depth.npz')
    depth = scene['depths'][frame]
    y, x = np.mgrid[:depth.shape[0], :depth.shape[1]]
    z = depth.ravel(); pixels = np.c_[x.ravel()*4, y.ravel()*4]
    valid = np.isfinite(z) & (z > .15) & (z < 5.)
    pixels, z = pixels[valid], z[valid]; K = scene['K']
    points = np.c_[(pixels[:, 0]-K[0, 2])*z/K[0, 0],
                   (pixels[:, 1]-K[1, 2])*z/K[1, 1], z]
    baseline = extract_planes(points, pixels, pixel_stride=4, structure='hbeam')
    planes = extract_planes(points, pixels, pixel_stride=4, rgb=scene['rgb'], structure='hbeam')
    assert len(planes) == len(baseline)
    for before, after in zip(baseline, planes):
        np.testing.assert_allclose(after['normal'], before['normal'], atol=1e-12)
        assert np.dot(after['normal'], after['center']) == pytest.approx(
            np.dot(before['normal'], before['center']), abs=1e-12)
    target = [p for p in planes if 116 <= np.mean(np.array(p['polygon_px'])[:, 0]) <= 208]
    main = max(target, key=lambda p: p['inlier_count'])
    assert abs(np.dot(main['normal'], [-.634, -.052, -.772])) > .99
    assert main['inlier_count'] > 1000
    for plane in planes:
        ratio = plane['interior_support_ratio']
        assert ratio is None or 0. <= ratio <= 1.
        assert plane['depth_consistency_warning'] == (ratio is not None and ratio < .8)


@pytest.mark.parametrize('noise', [.001, .003, .006, .010])
def test_depth_interior_validation_keeps_a_noisy_true_plane(noise):
    y, x = np.mgrid[0:320:4, 0:240:4]
    pixels = np.c_[x.ravel(), y.ravel()]
    z = 1.2+np.random.default_rng(25).normal(0, noise, x.size)
    points = np.c_[(pixels[:, 0]-120)*z/500, (pixels[:, 1]-160)*z/500, z]
    rgb = np.full((320, 240, 3), 180, np.uint8); rgb[:, 100:145] = 25
    planes = extract_planes(points, pixels, pixel_stride=4, rgb=rgb,
                            structure='hbeam', iterations=300)
    assert len(planes) == 1
    assert abs(planes[0]['normal'][2]) > .995
    assert planes[0]['inlier_count'] > .75*len(points)


@pytest.mark.parametrize('step', [.020, .026, .300])
def test_depth_interior_validation_keeps_a_plane_behind_a_real_occluder(step):
    y, x = np.mgrid[0:320:4, 0:240:4]
    pixels = np.c_[x.ravel(), y.ravel()]
    foreground = (abs(x-120) < 64) & (abs(y-160) < 96)
    z = np.where(foreground, 1.2-step, 1.2).ravel()
    points = np.c_[(pixels[:, 0]-120)*z/500, (pixels[:, 1]-160)*z/500, z]
    planes = extract_planes(points, pixels, pixel_stride=4, structure='hbeam', iterations=300)
    assert len(planes) == 2
    np.testing.assert_allclose(sorted(p['center'][2] for p in planes), [1.2-step, 1.2], atol=1e-9)
    if step == .020:
        background = max(planes, key=lambda p: p['center'][2])
        assert background['interior_support_ratio'] < .8
        assert background['depth_consistency_warning']


@pytest.mark.parametrize('narrow', [24, 40, 64])
@pytest.mark.parametrize('noise', [.003, .004])
def test_noisy_hbeam_crease_keeps_the_real_web(narrow, noise):
    points, pixels, truth, normals, _ = _folded_beam(narrow=narrow)
    rays = points/points[:, 2, None]
    z = points[:, 2]+np.random.default_rng(55).normal(0, noise, len(points))
    points = rays*z[:, None]
    planes = extract_planes(points, pixels, pixel_stride=4,
                            structure='hbeam', iterations=300)
    assert len(planes) == 3
    for label, expected in enumerate(normals):
        target = pixels[truth == label]
        plane = min(planes, key=lambda p: np.linalg.norm(
            np.mean(p['polygon_px'], axis=0)-target.mean(axis=0)))
        assert abs(np.dot(plane['normal'], expected)) > .995
        assert plane['inlier_count'] >= .8*len(target)


def test_reclaim_cannot_steal_a_narrow_faces_crease_samples():
    from sketch_control.organized_planes import _lattice, _reclaim_support
    y, x = np.mgrid[0:40, -20:4]
    pixels = np.c_[4*x.ravel(), 4*y.ravel()]
    first = x.ravel() < 0
    z = np.where(first, 1., 1/(1-.008*x.ravel()))
    points = np.c_[.008*x.ravel()*z, .008*y.ravel()*z, z]
    xy, a, b = _lattice(pixels, 4)
    segments = []
    for mask, model in ((first, np.array([0., 0., 1., -1.])),
                        (~first, np.array([-1., 0., 1., -1.])/np.sqrt(2))):
        indices = np.flatnonzero(mask)
        segments.append(dict(model=model, inlier_indices=indices,
                             inlier_count=len(indices), rms_m=0.,
                             segment_inlier_ratio=1., region_rms_m=0.))
    refined = _reclaim_support(points, pixels, xy, a, b, np.ones(len(points), int),
                               segments, .015, 80, 4)
    assert len(refined) == 2
    assert not np.intersect1d(refined[0]['inlier_indices'], refined[1]['inlier_indices']).size
    assert set(refined[1]['inlier_indices']) >= set(segments[1]['inlier_indices'])
