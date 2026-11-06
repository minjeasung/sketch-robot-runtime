"""Geometric spray is bounded by the original cone and real first-hit CAD faces."""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
ENVELOPE = dict(axial_start_m=-.002, start_radius_m=.003,
                length_m=.5, end_radius_m=.300)


def module():
    assert importlib.util.find_spec('platform_spray_visual'), 'Spray visual module is required'
    import platform_spray_visual
    return platform_spray_visual


def plate(z=.2, half=.2):
    return dict(vertices=[[-half, -half, z], [half, -half, z],
                          [half, half, z], [-half, half, z]],
                triangles=[[0, 1, 2], [0, 2, 3]])


def visual(*meshes, **kwargs):
    return module().SprayVisual(None, ENVELOPE,
        dict(frame_id='World', meshes=list(meshes or [plate()])),
        cell_size_m=.02, max_cells=5000, **kwargs)


def test_original_frustum_is_not_resized_to_the_surface():
    v = visual(plate(half=.01))
    assert v.radius_at(.5) == pytest.approx(.301188)
    points = v.cone_points(np.eye(4))
    assert points[:, 2].min() == pytest.approx(-.002)
    assert points[:, 2].max() == pytest.approx(.498)
    assert np.linalg.norm(points[:, :2], axis=1).max() == pytest.approx(.300)


def test_enabled_spray_hits_real_200mm_plate_in_source_envelope():
    v = visual()
    report = v.update(np.eye(4), True, 'first')
    hit = v.cell_centers[v.covered]
    assert len(hit) > 20
    assert np.allclose(hit[:, 2], .2)
    assert np.linalg.norm(hit[:, :2], axis=1).max() <= .122988 + 1e-9
    assert np.linalg.norm(hit[:, :2], axis=1).max() > .11
    assert 0.035 < report['covered_area_m2'] < .055
    assert report['physical_output_enabled'] is False
    assert report['coverage_kind'] == 'geometric_surface_cells'


def test_beyond_plate_edges_cannot_create_imaginary_paint():
    v = visual(plate(half=.04))
    report = v.update(np.eye(4), True, 'small')
    assert report['covered_area_m2'] == pytest.approx(.0064)
    assert np.all(np.abs(v.cell_vertices[v.covered, :, :2]) <= .04)


def test_occluding_plate_prevents_coverage_on_back_plate():
    # More than eight source triangles exercises traversal across BVH leaves.
    v = visual(*(plate(z=z, half=.2) for z in [.1, .2, .3, .4, .5]))
    v.update(np.eye(4), True, 'blocked')
    assert v.covered.any()
    assert np.allclose(v.cell_centers[v.covered, 2], .1)


def test_disabled_update_preserves_history_without_new_coverage():
    v = visual()
    v.update(np.eye(4), True, 'first')
    before = v.covered.copy()
    moved = np.eye(4); moved[0, 3] = .15
    report = v.update(moved, False, 'first')
    assert np.array_equal(v.covered, before)
    assert not v.footprint.any()
    assert report['enabled'] is False


def test_new_plan_resets_history_even_when_spray_is_disabled():
    v = visual()
    v.update(np.eye(4), True, 'first')
    v.update(np.eye(4), False, 'second')
    assert not v.covered.any()


def test_motion_accumulates_coverage_beyond_initial_footprint():
    v = visual()
    v.update(np.eye(4), True, 'first')
    before = v.covered.copy()
    moved = np.eye(4); moved[0, 3] = .15
    v.update(moved, True, 'first')
    assert np.all(v.covered[before])
    assert np.count_nonzero(v.covered) > np.count_nonzero(before)
    assert np.max(v.cell_centers[v.covered, 0]) > .18


def test_cells_behind_nozzle_are_not_covered():
    for z in [-.02, -.1]:
        v = visual(plate(z=z))
        v.update(np.eye(4), True, 'outside')
        assert not v.covered.any()


def test_world_pose_rotates_axis_to_world_x():
    mesh = plate()
    # Rotate both plate and nozzle by +90 degrees about world Y.
    rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    mesh['vertices'] = (np.array(mesh['vertices']) @ rotation.T).tolist()
    pose = np.eye(4); pose[:3, :3] = rotation
    v = visual(mesh)
    v.update(pose, True, 'rotated')
    assert v.covered.any()
    assert np.allclose(v.cell_centers[v.covered, 0], .2)


def test_mesh_budget_is_bounded_without_removing_cad_faces():
    v = module().SprayVisual(None, ENVELOPE, dict(frame_id='World', meshes=[plate(half=100)]),
                            cell_size_m=.001, max_cells=100)
    assert len(v.cell_centers) <= 100
    assert np.sum(v.cell_areas) == pytest.approx(40000.)


def test_bad_surface_frame_and_nonrigid_pose_are_rejected():
    with pytest.raises(ValueError, match='World'):
        module().SprayVisual(None, ENVELOPE, dict(frame_id='camera', meshes=[plate()]))
    v = visual()
    pose = np.eye(4); pose[0, 0] = 2
    with pytest.raises(ValueError, match='rigid'):
        v.update(pose, True, 'invalid')


def test_source_slope_projects_coverage_beyond_the_cad_visual_cap():
    v = visual(plate(z=.5001, half=.4), projection_end_m=.5)
    report = v.update(np.eye(4), True, '500mm')
    assert v.covered.any()
    assert np.max(np.linalg.norm(v.cell_centers[v.covered, :2], axis=1)) > .28
    points = v.cone_points(np.eye(4))
    assert points[:, 2].max() == pytest.approx(.5)
    assert np.linalg.norm(points[:, :2], axis=1).max() == pytest.approx(.301188)
    assert report['source_axial_end_m'] == pytest.approx(.498)
    assert report['visual_extrapolation_m'] == pytest.approx(.002)
    assert report['coverage_axial_limit_m'] is None


def test_real_usd_batches_geometry_and_sensor_visibility():
    pytest.importorskip('pxr')
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    v = module().SprayVisual(stage, ENVELOPE, dict(frame_id='World', meshes=[plate()]))
    v.update(np.eye(4), True, 'usd')
    root = stage.GetPrimAtPath(v.root_path)
    cone = UsdGeom.BasisCurves(stage.GetPrimAtPath(v.root_path + '/Envelope'))
    footprint = UsdGeom.Mesh(stage.GetPrimAtPath(v.root_path + '/Footprint'))
    assert len(cone.GetPointsAttr().Get()) > 80
    assert len(footprint.GetFaceVertexCountsAttr().Get()) > 0
    assert len(list(root.GetChildren())) == 3
    assert all(not any('Collision' in api or 'RigidBody' in api for api in prim.GetAppliedSchemas())
               for prim in stage.Traverse())
    v.set_sensor_capture(True)
    assert UsdGeom.Imageable(root).ComputeVisibility() == 'invisible'
    v.set_sensor_capture(False)
    assert UsdGeom.Imageable(root).ComputeVisibility() == 'inherited'
    v.update(np.eye(4), False, 'usd')
    assert cone.GetVisibilityAttr().Get() == 'invisible'
    assert len(footprint.GetFaceVertexCountsAttr().Get()) == 0
    coverage = UsdGeom.Mesh(stage.GetPrimAtPath(v.root_path + '/Coverage'))
    assert len(coverage.GetFaceVertexCountsAttr().Get()) > 0
