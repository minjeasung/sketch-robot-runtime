"""D405 canvas preserves plane axes while using safely inscribed coverage."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

projector = pytest.importorskip('sketch_control.wall_projector_node')
from sketch_control.d405_view_geometry import measurement_samples
from sketch_control.rotation_utils import quat_to_matrix


def _front_extent(rotation, *, focal_length=444.444444, camera=(0., 0., 0.),
                  centroid=(0., 0., .32), normal=(0., 0., -1.)):
    captured = []
    node = SimpleNamespace(
        locked_work_area=None, _locked_extent=None,
        d405_K=np.array([[focal_length, 0., 319.5], [0., focal_length, 239.5], [0., 0., 1.]]),
        _d405_warn=lambda message: pytest.fail(message),
        _warp_overlay_publish=lambda rgb, extent, *args: captured.append(extent),
    )
    node._front_view_size = MethodType(projector.WallProjectorNode._front_view_size, node)
    projector.WallProjectorNode._d405_phase_a(
        node, np.zeros((480, 640, 3), dtype=np.uint8), rotation, np.asarray(camera),
        np.asarray(centroid), np.asarray(normal), None)
    assert len(captured) == 1
    return captured[0], node.d405_K


def _assert_visible_rectangle(extent, rotation, intrinsics, camera=(0., 0., 0.)):
    assert abs(np.dot(extent[1] - extent[0], extent[3] - extent[0])) < 1e-9
    np.testing.assert_allclose(extent[2], extent[1] + extent[3] - extent[0], atol=1e-9)
    pixels = ((extent - camera) @ rotation) @ intrinsics.T
    assert (pixels[:, 2] > 0).all()
    pixels = pixels[:, :2] / pixels[:, 2:]
    assert (pixels >= -1e-6).all()
    assert (pixels <= [639 + 1e-6, 479 + 1e-6]).all()


@pytest.mark.parametrize('roll_degrees', [0., 30., 60., 150.])
def test_canvas_preserves_original_plane_axes_and_image_signs_at_camera_roll(roll_degrees):
    angle = np.deg2rad(roll_degrees)
    rotation = quat_to_matrix([0., 0., np.sin(angle / 2), np.cos(angle / 2)])
    extent, intrinsics = _front_extent(rotation)
    right = extent[1] - extent[0]
    down = extent[3] - extent[0]
    # The pre-existing canvas axes on z=.32 are X/Y, with camera-facing signs.
    sign = 1. if roll_degrees < 90. else -1.
    np.testing.assert_allclose(right / np.linalg.norm(right), [sign, 0, 0], atol=1e-9)
    np.testing.assert_allclose(down / np.linalg.norm(down), [0, sign, 0], atol=1e-9)
    _assert_visible_rectangle(extent, rotation, intrinsics)


def test_recorded_right_camera_footprint_supports_measurement_without_rotating_canvas():
    # 2026-10-16 quiet right run: recorded target camera plan and static calibration.
    zed_to_world = np.array([
        [-.9317773919853206, .14158098520221657, -.3342838859625215],
        [.36303015272155337, .3633911953523559, -.8579953073038055],
        [0., -.9208157599485122, -.389997866956275],
    ])
    tcp = quat_to_matrix([-.00020604398756828176, .49021017081649026,
                          .00004098765451503522, .8716042360452103])
    mount = quat_to_matrix([0., -np.sqrt(.5), np.sqrt(.5), 0.])
    rotation = zed_to_world.T @ tcp @ mount
    normal = zed_to_world.T @ np.array([0., 1., 0.])
    camera = .32 * normal
    extent, intrinsics = _front_extent(
        rotation, camera=camera, centroid=np.zeros(3), normal=normal)
    width = np.linalg.norm(extent[1] - extent[0])
    height = np.linalg.norm(extent[3] - extent[0])
    assert [width, height] == pytest.approx([.25448, .28647], abs=2e-4)
    assert min(width, height) * .8 > .16
    # Existing ZED-referenced canvas right axis expressed back in the world.
    right_world = zed_to_world @ (extent[1] - extent[0]) / width
    assert abs(np.dot(right_world, [.98838498, 0., .15197085])) == pytest.approx(1., abs=1e-6)
    _assert_visible_rectangle(extent, rotation, intrinsics, camera)
    area = extent.mean(axis=0) + .8 * (extent - extent.mean(axis=0))
    assert measurement_samples(camera, area, normal, margin=.08)


@pytest.mark.parametrize('tilt_degrees', [-30., 25.])
@pytest.mark.parametrize('roll_degrees', [0., 60.])
def test_oblique_canvas_is_rectangular_and_all_corners_remain_in_measured_image(
        tilt_degrees, roll_degrees):
    tilt, roll = np.deg2rad([tilt_degrees, roll_degrees])
    rotation = (quat_to_matrix([0., np.sin(tilt / 2), 0., np.cos(tilt / 2)])
                @ quat_to_matrix([0., 0., np.sin(roll / 2), np.cos(roll / 2)]))
    extent, intrinsics = _front_extent(rotation)
    np.testing.assert_allclose(extent[:, 2], .32, atol=1e-9)
    _assert_visible_rectangle(extent, rotation, intrinsics)


def test_genuinely_narrow_optical_coverage_still_rejects_eight_cm_inset():
    extent, _ = _front_extent(np.eye(3), focal_length=1800.)
    area = extent.mean(axis=0) + .8 * (extent - extent.mean(axis=0))
    with pytest.raises(ValueError, match='too small'):
        measurement_samples([0, 0, 0], area, [0, 0, -1], margin=.08)
