import numpy as np
import pytest

from sketch_control.work_area_geometry import (
    WorkAreaGeometryError,
    bilinear_quad_point,
    generate_fill_strokes,
    outside_pixel_rect_indices,
    outside_quad_3d_indices,
    pixel_rect_from_points,
    point_in_quad_3d,
    quad_size_m,
)


def test_fill_points_stay_inside_selected_pixel_rectangle():
    rect = (120.0, 80.0, 520.0, 680.0)
    strokes = generate_fill_strokes(
        rect,
        work_area_width_m=0.40,
        work_area_height_m=0.60,
        roller_length_m=0.175,
        overlap=0.30,
    )

    assert len(strokes) >= 2
    points = [point for stroke in strokes for point in stroke]
    assert outside_pixel_rect_indices(points, rect, tolerance_px=0.0) == []
    # The first/last centers retain exactly half a roller of physical margin.
    expected_margin_px = 0.5 * 0.175 / 0.40 * (rect[2] - rect[0])
    assert strokes[0][0][0] == pytest.approx(rect[0] + expected_margin_px)
    assert strokes[-1][0][0] == pytest.approx(rect[2] - expected_margin_px)


def test_fill_rejects_work_area_narrower_than_measured_roller_length():
    with pytest.raises(WorkAreaGeometryError, match="smaller than roller"):
        generate_fill_strokes(
            (10.0, 20.0, 110.0, 220.0),
            work_area_width_m=0.17,
            work_area_height_m=0.30,
            roller_length_m=0.175,
            overlap=0.30,
        )


def test_fill_rejects_area_without_valid_stroke_height():
    with pytest.raises(WorkAreaGeometryError, match="valid stroke"):
        generate_fill_strokes(
            (10.0, 20.0, 210.0, 21.0),
            work_area_width_m=0.30,
            work_area_height_m=0.001,
            roller_length_m=0.175,
            overlap=0.30,
            minimum_stroke_length_m=0.005,
        )


def test_free_sketch_outside_selected_rect_is_reported_not_clipped():
    rect = pixel_rect_from_points(
        [(100.0, 100.0), (400.0, 300.0)], 800, 600
    )
    points = [(100.0, 100.0), (250.0, 200.0), (410.0, 200.0), (-1.0, 120.0)]
    assert outside_pixel_rect_indices(points, rect, tolerance_px=0.5) == [2, 3]


def test_bilinear_mapping_and_3d_quad_containment_agree():
    corners = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.4, 0.0, 0.0],
            [0.4, 0.6, 0.0],
            [0.0, 0.6, 0.0],
        ]
    )
    inside = bilinear_quad_point(corners, 0.25, 0.75)
    boundary = bilinear_quad_point(corners, 1.0, 0.5)
    outside = np.array([0.4015, 0.3, 0.0])

    assert point_in_quad_3d(inside, corners)
    assert point_in_quad_3d(boundary, corners)
    assert not point_in_quad_3d(outside, corners, boundary_tolerance_m=0.001)
    assert outside_quad_3d_indices([inside, boundary, outside], corners) == [2]
    assert quad_size_m(corners) == pytest.approx((0.4, 0.6))


def test_3d_containment_rejects_point_off_plane():
    corners = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.4, 0.0, 1.0],
            [0.4, 0.6, 1.0],
            [0.0, 0.6, 1.0],
        ]
    )
    assert not point_in_quad_3d(
        [0.2, 0.3, 1.01], corners, plane_tolerance_m=0.002
    )
