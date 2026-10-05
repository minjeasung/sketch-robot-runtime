"""Hand-computed concave selections must constrain full spray segments."""
import numpy as np
import pytest
from sketch_control import work_area_geometry as geometry
from sketch_control.zed_spray_projection import select_work_area, validate_work_area_request

U = [[10, 10], [90, 10], [90, 90], [60, 90], [60, 40], [40, 40], [40, 90], [10, 90]]


def test_polygon_selection_retains_concavity_and_bounds():
    assert geometry.work_area_polygon(U, 101, 101).tolist() == U
    quad = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]
    np.testing.assert_allclose(select_work_area(quad, (101, 101), U),
                               [[.1, .1, 0], [.9, .1, 0], [.9, .9, 0], [.1, .9, 0]])


@pytest.mark.parametrize('points', [
    [[10, 10], [90, 90], [10, 90], [90, 10]],
    [[10, 10], [30, 30], [50, 50]],
    [[-1, 10], [90, 10], [90, 90]],
])
def test_bad_work_boundaries_are_rejected(points):
    with pytest.raises(ValueError):
        geometry.work_area_polygon(points, 101, 101)


def test_spray_does_not_cross_concavity_even_when_both_endpoints_are_inside():
    assert not geometry.strokes_inside_polygon([[(20, 80), (80, 80)]], U)
    assert geometry.strokes_inside_polygon([[(20, 80), (20, 20), (80, 20), (80, 80)]], U)


def test_fill_clips_each_pass_to_the_actual_selection():
    strokes = geometry.clip_strokes_to_polygon([[(50, 10), (50, 90)], [(20, 90), (20, 10)]], U)
    assert strokes == [((50., 10.), (50., 40.)), ((20., 90.), (20., 10.))]
    assert geometry.strokes_inside_polygon(strokes, U)


def test_work_request_accepts_one_polygon_without_changing_identity():
    stamp, points = validate_work_area_request(dict(source='zed', plane_generation_id='zed:g',
        header=dict(frame_id='wall_front', stamp=dict(sec=5, nanosec=7)), pixels=U), 'zed:g')
    assert stamp == dict(sec=5, nanosec=7)
    assert points.tolist() == U
