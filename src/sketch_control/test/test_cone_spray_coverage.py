"""Transverse cone overlap fixes spacing while overspray is permitted."""
import numpy as np
import pytest

from sketch_control.work_area_geometry import generate_spray_fill_strokes, WorkAreaGeometryError


def strokes(width, height, direction='auto', diameter=.602376):
    return generate_spray_fill_strokes((0, 0, width*1000, height*1000),
        work_area_width_m=width, work_area_height_m=height,
        footprint_width_m=diameter, overlap=.1, direction=direction)


@pytest.mark.parametrize('width,height,expected', [
    (.6, .2, (((0., 100.), (600., 100.)),)),
    (.2, .6, (((100., 0.), (100., 600.)),)),
    (.01, .02, (((5., 0.), (5., 20.)),)),
])
def test_long_axis_one_centered_pass_even_when_cone_is_wider_than_area(width, height, expected):
    np.testing.assert_allclose(strokes(width, height), expected)


@pytest.mark.parametrize('direction,width,height,cross_axis', [
    ('horizontal', 3., 1.2, 1), ('vertical', 1.2, 3., 0),
    ('auto', 3., 1.2, 1), ('auto', 1.2, 3., 0),
])
def test_three_passes_have_exact_ten_percent_overlap_and_centered_overhang(direction, width, height, cross_axis):
    result = np.asarray(strokes(width, height, direction)) / 1000
    assert len(result) == 3
    centers = result[:, 0, cross_axis]
    assert centers == pytest.approx([.0578616, .6, 1.1421384])
    assert np.diff(centers) == pytest.approx([.5421384, .5421384])
    assert centers[0] - .301188 < 0
    assert centers[-1] + .301188 > 1.2
    assert np.sign(np.diff(result[:, :, 1-cross_axis], axis=1).ravel()).tolist() == [1, -1, 1]


def test_two_passes_cover_width_without_compressing_spacing():
    result = np.asarray(strokes(1., 3., 'vertical')) / 1000
    assert len(result) == 2
    assert result[:, 0, 0] == pytest.approx([.2289308, .7710692])


@pytest.mark.parametrize('width,height,direction,diameter', [
    (0., 1., 'auto', .6), (1., -1., 'auto', .6),
    (1., 1., 'diagonal', .6), (1., 1., 'auto', float('nan')),
])
def test_invalid_coverage_geometry_rejected(width, height, direction, diameter):
    with pytest.raises(WorkAreaGeometryError):
        strokes(width, height, direction, diameter)


def test_exact_eight_pass_coverage_boundary_does_not_gain_a_rounding_pass():
    # 0.602376 + 7 * 0.5421384 = 4.3973448 m of transverse coverage.
    assert len(strokes(4.397344800000001, 5., 'vertical')) == 8
