import copy

import numpy as np
import pytest

from sketch_control.work_area_geometry import (
    WorkAreaGeometryError, generate_spray_fill_strokes, validate_zed_surface_status,
)


def atomic_status():
    corners = [[0., 0., 0.], [1., 0., 0.], [1., .5, 0.], [0., .5, 0.]]
    return dict(source="zed", mode="work_area", accepted=True, state="locked",
                plane_generation_id="zed:catalog:plane:1000000000", work_area_id="work-1",
                selection_id="2000000000", frame_id="World", position=[.5, .25, 0.],
                orientation=[0., 0., 0., 1.], corners=corners, front_extent=copy.deepcopy(corners),
                view_width=801, view_height=401, target_stamp=dict(sec=1, nanosec=0))


def test_atomic_zed_work_area_validates():
    status = atomic_status()
    assert validate_zed_surface_status(status) is status


@pytest.mark.parametrize("field,value", [
    ("source", "d405"), ("mode", "target"), ("accepted", "true"), ("state", "invalidated"),
    ("plane_generation_id", "d405:old"), ("selection_id", "selection-1"),
    ("selection_id", "02000000000"), ("selection_id", "0"), ("work_area_id", ""),
    ("view_width", 1), ("view_height", 401.5), ("target_stamp", {"sec": 1, "nanosec": -1}),
    ("position", [.5, .25, float("nan")]), ("position", [.1, .1, 0.]),
    ("orientation", [0., 0., 0., 0.]), ("corners", [[0., 0., 0.]] * 4),
    ("corners", [[0., 0., 0.], [1., 0., 0.], [1., .5, .1], [0., .5, 0.]]),
])
def test_bad_atomic_geometry_rejected(field, value):
    status = atomic_status()
    status[field] = value
    with pytest.raises(WorkAreaGeometryError):
        validate_zed_surface_status(status)


def test_fill_coverage_uses_spray_footprint_and_overlap():
    strokes = generate_spray_fill_strokes(
        (0., 0., 800., 400.), work_area_width_m=1., work_area_height_m=.5,
        footprint_width_m=.35, overlap=.30)
    assert len(strokes) == 4
    centers = np.array([stroke[0][0] / 800 for stroke in strokes])
    assert centers[0] == pytest.approx(.175)
    assert centers[-1] == pytest.approx(.825)
    assert np.max(np.diff(centers)) <= .35 * (1 - .30)
    assert [stroke[0][1] for stroke in strokes] == [0., 400., 0., 400.]


def test_fill_rejects_work_area_narrower_than_footprint():
    with pytest.raises(WorkAreaGeometryError):
        generate_spray_fill_strokes((0., 0., 800., 400.), work_area_width_m=.2,
                                   work_area_height_m=.5, footprint_width_m=.35, overlap=.3)
