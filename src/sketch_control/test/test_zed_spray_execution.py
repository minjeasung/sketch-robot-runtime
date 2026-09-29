import json
from types import SimpleNamespace

import numpy as np
import pytest

from sketch_control.painting_execution import real_plan_gate_blockers
from sketch_control.zed_spray_execution import ZedSprayExecutionMixin, validate_zed_work_area


def locked_area():
    return dict(source="zed", mode="work_area", accepted=True, state="locked",
                work_area_id="area-1", plane_generation_id="zed:1:1",
                selection_id="123", frame_id="camera", position=[0., 0., 1.],
                orientation=[1., 0., 0., 0.],
                corners=[[-.2, -.2, 1.], [.2, -.2, 1.],
                         [.2, .2, 1.], [-.2, .2, 1.]])


def test_zed_area_requires_current_lock_and_selection():
    payload = locked_area()
    point, normal, corners = validate_zed_work_area(payload, "zed:1:1", "123")
    np.testing.assert_allclose(normal, [0., 0., -1.])
    np.testing.assert_allclose(point, corners.mean(axis=0))
    for generation, selection in [("old", "123"), ("zed:1:1", "122"), ("", "123")]:
        with pytest.raises(ValueError):
            validate_zed_work_area(payload, generation, selection)


@pytest.mark.parametrize("field,value", [
    ("accepted", False), ("source", "d405"), ("mode", "target"),
    ("state", "pending"), ("frame_id", ""), ("work_area_id", ""),
    ("position", [float("nan"), 0., 1.]), ("orientation", [0., 0., 0., 0.]),
    ("corners", [[0., 0., 1.]] * 4),
    ("corners", [[-.2, -.2, 1.], [.2, -.2, 1.], [.2, .2, 1.1], [-.2, .2, 1.]]),
])
def test_invalid_zed_geometry_fails_closed(field, value):
    payload = locked_area()
    payload[field] = value
    with pytest.raises(ValueError):
        validate_zed_work_area(payload, "zed:1:1", "123")


def test_spray_gate_requires_zed_and_retains_plan_identity_checks():
    args = dict(segment_present=True, segment_version=3,
                segment_path_id="path", waypoint_path_id="path",
                segment_plan_hash="hash", accepted_plan_hash="hash",
                accepted_plan_path_id="path", segment_work_area_id="area",
                current_work_area_id="area", segment_plane_generation_id="zed:1",
                current_plane_generation_id="zed:1", d405_plane_accepted=False,
                process_mode="spray", zed_plane_accepted=True)
    assert real_plan_gate_blockers(**args) == ()
    args["zed_plane_accepted"] = False
    assert "ZED_PLANE_NOT_ACCEPTED" in real_plan_gate_blockers(**args)
    args["zed_plane_accepted"] = True
    args["accepted_plan_hash"] = "old"
    assert "PLAN_HASH_MISMATCH" in real_plan_gate_blockers(**args)
    args["process_mode"] = "paint"
    assert "D405_PLANE_NOT_ACCEPTED" in real_plan_gate_blockers(**args)


class Executor(ZedSprayExecutionMixin):
    def __init__(self):
        self.process_mode = "spray"
        self.executing = False
        self._zed_work_area_selection_id = ""
        self._zed_plane_accepted = False
        self._zed_pending_area = None
        self._zed_target_lock = dict(plane_generation_id="zed:1:1", frame_id="link0",
                                    normal=[0., 0., -1.], corners=locked_area()["corners"])
        self.events = []
        self._execution_snapshot_updates_locked = lambda *_: False
        self._publish_execution_status = lambda *args: self.events.append(args)
        self._mark_scene_dirty = lambda *_: None
        self._canonical_world_frame = lambda frame: frame
        self._set_dynamic_work_area_corners = lambda corners: setattr(self, "dynamic_work_area_corners", corners)
        self._lookup_transform_to_base = lambda *_, **__: None


def pixels(stamp=123, frame="wall_front"):
    return SimpleNamespace(header=SimpleNamespace(
        frame_id=frame, stamp=SimpleNamespace(sec=0, nanosec=stamp)), poses=[object()] * 4)


def status(**updates):
    payload = locked_area()
    payload["frame_id"] = "link0"
    payload.update(updates)
    return SimpleNamespace(data=json.dumps(payload))


@pytest.mark.parametrize("status_first", [True, False])
def test_atomic_status_and_pixels_can_arrive_in_either_order(status_first):
    node = Executor()
    operations = [lambda: node.on_zed_surface_status(status()), lambda: node.on_zed_area_pixels(pixels())]
    if not status_first:
        operations.reverse()
    operations[0]()
    assert not node._zed_plane_accepted
    operations[1]()
    assert node._zed_plane_accepted
    assert node.dynamic_surface_source == "zed_locked"
    assert node._d405_plane_accepted is False


def test_reselection_invalidates_plan_and_rejects_old_area_or_raw_pixels():
    node = Executor()
    node.on_zed_area_pixels(pixels())
    node.on_zed_surface_status(status())
    node._accepted_plan_hash = "approved"
    node.on_zed_area_pixels(pixels(124))
    node.on_zed_surface_status(status())
    assert not node._zed_plane_accepted and not node._accepted_plan_hash
    node.on_zed_area_pixels(pixels(123, "zed_raw"))
    node.on_zed_surface_status(status())
    assert not node._zed_plane_accepted


def test_tf_unavailable_never_accepts_then_retry_uses_same_atomic_geometry():
    node = Executor()
    node._zed_target_lock["frame_id"] = "camera"
    node.on_zed_area_pixels(pixels())
    node.on_zed_surface_status(status(frame_id="camera"))
    assert not node._zed_plane_accepted
    node._lookup_transform_to_base = lambda *_, **__: SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    node._accept_pending_zed_area()
    assert node._zed_plane_accepted
    np.testing.assert_allclose(node.dynamic_surface_point, [1., 0., 1.])


def test_changed_geometry_under_accepted_identity_invalidates_it():
    node = Executor()
    node.on_zed_area_pixels(pixels())
    node.on_zed_surface_status(status())
    node.on_zed_surface_status(status(position=[0., 0., 1.01]))
    assert not node._zed_plane_accepted
    assert node.events[-1] == ("PLAN_INVALIDATED", "ZED_LOCK_GEOMETRY_CHANGED")
    node.on_zed_surface_status(status())
    assert not node._zed_plane_accepted


def test_explicit_invalidation_cannot_be_undone_by_old_accepted_status():
    node = Executor()
    node.on_zed_area_pixels(pixels())
    node.on_zed_surface_status(status())
    node.on_zed_surface_status(status(accepted=False, state="invalidated"))
    node.on_zed_surface_status(status())
    assert not node._zed_plane_accepted


def test_paint_ignores_all_zed_status_and_selection():
    node = Executor()
    node.process_mode = "paint"
    node.on_zed_area_pixels(pixels())
    node.on_zed_surface_status(status())
    assert not node._zed_plane_accepted
    assert node.events == []
