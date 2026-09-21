from dataclasses import replace
import json
from types import SimpleNamespace

import numpy as np
import pytest

from sketch_control.plane_lifecycle import (
    PlaneLifecycle,
    SinglePlaneFitConfig,
    calibration_file_sha256,
    canonical_work_area_id,
    capture_once_and_fit_plane,
    validate_single_plane_result,
)

try:
    from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
except ModuleNotFoundError as exc:
    if exc.name != "rclpy":
        raise
    D405SurfaceRefinerNode = None


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _plane_points(z=0.02, noise=0.0004):
    rng = np.random.default_rng(11)
    xy = rng.uniform(-0.15, 0.15, size=(500, 2))
    zz = np.full((xy.shape[0], 1), z)
    zz += rng.normal(0.0, noise, size=zz.shape)
    return np.column_stack([xy, zz])


def _fit_config(**overrides):
    config = SinglePlaneFitConfig(
        voxel_size_m=0.0,
        ransac_distance_threshold_m=0.002,
        ransac_iterations=120,
        min_roi_points=100,
        min_inliers=90,
        min_inlier_ratio=0.80,
        max_rms_residual_m=0.001,
        max_residual_m=0.002,
        max_normal_delta_deg=3.0,
        max_plane_shift_m=0.05,
        max_tf_age_s=0.20,
    )
    return replace(config, **overrides)


def test_single_cloud_fit_is_accepted_with_complete_metrics():
    result = capture_once_and_fit_plane(
        _plane_points(),
        reference_point=[0.0, 0.0, 0.0],
        reference_normal=[0.0, 0.0, 1.0],
        config=_fit_config(),
    )
    decision = validate_single_plane_result(result, _fit_config(), 0.01)

    assert decision.accepted
    assert result.roi_point_count == 500
    assert result.voxel_point_count == 500
    assert result.inlier_count >= 490
    assert result.inlier_ratio >= 0.98
    assert result.rms_residual_m < 0.001
    assert result.max_residual_m < 0.002
    assert result.normal_delta_deg_from_zed < 1.0
    assert abs(abs(result.plane_shift_m_from_zed) - 0.02) < 0.001


def test_validation_rejects_stale_transform_and_low_ratio():
    config = _fit_config()
    result = capture_once_and_fit_plane(
        _plane_points(), [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], config
    )
    stale = validate_single_plane_result(result, config, 0.21)
    assert not stale.accepted
    assert stale.rejection_reason == "transform_too_old"

    low_ratio_result = replace(result, inlier_ratio=0.79)
    low_ratio = validate_single_plane_result(low_ratio_result, config, 0.01)
    assert not low_ratio.accepted
    assert low_ratio.rejection_reason == "inlier_ratio_rejected"

    invalid_metrics = validate_single_plane_result(
        replace(result, rms_residual_m=float("nan")), config, 0.01
    )
    assert not invalid_metrics.accepted
    assert invalid_metrics.rejection_reason == "invalid_fit_metrics"


def test_fit_rejects_nonfinite_reference_plane_fail_closed():
    result = capture_once_and_fit_plane(
        _plane_points(),
        reference_point=[0.0, 0.0, float("nan")],
        reference_normal=[0.0, 0.0, 1.0],
        config=_fit_config(),
    )

    assert not result.fit_succeeded
    assert result.fit_error == "invalid_reference_plane"
    assert not validate_single_plane_result(result, _fit_config(), 0.01).accepted


def test_one_trigger_consumes_exactly_one_capture_and_reject_needs_rearm():
    lifecycle = PlaneLifecycle("work_area", "area-a")
    assert lifecycle.arm_capture(10.0, 1.2).accepted
    assert lifecycle.consume_capture(10.1).accepted
    assert not lifecycle.consume_capture(10.2).accepted

    lifecycle.reject("too_few_inliers")
    assert not lifecycle.capture_armed
    assert lifecycle.state == "rejected"
    assert lifecycle.arm_capture(11.0, 1.2).accepted
    assert lifecycle.consume_capture(11.1).accepted
    lifecycle.accept()

    assert lifecycle.accepted
    assert not lifecycle.arm_capture(12.0, 1.2).accepted
    assert lifecycle.snapshot()["rejection_reason"] == ""


def test_accepted_plane_does_not_expire_from_elapsed_time():
    lifecycle = PlaneLifecycle("work_area", "area-a")
    lifecycle.arm_capture(0.0, 1.0)
    lifecycle.consume_capture(0.1)
    lifecycle.accept()

    assert not lifecycle.expire_capture(1_000_000.0)
    assert lifecycle.accepted
    assert lifecycle.state == "accepted"


def test_paint_lock_defers_invalidation_until_paint_exits():
    lifecycle = PlaneLifecycle("work_area", "area-a")
    lifecycle.arm_capture(0.0, 1.0)
    lifecycle.consume_capture(0.1)
    lifecycle.accept()
    accepted_generation = lifecycle.plane_generation_id

    lifecycle.set_paint_active(True)
    applied = lifecycle.invalidate(
        "work_area_selected", work_area_id="area-b", token="event-2"
    )

    assert not applied
    assert lifecycle.accepted
    assert lifecycle.plane_generation_id == accepted_generation
    assert lifecycle.state == "paint_locked"
    assert lifecycle.arm_capture(2.0, 1.0).rejection_reason == "paint_locked"

    assert lifecycle.set_paint_active(False)
    assert not lifecycle.accepted
    assert lifecycle.work_area_id == "area-b"
    assert lifecycle.plane_generation_id != accepted_generation


def test_multiple_paint_invalidations_keep_pending_new_work_area_identity():
    lifecycle = PlaneLifecycle("work_area", "area-a")
    lifecycle.arm_capture(0.0, 1.0)
    lifecycle.consume_capture(0.1)
    lifecycle.accept()
    lifecycle.set_paint_active(True)

    lifecycle.invalidate(
        "work_area_selected", work_area_id="area-b", token="selection-2"
    )
    lifecycle.invalidate("calibration_file_changed", token="hash-2")
    lifecycle.set_paint_active(False)

    assert lifecycle.work_area_id == "area-b"
    assert not lifecycle.accepted
    assert lifecycle.rejection_reason == "calibration_file_changed"


def test_ids_and_generations_are_deterministic():
    corners = [
        [0.0, 0.0, 0.0],
        [0.4, 0.0, 0.0],
        [0.4, 0.3, 0.0],
        [0.0, 0.3, 0.0],
    ]
    area_id = canonical_work_area_id("link0", corners)
    assert area_id == canonical_work_area_id("link0", corners)
    assert area_id != canonical_work_area_id("camera", corners)

    first = PlaneLifecycle("work_area")
    second = PlaneLifecycle("work_area")
    first.invalidate(
        "work_area_selected", work_area_id=area_id, token="selection-7"
    )
    second.invalidate(
        "work_area_selected", work_area_id=area_id, token="selection-7"
    )
    assert first.plane_generation_id == second.plane_generation_id


def test_calibration_hash_detects_content_change(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"value": 1}', encoding="utf-8")
    before = calibration_file_sha256(str(path))
    path.write_text('{"value": 2}', encoding="utf-8")
    after = calibration_file_sha256(str(path))
    assert before != after


def _fake_pose_stamped(z=0.0):
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    position = SimpleNamespace(x=0.0, y=0.0, z=float(z))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="camera", stamp=None),
        pose=SimpleNamespace(position=position, orientation=orientation),
    )


def _fake_cloud(stamp_sec=1):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="camera",
            stamp=SimpleNamespace(sec=stamp_sec, nanosec=0),
        )
    )


def _node_stub(points):
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {"work_area": PlaneLifecycle("work_area", "area-a")}
    node._active_capture_mode = None
    node._active_capture_reference = None
    node.capture_timeout_s = 1.2
    node.fit_config = _fit_config()
    node._reference_plane_for_mode = lambda _mode: _fake_pose_stamped()
    node._cloud_to_numpy = lambda _msg: points
    node._select_surface_roi = lambda values, *_args, **_kwargs: values
    node._publish_status_messages = []
    node._publication_events = []
    node._publish_status = lambda *args, **kwargs: (
        node._publish_status_messages.append((args, kwargs)),
        node._publication_events.append(("status", args[1])),
    )
    node._published_planes = []
    node._publish_refined_plane = lambda *args: (
        node._published_planes.append(args),
        node._publication_events.append(("pose", args[-1])),
    )
    node._set_expected_refined_corner_signature = lambda *_args: None
    node.get_logger = lambda: _Logger()
    return node


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_ros_adapter_rejects_one_cloud_then_requires_next_trigger():
    node = _node_stub(np.empty((0, 3), dtype=float))
    assert D405SurfaceRefinerNode._start_capture(node, "work_area")

    D405SurfaceRefinerNode._on_cloud(node, _fake_cloud())
    assert node._active_capture_mode is None
    assert node._lifecycles["work_area"].state == "rejected"

    # A second cloud is ignored because the rejected attempt was consumed.
    message_count = len(node._publish_status_messages)
    D405SurfaceRefinerNode._on_cloud(node, _fake_cloud(stamp_sec=2))
    assert len(node._publish_status_messages) == message_count

    node._cloud_to_numpy = lambda _msg: _plane_points()
    assert D405SurfaceRefinerNode._start_capture(node, "work_area")
    # The reference plane belongs to the trigger.  A later raw-plane callback
    # must not change the interpretation of the already armed cloud.
    node._reference_plane_for_mode = lambda _mode: _fake_pose_stamped(z=0.10)
    node._publication_events.clear()
    D405SurfaceRefinerNode._on_cloud(node, _fake_cloud(stamp_sec=3))
    assert node._lifecycles["work_area"].accepted
    assert len(node._published_planes) == 1
    assert node._publication_events == [
        ("status", "accepted"),
        ("pose", "work_area"),
    ]

    # First accepted is locked; another trigger cannot arm a capture.
    assert not D405SurfaceRefinerNode._start_capture(node, "work_area")


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_status_json_always_contains_required_contract_fields():
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {"work_area": PlaneLifecycle("work_area", "area-a")}
    node.status_pub = _CapturePublisher()
    node.target_status_pub = _CapturePublisher()
    node.get_logger = lambda: _Logger()

    D405SurfaceRefinerNode._publish_status(
        node,
        False,
        "rejected",
        mode="work_area",
        rejection_reason="too_few_roi_points",
    )
    payload = json.loads(node.status_pub.messages[-1].data)
    required = {
        "ok",
        "state",
        "accepted",
        "rejection_reason",
        "work_area_id",
        "plane_generation_id",
        "roi_point_count",
        "inlier_count",
        "inlier_ratio",
        "rms_residual_m",
        "max_residual_m",
        "normal_delta_deg_from_zed",
        "plane_shift_m_from_zed",
        "capture_stamp",
        "source_frame",
        "transform_age_s",
    }
    assert required <= payload.keys()


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_target_status_does_not_overwrite_latched_work_area_status():
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {
        "work_area": PlaneLifecycle("work_area", "area-a"),
        "target": PlaneLifecycle("target"),
    }
    node.status_pub = _CapturePublisher()
    node.target_status_pub = _CapturePublisher()
    node.get_logger = lambda: _Logger()

    D405SurfaceRefinerNode._publish_status(
        node,
        False,
        "invalidated",
        mode="work_area",
        rejection_reason="not_refined",
    )
    D405SurfaceRefinerNode._publish_status(
        node,
        False,
        "invalidated",
        mode="target",
        rejection_reason="target_surface_changed",
    )

    assert len(node.status_pub.messages) == 1
    assert len(node.target_status_pub.messages) == 1
    assert json.loads(node.status_pub.messages[0].data)["mode"] == "work_area"


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_explicit_cleared_state_prevents_corner_hash_fallback():
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {"work_area": PlaneLifecycle("work_area", "area-a")}
    node._active_capture_mode = None
    node._external_work_area_state_seen = False
    node._external_work_area_state_key = None
    node._external_work_area_id = ""
    node._target_signature = "target-a"
    node._corner_signature = None
    node._expected_refined_corner_signature = None
    node.latest_work_area_plane = object()
    node.latest_corners = object()
    node._publish_status = lambda *_args, **_kwargs: None
    node.get_logger = lambda: _Logger()

    D405SurfaceRefinerNode._on_work_area_state(
        node,
        SimpleNamespace(
            data=json.dumps(
                {
                    "selected": False,
                    "work_area_id": "",
                    "invalidation_seq": 4,
                }
            )
        ),
    )
    invalidation_seq = node._lifecycles["work_area"].invalidation_seq
    poses = []
    for x, y in ((0.0, 0.0), (0.4, 0.0), (0.4, 0.3), (0.0, 0.3)):
        poses.append(
            SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0)
            )
        )
    corners = SimpleNamespace(
        header=SimpleNamespace(frame_id="link0"), poses=poses
    )
    D405SurfaceRefinerNode._on_work_area_corners(node, corners)

    assert node._lifecycles["work_area"].invalidation_seq == invalidation_seq
    assert node._lifecycles["work_area"].work_area_id == ""
    assert node.latest_work_area_plane is None
    assert node.latest_corners is corners


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_new_work_area_identity_requires_geometry_received_after_state_edge():
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {"work_area": PlaneLifecycle("work_area", "old-area")}
    node._active_capture_mode = None
    node._active_capture_reference = None
    node._external_work_area_state_seen = True
    node._external_work_area_state_key = (True, "old-area", "", "1")
    node._external_work_area_id = "old-area"
    node._target_signature = "target-a"
    node._corner_signature = ("link0", ((0.0, 0.0, 0.0),))
    node._expected_refined_corner_signature = object()
    node.latest_work_area_plane = object()
    node.latest_corners = object()
    node._publish_status = lambda *_args, **_kwargs: None

    D405SurfaceRefinerNode._on_work_area_state(
        node,
        SimpleNamespace(
            data=json.dumps(
                {
                    "selected": True,
                    "work_area_id": "new-area",
                    "invalidation_seq": 2,
                }
            )
        ),
    )

    assert node._lifecycles["work_area"].work_area_id == "new-area"
    assert node.latest_work_area_plane is None
    assert node.latest_corners is None
    assert node._corner_signature is None


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_same_geometry_new_target_stamp_is_a_new_invalidation_event():
    node = object.__new__(D405SurfaceRefinerNode)
    node._lifecycles = {
        "work_area": PlaneLifecycle("work_area", "area-a"),
        "target": PlaneLifecycle("target"),
    }
    node._active_capture_mode = None
    node._external_work_area_id = "area-a"
    node._target_signature = None
    node._publish_status = lambda *_args, **_kwargs: None
    capture_requests = []
    node._start_capture = lambda mode: capture_requests.append(
        (mode, node.latest_target_surface)
    ) or True

    pose = _fake_pose_stamped()
    pose.header.stamp = SimpleNamespace(sec=10, nanosec=0)
    D405SurfaceRefinerNode._on_target_surface(node, pose)
    first_target_seq = node._lifecycles["target"].invalidation_seq
    first_work_seq = node._lifecycles["work_area"].invalidation_seq
    assert capture_requests == [("target", pose)]

    # A duplicate delivery of the exact latched event is ignored.
    D405SurfaceRefinerNode._on_target_surface(node, pose)
    assert node._lifecycles["target"].invalidation_seq == first_target_seq
    assert node._lifecycles["work_area"].invalidation_seq == first_work_seq
    assert capture_requests == [("target", pose)]

    # Reselecting the same geometry has a new source stamp and invalidates.
    pose.header.stamp = SimpleNamespace(sec=11, nanosec=0)
    D405SurfaceRefinerNode._on_target_surface(node, pose)
    assert node._lifecycles["target"].invalidation_seq == first_target_seq + 1
    assert node._lifecycles["work_area"].invalidation_seq == first_work_seq + 1
    assert capture_requests == [("target", pose), ("target", pose)]
