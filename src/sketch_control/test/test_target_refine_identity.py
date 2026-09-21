import json
from types import SimpleNamespace

import numpy as np
import pytest

from sketch_control.plane_lifecycle import PlaneLifecycle

try:
    from geometry_msgs.msg import PoseArray, PoseStamped
    from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
    from sketch_control.target_selector_node import TargetSelectorNode
except ModuleNotFoundError as exc:
    if exc.name not in {"geometry_msgs", "rclpy"}:
        raise
    PoseArray = None
    PoseStamped = None
    D405SurfaceRefinerNode = None
    TargetSelectorNode = None


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _target_pose(sec=123, nanosec=456_000_000):
    msg = PoseStamped()
    msg.header.frame_id = "zed_left_camera_frame_optical"
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    return msg


def _refiner_stub():
    node = object.__new__(D405SurfaceRefinerNode)
    node.latest_target_surface = None
    node.latest_work_area_plane = None
    node._target_signature = None
    node._target_id = ""
    node._target_stamp = {"sec": 0, "nanosec": 0}
    node._external_work_area_id = ""
    node._active_capture_mode = None
    node._active_capture_reference = None
    node._pending_cloud_capture = None
    node._lifecycles = {
        "target": PlaneLifecycle("target"),
        "work_area": PlaneLifecycle("work_area"),
    }
    node.capture_timeout_s = 5.0
    node.status_pub = _Publisher()
    node.target_status_pub = _Publisher()
    node.get_logger = lambda: _Logger()
    return node


@pytest.mark.skipif(
    TargetSelectorNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_target_selector_preserves_nonzero_selection_stamp_exactly():
    node = object.__new__(TargetSelectorNode)
    node.K = np.eye(3)
    node.latest_depth = np.ones((100, 100), dtype=np.float32)
    node.latest_depth_header = SimpleNamespace(
        frame_id="zed_left_camera_frame_optical"
    )
    points = np.column_stack(
        (
            np.linspace(-0.2, 0.2, 100),
            np.linspace(-0.1, 0.1, 100),
            np.ones(100),
        )
    )
    node._points_from_roi = lambda *_args: points
    node._ransac_plane = lambda values: (
        np.array([0.0, 0.0, 1.0, -1.0]),
        np.arange(values.shape[0]),
    )
    node.pub = _Publisher()
    node.catalog_pub = _Publisher()
    node.get_logger = lambda: _Logger()
    node.get_clock = lambda: (_ for _ in ()).throw(
        AssertionError("selector must not invent a replacement stamp")
    )

    selection = PoseArray()
    selection.header.frame_id = "zed_raw"
    selection.header.stamp.sec = 123
    selection.header.stamp.nanosec = 456_000_000
    for u, v in ((20.0, 20.0), (80.0, 80.0)):
        pose = PoseStamped().pose
        pose.position.x = u
        pose.position.y = v
        selection.poses.append(pose)

    TargetSelectorNode._on_selection(node, selection)

    assert node.pub.messages == []  # Selection is explicit, not largest-plane auto-selection.
    assert len(node.catalog_pub.messages) == 1
    output = json.loads(node.catalog_pub.messages[0].data)
    assert output["generation"] == str(selection.header.stamp.sec * 1_000_000_000 + selection.header.stamp.nanosec)
    assert len(output["planes"]) == 1
    assert output["planes"][0]["id"].startswith(output["generation"] + ":")


@pytest.mark.skipif(
    TargetSelectorNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_target_selector_rejects_zero_request_stamp():
    node = object.__new__(TargetSelectorNode)
    node.K = np.eye(3)
    node.latest_depth = np.ones((10, 10), dtype=np.float32)
    node.pub = _Publisher()
    node.get_logger = lambda: _Logger()
    selection = PoseArray()
    selection.header.frame_id = "zed_raw"
    selection.poses.append(PoseStamped().pose)

    TargetSelectorNode._on_selection(node, selection)

    assert node.pub.messages == []


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_auto_arm_status_and_pending_cloud_keep_target_identity():
    node = _refiner_stub()
    target = _target_pose()

    D405SurfaceRefinerNode._on_target_surface(node, target)

    target_statuses = [json.loads(msg.data) for msg in node.target_status_pub.messages]
    assert [item["state"] for item in target_statuses] == [
        "invalidated",
        "capture_armed",
    ]
    expected_stamp = {"sec": 123, "nanosec": 456_000_000}
    assert all(item["target_stamp"] == expected_stamp for item in target_statuses)
    target_id = target_statuses[0]["target_id"]
    assert target_id
    assert all(item["target_id"] == target_id for item in target_statuses)

    node._lookup_transform = lambda *_args: (None, None)
    cloud = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="d405_depth_optical_frame",
            stamp=SimpleNamespace(sec=124, nanosec=0),
        )
    )
    D405SurfaceRefinerNode._on_cloud(node, cloud)

    pending = node._pending_cloud_capture
    assert pending is not None
    assert pending.target_id == target_id
    assert (pending.target_stamp_sec, pending.target_stamp_nanosec) == (
        123,
        456_000_000,
    )
    waiting = json.loads(node.target_status_pub.messages[-1].data)
    assert waiting["state"] == "waiting_for_tf"
    assert waiting["target_id"] == target_id
    assert waiting["target_stamp"] == expected_stamp


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_pending_cloud_cannot_cross_target_identity_at_same_generation():
    node = _refiner_stub()
    D405SurfaceRefinerNode._on_target_surface(node, _target_pose())
    node._lookup_transform = lambda *_args: (None, None)
    cloud = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="d405_depth_optical_frame",
            stamp=SimpleNamespace(sec=124, nanosec=0),
        )
    )
    D405SurfaceRefinerNode._on_cloud(node, cloud)
    assert node._pending_cloud_capture is not None

    # Model an accidental identity overwrite without a lifecycle invalidation;
    # the independent pending guard must still fail closed.
    node._target_stamp = {"sec": 999, "nanosec": 0}
    D405SurfaceRefinerNode._retry_pending_cloud(node)

    assert node._pending_cloud_capture is None
    assert node._lifecycles["target"].state == "rejected"
    assert node._lifecycles["target"].rejection_reason == "target_identity_changed"
