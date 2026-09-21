from types import SimpleNamespace

import pytest

from sketch_control.plane_lifecycle import PlaneLifecycle

try:
    from builtin_interfaces.msg import Time as TimeMsg
    import sketch_control.d405_surface_refiner_node as refiner_module
    from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
except ModuleNotFoundError as exc:
    if exc.name not in {"builtin_interfaces", "rclpy"}:
        raise
    TimeMsg = None
    refiner_module = None
    D405SurfaceRefinerNode = None


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _pose(frame_id="zed_frame"):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=1, nanosec=0),
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _cloud(stamp_sec, frame_id="d405_frame"):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=int(stamp_sec), nanosec=0),
        )
    )


def _node_stub():
    node = object.__new__(D405SurfaceRefinerNode)
    target = PlaneLifecycle("target")
    target.invalidate("target_surface_changed", token="target-1")
    node._lifecycles = {
        "target": target,
        "work_area": PlaneLifecycle("work_area", "area-1"),
    }
    node.latest_target_surface = _pose()
    node.latest_work_area_plane = _pose()
    node._active_capture_mode = None
    node._active_capture_reference = None
    node._pending_cloud_capture = None
    node.capture_timeout_s = 1.0
    node.published_status = []
    node.evaluated = []
    node._publish_status = lambda *args, **kwargs: node.published_status.append(
        (args, kwargs)
    )
    node._evaluate_captured_cloud = (
        lambda mode, cloud, plane, target_frame, transform, age, metadata:
        node.evaluated.append(
            {
                "mode": mode,
                "cloud": cloud,
                "plane": plane,
                "target_frame": target_frame,
                "transform": transform,
                "age": age,
                "metadata": metadata,
            }
        )
    )
    node.get_logger = lambda: _Logger()
    return node


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_first_cloud_waits_for_exact_time_tf_and_later_clouds_are_ignored(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(refiner_module.time, "monotonic", lambda: now[0])
    node = _node_stub()
    lookups = []
    transform_available = [False]
    transform = object()

    def lookup(target_frame, source_frame, stamp):
        lookups.append((target_frame, source_frame, stamp))
        return (transform, 0.0) if transform_available[0] else (None, None)

    node._lookup_transform = lookup
    assert D405SurfaceRefinerNode._start_capture(node, "target")

    first = _cloud(101)
    D405SurfaceRefinerNode._on_cloud(node, first)
    pending = node._pending_cloud_capture
    assert pending is not None
    assert pending.cloud is first
    assert pending.reference_plane is node.latest_target_surface
    assert node._active_capture_mode is None
    assert node._lifecycles["target"].state == "evaluating"

    second = _cloud(102)
    D405SurfaceRefinerNode._on_cloud(node, second)
    assert node._pending_cloud_capture.cloud is first
    assert len(lookups) == 1
    assert node.evaluated == []

    transform_available[0] = True
    now[0] = 10.1
    D405SurfaceRefinerNode._check_capture_timeout(node)
    assert node._pending_cloud_capture is None
    assert len(node.evaluated) == 1
    assert node.evaluated[0]["cloud"] is first
    assert node.evaluated[0]["transform"] is transform
    assert node.evaluated[0]["metadata"]["tf_wait_s"] == pytest.approx(0.1)


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_tf_absence_is_terminal_only_after_pending_deadline(monkeypatch):
    now = [20.0]
    monkeypatch.setattr(refiner_module.time, "monotonic", lambda: now[0])
    node = _node_stub()
    node._lookup_transform = lambda *_args: (None, None)
    assert D405SurfaceRefinerNode._start_capture(node, "target")

    first = _cloud(201)
    now[0] = 20.1
    D405SurfaceRefinerNode._on_cloud(node, first)
    assert node._pending_cloud_capture is not None
    assert node._lifecycles["target"].state == "evaluating"

    now[0] = 20.9
    D405SurfaceRefinerNode._check_capture_timeout(node)
    assert node._pending_cloud_capture is not None
    assert node._lifecycles["target"].state == "evaluating"

    now[0] = 21.01
    D405SurfaceRefinerNode._check_capture_timeout(node)
    assert node._pending_cloud_capture is None
    assert node._lifecycles["target"].state == "rejected"
    assert node._lifecycles["target"].rejection_reason == "tf_missing"
    rejected = [
        item
        for item in node.published_status
        if item[1].get("rejection_reason") == "tf_missing"
    ]
    assert len(rejected) == 1
    assert rejected[0][1]["capture_stamp"] == {"sec": 201, "nanosec": 0}

    D405SurfaceRefinerNode._check_capture_timeout(node)
    D405SurfaceRefinerNode._on_cloud(node, _cloud(202))
    rejected_again = [
        item
        for item in node.published_status
        if item[1].get("rejection_reason") == "tf_missing"
    ]
    assert len(rejected_again) == 1
    assert node.evaluated == []


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
@pytest.mark.parametrize(
    "reason", ["target_surface_changed", "calibration_file_changed"]
)
def test_invalidation_cancels_pending_cloud_without_late_processing(
    monkeypatch, reason
):
    now = [30.0]
    monkeypatch.setattr(refiner_module.time, "monotonic", lambda: now[0])
    node = _node_stub()
    node._lookup_transform = lambda *_args: (None, None)
    assert D405SurfaceRefinerNode._start_capture(node, "target")
    D405SurfaceRefinerNode._on_cloud(node, _cloud(301))
    assert node._pending_cloud_capture is not None

    D405SurfaceRefinerNode._invalidate_lifecycle(
        node,
        "target",
        reason,
        token="target-2",
    )
    assert node._pending_cloud_capture is None
    assert node._lifecycles["target"].state == "invalidated"

    now[0] = 32.0
    D405SurfaceRefinerNode._check_capture_timeout(node)
    assert node.evaluated == []
    assert node._lifecycles["target"].state == "invalidated"


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_new_explicit_capture_supersedes_pending_and_owns_next_cloud(
    monkeypatch,
):
    now = [40.0]
    monkeypatch.setattr(refiner_module.time, "monotonic", lambda: now[0])
    node = _node_stub()
    transform = object()
    transform_available = [False]
    node._lookup_transform = lambda *_args: (
        (transform, 0.0) if transform_available[0] else (None, None)
    )
    assert D405SurfaceRefinerNode._start_capture(node, "target")
    old_cloud = _cloud(401)
    D405SurfaceRefinerNode._on_cloud(node, old_cloud)
    assert node._pending_cloud_capture.cloud is old_cloud

    now[0] = 40.2
    assert D405SurfaceRefinerNode._start_capture(node, "target")
    assert node._pending_cloud_capture is None
    assert node._active_capture_mode == "target"

    transform_available[0] = True
    new_cloud = _cloud(402)
    D405SurfaceRefinerNode._on_cloud(node, new_cloud)
    assert len(node.evaluated) == 1
    assert node.evaluated[0]["cloud"] is new_cloud
    assert node.evaluated[0]["cloud"] is not old_cloud


@pytest.mark.skipif(
    D405SurfaceRefinerNode is None,
    reason="ROS 2 Python environment is not sourced",
)
def test_exact_time_tf_lookup_is_nonblocking():
    node = object.__new__(D405SurfaceRefinerNode)
    transform = SimpleNamespace(header=SimpleNamespace(stamp=TimeMsg()))

    class _Buffer:
        timeout = None

        def lookup_transform(
            self, _target_frame, _source_frame, _query_time, *, timeout
        ):
            self.timeout = timeout
            return transform

    node.tf_buffer = _Buffer()
    stamp = TimeMsg(sec=501, nanosec=123)
    result, age_s = D405SurfaceRefinerNode._lookup_transform(
        node, "zed_frame", "d405_frame", stamp
    )

    assert result is transform
    assert age_s == 0.0
    assert node.tf_buffer.timeout.nanoseconds == 0
