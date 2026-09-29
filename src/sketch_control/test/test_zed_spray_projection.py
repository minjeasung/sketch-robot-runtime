"""ZED-only projection geometry and ROS callback regression tests."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from sketch_control.zed_spray_projection import (
    project_target_rectangle,
    select_work_area,
    validate_target_lock,
)


def _target(sec=10):
    return dict(
        accepted=True, source="zed", state="locked",
        plane_generation_id=f"zed:catalog:catalog:1:{sec * 1_000_000_000}",
        catalog_generation="catalog", plane_id="catalog:1", frame_id="zed_optical",
        center=[0.0, 0.0, 2.0], normal=[0.0, 0.0, -1.0],
        corners=[[-0.5, -0.3, 2.0], [0.5, -0.3, 2.0],
                 [0.5, 0.3, 2.0], [-0.5, 0.3, 2.0]],
        stamp=dict(sec=sec, nanosec=0),
    )


K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])


def test_tilted_plane_rectifies_to_metric_rectangle_before_area_selection():
    data = _target()
    angle = np.deg2rad(35)
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    center = np.asarray(data["center"])
    data["corners"] = ((np.asarray(data["corners"]) - center) @ rotation.T + center).tolist()
    data["normal"] = (rotation @ np.asarray(data["normal"])).tolist()
    target = validate_target_lock(data)
    extent, pixels, size = project_target_rectangle(target, K, (640, 480))
    assert np.linalg.norm(extent[1] - extent[0]) == pytest.approx(1.0)
    assert np.linalg.norm(extent[3] - extent[0]) == pytest.approx(0.6)
    assert np.dot(extent[1] - extent[0], extent[3] - extent[0]) == pytest.approx(0)
    assert size == (900, 540)
    assert abs(pixels[0, 1] - pixels[1, 1]) > 5  # Raw projection is a trapezoid.
    corners = select_work_area(extent, size, [[0, 0], [449.5, 269.5]])
    assert np.linalg.norm(corners[1] - corners[0]) == pytest.approx(0.5)
    assert np.linalg.norm(corners[3] - corners[0]) == pytest.approx(0.3)
    assert corners[2] == pytest.approx([0.0, 0.0, 2.0])


@pytest.mark.parametrize("field,value", [
    ("accepted", False), ("accepted", "true"), ("source", "d405"),
    ("state", "measured"), ("plane_generation_id", "zed:wrong"),
    ("catalog_generation", ""), ("frame_id", ""), ("plane_id", ""),
    ("stamp", {"sec": 0, "nanosec": 0}),
    ("stamp", {"sec": 10, "nanosec": 1_000_000_000}),
    ("normal", [0, 0, 0]), ("normal", [0, 0, -2]),
    ("center", [0, 0, float("nan")]), ("center", [0, 0, 2.02]),
])
def test_target_lock_rejects_invalid_identity_or_geometry(field, value):
    data = _target()
    data[field] = value
    with pytest.raises(ValueError):
        validate_target_lock(data)


@pytest.mark.parametrize("corners", [
    [[0, 0, 2]] * 4,
    [[-.5, -.3, 2], [.5, -.3, 2], [.4, .3, 2], [-.5, .3, 2]],
    [[-.5, -.3, 2], [.5, -.3, 2], [.5, .3, 2.1], [-.5, .3, 2]],
    [[-.5, -.3, 2], [.5, .3, 2], [.5, -.3, 2], [-.5, .3, 2]],
])
def test_catalog_support_must_be_a_nondegenerate_planar_rectangle(corners):
    data = _target()
    data["corners"] = corners
    with pytest.raises(ValueError):
        validate_target_lock(data)


@pytest.mark.parametrize("quality", [
    {"inlier_count": 79, "rms_m": 0.005},
    {"inlier_count": 100, "rms_m": 0.016},
    {"inlier_count": 100, "rms_m": float("nan")},
    {"inlier_count": 100}, {"rms_m": 0.005},
])
def test_rejects_failed_or_incomplete_quality_when_producer_supplies_it(quality):
    data = _target()
    data.update(quality)
    with pytest.raises(ValueError):
        validate_target_lock(data)


@pytest.mark.parametrize("translation", [[0, 0, -3], [3, 0, 0], [float("nan"), 0, 0]])
def test_projection_rejects_behind_camera_outside_image_or_invalid_tf(translation):
    with pytest.raises(ValueError):
        project_target_rectangle(validate_target_lock(_target()), K, (640, 480),
                                 translation=translation)


def test_projection_transforms_surface_frame_into_zed_optical_frame():
    data = _target()
    data["corners"] = (np.asarray(data["corners"]) + [1, 0, 0]).tolist()
    data["center"][0] = 1.0
    extent, pixels, size = project_target_rectangle(
        validate_target_lock(data), K, (640, 480), translation=[-1, 0, 0])
    assert extent[0] == pytest.approx([.5, -.3, 2])
    assert pixels[0] == pytest.approx([170, 150])
    assert size == (900, 540)


def test_projection_orients_reversed_catalog_corners_like_camera():
    data = _target()
    data["corners"] = [data["corners"][i] for i in (2, 3, 0, 1)]
    extent, pixels, _size = project_target_rectangle(validate_target_lock(data), K, (640, 480))
    assert extent == pytest.approx(np.asarray(_target()["corners"]))
    assert pixels == pytest.approx(np.array([[170, 150], [470, 150], [470, 330], [170, 330]]))


@pytest.mark.parametrize("points", [
    [[-0.001, 0], [100, 100]], [[0, 0], [900, 100]],
    [[0, 0], [100, 540]], [[0, 0], [float("inf"), 100]],
    [[0, 0], [float("nan"), 100]], [[0, 0], [10, 100]],
    [[0, 0], [100, 0], [90, 100], [0, 100]], [[0, 0]],
])
def test_area_rejects_invalid_pixels_without_clamping(points):
    with pytest.raises(ValueError):
        select_work_area(np.asarray(_target()["corners"]), (900, 540), points)


def test_full_frame_area_stays_exactly_on_catalog_support():
    extent = np.asarray(_target()["corners"])
    result = select_work_area(extent, (900, 540),
                             [[0, 0], [899, 0], [899, 539], [0, 539]])
    assert result == pytest.approx(extent)


try:
    from geometry_msgs.msg import Pose, PoseArray
    from std_msgs.msg import String
    from sketch_control.wall_projector_node import WallProjectorNode
except ModuleNotFoundError as exc:
    if exc.name not in {"geometry_msgs", "rclpy"}:
        raise
    WallProjectorNode = None


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


def _projector_without_ros(monkeypatch):
    """Run production callbacks with transport-only doubles on a non-ROS host."""
    class Pose:
        def __init__(self):
            self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)

    class Header:
        def __init__(self):
            self.stamp = SimpleNamespace(sec=0, nanosec=0)
            self.frame_id = ""

    class PoseArray:
        def __init__(self):
            self.header, self.poses = Header(), []

    class PoseStamped:
        def __init__(self):
            self.header, self.pose = Header(), Pose()

    class Image:
        def __init__(self):
            self.header = Header()

    class String:
        def __init__(self, data=""):
            self.data = data

    modules = {
        "rclpy": dict(time=SimpleNamespace(Time=SimpleNamespace(from_msg=lambda stamp: stamp))),
        "rclpy.duration": dict(Duration=lambda **kw: None),
        "rclpy.node": dict(Node=type("Node", (), {})),
        "rclpy.qos": dict(DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
                          HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                          QoSProfile=lambda **kw: None, qos_profile_sensor_data=None),
        "geometry_msgs": {},
        "geometry_msgs.msg": dict(Pose=Pose, PoseArray=PoseArray, PoseStamped=PoseStamped),
        "sensor_msgs": {},
        "sensor_msgs.msg": dict(Image=Image, CameraInfo=type("CameraInfo", (), {})),
        "std_msgs": {},
        "std_msgs.msg": dict(Empty=type("Empty", (), {}), String=String),
        "tf2_ros": dict(Buffer=object, TransformListener=object,
                        TransformException=type("TransformException", (Exception,), {})),
    }
    # Restore sys.modules immediately so doubles cannot leak into other tests.
    with monkeypatch.context() as patch:
        for name, values in modules.items():
            module = ModuleType(name)
            module.__dict__.update(values)
            patch.setitem(sys.modules, name, module)
        path = Path(__file__).parents[1] / "sketch_control" / "wall_projector_node.py"
        spec = importlib.util.spec_from_file_location("_zed_projector_callback_tests", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    for name, value in (("Pose", Pose), ("PoseArray", PoseArray), ("String", String)):
        monkeypatch.setattr(sys.modules[__name__], name, value, raising=False)
    return module.WallProjectorNode


@pytest.fixture
def node(monkeypatch):
    projector = WallProjectorNode or _projector_without_ros(monkeypatch)
    result = object.__new__(projector)
    result.process_mode = "spray"
    result.latest_surface = None
    result.latest_work_area_pixels = None
    result.locked_work_area = None
    result._work_area_id = ""
    result._work_area_invalidation_seq = 0
    result.front_view_extent = None
    result.front_view_size = None
    result._locked_extent = None
    result._locked_extent_size = None
    result._show_fill_preview = False
    result._fill_preview_strokes = []
    result._zed_target = None
    result._zed_target_stamp_ns = 0
    result._zed_selection_stamp_ns = 0
    result._zed_front_generation = ""
    result._zed_front_stamp_ns = 0
    result._zed_image_stamp_ns = 0
    result._zed_first_front_stamp_ns = 0
    result._zed_last_status = None
    result._zed_info_frame = "zed_optical"
    result._zed_info_size = (640, 480)
    result.K = K.copy()
    result.front_view_source = "d405"
    for name in ("zed_status_pub", "work_area_state_pub", "work_area_corners_pub",
                 "work_area_pub", "front_extent_pub", "front_pub"):
        setattr(result, name, _Publisher())
    result.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    result.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=12_000_000_000,
        to_msg=lambda: SimpleNamespace(sec=12, nanosec=0)))
    return result


def _area(frame="wall_front", points=((0, 0), (899, 539)), sec=12):
    msg = PoseArray()
    msg.header.frame_id = frame
    msg.header.stamp.sec = sec
    for x, y in points:
        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        msg.poses.append(pose)
    return msg


def _request(message=None, *, generation=None, sec=12, nanosec=0):
    message = message or _area(sec=sec)
    return String(data=json.dumps(dict(
        source="zed", plane_generation_id=generation or _target()["plane_generation_id"],
        header=dict(frame_id=message.header.frame_id, stamp=dict(
            sec=message.header.stamp.sec, nanosec=nanosec)),
        pixels=[[pose.position.x, pose.position.y] for pose in message.poses],
    )))


def _ready(node, target_sec=10):
    target = _target(sec=target_sec)
    node._on_zed_target_lock(String(data=json.dumps(target)))
    node.front_view_extent = np.asarray(target["corners"])
    node.front_view_size = (900, 540)
    node._zed_front_generation = target["plane_generation_id"]
    node._zed_front_stamp_ns = (target_sec + 1) * 1_000_000_000
    node._zed_first_front_stamp_ns = (target_sec + 1) * 1_000_000_000


def _clock(node, sec):
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=sec * 1_000_000_000,
        to_msg=lambda: SimpleNamespace(sec=sec, nanosec=0)))


def test_atomic_status_binds_exact_plane_and_selection_identity(node):
    _ready(node)
    node._on_zed_work_area_request(_request())
    status = json.loads(node.zed_status_pub.messages[-1].data)
    assert status["accepted"] is True
    assert status["mode"] == "work_area"
    assert status["plane_generation_id"] == _target()["plane_generation_id"]
    assert status["selection_id"] == "12000000000"
    assert status["target_stamp"] == {"sec": 10, "nanosec": 0}
    assert status["corners"] == _target()["corners"]
    assert status["front_extent"] == _target()["corners"]
    assert (status["view_width"], status["view_height"]) == (900, 540)
    assert len(node.work_area_pub.messages) == 1
    assert len(node.work_area_corners_pub.messages) == 1
    from sketch_control.zed_spray_execution import validate_zed_work_area
    point, normal, corners = validate_zed_work_area(
        status, _target()["plane_generation_id"], "12000000000")
    assert point == pytest.approx([0, 0, 2])
    assert normal == pytest.approx([0, 0, -1])
    assert corners == pytest.approx(np.asarray(_target()["corners"]))
    from sketch_control.work_area_geometry import validate_zed_surface_status
    assert validate_zed_surface_status(status) == status


@pytest.mark.parametrize("event", ["empty", "invalidated", "mode"])
def test_invalidation_revokes_area_and_preserves_revoked_identity(node, event):
    _ready(node)
    node._on_zed_work_area_request(_request())
    before = json.loads(node.zed_status_pub.messages[-1].data)
    if event == "empty":
        node._on_work_area_pixels(_area(points=()))
    elif event == "invalidated":
        node._on_zed_target_lock(String(data=json.dumps(dict(accepted=False, state="invalidated"))))
    else:
        node._on_process_mode(String(data=json.dumps(dict(mode="paint"))))
    after = json.loads(node.zed_status_pub.messages[-1].data)
    assert after["accepted"] is False
    assert after["state"] == "invalidated"
    for key in ("work_area_id", "selection_id", "plane_generation_id"):
        assert after[key] == before[key]
    assert node.locked_work_area is None


@pytest.mark.parametrize("kind", ["raw", "zero_selection", "old_extent", "no_extent"])
def test_selection_requires_current_rectified_zed_view(node, kind):
    _ready(node)
    message = _area()
    if kind == "raw":
        message.header.frame_id = "zed_raw"
    elif kind == "zero_selection":
        message.header.stamp.sec = 0
    elif kind == "old_extent":
        node._zed_front_generation = "previous"
    else:
        node.front_view_extent = None
    node._on_zed_work_area_request(_request(message))
    assert node.locked_work_area is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_late_d405_and_generic_surface_callbacks_do_not_mutate_spray(node):
    _ready(node)
    before = copy.deepcopy(node.latest_surface)
    for callback in (node._on_wall, node._on_target_surface, node._on_refined_target_surface,
                     node._on_refined_work_area, node._on_d405_image, node._on_d405_info):
        callback(None)
    assert node.latest_surface[0] == pytest.approx(before[0])
    assert node.latest_surface[1] == pytest.approx(before[1])
    assert node.latest_surface[2:] == before[2:]
    assert node._zed_target["plane_generation_id"] == _target()["plane_generation_id"]


def _image(sec=11):
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[:, :, 0] = np.arange(640, dtype=np.uint16)[None, :] // 3
    rgb[:, :, 1] = np.arange(480, dtype=np.uint16)[:, None] // 2
    rgb[:, :, 2] = 33
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="zed_optical", stamp=SimpleNamespace(sec=sec, nanosec=0)),
        width=640, height=480, encoding="rgb8", data=rgb.tobytes(), step=640 * 3)


def test_zed_rgb_is_rectified_before_selection_and_is_not_mirrored(node):
    data = _target()
    data["corners"] = [data["corners"][i] for i in (2, 3, 0, 1)]
    node._on_zed_target_lock(String(data=json.dumps(data)))
    node._on_image(_image())
    assert len(node.front_pub.messages) == 1
    assert node.locked_work_area is None
    assert node.work_area_pub.messages == []
    front = node.front_pub.messages[-1]
    rgb = np.frombuffer(front.data, dtype=np.uint8).reshape(front.height, front.width, 3)
    assert rgb[0, 0].tolist() == [56, 75, 33]
    assert rgb[0, -1].tolist() == [156, 75, 33]
    assert rgb[-1, 0].tolist() == [56, 165, 33]
    assert node.front_view_extent == pytest.approx(np.asarray(_target()["corners"]))


def test_old_zed_frames_cannot_restore_previous_extent(node):
    node._on_zed_target_lock(String(data=json.dumps(_target())))
    node._on_image(_image())
    node._on_zed_target_lock(String(data=json.dumps(_target(sec=12))))
    node._on_image(_image(sec=11))
    assert len(node.front_pub.messages) == 1
    assert node.front_view_extent is None
    assert node._zed_front_generation == ""
    node._on_zed_work_area_request(_request(generation=_target(sec=12)["plane_generation_id"]))
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_revoked_target_cannot_be_replayed_to_restore_area(node):
    _ready(node)
    node._on_zed_target_lock(String(data='{"accepted":false,"state":"invalidated"}'))
    node._on_zed_target_lock(String(data=json.dumps(_target())))
    assert node._zed_target is None
    assert node.front_view_extent is None


def test_target_generation_cannot_change_geometry_in_place(node):
    _ready(node)
    data = _target()
    data["corners"] = (np.asarray(data["corners"]) + [0.01, 0, 0]).tolist()
    data["center"][0] = 0.01
    node._on_zed_target_lock(String(data=json.dumps(data)))
    assert node._zed_target is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_current_bad_camera_frame_revokes_area_and_clears_extent(node):
    _ready(node)
    node._on_zed_work_area_request(_request())
    image = _image(sec=12)
    image.header.frame_id = "d405_optical"
    node._on_image(image)
    assert node.locked_work_area is None
    assert node.front_view_extent is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_mode_round_trip_requires_fresh_target_and_area(node):
    _ready(node)
    node._on_zed_work_area_request(_request())
    for mode in ("paint", "spray"):
        node._on_process_mode(String(data=json.dumps({"mode": mode})))
    node._on_zed_target_lock(String(data=json.dumps(_target())))
    node._on_image(_image())
    node._on_zed_work_area_request(_request())
    assert node._zed_target is None
    assert node.locked_work_area is None
    assert node.front_pub.messages == []


def test_paint_still_uses_d405_and_ignores_zed_lock(node):
    node.process_mode = "paint"
    node._on_zed_target_lock(String(data=json.dumps(_target())))
    assert node._zed_target is None
    node._on_d405_info(SimpleNamespace(k=K.flatten()))
    assert node.d405_K == pytest.approx(K)
    pose = SimpleNamespace(header=SimpleNamespace(frame_id="d405_optical"), pose=SimpleNamespace(
        position=SimpleNamespace(x=0, y=0, z=2), orientation=SimpleNamespace(x=1, y=0, z=0, w=0)))
    node._on_refined_target_surface(pose)
    assert node.latest_surface[3] == "target_refined"
    assert node.latest_surface[2] == "d405_optical"


def test_unknown_mode_does_not_crash_or_change_active_lock(node):
    _ready(node)
    node._on_process_mode(String(data='{"mode": []}'))
    assert node.process_mode == "spray"
    assert node._zed_target is not None


def test_rejected_newer_frame_does_not_allow_older_frame_to_restore_extent(node):
    node._on_zed_target_lock(String(data=json.dumps(_target())))
    node._on_image(_image(sec=11))
    bad = _image(sec=12)
    bad.header.frame_id = "d405_optical"
    node._on_image(bad)
    node._on_image(_image(sec=11))
    assert node.front_view_extent is None
    assert len(node.front_pub.messages) == 1


def test_rejected_newer_selection_does_not_allow_older_area_to_lock(node):
    _ready(node)
    node._on_zed_work_area_request(_request(_area(points=((-1, 0), (899, 539)), sec=12)))
    node._on_zed_work_area_request(_request(sec=11))
    assert node.locked_work_area is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_first_area_selection_never_publishes_target_invalidation(node):
    _ready(node)
    before = len(node.zed_status_pub.messages)
    node._on_zed_work_area_request(_request())
    statuses = [json.loads(msg.data) for msg in node.zed_status_pub.messages[before:]]
    assert len(statuses) == 1
    assert statuses[0]["mode"] == "work_area"
    assert statuses[0]["accepted"] is True


def test_replacing_area_invalidates_only_previous_selection(node):
    _ready(node)
    node._on_zed_work_area_request(_request())
    previous = json.loads(node.zed_status_pub.messages[-1].data)
    before = len(node.zed_status_pub.messages)
    node._on_zed_work_area_request(_request(sec=13))
    statuses = [json.loads(msg.data) for msg in node.zed_status_pub.messages[before:]]
    assert len(statuses) == 2
    assert statuses[0]["mode"] == "work_area"
    assert statuses[0]["accepted"] is False
    assert statuses[0]["selection_id"] == previous["selection_id"] == "12000000000"
    assert statuses[0]["work_area_id"] == previous["work_area_id"]
    assert statuses[1]["accepted"] is True
    assert statuses[1]["selection_id"] == "13000000000"
    assert statuses[1]["work_area_id"] != previous["work_area_id"]


@pytest.mark.parametrize("offset", [-3600, 3600])
def test_browser_clock_offset_does_not_change_generation_bound_acceptance(node, offset):
    _clock(node, 7200)
    _ready(node, target_sec=7198)
    generation = _target(sec=7198)["plane_generation_id"]
    node._on_zed_work_area_request(_request(generation=generation, sec=7200 + offset, nanosec=123))
    status = json.loads(node.zed_status_pub.messages[-1].data)
    assert status["mode"] == "work_area" and status["accepted"] is True
    assert status["selection_id"] == str((7200 + offset) * 1_000_000_000 + 123)
    assert status["target_stamp"] == {"sec": 7198, "nanosec": 0}
    assert node.work_area_pub.messages[-1].header.stamp.nanosec == 123


@pytest.mark.parametrize("legacy_first", [True, False])
def test_legacy_pixels_cannot_accept_or_invalidate_json_selection(node, legacy_first):
    _ready(node)
    if legacy_first:
        node._on_work_area_pixels(_area())
        assert node.locked_work_area is None
        assert node.work_area_pub.messages == []
    node._on_zed_work_area_request(_request())
    before = json.loads(node.zed_status_pub.messages[-1].data)
    node._on_work_area_pixels(_area(points=((100, 100), (500, 400)), sec=13))
    node._on_work_area_pixels(_area(frame="zed_raw", sec=14))
    assert json.loads(node.zed_status_pub.messages[-1].data) == before
    assert node.locked_work_area["selection_id"] == "12000000000"


def test_sim_time_and_browser_wall_clock_do_not_need_the_same_epoch(node):
    _ready(node)
    node._on_zed_work_area_request(_request(sec=1_800_000_000))
    assert node.locked_work_area["selection_id"] == "1800000000000000000"


def test_old_generation_request_cannot_bind_to_new_target_or_poison_its_order(node):
    _ready(node)
    node._on_zed_work_area_request(_request(sec=10800))
    _clock(node, 22)
    _ready(node, target_sec=20)
    node._on_zed_work_area_request(_request(sec=10801))
    assert node.locked_work_area is None
    node._on_zed_work_area_request(_request(generation=_target(sec=20)["plane_generation_id"], sec=3600))
    accepted = json.loads(node.zed_status_pub.messages[-1].data)
    assert accepted["accepted"] is True and accepted["selection_id"] == "3600000000000"
    node._on_zed_work_area_request(_request(sec=10802))
    assert json.loads(node.zed_status_pub.messages[-1].data) == accepted


def test_replay_after_clear_cannot_reopen_selection_and_clear_uses_no_ros_time(node):
    _clock(node, 7200)
    _ready(node, target_sec=7198)
    generation = _target(sec=7198)["plane_generation_id"]
    request = _request(generation=generation, sec=3600)
    node._on_zed_work_area_request(request)
    node._on_work_area_pixels(_area(points=(), sec=3601))
    node._on_zed_work_area_request(request)
    assert node.locked_work_area is None
    node._on_zed_work_area_request(_request(generation=generation, sec=3602))
    assert node.locked_work_area["selection_id"] == "3602000000000"


def test_atomic_request_cannot_reopen_revoked_target_generation(node):
    _ready(node)
    request = _request()
    node._on_zed_work_area_request(request)
    node._on_zed_target_lock(String(data='{"source":"zed","accepted":false,"state":"invalidated"}'))
    node._on_zed_work_area_request(request)
    assert node._zed_target is None and node.locked_work_area is None


@pytest.mark.parametrize("patch", [
    {"source": "d405"}, {"header": {}}, {"header": {"frame_id": "zed_raw", "stamp": {"sec": 12, "nanosec": 0}}},
    {"pixels": [[0, 0], [float("nan"), 539]]}, {"pixels": [[0, 0], [True, 539]]},
    {"pixels": [[0, 0, 0], [899, 539, 0]]}, {"pixels": [[0, 0], ["899", 539]]},
    {"pixels": [[0, 0]]}, {"pixels": None},
])
def test_atomic_request_rejects_invalid_source_header_or_pixels(node, patch):
    _ready(node)
    payload = json.loads(_request().data)
    payload.update(patch)
    node._on_zed_work_area_request(String(data=json.dumps(payload)))
    assert node.locked_work_area is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]


def test_empty_atomic_request_clears_area_and_replay_cannot_restore_it(node):
    _ready(node)
    request = _request(sec=3600)
    node._on_zed_work_area_request(request)
    accepted = json.loads(node.zed_status_pub.messages[-1].data)
    node._on_zed_work_area_request(_request(_area(points=(), sec=3601)))
    cleared = json.loads(node.zed_status_pub.messages[-1].data)
    assert cleared["accepted"] is False
    assert cleared["selection_id"] == accepted["selection_id"]
    assert cleared["work_area_id"] == accepted["work_area_id"]
    node._on_zed_work_area_request(request)
    assert node.locked_work_area is None
    assert json.loads(node.zed_status_pub.messages[-1].data) == cleared


def test_duplicate_request_does_not_replace_active_area(node):
    _ready(node)
    request = _request()
    node._on_zed_work_area_request(request)
    accepted = json.loads(node.zed_status_pub.messages[-1].data)
    before = len(node.zed_status_pub.messages)
    node._on_zed_work_area_request(request)
    assert len(node.zed_status_pub.messages) == before
    assert node.locked_work_area["work_area_id"] == accepted["work_area_id"]


def test_atomic_zed_requests_do_not_change_paint(node):
    node.process_mode = "paint"
    node._on_zed_work_area_request(_request())
    assert node.zed_status_pub.messages == []
    assert node.work_area_pub.messages == []


@pytest.mark.parametrize("stamp", [
    {"sec": True, "nanosec": 0}, {"sec": 12.5, "nanosec": 0},
    {"sec": 12, "nanosec": -1}, {"sec": 12, "nanosec": 1_000_000_000},
    {"sec": 2_147_483_648, "nanosec": 0},
])
def test_malformed_browser_stamp_is_rejected_before_ros_message_assignment(node, stamp):
    _ready(node)
    payload = json.loads(_request().data)
    payload["header"]["stamp"] = stamp
    node._on_zed_work_area_request(String(data=json.dumps(payload)))
    assert node.locked_work_area is None
    assert not json.loads(node.zed_status_pub.messages[-1].data)["accepted"]
