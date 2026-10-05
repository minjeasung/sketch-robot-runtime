from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import Image
from sketch_control.target_selector_node import TargetSelectorNode


def _node(depth_ns=100_000_000, shape=(2, 2)):
    node = object.__new__(TargetSelectorNode)
    node.rgb_frames = deque(maxlen=4)
    node.latest_depth = np.ones(shape)
    node.latest_depth_header = SimpleNamespace(
        stamp=SimpleNamespace(sec=10, nanosec=depth_ns))
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args: None)
    return node


def _image(ns=100_000_000):
    msg = Image()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = ns
    msg.height = msg.width = 2
    msg.encoding = 'bgr8'
    msg.step = 8
    msg.data = bytes([10, 20, 30]*2+[0, 0])*2
    return msg


def test_color_edges_use_matching_depth_frame_and_honor_row_padding():
    node = _node()
    node._on_rgb(_image())
    node._on_rgb(_image(400_000_000))
    rgb = node._rgb_for_depth()
    assert rgb.shape == (2, 2, 3)
    np.testing.assert_array_equal(rgb[1, 1], [30, 20, 10])


@pytest.mark.parametrize('shape,stamp', [((2, 2), 400_000_000), ((3, 3), 100_000_000)])
def test_stale_or_different_resolution_rgb_does_not_guide_depth(shape, stamp):
    node = _node(shape=shape)
    node._on_rgb(_image(stamp))
    assert node._rgb_for_depth() is None


def test_target_selection_uses_hbeam_profile_for_recorded_faces():
    import time
    from pathlib import Path
    from geometry_msgs.msg import Pose, PoseArray
    import json

    scene = np.load(Path(__file__).parent/'fixtures/hbeam_depth.npz')
    rgb = scene['rgb']; node = _node(shape=rgb.shape[:2])
    node.latest_depth[:] = np.nan
    node.latest_depth[::4, ::4] = scene['depths'][0]
    node.latest_depth_received_at = time.monotonic()
    node.latest_depth_header.frame_id = 'zed_left_camera_frame_optical'
    node.K = scene['K']
    node.plane_structure = 'hbeam'
    node.rgb_frames.append((10_100_000_000, rgb))
    published = []
    node.catalog_pub = SimpleNamespace(publish=lambda msg: published.append(json.loads(msg.data)))
    request = PoseArray()
    request.header.frame_id = 'zed_raw'; request.header.stamp.sec = 11
    for x, y in ((0., 0.), (155., 631.)):
        p = Pose(); p.position.x = x; p.position.y = y
        request.poses.append(p)
    node._on_selection(request)
    assert len(published) == 1
    assert len(published[0]['planes']) == 4
    assert published[0]['structure'] == 'hbeam'
    assert published[0]['extraction_method'] == 'rgb_depth_regions'
