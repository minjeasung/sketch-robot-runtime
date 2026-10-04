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
