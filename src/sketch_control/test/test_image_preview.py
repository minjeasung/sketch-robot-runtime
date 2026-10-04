"""Display compression must preserve pixel geometry and the raw ROS inputs."""
from types import SimpleNamespace

import cv2
import numpy as np
from std_msgs.msg import Header

from sketch_control.image_preview import ImagePreview
from sketch_control.outpost_bridge_node import OutpostBridge


class Publisher:
    def __init__(self, subscribers=1):
        self.subscribers = subscribers
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


class PreviewNode:
    def __init__(self):
        self.publisher = Publisher()

    def create_publisher(self, kind, topic, qos):
        self.topic, self.qos = topic, qos
        return self.publisher


def test_preview_preserves_dimensions_color_header_and_input():
    node = PreviewNode()
    preview = ImagePreview(node, '/camera/image')
    rgb = np.full((240, 320, 3), [220, 40, 15], dtype=np.uint8)
    before = rgb.copy()
    header = Header(frame_id='optical')
    header.stamp.sec = 42
    preview.publish(rgb, header)
    msg = node.publisher.messages[0]
    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (240, 320, 3)
    np.testing.assert_allclose(image[100, 100, ::-1], [220, 40, 15], atol=4)
    np.testing.assert_array_equal(rgb, before)
    assert msg.header == header
    assert 'jpeg' in msg.format
    assert len(msg.data) < rgb.nbytes / 10
    assert node.topic == '/camera/image/compressed'
    assert node.qos.depth == 1


def test_preview_skips_encoding_without_viewer_and_limits_rate(monkeypatch):
    node = PreviewNode()
    now = [10.0]
    monkeypatch.setattr('sketch_control.image_preview.time.monotonic', lambda: now[0])
    preview = ImagePreview(node, '/camera/image')
    node.publisher.subscribers = 0
    preview.publish(None, Header())  # Must not even try to encode.
    node.publisher.subscribers = 1
    rgb = np.zeros((24, 32, 3), np.uint8)
    preview.publish(rgb, Header())
    now[0] = 10.1
    preview.publish(rgb, Header())
    assert len(node.publisher.messages) == 1
    now[0] = 10.21
    preview.publish(rgb, Header())
    assert len(node.publisher.messages) == 2


def test_outpost_keeps_raw_rgb_and_depth_when_adding_preview():
    node = PreviewNode()
    preview = ImagePreview(node, '/camera/image')
    rgb = np.arange(24 * 32 * 3, dtype=np.uint8).reshape(24, 32, 3)
    depth = np.full((24, 32), 1.2345, np.float32)
    publishers = [Publisher(), Publisher(0), Publisher(), Publisher(0), Publisher(0)]
    camera = dict(image_frame='optical', publishers=publishers, preview=preview)
    OutpostBridge.publish_frame(SimpleNamespace(), camera, (42_000_000_000, 1, rgb, depth, None))
    assert bytes(publishers[0].messages[0].data) == rgb.tobytes()
    assert bytes(publishers[2].messages[0].data) == depth.tobytes()
    assert len(node.publisher.messages) == 1
