"""Bounded JPEG previews for browsers; raw perception topics stay unchanged."""
import time

import cv2
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage


class ImagePreview:
    def __init__(self, node, image_topic):
        self.publisher = node.create_publisher(
            CompressedImage, image_topic + '/compressed', QoSProfile(depth=1))
        self.next_publish = 0.0

    def publish(self, rgb, header):
        now = time.monotonic()
        if not self.publisher.get_subscription_count() or now < self.next_publish:
            return
        # Preserve the exact pixel grid used by sketches and calibrated rays.
        # Only this display copy is lossy; never resize or modify the input.
        ok, encoded = cv2.imencode(
            '.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return
        self.publisher.publish(CompressedImage(
            header=header, format='rgb8; jpeg compressed bgr8',
            data=encoded.tobytes()))
        self.next_publish = now + 0.2
