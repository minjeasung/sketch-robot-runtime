"""Request-bound projection of upstream plane snapshots into Sketch topics."""
import json
import time
import numpy as np
from .planes import fingerprint, project_catalogue


def run(config):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseArray
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException
    from scipy.spatial.transform import Rotation

    class Bridge(Node):
        def __init__(self):
            super().__init__('snucem_sketch_plane_bridge')
            self.catalog, self.info, self.image = None, None, None
            self.catalog_at = self.image_at = float('-inf')
            self.request, self.request_stamp = None, 0
            self.published_key = None
            self.tf = Buffer(node=self)
            self.listener = TransformListener(self.tf, self)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/perception/target_planes', latched)
            self.state_pub = self.create_publisher(String, '/snucem_sketch/source', latched)
            self.create_subscription(String, '/snucem_sketch/plane_catalog', self.on_catalog, latched)
            self.create_subscription(CameraInfo, config.camera_info_topic, self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, config.image_topic, self.on_image, qos_profile_sensor_data)
            self.create_subscription(PoseArray, '/target_selection_pixels', self.on_request, 10)
            self.create_timer(.2, self.tick)

        def on_catalog(self, msg):
            try:
                data = json.loads(msg.data)
                if data.get('schema_version') != 1:
                    raise ValueError('wrong source schema')
                self.catalog, self.catalog_at = data, time.monotonic()
            except (ValueError, TypeError, AttributeError):
                self.catalog = None

        def on_info(self, msg):
            self.info = msg

        def on_image(self, msg):
            self.image = msg.header
            self.image_size = (msg.width, msg.height)
            self.image_at = time.monotonic()

        def on_request(self, msg):
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp <= self.request_stamp or msg.header.frame_id != 'zed_raw':
                return
            self.request_stamp = stamp
            self.request = [[p.position.x, p.position.y] for p in msg.poses]
            self.published_key = None
            self.tick()

        def tick(self):
            revision, error = '', ''
            try:
                now = time.monotonic()
                if not self.catalog or now-self.catalog_at > 1.0 or not self.catalog.get('revision'):
                    raise ValueError('plane source unavailable/stale')
                if self.info is None or self.image is None or now-self.image_at > 2.0:
                    raise ValueError('camera image/CameraInfo unavailable')
                if self.image.frame_id != self.info.header.frame_id or self.image_size != (self.info.width, self.info.height):
                    raise ValueError('camera frame/size mismatch')
                transform = self.tf.lookup_transform(self.info.header.frame_id, self.catalog['frame_id'],
                    Time.from_msg(self.image.stamp), timeout=Duration(seconds=.05)).transform
                q, t = transform.rotation, transform.translation
                T = np.eye(4)
                T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                T[:3, 3] = [t.x, t.y, t.z]
                revision = fingerprint(dict(source=self.catalog['revision'], K=list(self.info.k),
                                            size=self.image_size, transform=np.round(T, 8).tolist()))
                key = (self.request_stamp, revision)
                if self.request and key != self.published_key:
                    payload = project_catalogue(self.catalog, np.asarray(self.info.k).reshape(3, 3),
                        self.image_size, T, str(self.request_stamp), self.request)
                    payload['source_revision'] = revision
                    self.pub.publish(String(data=json.dumps(payload, allow_nan=False)))
                    self.published_key = key
                elif self.request == [] and key != self.published_key:
                    self.pub.publish(String(data=json.dumps(dict(generation=str(self.request_stamp), planes=[], error='selection cleared'))))
                    self.published_key = key
            except (ValueError, KeyError, TypeError, TransformException) as exc:
                error, revision = str(exc), ''
                key = (self.request_stamp, error)
                if self.request_stamp and key != self.published_key:
                    self.pub.publish(String(data=json.dumps(dict(generation=str(self.request_stamp), planes=[], error=error))))
                    self.published_key = key
            self.state_pub.publish(String(data=json.dumps(dict(revision=revision, error=error))))

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
