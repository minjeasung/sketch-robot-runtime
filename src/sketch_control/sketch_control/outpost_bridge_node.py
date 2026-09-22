"""Outpost IPC -> the sketch system's existing calibrated ROS camera topics."""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
import zmq

from .outpost_camera import camera_status, decode_frame, FrameGuard


class OutpostBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('sketch_outpost_bridge', **kwargs)
        self.origin = self.declare_parameter('outpost_http', 'http://127.0.0.1:8100').value
        publish_hz = float(self.declare_parameter('publish_hz', 10.0).value)
        if not math.isfinite(publish_hz) or not 0 < publish_hz <= 30:
            raise ValueError('publish_hz must be in (0, 30]')
        self.publish_period = 1.0 / publish_hz
        self.cameras = []
        self._zmq_context = zmq.Context()
        # Reliable publishers serve both best-effort image subscribers and the
        # refiner's reliable PointCloud2 subscriber, with a bounded queue.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        try:
            for name, kind, frame, topics in (
                ('zed', 'zed', 'zed_left_camera_frame_optical', (
                    '/zed/zed_node/rgb/color/rect/image', '/zed/zed_node/rgb/color/rect/camera_info',
                    '/zed/zed_node/depth/depth_registered', '/zed/zed_node/depth/camera_info',
                    '/zed/zed_node/point_cloud/cloud_registered')),
                ('d405', 'realsense', 'd405_color_optical_frame', (
                    '/d405/d405/color/image_raw', '/d405/d405/color/camera_info',
                    '/d405/d405/depth/image_rect_raw', '/d405/d405/depth/camera_info',
                    '/d405/d405/depth/color/points')),
            ):
                hw = self.declare_parameter(f'outpost_{name}_hw_id', '').value
                serial = self.declare_parameter(f'outpost_{name}_serial', '').value
                status = camera_status(self.origin, hw, serial, kind)
                sock = self._zmq_context.socket(zmq.SUB)
                sock.setsockopt(zmq.RCVHWM, 1)
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.MAXMSGSIZE, 128*1024*1024)
                sock.setsockopt(zmq.SUBSCRIBE, hw.encode())
                sock.connect(status['local_raw_endpoint'])
                self.cameras.append(dict(name=name, kind=kind, frame=frame, status=status,
                    hw=hw, serial=serial, sock=sock, guard=FrameGuard(), last=time.monotonic(), next_publish=0.,
                    publishers=[self.create_publisher(t, topic, qos) for t, topic in
                                zip((Image, CameraInfo, Image, CameraInfo, PointCloud2), topics)]))
        except Exception:
            self.close()
            raise

    def close(self):
        for camera in self.cameras:
            camera['sock'].close(linger=0)
        self._zmq_context.term()

    def publish_frame(self, camera, data):
        stamp, _, rgb, depth, cloud = data
        header = Header(frame_id=camera['frame'])
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        h, w = depth.shape
        info = CameraInfo(header=header, width=w, height=h, distortion_model='plumb_bob')
        k = camera['status']['intrinsics']
        info.k = [float(k['fx']), 0., float(k['cx']), 0., float(k['fy']), float(k['cy']), 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [float(k['fx']), 0., float(k['cx']), 0., 0., float(k['fy']), float(k['cy']), 0., 0., 0., 1., 0.]
        for index, array, encoding in ((0, rgb, 'rgb8'), (2, depth, '32FC1')):
            array = np.ascontiguousarray(array)
            msg = Image(header=header, width=w, height=h, encoding=encoding,
                        is_bigendian=0, step=w*(3 if index == 0 else 4), data=array.tobytes())
            camera['publishers'][index].publish(msg)
        camera['publishers'][1].publish(info)
        camera['publishers'][3].publish(info)
        if not camera['publishers'][4].get_subscription_count():
            return
        packed = np.empty((h, w), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        for index, axis in enumerate(('x','y','z')):
            packed[axis] = cloud[..., index]
        packed['rgb'] = ((rgb[...,0].astype(np.uint32)<<16)
                         | (rgb[...,1].astype(np.uint32)<<8) | rgb[...,2].astype(np.uint32))
        fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, axis in enumerate(('x','y','z','rgb'))]
        camera['publishers'][4].publish(PointCloud2(header=header, height=h, width=w,
            fields=fields, is_bigendian=False, point_step=16, row_step=w*16,
            data=packed.tobytes(), is_dense=False))

    def run(self):
        next_status = 0.0
        while rclpy.ok(context=self.context):
            now = time.monotonic()
            for camera in self.cameras:
                if now - camera['last'] > 3.0:
                    raise TimeoutError(camera['name'] + ': fresh frames lost; restart perception')
            if now >= next_status:
                for camera in self.cameras:
                    status = camera_status(self.origin, camera['hw'], camera['serial'], camera['kind'])
                    for key in ('generation', 'intrinsics', 'resolution', 'local_raw_endpoint'):
                        if status[key] != camera['status'][key]:
                            raise ValueError(camera['name'] + ': stream changed; restart perception')
                next_status = time.monotonic() + 1.0
            for camera in self.cameras:
                sock = camera['sock']
                if not sock.poll(timeout=10):
                    continue
                parts = sock.recv_multipart()
                for _ in range(8):
                    if not sock.getsockopt(zmq.EVENTS) & zmq.POLLIN:
                        break
                    parts = sock.recv_multipart()
                frame = decode_frame(parts, camera['status'], camera['kind'])
                if not camera['guard'].accept(frame[0], frame[1]) or not np.isfinite(frame[4][...,2]).any():
                    continue
                camera['last'] = time.monotonic()
                if camera['last'] < camera['next_publish']:
                    continue
                camera['next_publish'] = max(camera['next_publish'] + self.publish_period,
                                             camera['last'])
                self.publish_frame(camera, frame)
            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OutpostBridge()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
