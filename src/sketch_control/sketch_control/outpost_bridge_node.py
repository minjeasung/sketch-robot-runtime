"""Outpost raw IPC -> Sketch ROS camera topics.

The launch file normally starts one process per physical camera.  Keeping ZED
and D405 isolated prevents a slow global cloud conversion from delaying the
wrist camera, while this module still supports camera_name=all for offline
smoke tests and backwards-compatible manual use.
"""
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


CAMERA_SPECS = {
    "zed": {
        "kind": "zed",
        "frame": "zed_left_camera_frame_optical",
        "topics": (
            "/zed/zed_node/rgb/color/rect/image",
            "/zed/zed_node/rgb/color/rect/camera_info",
            "/zed/zed_node/depth/depth_registered",
            "/zed/zed_node/depth/camera_info",
            "/zed/zed_node/point_cloud/cloud_registered",
        ),
    },
    "d405": {
        "kind": "realsense",
        "frame": "d405_color_optical_frame",
        "topics": (
            "/d405/d405/color/image_raw",
            "/d405/d405/color/camera_info",
            "/d405/d405/depth/image_rect_raw",
            "/d405/d405/depth/camera_info",
            "/d405/d405/depth/color/points",
        ),
    },
}


def sample_pointcloud_grid(cloud, rgb, stride):
    """Downsample only the ROS point cloud grid, preserving full RGB/depth images."""
    stride = int(stride)
    if stride < 1:
        raise ValueError("point_stride must be >= 1")
    cloud = np.asarray(cloud)
    rgb = np.asarray(rgb)
    if cloud.shape[:2] != rgb.shape[:2]:
        raise ValueError("RGB and point cloud grids differ")
    return cloud[::stride, ::stride], rgb[::stride, ::stride]


class OutpostBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("sketch_outpost_bridge", **kwargs)
        self.origin = self.declare_parameter(
            "outpost_http", "http://127.0.0.1:8100"
        ).value
        camera_name = str(self.declare_parameter("camera_name", "all").value).lower()
        if camera_name not in ("all", "zed", "d405"):
            raise ValueError("camera_name must be all, zed, or d405")
        publish_hz = float(self.declare_parameter("publish_hz", 10.0).value)
        point_stride = int(self.declare_parameter("point_stride", 1).value)
        frame_timeout = float(self.declare_parameter("frame_timeout_s", 3.0).value)
        max_frame_age = float(self.declare_parameter("max_frame_age_s", 1.0).value)
        status_period = float(self.declare_parameter("status_period_s", 1.0).value)
        if not math.isfinite(publish_hz) or not 0 < publish_hz <= 30:
            raise ValueError("publish_hz must be in (0, 30]")
        if point_stride < 1:
            raise ValueError("point_stride must be >= 1")
        for name, value in (
            ("frame_timeout_s", frame_timeout),
            ("max_frame_age_s", max_frame_age),
            ("status_period_s", status_period),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")

        self.publish_period = 1.0 / publish_hz
        self.frame_timeout = frame_timeout
        self.max_frame_age = max_frame_age
        self.status_period = status_period
        self.cameras = []
        self._zmq_context = zmq.Context()

        # Keep the established reliable depth-1 contract.  Heavy copies are
        # avoided below when a topic has no subscriber.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        selected = CAMERA_SPECS if camera_name == "all" else {
            camera_name: CAMERA_SPECS[camera_name]
        }
        try:
            for name, spec in selected.items():
                kind, frame, topics = spec["kind"], spec["frame"], spec["topics"]
                hw = self.declare_parameter(f"outpost_{name}_hw_id", "").value
                serial = self.declare_parameter(f"outpost_{name}_serial", "").value
                status = camera_status(self.origin, hw, serial, kind)
                self.cameras.append(
                    dict(
                        name=name,
                        kind=kind,
                        frame=frame,
                        status=status,
                        hw=hw,
                        serial=serial,
                        sock=self._open_socket(status["local_raw_endpoint"], hw),
                        guard=FrameGuard(max_age=max_frame_age),
                        last=time.monotonic(),
                        next_publish=0.0,
                        next_status=0.0,
                        point_stride=point_stride,
                        publishers=[
                            self.create_publisher(t, topic, qos)
                            for t, topic in zip(
                                (Image, CameraInfo, Image, CameraInfo, PointCloud2),
                                topics,
                            )
                        ],
                    )
                )
        except Exception:
            self.close()
            raise

    def _open_socket(self, endpoint, hw):
        sock = self._zmq_context.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.MAXMSGSIZE, 128 * 1024 * 1024)
        sock.setsockopt(zmq.SUBSCRIBE, hw.encode())
        sock.connect(endpoint)
        return sock

    def close(self):
        for camera in self.cameras:
            camera["sock"].close(linger=0)
        self.cameras.clear()
        self._zmq_context.term()

    def _refresh_status(self, camera):
        """Refresh stream metadata; reconnect safely when generation/IPC changes."""
        status = camera_status(
            self.origin, camera["hw"], camera["serial"], camera["kind"]
        )
        previous = camera["status"]

        # Intrinsics or image geometry changes invalidate pixel-to-ray calibration
        # and must not be silently accepted during an active perception session.
        for key in ("intrinsics", "resolution"):
            if status[key] != previous[key]:
                raise ValueError(
                    f"{camera['name']}: {key} changed; restart perception/calibration"
                )

        reconnect = (
            status["generation"] != previous["generation"]
            or status["local_raw_endpoint"] != previous["local_raw_endpoint"]
        )
        camera["status"] = status
        if reconnect:
            camera["sock"].close(linger=0)
            camera["sock"] = self._open_socket(
                status["local_raw_endpoint"], camera["hw"]
            )
            camera["guard"] = FrameGuard(max_age=self.max_frame_age)
            camera["last"] = time.monotonic()
            camera["next_publish"] = 0.0
            self.get_logger().info(
                f"{camera['name']}: Outpost stream generation/IPC changed; "
                "subscriber reconnected"
            )

    @staticmethod
    def _camera_info(header, width, height, intrinsics):
        info = CameraInfo(
            header=header,
            width=width,
            height=height,
            distortion_model="plumb_bob",
        )
        k = intrinsics
        info.k = [
            float(k["fx"]), 0.0, float(k["cx"]),
            0.0, float(k["fy"]), float(k["cy"]),
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            float(k["fx"]), 0.0, float(k["cx"]), 0.0,
            0.0, float(k["fy"]), float(k["cy"]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def publish_frame(self, camera, data):
        stamp, _, rgb, depth, cloud = data
        header = Header(frame_id=camera["frame"])
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        h, w = depth.shape
        pubs = camera["publishers"]

        # Preserve full-resolution image + CameraInfo for sketch pixel -> 3D ROI.
        if pubs[0].get_subscription_count():
            array = np.ascontiguousarray(rgb)
            pubs[0].publish(
                Image(
                    header=header,
                    width=w,
                    height=h,
                    encoding="rgb8",
                    is_bigendian=0,
                    step=w * 3,
                    data=array.tobytes(),
                )
            )
        if pubs[2].get_subscription_count():
            array = np.ascontiguousarray(depth, dtype=np.float32)
            pubs[2].publish(
                Image(
                    header=header,
                    width=w,
                    height=h,
                    encoding="32FC1",
                    is_bigendian=0,
                    step=w * 4,
                    data=array.tobytes(),
                )
            )

        if pubs[1].get_subscription_count() or pubs[3].get_subscription_count():
            info = self._camera_info(header, w, h, camera["status"]["intrinsics"])
            if pubs[1].get_subscription_count():
                pubs[1].publish(info)
            if pubs[3].get_subscription_count():
                pubs[3].publish(info)

        # The global ZED cloud can be sparse without losing sketch pixel
        # precision because sketch ROI reconstruction uses full depth+CameraInfo.
        # D405 uses stride=1 in the launch file to preserve local surface detail.
        if not pubs[4].get_subscription_count():
            return
        sampled_cloud, sampled_rgb = sample_pointcloud_grid(
            cloud, rgb, camera["point_stride"]
        )
        ph, pw = sampled_cloud.shape[:2]
        packed = np.empty(
            (ph, pw),
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
        )
        for index, axis in enumerate(("x", "y", "z")):
            packed[axis] = sampled_cloud[..., index]
        packed["rgb"] = (
            (sampled_rgb[..., 0].astype(np.uint32) << 16)
            | (sampled_rgb[..., 1].astype(np.uint32) << 8)
            | sampled_rgb[..., 2].astype(np.uint32)
        )
        fields = [
            PointField(
                name=axis,
                offset=i * 4,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for i, axis in enumerate(("x", "y", "z", "rgb"))
        ]
        pubs[4].publish(
            PointCloud2(
                header=header,
                height=ph,
                width=pw,
                fields=fields,
                is_bigendian=False,
                point_step=16,
                row_step=pw * 16,
                data=packed.tobytes(),
                is_dense=False,
            )
        )

    def run(self):
        while rclpy.ok(context=self.context):
            now = time.monotonic()
            for camera in self.cameras:
                if now - camera["last"] > self.frame_timeout:
                    raise TimeoutError(
                        camera["name"]
                        + ": fresh frames lost; restart perception if stream is not recovered"
                    )

                if now >= camera["next_status"]:
                    self._refresh_status(camera)
                    camera["next_status"] = time.monotonic() + self.status_period

                sock = camera["sock"]
                if not sock.poll(timeout=10):
                    continue
                parts = sock.recv_multipart()
                # Drain complete multipart messages so processing follows the
                # newest frame instead of building latency behind the camera.
                for _ in range(8):
                    if not sock.getsockopt(zmq.EVENTS) & zmq.POLLIN:
                        break
                    parts = sock.recv_multipart()

                try:
                    frame = decode_frame(parts, camera["status"], camera["kind"])
                except ValueError as exc:
                    # A generation race can leave one old multipart frame queued
                    # immediately after a safe reconnect.  Refresh once; any
                    # persistent contract violation still fails closed.
                    if "generation" not in str(exc):
                        raise
                    self._refresh_status(camera)
                    continue

                if (
                    not camera["guard"].accept(frame[0], frame[1])
                    or not np.isfinite(frame[4][..., 2]).any()
                ):
                    continue
                camera["last"] = time.monotonic()
                if camera["last"] < camera["next_publish"]:
                    continue
                camera["next_publish"] = max(
                    camera["next_publish"] + self.publish_period,
                    camera["last"],
                )
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


if __name__ == "__main__":
    main()
