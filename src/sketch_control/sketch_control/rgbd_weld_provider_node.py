"""ROS adapter for physical RGB-D selection; also builds the measured task mesh."""
from dataclasses import asdict
import json
import time
import numpy as np
from .camera_registration import load_registration
from .mixed_frame_sync import FrameBinding
from .rgbd_weld_provider import RGBDFrameStore, project_seam, project_surface_measurement
from .weld_contract import validate_target


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, CameraInfo
    class RGBDWeldProvider(Node):
        def __init__(self):
            super().__init__('rgbd_weld_provider')
            self.registration=load_registration(self.declare_parameter('camera_registration_file','').value,self.declare_parameter('camera_name','zed').value)
            self.frames=RGBDFrameStore(self.registration)
            self.result=self.create_publisher(String,'/welding/extraction_result',10)
            self.scene=self.create_publisher(String,'/task_scene/request',10)
            qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',self.rgb,qos)
            self.create_subscription(Image,'/zed/zed_node/depth/depth_registered',self.depth,qos)
            self.create_subscription(CameraInfo,'/zed/zed_node/rgb/color/rect/camera_info',self.info,qos)
            self.create_subscription(String,'/perception/zed_frame_identity',self.identity,10)
            self.create_subscription(String,'/welding/extraction_request',self.request,10)
            self.create_subscription(String,'/perception/selection_pin',self.pin,10)
            # Spray/paint target selection shares the same physical RGB-D cache.
            self.create_subscription(String,'/task_scene/selection',self.selection,10)
        @staticmethod
        def stamp(msg): return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        def rgb(self,msg):
            if msg.header.frame_id==self.registration.optical_frame: self.frames.add_rgb(self.stamp(msg),(msg.height,msg.width))
        def depth(self,msg):
            if msg.header.frame_id!=self.registration.optical_frame: return
            try:
                if msg.encoding not in ('32FC1','16UC1'): raise ValueError('unsupported depth encoding')
                dtype=np.dtype(('>' if msg.is_bigendian else '<')+('f4' if msg.encoding=='32FC1' else 'u2'))
                if msg.step<msg.width*dtype.itemsize or len(msg.data)!=msg.height*msg.step: raise ValueError('invalid depth stride')
                depth=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=bytes(msg.data),strides=(msg.step,dtype.itemsize)).astype(float)
                if msg.encoding=='16UC1': depth*=.001
                self.frames.add_depth(self.stamp(msg),depth)
            except (ValueError,TypeError): self.frames.clear()
        def info(self,msg):
            if msg.header.frame_id!=self.registration.optical_frame: return
            try: self.frames.add_intrinsics(self.stamp(msg),dict(fx=msg.k[0],fy=msg.k[4],cx=msg.k[2],cy=msg.k[5]))
            except (ValueError,TypeError): self.frames.clear()
        def identity(self,msg):
            try:
                p=json.loads(msg.data)
                if p.get('valid') is False: raise ValueError('camera frame invalidated')
                self.frames.add_identity(p)
            except (ValueError,KeyError,TypeError): self.frames.clear()
        def pin(self,msg):
            try: self.frames.pin(json.loads(msg.data),time.monotonic())
            except (ValueError,KeyError,TypeError) as exc: self.get_logger().error('Frame pin rejected: '+str(exc))
        def selected(self,p):
            _identity,selected=self.frames.selected_payload(p,time.monotonic())
            return selected
        def publish_surface(self,p,depth,intrinsics,binding):
            measurement=project_surface_measurement(p['polygon'],depth,intrinsics,binding,p['target_revision'])
            self.scene.publish(String(data=json.dumps(measurement)))
        def selection(self,msg):
            try:
                p=json.loads(msg.data); depth,intrinsics,binding=self.selected(p)
                self.publish_surface(p,depth,intrinsics,binding)
            except (ValueError,KeyError,TypeError) as exc: self.get_logger().error('Measured selection rejected: '+str(exc))
        def request(self,msg):
            p={}
            try:
                p=json.loads(msg.data); target=validate_target(p)
                depth,intrinsics,binding=self.selected(p)
                catalog=project_seam(p['polyline_px'],depth,intrinsics,binding,self.registration,target=target,reference_up=p.get('reference_up'),selection_lease=self.frames.lease)
                self.publish_surface(p,depth,intrinsics,binding)
                payload=asdict(catalog)
                payload.update(clock_domain='ros',stamp=p['stamp'],frame_binding=asdict(binding),camera_source='physical',orientation_source='user' if p.get('reference_up') is not None else 'measured_adjacent_plane')
            except (ValueError,KeyError,TypeError) as exc:
                payload=dict(error=str(exc),request_id=p.get('request_id',''),target_revision=p.get('target_revision',''),generation=p.get('generation',''))
            self.result.publish(String(data=json.dumps(payload)))
    rclpy.init(args=args); node=RGBDWeldProvider()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
