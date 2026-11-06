"""Replaceable seam provider using the original ZED target envelope."""
from dataclasses import asdict, replace
from pathlib import Path
import json
import time
from .mixed_frame_sync import FrameBinding, SelectionLease
from .weld_contract import validate_catalog, validate_target
from .weld_geometry import transform_catalog


def check_frame_identity(target,metadata):
    for key in ('generation','width','height','frame_id','clock_domain'):
        if getattr(target,key)!=metadata.get(key): raise ValueError('ZED frame identity mismatch: '+key)
    stamp=metadata.get('stamp',{})
    if target.stamp!=(stamp.get('sec'),stamp.get('nanosec')): raise ValueError('ZED frame stamp mismatch')


def load_fixture(path,target,scene_revision,simulation=True):
    if not simulation or target.provider!='fixture': raise ValueError('fixture is only available in explicit simulation mode')
    payload=json.loads(Path(path).read_text())
    if payload.get('scene_revision')!=scene_revision: raise ValueError('fixture scene mismatch')
    payload.update(request_id=target.request_id,target_revision=target.target_revision,generation=target.generation,source='fixture')
    return validate_catalog(payload,target)


def normalize_result(payload,target,frame_id,transform):
    catalog=validate_catalog(payload,target)
    if catalog.frame_id==frame_id: return catalog
    if transform is None or payload.get('clock_domain')!='ros' or not isinstance(payload.get('stamp'),dict): raise ValueError('capture-time ROS transform required')
    if target.clock_domain!='ros' or target.stamp!=(payload['stamp'].get('sec'),payload['stamp'].get('nanosec')): raise ValueError('result capture stamp differs from target')
    return replace(transform_catalog(catalog,transform),frame_id=frame_id)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import CameraInfo
    from .weld_overlay import project_catalog
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    import numpy as np
    from collections import deque
    class WeldInput(Node):
        def __init__(self):
            super().__init__('weld_input')
            self.fixture=self.declare_parameter('weld_fixture_file','').value
            self.scene_revision=self.declare_parameter('weld_scene_revision','1').value
            self.simulation=self.declare_parameter('simulation',False).value
            self.provider=self.declare_parameter('weld_input_provider','fixture').value
            self.base_frame=self.declare_parameter('base_frame','World').value
            self.execution_mode=self.declare_parameter('execution_mode','simulation' if self.simulation else 'real').value
            if self.execution_mode=='mixed' and self.provider!='external': raise ValueError('mixed requires external physical RGB-D provider')
            self.target=None; self.frames=deque(maxlen=100); self.frame_context=None; self.lease=None; self.pinned_frame=None; self.last_physical_frame=0.
            self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
            self.overlay_catalog=None; self.overlay_infos={}; self.overlay_sent=deque(maxlen=100)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub=self.create_publisher(String,'/welding/seams',qos)
            self.overlay_pub=self.create_publisher(String,'/welding/seam_overlay',10)
            info_topic=self.declare_parameter('zed_camera_info_topic','/zed/zed_node/rgb/color/rect/camera_info').value
            self.create_subscription(CameraInfo,info_topic,self.on_overlay_info,10)
            self.create_timer(.05,self.publish_overlay)
            self.external=self.create_publisher(String,'/welding/extraction_request',10)
            self.create_subscription(String,'/perception/zed_frame_identity',self.on_frame,10)
            self.create_subscription(String,'/welding/target_request',self.on_target,10)
            self.create_subscription(String,'/perception/selection_pin',self.on_pin,10)
            self.create_subscription(String,'/welding/extraction_result',self.on_result,10)
        def on_overlay_info(self,msg):
            key=(msg.header.frame_id,msg.header.stamp.sec,msg.header.stamp.nanosec)
            self.overlay_infos[key]=dict(frame_id=msg.header.frame_id,
                stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                width=msg.width,height=msg.height,p=list(msg.p))
            while len(self.overlay_infos)>100:self.overlay_infos.pop(next(iter(self.overlay_infos)))
        def publish_catalog(self,catalog):
            self.overlay_catalog=catalog; self.overlay_sent.clear()
            self.pub.publish(String(data=json.dumps(asdict(catalog))))
        def publish_overlay(self):
            catalog=self.overlay_catalog
            if catalog is None:return
            candidates=list(self.frames)[-6:]
            if self.pinned_frame:candidates.append(self.pinned_frame)
            for frame in candidates:
                stamp=frame.get('stamp',{});key=(frame.get('frame_id'),stamp.get('sec'),stamp.get('nanosec'))
                if key in self.overlay_sent or key not in self.overlay_infos:continue
                try:
                    tr=self.tf.lookup_transform(frame['frame_id'],catalog.frame_id,
                        rclpy.time.Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec'])).transform
                    matrix=np.eye(4)
                    matrix[:3,:3]=Rotation.from_quat([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]).as_matrix()
                    matrix[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
                    payload=project_catalog(catalog,frame,self.overlay_infos[key],matrix)
                except Exception:
                    # Missing capture-time TF/intrinsics must not move the line
                    # using a different frame or affect the control workflow.
                    continue
                self.overlay_pub.publish(String(data=json.dumps(payload)));self.overlay_sent.append(key)
        def on_pin(self,msg):
            try:
                p=json.loads(msg.data); b=FrameBinding.from_dict(p['frame_binding'])
                if not self.lease or p not in self.frames: raise ValueError('pin frame unavailable')
                self.lease.pin(b,time.monotonic()); self.pinned_frame=p
            except (ValueError,TypeError,KeyError):
                self.pinned_frame=None
                if self.lease: self.lease.clear()
        def on_frame(self,msg):
            try:
                p=json.loads(msg.data)
                if self.execution_mode=='mixed':
                    if p.get('valid') is False: raise ValueError(p.get('error','physical frame invalidated'))
                    b=FrameBinding.from_dict(p['frame_binding'])
                    context=(b.source_session,b.clock_epoch,b.registration_revision)
                    if self.frame_context is not None and context!=self.frame_context:
                        self.frames.clear(); self.target=None; self.fail('physical frame context changed')
                    if self.frame_context!=context:
                        self.lease=SelectionLease(b.registration_revision); self.pinned_frame=None
                    self.frame_context=context; self.last_physical_frame=time.monotonic()
                self.frames.append(p)
            except (ValueError,TypeError,KeyError):
                self.frames.clear(); self.target=None; self.pinned_frame=None
                if self.lease: self.lease.clear()
                self.fail('physical frame identity invalidated')
        def on_target(self,msg):
            request_id=''
            self.target=None; self.overlay_catalog=None
            try:
                payload=json.loads(msg.data); target=validate_target(payload); request_id=target.request_id
                if target.provider!=self.provider: raise ValueError('provider does not match configured runtime')
                matches=[f for f in list(self.frames)+([self.pinned_frame] if self.pinned_frame else []) if f.get('stamp')==payload['stamp']]
                if not matches: raise ValueError('ZED target frame expired')
                check_frame_identity(target,matches[-1])
                if self.execution_mode=='mixed':
                    if time.monotonic()-self.last_physical_frame>1.: raise ValueError('live camera frame expired')
                    b=FrameBinding.from_dict(matches[-1]['frame_binding'])
                    self.lease.require(b,time.monotonic())
                    if (b.source_session,b.clock_epoch,b.registration_revision)!=self.frame_context: raise ValueError('physical frame context changed')
                    if payload.get('frame_binding')!=matches[-1]['frame_binding']: raise ValueError('selected frame binding mismatch')
                    if not payload.get('polyline_px'): raise ValueError('user selected seam polyline required')
                self.target=target
                if target.provider=='fixture': self.publish_catalog(load_fixture(self.fixture,target,self.scene_revision,self.simulation))
                else: self.external.publish(msg)
            except (ValueError,KeyError,TypeError,OSError) as exc: self.fail(str(exc),request_id)
        def on_result(self,msg):
            if not self.target or self.provider!='external': return
            try:
                p=json.loads(msg.data); matrix=None
                if p.get('request_id')!=self.target.request_id or p.get('target_revision')!=self.target.target_revision or p.get('generation')!=self.target.generation: return
                if p.get('error'): raise ValueError(p['error'])
                if self.execution_mode=='mixed':
                    if time.monotonic()-self.last_physical_frame>1.: raise ValueError('live camera frame expired')
                    b=FrameBinding.from_dict(p['frame_binding'])
                    self.lease.require(b,time.monotonic())
                    if (b.source_session,b.clock_epoch,b.registration_revision)!=self.frame_context: raise ValueError('physical frame context changed')
                    if b.source_session!=self.target.generation: raise ValueError('physical source session changed')
                if p.get('frame_id')!=self.base_frame:
                    if p.get('clock_domain')!='ros': raise ValueError('external clock conversion required')
                    stamp=p['stamp']; transform=self.tf.lookup_transform(self.base_frame,p['frame_id'],rclpy.time.Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec']))
                    tr=transform.transform; matrix=np.eye(4)
                    matrix[:3,:3]=Rotation.from_quat([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]).as_matrix()
                    matrix[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
                result=normalize_result(p,self.target,self.base_frame,matrix)
                self.publish_catalog(result)
            except Exception as exc: self.fail(str(exc))
        def fail(self,error,request_id=None):
            self.overlay_catalog=None
            self.pub.publish(String(data=json.dumps({'error':error,'request_id':request_id if request_id is not None else (self.target.request_id if self.target else '')})))
    rclpy.init(args=args); node=WeldInput()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
