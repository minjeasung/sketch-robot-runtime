"""Replaceable seam provider using the original ZED target envelope."""
from dataclasses import asdict, replace
from pathlib import Path
import json
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
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    import numpy as np
    from collections import deque
    class WeldInput(Node):
        def __init__(self):
            super().__init__('weld_input')
            self.fixture=self.declare_parameter('weld_fixture_file','').value
            self.simulation=self.declare_parameter('simulation',False).value
            self.provider=self.declare_parameter('weld_input_provider','fixture').value
            self.target=None; self.frames=deque(maxlen=100)
            self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub=self.create_publisher(String,'/welding/seams',qos)
            self.external=self.create_publisher(String,'/welding/extraction_request',10)
            self.create_subscription(String,'/perception/zed_frame_identity',self.on_frame,10)
            self.create_subscription(String,'/welding/target_request',self.on_target,10)
            self.create_subscription(String,'/welding/extraction_result',self.on_result,10)
        def on_frame(self,msg):
            try: self.frames.append(json.loads(msg.data))
            except (ValueError,TypeError): pass
        def on_target(self,msg):
            request_id=''
            try:
                payload=json.loads(msg.data); target=validate_target(payload); request_id=target.request_id
                if target.provider!=self.provider: raise ValueError('provider does not match configured runtime')
                matches=[f for f in self.frames if f.get('stamp')==payload['stamp']]
                if not matches: raise ValueError('ZED target frame expired')
                check_frame_identity(target,matches[-1]); self.target=target
                if target.provider=='fixture': self.pub.publish(String(data=json.dumps(asdict(load_fixture(self.fixture,target,'1',self.simulation)))))
                else: self.external.publish(msg)
            except (ValueError,KeyError,TypeError,OSError) as exc: self.fail(str(exc),request_id)
        def on_result(self,msg):
            if not self.target or self.provider!='external': return
            try:
                p=json.loads(msg.data); matrix=None
                if p.get('frame_id')!='World':
                    if p.get('clock_domain')!='ros': raise ValueError('external clock conversion required')
                    stamp=p['stamp']; transform=self.tf.lookup_transform('World',p['frame_id'],rclpy.time.Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec']))
                    tr=transform.transform; matrix=np.eye(4)
                    matrix[:3,:3]=Rotation.from_quat([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]).as_matrix()
                    matrix[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
                result=normalize_result(p,self.target,'World',matrix)
                self.pub.publish(String(data=json.dumps(asdict(result))))
            except Exception as exc: self.fail(str(exc))
        def fail(self,error,request_id=None): self.pub.publish(String(data=json.dumps({'error':error,'request_id':request_id if request_id is not None else (self.target.request_id if self.target else '')})))
    rclpy.init(args=args); node=WeldInput()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
