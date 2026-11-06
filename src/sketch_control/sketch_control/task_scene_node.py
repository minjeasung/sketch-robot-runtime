"""Apply the same validated measured surface to MoveIt and Isaac, with ACKs."""
import json
import time
from dataclasses import asdict
from .camera_registration import load_registration
from .mixed_frame_sync import FrameBinding, SelectionLease, PhysicalCameraReadiness
from .task_scene import validate_scene_request, SceneCoordinator


def collision_object(surface):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle
    from geometry_msgs.msg import Point, Pose
    mesh=Mesh(vertices=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in surface.vertices_m],
              triangles=[MeshTriangle(vertex_indices=list(f)) for f in surface.faces])
    pose=Pose(); pose.orientation.w=1.
    obj=CollisionObject(id='measured_task_surface',operation=CollisionObject.ADD)
    obj.header.frame_id=surface.frame_id
    obj.meshes=[mesh]; obj.mesh_poses=[pose]
    return obj


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from moveit_msgs.srv import ApplyPlanningScene
    class TaskSceneNode(Node):
        def __init__(self):
            super().__init__('task_scene')
            path=self.declare_parameter('camera_registration_file','').value
            camera=self.declare_parameter('camera_name','zed').value
            self.registration=load_registration(path,camera)
            required=self.declare_parameter('required_camera_names',[camera]).value
            d405_path=self.declare_parameter('d405_registration_file',path).value
            registrations={name:load_registration(d405_path if name=='d405' else path,name) for name in required}
            self.camera_readiness=PhysicalCameraReadiness(registrations)
            self.lease=SelectionLease(self.registration.revision)
            self.coordinator=SceneCoordinator(); self.active=False; self.context=None; self.last_identity=0.
            self.timeout=float(self.declare_parameter('scene_ack_timeout_s',5.).value); self.deadline=0.
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub=self.create_publisher(String,'/task_scene/status',qos)
            self.apply_pub=self.create_publisher(String,'/task_scene/apply',qos)
            self.client=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
            self.create_subscription(String,'/task_scene/request',self.request,10)
            self.create_subscription(String,'/task_scene/ack',self.ack,10)
            self.create_subscription(String,'/task_scene/invalidate',self.invalidate,10)
            self.create_subscription(String,'/task_scene/activity',self.activity,10)
            self.create_subscription(String,'/perception/selection_pin',self.pin,10)
            for camera_name in required:
                self.create_subscription(String,'/perception/'+camera_name+'_frame_identity',lambda msg,name=camera_name:self.identity(msg,name),10)
            self.create_timer(.2,self.publish_status,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def pin(self,msg):
            try:
                binding=FrameBinding.from_dict(json.loads(msg.data)['frame_binding'])
                if self.context!=(binding.source_session,binding.clock_epoch,binding.registration_revision): raise ValueError('pin context changed')
                self.lease.pin(binding,time.monotonic())
            except (ValueError,TypeError,KeyError): self.lease.clear()
        def activity(self,msg):
            try: self.active=json.loads(msg.data).get('active') is True
            except (ValueError,TypeError): self.active=True
        def identity(self,msg,camera_name):
            try:
                p=json.loads(msg.data)
                changed=self.camera_readiness.observe(camera_name,p,time.monotonic())
                if changed: self.coordinator.invalidate('physical camera context changed'); self.lease.clear()
                if camera_name!=self.registration.camera_id: return
                b=p['frame_binding']; context=(b['source_session'],b['clock_epoch'],b['registration_revision'])
                if self.context is not None and context!=self.context:
                    self.coordinator.invalidate('physical camera context changed'); self.lease.clear()
                self.context=context; self.last_identity=time.monotonic()
            except (ValueError,KeyError,TypeError) as exc:
                self.coordinator.invalidate(str(exc)); self.lease.clear()
        def invalidate(self,msg):
            self.coordinator.invalidate(msg.data or 'runtime invalidated')
            self.publish_status()
        def request(self,msg):
            try:
                p=json.loads(msg.data)
                surface=validate_scene_request(p,self.registration.camera_id,self.camera_readiness.registrations,self.camera_readiness,self.lease,self.coordinator,time.monotonic())
                self.coordinator.request(surface,self.active); self.deadline=time.monotonic()+self.timeout
                self.apply_pub.publish(String(data=json.dumps(asdict(surface))))
                if not self.client.service_is_ready(): raise ValueError('MoveIt planning scene service unavailable')
                request=ApplyPlanningScene.Request(); request.scene.is_diff=True
                request.scene.world.collision_objects=[collision_object(surface)]
                future=self.client.call_async(request)
                def completed(result):
                    try: success=result.result().success; error=''
                    except Exception as exc: success=False; error=str(exc)
                    self.coordinator.ack('moveit',surface.surface_id,surface.revision,success,error)
                    self.publish_status()
                future.add_done_callback(completed)
            except (ValueError,KeyError,TypeError) as exc:
                self.coordinator.error=str(exc)
                if self.coordinator.surface: self.coordinator.ack('moveit',self.coordinator.surface.surface_id,self.coordinator.surface.revision,False,str(exc))
            self.publish_status()
        def ack(self,msg):
            try:
                p=json.loads(msg.data)
                # The MoveIt ACK belongs only to the actual service response above.
                if p.get('engine')!='isaac': return
                self.coordinator.ack(p['engine'],p['surface_id'],p['revision'],p.get('success'),p.get('error',''))
            except (ValueError,KeyError,TypeError) as exc: self.coordinator.error=str(exc)
            self.publish_status()
        def publish_status(self):
            if self.coordinator.surface and not self.camera_readiness.ready(time.monotonic()):
                self.coordinator.invalidate('live camera frame expired'); self.lease.clear()
            if self.coordinator.surface and not self.coordinator.ready and time.monotonic()>self.deadline:
                self.coordinator.invalidate('scene engine acknowledgement timed out')
            self.status_pub.publish(String(data=json.dumps(self.coordinator.status())))
    rclpy.init(args=args); node=TaskSceneNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
