"""One finite measured surface and a two-engine application barrier."""
from dataclasses import dataclass, asdict
import hashlib
import json
import numpy as np
from .mixed_frame_sync import FrameBinding


@dataclass(frozen=True)
class TaskSurface:
    surface_id: str
    revision: str
    frame_id: str
    vertices_m: tuple
    faces: tuple
    normal: tuple
    frame_binding: FrameBinding

    @classmethod
    def from_dict(cls,p):
        return cls(p['surface_id'],p['revision'],p['frame_id'],tuple(map(tuple,p['vertices_m'])),tuple(map(tuple,p['faces'])),tuple(p['normal']),FrameBinding.from_dict(p['frame_binding']))


def make_surface(measurement,registration):
    binding=measurement['frame_binding']
    if isinstance(binding,dict): binding=FrameBinding.from_dict(binding)
    if binding.registration_revision!=registration.revision: raise ValueError('registration revision mismatch')
    vertices=np.asarray(measurement['vertices_m'],dtype=float)
    faces=np.asarray(measurement['faces'])
    normal=np.asarray(measurement['normal'],dtype=float)
    if vertices.ndim!=2 or vertices.shape[1:]!=(3,) or not 3<=len(vertices)<=100000 or not np.isfinite(vertices).all():
        raise ValueError('finite nonempty surface geometry required')
    if faces.ndim!=2 or faces.shape[1:]!=(3,) or not 1<=len(faces)<=200000 or faces.dtype.kind not in 'iu' or np.any(faces<0) or np.any(faces>=len(vertices)):
        raise ValueError('valid triangle indices required')
    if normal.shape!=(3,) or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>1e-6:
        raise ValueError('unit surface normal required')
    triangles=vertices[faces]
    cross=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    if np.any(cross @ normal<=1e-10): raise ValueError('degenerate or flipped surface')
    sid=measurement['surface_id']
    if not isinstance(sid,str) or not sid or len(sid)>128: raise ValueError('surface_id required')
    transformed=registration.transform_points(vertices)
    rotated=registration.rotation @ normal
    content=dict(surface_id=sid,frame_id=registration.target_frame,vertices_m=transformed.tolist(),faces=faces.tolist(),normal=rotated.tolist(),frame_binding=asdict(binding))
    revision=hashlib.sha256(json.dumps(content,sort_keys=True,allow_nan=False).encode()).hexdigest()[:24]
    return TaskSurface(sid,revision,registration.target_frame,tuple(map(tuple,transformed)),tuple(map(tuple,faces.tolist())),tuple(rotated),binding)


def make_refined_surface(surface_id, corners, center, normal, frame_id, binding, registration):
    """Project an observed finite work area onto its accepted measured plane.

    The caller supplies actual work-area corners and the D405 exposure binding;
    no nominal area, thickness, or TCP offset is synthesized.
    """
    if frame_id != registration.target_frame:
        raise ValueError('refined plane must use registered target frame')
    corners=np.asarray(corners,dtype=float)
    center=np.asarray(center,dtype=float); normal=np.asarray(normal,dtype=float)
    if corners.shape != (4,3) or not np.isfinite(corners).all():
        raise ValueError('four measured work-area corners required')
    if center.shape != (3,) or normal.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>1e-6:
        raise ValueError('finite measured plane and unit normal required')
    projected=corners-((corners-center)@normal)[:,None]*normal
    faces=np.array([[0,1,2],[0,2,3]])
    if np.dot(np.cross(projected[1]-projected[0],projected[2]-projected[0]),normal)<0:
        faces=faces[:,::-1]
    vertices=(projected-np.asarray(registration.translation_m))@registration.rotation
    return make_surface(dict(surface_id=surface_id,vertices_m=vertices.tolist(),faces=faces.tolist(),normal=(registration.rotation.T@normal).tolist(),frame_binding=binding),registration)


def validate_scene_request(payload, default_camera, registrations, readiness, lease, coordinator, now):
    camera=payload.get('camera_name',default_camera)
    if camera not in registrations: raise ValueError('unregistered task-scene camera')
    registration=registrations[camera]
    surface=make_surface(payload.get('measurement',payload),registration)
    binding=surface.frame_binding
    if camera==default_camera:
        lease.require(binding,now)
    else:
        binding.require_current(now,registration.revision)
        if not coordinator.ready or payload.get('parent_revision')!=coordinator.surface.revision:
            raise ValueError('refinement parent scene changed or is not acknowledged')
        if surface.surface_id!=coordinator.surface.surface_id:
            raise ValueError('refinement must retain the selected measured surface')
    if not readiness.ready(now): raise ValueError('required physical camera input unavailable')
    current=readiness.frames.get(camera)
    if current is None or (binding.source_session,binding.clock_epoch,binding.registration_revision)!=(current.source_session,current.clock_epoch,current.registration_revision):
        raise ValueError('measured surface does not match current camera context')
    return surface


class SceneCoordinator:
    def __init__(self):
        self.surface=None
        self.acks={}
        self.error='waiting for measured surface'

    def request(self,surface,task_active=False):
        if task_active: raise ValueError('cannot change scene during active task')
        self.surface=surface
        self.acks={}
        self.error='waiting for Isaac and MoveIt acknowledgement'

    def ack(self,engine,surface_id,revision,success,error=''):
        if engine not in ('isaac','moveit'): raise ValueError('unknown scene engine')
        if not self.surface or (surface_id,revision)!=(self.surface.surface_id,self.surface.revision): return False
        self.acks[engine]=success is True
        if not success: self.error=error or engine+' scene application failed'
        if self.ready: self.error=''
        return True

    @property
    def ready(self):
        return self.surface is not None and all(self.acks.get(engine) is True for engine in ('isaac','moveit'))

    def invalidate(self,reason):
        self.surface=None
        self.acks={}
        self.error=reason

    def status(self):
        return dict(ready=self.ready,surface_id=self.surface.surface_id if self.surface else '',revision=self.surface.revision if self.surface else '',acks=dict(self.acks),error=self.error)
