"""Measured physical-camera registration, independent of virtual robot TF."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class CameraRegistration:
    camera_id: str
    serial: str
    optical_frame: str
    target_frame: str
    translation_m: tuple
    quaternion_xyzw: tuple
    revision: str
    pose_source: str

    def __post_init__(self):
        for key in ('camera_id','serial','optical_frame','target_frame','revision'):
            if not isinstance(getattr(self,key),str) or not getattr(self,key).strip():
                raise ValueError(key+' required')
        if self.optical_frame in ('d405_color_optical_frame','zed_left_camera_frame_optical'):
            raise ValueError('physical registration must not overwrite a virtual robot camera link; use a distinct physical optical frame')
        if self.serial.startswith(('stream://','virtual:','sim:')):
            raise ValueError('physical camera serial required')
        if self.pose_source != 'fixed_measured':
            raise ValueError('pose_source must be fixed_measured; per-capture measured poses are not configured')
        xyz=np.asarray(self.translation_m,dtype=float); q=np.asarray(self.quaternion_xyzw,dtype=float)
        if xyz.shape!=(3,) or q.shape!=(4,) or not np.isfinite(xyz).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:
            raise ValueError('finite translation and normalized quaternion required')

    @classmethod
    def from_dict(cls, payload):
        return cls(**{k:payload[k] for k in cls.__dataclass_fields__})

    @property
    def rotation(self):
        x,y,z,w=self.quaternion_xyzw
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    def transform_points(self, points):
        return np.asarray(points,dtype=float) @ self.rotation.T + np.asarray(self.translation_m)


def load_registration(path, camera_id):
    payload=json.loads(Path(path).read_text())
    registration=CameraRegistration.from_dict(payload.get('cameras',{}).get(camera_id,payload))
    if registration.camera_id!=camera_id: raise ValueError('camera_id does not match registration')
    return registration
