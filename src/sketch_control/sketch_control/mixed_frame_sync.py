"""Fail-closed binding of one physical RGB-D exposure to simulation time."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FrameBinding:
    source_session: str
    source_frame_id: int
    capture_stamp: int
    receipt_monotonic: float
    sim_stamp: int
    clock_epoch: int
    registration_revision: str

    def __post_init__(self):
        if not self.source_session or not self.registration_revision:
            raise ValueError('source session and registration revision required')
        for key in ('source_frame_id','capture_stamp','sim_stamp','clock_epoch'):
            value=getattr(self,key)
            if isinstance(value,bool) or not isinstance(value,int) or value<0: raise ValueError(key+' must be nonnegative integer')
        if not math.isfinite(self.receipt_monotonic): raise ValueError('receipt clock must be finite')

    @classmethod
    def from_dict(cls,payload):
        return cls(**{k:payload[k] for k in cls.__dataclass_fields__})

    def require_current(self, steady_now, registration_revision, *, epoch=None, max_age_s=1.):
        age=steady_now-self.receipt_monotonic
        if not math.isfinite(age) or age<0 or age>max_age_s: raise ValueError('selected frame expired')
        if self.registration_revision!=registration_revision: raise ValueError('registration revision changed')
        if epoch is not None and epoch!=self.clock_epoch: raise ValueError('clock epoch changed')


class MixedFrameSync:
    def __init__(self,registration,max_age_s=1.):
        self.registration=registration
        self.max_age_s=max_age_s
        self.session=None
        self.last_id=-1
        self.last_capture=-1
        self.last_ros=0
        self.epoch=None

    def bind(self,frame,ros_now_ns,steady_now,epoch):
        r=self.registration
        for key,want in (('camera_id',r.camera_id),('serial',r.serial),('registration_revision',r.revision)):
            if frame.get(key)!=want: raise ValueError('camera binding mismatch: '+key)
        binding=FrameBinding(frame['source_session'],frame['source_frame_id'],frame['capture_stamp'],frame['receipt_monotonic'],ros_now_ns,epoch,r.revision)
        binding.require_current(steady_now,r.revision,max_age_s=self.max_age_s)
        if self.session is not None and binding.source_session!=self.session:
            raise ValueError('source session changed; restart perception')
        if binding.source_frame_id<=self.last_id or binding.capture_stamp<self.last_capture:
            raise ValueError('duplicate or out-of-order source frame')
        self.session=binding.source_session
        self.last_id=binding.source_frame_id
        self.last_capture=binding.capture_stamp
        previous_epoch=self.epoch
        self.epoch=epoch
        if previous_epoch is not None and previous_epoch!=epoch:
            self.last_ros=ros_now_ns
            raise ValueError('clock epoch changed; discard frame and previous plans')
        if ros_now_ns<=self.last_ros:
            raise ValueError('simulation clock paused or reset')
        self.last_ros=ros_now_ns
        return binding


class SelectionLease:
    """Bounded human-edit interval acquired only from a fresh physical frame."""
    def __init__(self,registration_revision,duration_s=30.):
        self.revision=registration_revision; self.duration=duration_s
        self.binding=None; self.deadline=0.

    def pin(self,binding,steady_now):
        binding.require_current(steady_now,self.revision)
        if self.binding!=binding:
            self.binding=binding; self.deadline=steady_now+self.duration

    def require(self,binding,steady_now):
        if self.binding==binding:
            if steady_now>self.deadline or steady_now<binding.receipt_monotonic: raise ValueError('selected frame expired')
            return
        binding.require_current(steady_now,self.revision)

    def clear(self):
        self.binding=None; self.deadline=0.


class PhysicalCameraReadiness:
    """All process-required physical streams must have fresh calibrated input."""
    def __init__(self,registrations):
        self.registrations=registrations; self.frames={}

    def observe(self,camera,payload,steady_now):
        previous=self.frames.pop(camera,None)
        registration=self.registrations[camera]
        if payload.get('valid') is not True: raise ValueError('physical camera frame invalid')
        for key,value in (('camera_id',camera),('serial',registration.serial),('frame_id',registration.optical_frame)):
            if payload.get(key)!=value: raise ValueError('physical camera identity mismatch: '+key)
        binding=FrameBinding.from_dict(payload['frame_binding'])
        binding.require_current(steady_now,registration.revision)
        self.frames[camera]=binding
        return previous is not None and (previous.source_session,previous.clock_epoch,previous.registration_revision)!=(binding.source_session,binding.clock_epoch,binding.registration_revision)

    def ready(self,steady_now):
        try:
            for camera,registration in self.registrations.items():
                self.frames[camera].require_current(steady_now,registration.revision)
            return True
        except (KeyError,ValueError): return False
