"""Sensor math and simulator command lease; imports no Isaac/ROS at module load."""
from dataclasses import dataclass, field
from contextlib import contextmanager
import os
import json
from collections import defaultdict, deque
import time
import math
import numpy as np

class LoopTrace:
    """Opt-in attribution of source-side stalls; never alters sensor stamps or leases."""
    def __init__(self, enabled=False):
        self.enabled=enabled; self.samples=defaultdict(lambda:deque(maxlen=4096))
        self.slow_events=deque(maxlen=256); self.state_gaps=deque(maxlen=256)
        self.critical_events=deque(maxlen=128); self.cumulative_max=defaultdict(float)
        self.last_state=None; self.last_step=None; self.max_gap=0.

    def record(self, phase, duration, now, step=0, cpu_duration=None):
        if not self.enabled:
            return
        self.samples[phase].append(duration)
        self.cumulative_max[phase]=max(self.cumulative_max[phase],duration)
        if duration>=.02:
            event={'phase':phase,'duration_s':duration,'monotonic':now,'step_index':step}
            if cpu_duration is not None:
                event['cpu_duration_s']=cpu_duration
            self.slow_events.append(event)
            if duration>=.20:
                self.critical_events.append(event)

    def call(self, phase, function, *args, step=0, **kwargs):
        if not self.enabled:
            return function(*args,**kwargs)
        begin=time.monotonic(); cpu_begin=time.thread_time()
        try:
            return function(*args,**kwargs)
        finally:
            self.record(phase,time.monotonic()-begin,begin,step,time.thread_time()-cpu_begin)

    def state_published(self, now, step):
        if self.last_state is not None:
            gap=now-self.last_state; self.max_gap=max(self.max_gap,gap)
            if self.enabled and gap>=.05:
                self.state_gaps.append({'duration_s':gap,'monotonic':now,'previous_step_index':self.last_step,'step_index':step})
        self.last_state=now; self.last_step=step

    def report(self):
        return {'state_max_gap_s':self.max_gap,'phases':{name:{'samples':len(samples),
                'cumulative_max_ms':1000*self.cumulative_max[name],'max_ms':1000*max(samples),'mean_ms':1000*sum(samples)/len(samples),
                'p99_ms':float(np.percentile(samples,99))*1000} for name,samples in self.samples.items()},
                'slow_events':list(self.slow_events),'state_gaps':list(self.state_gaps),'critical_events':list(self.critical_events)}


@contextmanager
def publication_mode(mode):
    # Jazzy rmw_fastrtps stores this mode on participant creation, not each write.
    key='RMW_FASTRTPS_PUBLICATION_MODE'; previous=os.environ.get(key)
    os.environ[key]=mode
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop(key,None)
        else:
            os.environ[key]=previous


def transport_qos_profiles(qos_module):
    """Measured streaming drops superseded samples; commands retain reliable delivery."""
    stream=qos_module.QoSProfile(depth=1,reliability=qos_module.ReliabilityPolicy.BEST_EFFORT,durability=qos_module.DurabilityPolicy.VOLATILE)
    command=qos_module.QoSProfile(depth=2,reliability=qos_module.ReliabilityPolicy.RELIABLE,durability=qos_module.DurabilityPolicy.VOLATILE)
    return stream,command


@dataclass
class CommandLease:
    session_id: str
    scene_revision: str
    timeout_s: float = .20
    sequence: int = -1
    received_monotonic: float = -math.inf
    target: object = None
    source_deadline: float = -math.inf
    source_states: dict = field(default_factory=dict)
    last_source_stamp: tuple = (-1,-1)
    last_source_monotonic: float = -math.inf

    def record_state(self, sec, nanosec, sampled_monotonic):
        """Bind an advancing published physics sample to its original steady time."""
        key=(sec,nanosec)
        if (not isinstance(sec,int) or not isinstance(nanosec,int) or sec<0 or
            not 0<=nanosec<1000000000 or key<=self.last_source_stamp or
            not math.isfinite(sampled_monotonic) or sampled_monotonic<self.last_source_monotonic):
            return
        self.last_source_stamp=key; self.last_source_monotonic=sampled_monotonic
        self.source_states[key]=sampled_monotonic
        self.source_states={k:v for k,v in self.source_states.items()
                            if sampled_monotonic-v<=self.timeout_s}
        while len(self.source_states)>128:
            del self.source_states[next(iter(self.source_states))]

    def accept(self, command, now):
        if (command.session_id != self.session_id or command.scene_revision != self.scene_revision or
            command.sequence <= self.sequence or not math.isfinite(now)):
            return False
        if not command.enabled:
            self.sequence=command.sequence; self.invalidate(); return True
        if len(command.positions)!=6 or not all(math.isfinite(v) for v in command.positions):
            return False
        stamp=getattr(getattr(command,'header',None),'stamp',None)
        sampled=self.source_states.get((getattr(stamp,'sec',None),getattr(stamp,'nanosec',None)))
        if sampled is None or not 0<=now-sampled<=self.timeout_s:
            return False
        self.sequence=command.sequence
        self.received_monotonic=now
        self.source_deadline=sampled+self.timeout_s
        self.target=np.array(command.positions,dtype=float)
        return True

    def active(self, now):
        return (self.target is not None and 0 <= now-self.received_monotonic <= self.timeout_s
                and now<=self.source_deadline)

    def active_at_current_time(self, clock=time.monotonic):
        return self.active(clock())

    def invalidate(self):
        self.target=None; self.received_monotonic=-math.inf; self.source_deadline=-math.inf
        self.source_states.clear()


def transform_wrench(wrench, target_from_source):
    tf=np.asarray(target_from_source); wrench=np.asarray(wrench)
    force=tf[:3,:3]@wrench[:3]
    return np.r_[force,tf[:3,:3]@wrench[3:]+np.cross(tf[:3,3],force)]


def gravity_wrench(mass_kg, cog_ft_m, world_from_ft_rotation):
    force=np.asarray(world_from_ft_rotation).T@np.array([0.,0.,-9.81*mass_kg])
    return np.r_[force,np.cross(cog_ft_m,force)]

@dataclass(frozen=True)
class AveragedWrench:
    raw: np.ndarray
    gravity: np.ndarray
    step_index: int
    stamp: float
    sample_count: int


class WrenchWindowAverage:
    """Five consecutive 500 Hz physical samples, expressed in attached ft_link axes.

    Raw reaction and gravity remain separate. The result owns the original
    last-step stamp; no clock read, zero fallback, clipping or target input exists.
    """
    def __init__(self, size=5):
        self.size=size
        self.samples=deque(maxlen=size)
        self.identity=None; self.last_step=None; self.last_stamp=None

    def flush(self):
        self.samples.clear()
        self.identity=None; self.last_step=None; self.last_stamp=None

    def add(self, raw, gravity, *, session_id, scene_revision, step_index, stamp,
            valid=True, playing=True):
        raw=np.asarray(raw,dtype=float); gravity=np.asarray(gravity,dtype=float)
        if (not valid or not playing or not session_id or not scene_revision or
            raw.shape!=(6,) or gravity.shape!=(6,) or not np.all(np.isfinite(raw)) or
            not np.all(np.isfinite(gravity)) or not math.isfinite(stamp) or stamp<0):
            self.flush(); return None
        identity=(session_id,scene_revision)
        if identity!=self.identity:
            self.flush()
        elif self.last_step is not None:
            if step_index<=self.last_step or stamp<=self.last_stamp:
                self.flush(); return None
            if step_index!=self.last_step+1:
                self.flush()
        self.identity=identity; self.last_step=step_index; self.last_stamp=stamp
        self.samples.append((raw.copy(),gravity.copy()))
        if len(self.samples)<self.size:
            return None
        return AveragedWrench(np.mean([v[0] for v in self.samples],axis=0),
                              np.mean([v[1] for v in self.samples],axis=0),
                              step_index,stamp,len(self.samples))


def invalid_measurements():
    # Missing physics must stay explicitly invalid; no zero-filled sensor fallback.
    return tuple(np.full(6,np.nan) for _ in range(5))


def camera_frame_index(value):
    if isinstance(value, dict):
        numerator=value.get('referenceTimeNumerator')
        denominator=value.get('referenceTimeDenominator')
        if numerator is None or denominator is None or denominator <= 0:
            return None
        return numerator/denominator
    return float(value) if isinstance(value,(int,float)) else None


CAMERA_TOPICS = {
    'zed':('/zed/zed_node/rgb/color/rect/image','/zed/zed_node/depth/depth_registered','/zed/zed_node/rgb/color/rect/camera_info','/zed/zed_node/depth/camera_info'),
    'd405':('/d405/d405/color/image_raw','/d405/d405/depth/image_rect_raw','/d405/d405/color/camera_info','/d405/d405/depth/camera_info'),
}


def render_sensor_and_viewport(world, cameras, overlay, *, playing, phase_timing=None):
    """Share one illustrated render between viewport and RGB/depth cameras.

    Spray illustrations are intentionally visible in simulation camera images.
    Keep render products running so deferred readbacks complete on subsequent
    scheduled frames, without blocking physics on extra capture passes.
    """
    world.render()
    if phase_timing is not None:
        phase_timing.mark('render', time.monotonic())
    if cameras is not None and playing:
        cameras.publish(time.monotonic())
    if phase_timing is not None:
        phase_timing.mark('camera_readback_publish', time.monotonic())


class RenderedCameras:
    """Real Isaac RGB and registered metric image-plane depth, published once per frame."""
    def __init__(self, node, manifest, scene):
        from isaacsim.sensors.camera import Camera
        from sensor_msgs.msg import Image, CameraInfo
        self.node=node; self.Image=Image; self.CameraInfo=CameraInfo
        self.cameras={}; self.frames={}; self.last_monotonic={}; self.manifest=manifest
        self.previews={}; self.identity_pubs={}; self.identities={}
        self._last_publish_diagnostic={}
        try:
            from omni.kit.viewport.utility import get_active_viewport
        except ImportError:
            self.viewport=None
        else:
            self.viewport=get_active_viewport()
        self.session_id=manifest.get('session_id')
        if self.session_id:
            from std_msgs.msg import String
            from sketch_control.image_preview import ImagePreview
            self.String=String
        names=list(scene.get('camera_names',('zed','d405')))
        if len(set(names))!=len(names) or any(name not in CAMERA_TOPICS for name in names):
            raise ValueError('Unknown or duplicate rendered camera name')
        frames={'zed':'zed_left_camera_frame_optical','d405':'d405_color_optical_frame',
                **scene.get('camera_frames',{})}
        for name in names:
            frame=frames[name]
            camera=Camera('/World/Cameras/'+name,name=name,frequency=-1,resolution=tuple(scene['camera_resolution']))
            camera.set_focal_length(2.5); camera.set_horizontal_aperture(3.6)
            camera.set_clipping_range(.01,5.)
            camera.initialize(); camera.add_distance_to_image_plane_to_frame()
            rgb_topic,depth_topic,info_rgb,info_depth=CAMERA_TOPICS[name]
            pubs=[node.create_publisher(t,topic,2) for t,topic in [(Image,rgb_topic),(Image,depth_topic),(CameraInfo,info_rgb),(CameraInfo,info_depth)]]
            self.cameras[name]=(camera,frame,pubs); self.frames[name]=-1; self.last_monotonic[name]=-math.inf
            if self.session_id:
                self.previews[name]=ImagePreview(node,rgb_topic)
                self.identity_pubs[name]=node.create_publisher(String,f'/perception/{name}_frame_identity',10)
        self.zed_pose=np.array(scene['zed_world_optical'])

    def update_poses(self, world_from_ft, world_from_zed=None):
        from scipy.spatial.transform import Rotation
        for name,(camera,frame,_) in self.cameras.items():
            tf=(self.zed_pose if world_from_zed is None else world_from_zed) if name=='zed' else world_from_ft@np.array(self.manifest['camera_ft_transforms'][frame])
            q=Rotation.from_matrix(tf[:3,:3]).as_quat()
            camera.set_world_pose(position=tf[:3,3],orientation=np.r_[q[3],q[:3]],camera_axes='ros')

    def set_render_updates(self, enabled):
        """Gate the owned Isaac 6.1 camera render products, not just callbacks.

        Camera does not expose this gate itself. Its initialize() creates and
        retains the Replicator HydraTexture wrapper in _render_product. Validate
        every handle before changing any camera so unsupported versions fail
        closed rather than silently producing illustrated perception images.
        """
        textures=[]
        for camera,_,_ in self.cameras.values():
            texture=getattr(getattr(camera,'_render_product',None),'hydra_texture',None)
            if not callable(getattr(texture,'set_updates_enabled',None)):
                raise RuntimeError('Camera render product cannot isolate simulation overlays')
            textures.append(texture)
        for texture in textures:
            texture.set_updates_enabled(bool(enabled))

    def capture_ready(self):
        """Whether every product has a genuinely new complete sensor frame."""
        for name,(camera,_,_) in self.cameras.items():
            data=camera.get_current_frame()
            index=camera_frame_index(data.get('rendering_frame'))
            stamp=float(data.get('rendering_time',0.))
            rgb=data.get('rgb') if 'rgb' in data else camera.get_rgba()
            depth=data.get('distance_to_image_plane')
            if (index is None or index<=self.frames[name] or not math.isfinite(stamp) or stamp<=0
                    or rgb is None or depth is None or np.size(rgb)==0 or np.size(depth)==0):
                return False
        return True

    def _publish_diagnostic(self, name, camera, data, reason, now):
        if now-self._last_publish_diagnostic.get(name,-math.inf)<5.:
            return
        self._last_publish_diagnostic[name]=now
        texture=getattr(getattr(camera,'_render_product',None),'hydra_texture',None)
        payload=dict(camera=name,reason=reason,
            rendering_frame=data.get('rendering_frame'),rendering_time=data.get('rendering_time'),
            last_published_frame=self.frames[name],frame_keys=sorted(data),
            internal_frame_keys=sorted(vars(camera).get('_current_frame',{})),
            rgb_shape=list(np.shape(data.get('rgb'))),
            depth_shape=list(np.shape(data.get('distance_to_image_plane'))),
            paused=camera.is_paused() if callable(getattr(camera,'is_paused',None)) else None,
            render_updates_enabled=getattr(texture,'updates_enabled',None))
        print('PLATFORM_CAMERA_NO_PUBLISH '+json.dumps(payload,default=str),flush=True)

    def publish(self, now):
        from builtin_interfaces.msg import Time
        for name,(camera,frame,pubs) in self.cameras.items():
            data=camera.get_current_frame()
            index=camera_frame_index(data.get('rendering_frame')); stamp=float(data.get('rendering_time',0.))
            if index is None or index<=self.frames[name] or not math.isfinite(stamp) or stamp<=0:
                self._publish_diagnostic(name,camera,data,'frame_identity_not_advanced',now)
                continue
            # Isaac 6.1's callback snapshots rgb and depth together. get_rgba()
            # reads the live annotator and may already belong to a later render.
            # Older Camera versions without the rgb frame key keep their API.
            rgba=data.get('rgb') if 'rgb' in data else camera.get_rgba()
            depth=data.get('distance_to_image_plane')
            if rgba is None or depth is None or np.size(rgba)==0 or np.size(depth)==0:
                self._publish_diagnostic(name,camera,data,'missing_rgb_or_depth',now)
                continue
            rgba=np.asarray(rgba); depth=np.asarray(depth,dtype=np.float32)
            if rgba.ndim!=3 or depth.shape!=rgba.shape[:2] or not np.any(np.isfinite(depth)&(depth>0)):
                self._publish_diagnostic(name,camera,data,'invalid_rgb_depth_shape_or_depth',now)
                continue
            sec=int(stamp); ros_stamp=Time(sec=sec,nanosec=int((stamp-sec)*1e9))
            rgb=self.Image(); rgb.header.stamp=ros_stamp; rgb.header.frame_id=frame
            rgb.height,rgb.width=rgba.shape[:2]; rgb.encoding='rgb8'; rgb.step=rgb.width*3
            rgb.data=np.ascontiguousarray(rgba[:,:,:3],dtype=np.uint8).tobytes()
            image=self.Image(); image.header=rgb.header; image.height,image.width=depth.shape
            image.encoding='32FC1'; image.step=image.width*4; image.data=np.ascontiguousarray(depth).tobytes()
            info=self.CameraInfo(); info.header=rgb.header; info.height=rgb.height; info.width=rgb.width
            k=np.asarray(camera.get_intrinsics_matrix()); info.k=k.flatten().tolist()
            info.distortion_model='plumb_bob'; info.d=[0.]*5; info.r=np.eye(3).flatten().tolist()
            info.p=np.column_stack((k,np.zeros(3))).flatten().tolist()
            pubs[0].publish(rgb); pubs[1].publish(image); pubs[2].publish(info); pubs[3].publish(info)
            if self.session_id:
                self.previews[name].publish(rgba[:,:,:3],rgb.header)
                identity=dict(generation=self.session_id,frame_id=frame,
                    stamp=dict(sec=ros_stamp.sec,nanosec=ros_stamp.nanosec),clock_domain='ros',
                    width=rgb.width,height=rgb.height,valid=True,source='simulation',camera_id=name)
                self.identity_pubs[name].publish(self.String(data=json.dumps(identity)))
                self.identities[name]=identity
            self.frames[name]=index; self.last_monotonic[name]=now

    def ready(self,now):
        return all(0<=now-v<=1. for v in self.last_monotonic.values())

    def invalidate(self):
        for name,identity in self.identities.items():
            self.identity_pubs[name].publish(self.String(data=json.dumps({**identity,'valid':False})))
        self.identities.clear()
        self.last_monotonic={k:-math.inf for k in self.cameras}
        # Reset creates new sensor frame counters; never reuse pre-reset buffers.
        self.frames={k:-1 for k in self.cameras}


@dataclass(frozen=True)
class RollerSensorContract:
    """Arm-specific wiring around the RB20 measured reaction and command lease.

    Callers must give each arm its own session ID and namespace. Commands retain
    the six-axis wire format; their ordering is exactly ``joints``. Scene revision
    must include the asset/tool revision and change when the contact scene changes.
    """
    joints: tuple
    sensor_joint: str
    wrench_frame: str
    contact_link: str
    base_frame: str
    tcp_link: str
    tool_revision: str
    session_id: str
    scene_revision: str
    topic_namespace: str

    @classmethod
    def from_arm_contract(cls, arm, *, session_id, scene_revision, topic_namespace):
        joints=tuple(arm.get('joints',()))
        if (arm.get('tool_id')!='roller' or len(joints)!=6 or
            any(not isinstance(name,str) or not name for name in joints) or len(set(joints))!=6):
            raise ValueError('Roller sensor requires six unique selected-arm joints and a roller tool')
        names=('sensor_joint','wrench_frame','contact_link','base_frame','tcp_link','tool_revision')
        if any(not isinstance(arm.get(name),str) or not arm[name] for name in names):
            raise ValueError('Roller sensor requires complete sensor, contact, frame and revision metadata')
        namespace=topic_namespace.rstrip('/')
        if not session_id or not scene_revision or not namespace.startswith('/') or namespace=='':
            raise ValueError('Roller sensor requires session, revision and absolute topic namespace')
        return cls(joints,*(arm[name] for name in names),session_id,scene_revision,namespace)

    def topic(self, leaf):
        return self.topic_namespace+'/'+leaf.strip('/')

    def joint_indices(self, articulation_joint_names):
        names=list(articulation_joint_names)
        if any(names.count(name)!=1 for name in self.joints):
            raise ValueError('Articulation does not contain exactly one of each selected joint')
        return [names.index(name) for name in self.joints]

    def physics_indices(self, metadata):
        try:
            # Isaac's incoming joint wrench rows include the root body at row 0.
            return metadata.joint_indices[self.sensor_joint]+1,metadata.link_indices[self.wrench_frame]
        except KeyError as exc:
            raise ValueError('PhysX must retain the selected sensor joint and wrench body') from exc

    def command_lease(self):
        return CommandLease(self.session_id,self.scene_revision)

    def measure_wrench(self, incoming, joint_to_ft, world_from_ft_rotation, boundary):
        """Convert a genuine child-joint reaction; never takes a force target.

        ``joint_to_ft`` is from the USD fixed joint's localPos1/localRot1.
        ``boundary`` is the mass-preserving reduced subtree's mass and FT-frame
        centre of mass. Return raw reaction and gravity separately, so the
        hardware plugin performs compensation exactly once before free-space tare.
        """
        incoming=np.asarray(incoming,dtype=float)
        transform=np.asarray(joint_to_ft,dtype=float)
        rotation=np.asarray(world_from_ft_rotation,dtype=float)
        cog=np.asarray(boundary['cog_ft_m'],dtype=float)
        mass=float(boundary['mass_kg'])
        if (incoming.shape!=(6,) or transform.shape!=(4,4) or rotation.shape!=(3,3) or
            cog.shape!=(3,) or not math.isfinite(mass) or mass<=0 or
            any(not np.all(np.isfinite(value)) for value in (incoming,transform,rotation,cog))):
            raise ValueError('Invalid measured reaction, sensor transform or physical mass boundary')
        return -transform_wrench(incoming,transform),gravity_wrench(mass,cog,rotation)
