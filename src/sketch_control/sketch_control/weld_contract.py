"""Validated, immutable boundary objects for seam-based welding (SI units)."""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math


def number(value, low=-100., high=100.):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('finite numeric value outside allowed range')
    return float(value)


def ident(value):
    if not isinstance(value,str) or not value.strip() or len(value)>256:
        raise ValueError('nonempty identifier required')
    return value


def vectors(values, width, limit=20000):
    if not isinstance(values,(list,tuple)) or len(values)>limit:
        raise ValueError('invalid vector array')
    result=[]
    for v in values:
        if not isinstance(v,(list,tuple)) or len(v)!=width: raise ValueError('invalid vector dimensions')
        result.append(tuple(number(x) for x in v))
    return tuple(result)


@dataclass(frozen=True)
class TargetRequest:
    request_id: str
    target_revision: str
    generation: str
    frame_id: str
    stamp: tuple
    clock_domain: str
    width: int
    height: int
    polygon: tuple
    provider: str


@dataclass(frozen=True)
class Seam:
    seam_id: str
    points_xyz: tuple
    reference_up_vectors: tuple = ()
    quaternions_xyzw: tuple = ()
    pose_profile_id: str = ''


@dataclass(frozen=True)
class SeamCatalog:
    request_id: str
    target_revision: str
    generation: str
    source: str
    frame_id: str
    seams: tuple
    schema_version: int = 1
    length_unit: str = 'm'


@dataclass(frozen=True)
class WeldSettings:
    speed_mps: float = .01
    approach_m: float = .05
    retreat_m: float = .05
    standoff_m: float = .01
    work_angle_deg: float = 0.
    travel_angle_deg: float = 0.
    weave_enabled: bool = True
    width_m: float = .004
    frequency_hz: float = 1.
    left_dwell_s: float = 0.
    right_dwell_s: float = 0.
    current_a: float = 100.
    pose_profile_id: str = ''


@dataclass(frozen=True)
class ToolProfile:
    profile_id: str
    revision: str
    model_id: str
    tip_translation_m: tuple
    tip_quaternion_xyzw: tuple
    reference_up: tuple
    radius_m: float
    max_step_m: float = .1
    max_corner_deg: float = 45.
    sample_dt_s: float = .02
    sample_spacing_m: float = .001


@dataclass(frozen=True)
class PlanIdentity:
    server_session_id: str
    target_revision: str
    generation: str
    selection_revision: int
    settings_revision: int
    tool_revision: str
    scene_revision: str
    model_id: str
    planning_attempt: str = ''


@dataclass(frozen=True)
class WeldSamples:
    time_s: object
    center_distance_m: object
    tip_xyz_m: object
    tcp_xyz_m: object
    tcp_quat_xyzw: object
    phase: tuple


@dataclass(frozen=True)
class PlannedSegment:
    seam_id: str
    phase: str
    trajectory: object
    samples: object = None


@dataclass(frozen=True)
class WeldPlan:
    plan_id: str
    identity: PlanIdentity
    start_joints: tuple
    segments: tuple


def validate_target(p):
    if not isinstance(p,dict): raise ValueError('target object required')
    width=number(p.get('width'),1,8192); height=number(p.get('height'),1,8192)
    poly=p.get('polygon',[])
    if not isinstance(poly,(list,tuple)) or not 3 <= len(poly) <= 1024: raise ValueError('closed target polygon required')
    pts=tuple((number(v[0],0,width-1),number(v[1],0,height-1)) for v in poly if isinstance(v,(list,tuple)) and len(v)==2)
    if len(pts)!=len(poly): raise ValueError('invalid polygon')
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(pts,pts[1:]+pts[:1]))
    if abs(area)<1: raise ValueError('degenerate target polygon')
    stamp=p.get('stamp',{})
    if not isinstance(stamp,dict): raise ValueError('stamp object required')
    sec=number(stamp.get('sec'),0,1e12); ns=number(stamp.get('nanosec'),0,999999999)
    if p.get('provider') not in ('fixture','external') or p.get('clock_domain') not in ('ros','wall'):
        raise ValueError('explicit provider and clock domain required')
    return TargetRequest(*(ident(p.get(k)) for k in ('request_id','target_revision','generation','frame_id')),
        (int(sec),int(ns)),p['clock_domain'],int(width),int(height),pts,p['provider'])


def validate_catalog(p,target):
    if not isinstance(p,dict) or p.get('schema_version')!=1 or p.get('length_unit')!='m':
        raise ValueError('schema 1 and length_unit=m required')
    for key in ('request_id','target_revision','generation'):
        if p.get(key)!=getattr(target,key): raise ValueError('stale '+key)
    if p.get('source')!=target.provider: raise ValueError('unexpected provider')
    items=p.get('seams'); result=[]; ids=set()
    if not isinstance(items,list) or len(items)>128: raise ValueError('invalid seam list')
    for item in items:
        if not isinstance(item,dict): raise ValueError('seam object required')
        sid=ident(item.get('seam_id')); pts=vectors(item.get('points_xyz'),3)
        if sid in ids or len(pts)<2: raise ValueError('duplicate ID or insufficient seam points')
        if any(math.dist(a,b)<1e-8 for a,b in zip(pts,pts[1:])): raise ValueError('duplicate seam points')
        up=vectors(item.get('reference_up_vectors',[]),3); q=vectors(item.get('quaternions_xyzw',[]),4)
        if up and q: raise ValueError('choose one orientation source')
        for arr in (up,q):
            if arr and len(arr) not in (1,len(pts)): raise ValueError('orientation count mismatch')
            if any(abs(math.sqrt(sum(x*x for x in v))-1)>.001 for v in arr): raise ValueError('orientation must be normalized')
        ids.add(sid); result.append(Seam(sid,pts,up,q,item.get('pose_profile_id','')))
    return SeamCatalog(target.request_id,target.target_revision,target.generation,p['source'],ident(p.get('frame_id')),tuple(result))


def validate_settings(p,profile):
    if not isinstance(p,dict) or set(p)-set(asdict(WeldSettings())): raise ValueError('unknown settings')
    limits=dict(speed_mps=(.001,.05), approach_m=(.005,.3), retreat_m=(.005,.3),standoff_m=(.001,.1),
        work_angle_deg=(-60,60),travel_angle_deg=(-60,60),width_m=(0,.02),frequency_hz=(0,5),
        left_dwell_s=(0,5),right_dwell_s=(0,5),current_a=(0,500))
    values=asdict(WeldSettings()); values.update(p)
    if not isinstance(values['pose_profile_id'],str) or values['pose_profile_id'] not in ('',profile.profile_id):
        raise ValueError('unknown pose profile')
    if type(values['weave_enabled']) is not bool: raise ValueError('weave_enabled must be boolean')
    for k,bounds in limits.items(): values[k]=number(values[k],*bounds)
    if values['weave_enabled'] and (values['frequency_hz']<=0 or values['width_m']<=0 or
        values['left_dwell_s']+values['right_dwell_s']>=1/values['frequency_hz']): raise ValueError('invalid weave period or dwell')
    return WeldSettings(**values)


def load_tool_profile(path,model_id):
    p=json.loads(Path(path).read_text())
    if p.get('simulation_only') is not True or model_id not in p.get('models',[]): raise ValueError('profile not valid for model')
    xyz=vectors([p['tip_translation_m']],3)[0]; q=vectors([p['tip_quaternion_xyzw']],4)[0]; up=vectors([p['reference_up']],3)[0]
    if abs(math.sqrt(sum(x*x for x in q))-1)>.001 or abs(math.sqrt(sum(x*x for x in up))-1)>.001: raise ValueError('profile orientation not normalized')
    return ToolProfile(ident(p['profile_id']),ident(p['revision']),model_id,xyz,q,up,number(p['radius_m'],.001,.05))
