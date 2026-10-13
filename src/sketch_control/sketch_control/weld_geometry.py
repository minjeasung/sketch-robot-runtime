"""3D seam geometry. No camera pixels or ROS state enter these calculations."""
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .weld_contract import WeldSamples


def _unit(v):
    n=np.linalg.norm(v,axis=-1,keepdims=True)
    if np.any(n<1e-8): raise ValueError('degenerate orientation reference')
    return v/n


def tool_outward_axes(tcp_quaternions,profile):
    tip=Rotation.from_quat(tcp_quaternions)*Rotation.from_quat(profile.tip_quaternion_xyzw)
    return -tip.apply([0,0,1])


def transform_catalog(catalog,transform):
    t=np.asarray(transform,float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]): raise ValueError('invalid rigid transform')
    if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(t[:3,:3])<.999: raise ValueError('non-rigid transform')
    rot=Rotation.from_matrix(t[:3,:3]); seams=[]
    for s in catalog.seams:
        pts=np.asarray(s.points_xyz)@t[:3,:3].T+t[:3,3]
        up=rot.apply(s.reference_up_vectors) if s.reference_up_vectors else []
        q=(rot*Rotation.from_quat(s.quaternions_xyzw)).as_quat() if s.quaternions_xyzw else []
        seams.append(replace(s,points_xyz=tuple(map(tuple,pts)),reference_up_vectors=tuple(map(tuple,up)),quaternions_xyzw=tuple(map(tuple,q))))
    return replace(catalog,seams=tuple(seams))


def build_weld_samples(seam,settings,profile,reverse=False):
    points=np.asarray(seam.points_xyz,float)
    if reverse: points=points[::-1]
    delta=np.diff(points,axis=0); lengths=np.linalg.norm(delta,axis=1)
    if len(lengths)<1 or np.any(lengths<1e-8) or np.any(lengths>profile.max_step_m+1e-9): raise ValueError('seam discontinuity or duplicate')
    tangents=delta/lengths[:,None]
    if len(tangents)>1 and np.any(np.rad2deg(np.arccos(np.clip(np.sum(tangents[1:]*tangents[:-1],axis=1),-1,1)))>profile.max_corner_deg): raise ValueError('split seam at sharp corner')
    arc=np.r_[0,np.cumsum(lengths)]; duration=arc[-1]/settings.speed_mps
    dt=min(profile.sample_dt_s,profile.sample_spacing_m/settings.speed_mps,
           1/(max(settings.frequency_hz,.01)*100) if settings.weave_enabled else profile.sample_dt_s)
    count=int(np.ceil(duration/dt))+1
    if count>50000: raise ValueError('path too large; split seam')
    times=np.linspace(0,duration,count); dist=times*settings.speed_mps
    center=np.column_stack([np.interp(dist,arc,points[:,i]) for i in range(3)])
    # Average adjacent directions at vertices, then interpolate the directed
    # frame along arc length. The seam positions remain the supplied polyline.
    vertex_tangents=np.vstack([tangents[0],_unit(tangents[:-1]+tangents[1:]),tangents[-1]])
    tangent=_unit(np.column_stack([np.interp(dist,arc,vertex_tangents[:,i]) for i in range(3)]))
    if seam.quaternions_xyzw:
        qs=np.asarray(seam.quaternions_xyzw)
        if len(qs)==1: qs=np.tile(qs,(len(points),1))
        if reverse: qs=qs[::-1]
        for i in range(1,len(qs)):
            if qs[i]@qs[i-1]<0: qs[i]=-qs[i]
        if np.any((Rotation.from_quat(qs[:-1]).inv()*Rotation.from_quat(qs[1:])).magnitude()>np.deg2rad(profile.max_corner_deg)): raise ValueError('split seam at sharp orientation change')
        base=Slerp(arc,Rotation.from_quat(qs))(dist)
        normal=-base.apply(np.tile([0,0,1],(count,1)))
        lateral=_unit(np.cross(normal,tangent))
    else:
        if seam.reference_up_vectors:
            ups=np.asarray(seam.reference_up_vectors)
            if len(ups)==1: ups=np.tile(ups,(len(points),1))
            if reverse: ups=ups[::-1]
            normal=np.column_stack([np.interp(dist,arc,ups[:,i]) for i in range(3)])
        elif (seam.pose_profile_id or settings.pose_profile_id)==profile.profile_id:
            normal=np.tile(profile.reference_up,(count,1))
        else: raise ValueError('explicit pose profile or orientation required')
        lateral=_unit(np.cross(normal,tangent)); normal=_unit(np.cross(tangent,lateral))
        z=-normal; y=_unit(np.cross(z,tangent)); x=_unit(np.cross(y,z))
        base=Rotation.from_matrix(np.stack([x,y,z],axis=2))
    offset=np.zeros(count)
    if settings.weave_enabled:
        period=1/settings.frequency_hz; sweep=(period-settings.left_dwell_s-settings.right_dwell_s)/2
        if sweep<=0: raise ValueError('invalid dwell')
        phase=times%period
        offset=np.where(phase<settings.right_dwell_s,1.,np.where(phase<settings.right_dwell_s+sweep,
            np.cos(np.pi*(phase-settings.right_dwell_s)/sweep),np.where(phase<settings.right_dwell_s+sweep+settings.left_dwell_s,-1.,
            -np.cos(np.pi*(phase-settings.right_dwell_s-sweep-settings.left_dwell_s)/sweep))))
        envelope=np.minimum(np.minimum(times/period,(duration-times)/period),1).clip(0,1)
        envelope=envelope**2*(3-2*envelope)
        offset*=settings.width_m*.5*envelope
    tip=center+normal*settings.standoff_m+lateral*offset[:,None]
    # Work/travel axes follow the directed seam even when its base pose
    # comes from an explicit quaternion rather than a generated frame.
    work=Rotation.from_rotvec(tangent*np.deg2rad(settings.work_angle_deg))
    travel=Rotation.from_rotvec(-lateral*np.deg2rad(settings.travel_angle_deg))
    tip_rot=work*travel*base
    tcp_rot=tip_rot*Rotation.from_quat(profile.tip_quaternion_xyzw).inv()
    tcp=tip-tcp_rot.apply(np.tile(profile.tip_translation_m,(count,1)))
    arrays=[times,dist,tip,tcp,tcp_rot.as_quat()]
    for a in arrays: a.setflags(write=False)
    return WeldSamples(*arrays,tuple('WELD' for _ in times))
