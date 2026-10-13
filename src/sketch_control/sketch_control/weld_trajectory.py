"""Trajectory timing and measured tracking, independent of ROS transport."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def validate_weld_timing(samples,actual_time_s,tolerance=.05):
    actual=np.asarray(actual_time_s,float); desired=np.asarray(samples.time_s,float)
    if actual.shape!=desired.shape or not np.isfinite(actual).all() or np.any(np.diff(actual)<=0): raise ValueError('invalid sample times')
    ratios=np.diff(desired)/np.diff(actual)
    error=float(np.max(abs(ratios-1)))
    if error>tolerance: raise ValueError('requested speed/frequency cannot be maintained')
    return {'speed_relative_error':error,'frequency_relative_error':error}


def tracking_metrics(planned,measured):
    if len(measured)<2: raise ValueError('measured feedback required')
    t=np.asarray([m['time_s'] for m in measured]); xyz=np.asarray([m['xyz'] for m in measured]); q=np.asarray([m['quaternion'] for m in measured])
    if not all(np.isfinite(v).all() for v in (t,xyz,q)) or np.any(np.diff(t)<=0): raise ValueError('invalid feedback')
    if t[0]>planned.time_s[0]+.05 or t[-1]<planned.time_s[-1]-.05: raise ValueError('incomplete feedback')
    ts=np.clip(t,planned.time_s[0],planned.time_s[-1]); expected=np.column_stack([np.interp(ts,planned.time_s,planned.tcp_xyz_m[:,i]) for i in range(3)])
    expected_q=Slerp(planned.time_s,Rotation.from_quat(planned.tcp_quat_xyzw))(ts)
    pos=float(np.max(np.linalg.norm(xyz-expected,axis=1))); ang=float(np.rad2deg(np.max((expected_q.inv()*Rotation.from_quat(q)).magnitude())))
    return {'max_position_m':pos,'max_angle_deg':ang,'passed':pos<=.005 and ang<=5.,'feedback_count':len(t)}


def sample_joint_trajectory(trajectory,t):
    points=trajectory.points
    times=np.asarray([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points])
    if len(points)<2 or np.any(np.diff(times)<=0): raise ValueError('invalid joint trajectory')
    return np.asarray([np.interp(t,times,[p.positions[i] for p in points]) for i in range(len(trajectory.joint_names))])

def retime_travel(positions,times,max_speed=.1,max_accel=.5):
    """Uniformly slow non-welding travel; weld sample times never use this."""
    p=np.asarray(positions,float); t=np.asarray(times,float); dt=np.diff(t)
    if len(t)<2 or np.any(dt<=0) or not np.isfinite(p).all(): raise ValueError('invalid travel trajectory')
    velocity=np.diff(p,axis=0)/dt[:,None]
    acceleration=np.diff(velocity,axis=0)/((dt[1:]+dt[:-1])/2)[:,None]
    scale=max(1.,float(np.max(abs(velocity)))/max_speed,
              float(np.sqrt(np.max(abs(acceleration))/max_accel)) if acceleration.size else 1.)
    return (t-t[0])*scale*1.05
