"""Measured simulation report; missing feedback never means success."""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import find_peaks
from .weld_geometry import build_weld_samples
from .weld_trajectory import tracking_metrics


def report_segment(seam,settings,profile,measured,reverse=False):
    samples=build_weld_samples(seam,settings,profile,reverse)
    report=tracking_metrics(samples,measured); report['seam_id']=seam.seam_id
    t=np.asarray([p['time_s'] for p in measured]); xyz=np.asarray([p['xyz'] for p in measured])
    q=np.asarray([p['quaternion'] for p in measured]); tip=xyz+Rotation.from_quat(q).apply(np.tile(profile.tip_translation_m,(len(t),1)))
    pts=np.asarray(seam.points_xyz); pts=pts[::-1] if reverse else pts
    tangent=pts[-1]-pts[0]; tangent/=np.linalg.norm(tangent)
    # Integration fixture uses straight seams. Curved paths retain the tracking
    # metrics, but require a transported-frame metrology adapter for weave stats.
    if np.max(np.linalg.norm(np.cross(pts-pts[0],tangent),axis=1))>1e-6: raise ValueError('weave metrology requires straight fixture seam')
    up=np.asarray(seam.reference_up_vectors[0] if seam.reference_up_vectors else profile.reference_up)
    lateral=np.cross(up,tangent); lateral/=np.linalg.norm(lateral)
    margin=1/settings.frequency_hz if settings.weave_enabled else .1
    mask=(t>=margin)&(t<=samples.time_s[-1]-margin)
    if mask.sum()<5: raise ValueError('insufficient steady-state feedback')
    speed=float(np.polyfit(t[mask],(tip[mask]-pts[0])@tangent,1)[0]); report['speed_mps']=speed
    report['speed_relative_error']=abs(speed/settings.speed_mps-1)
    report['passed'] &= report['speed_relative_error']<=.05
    if settings.weave_enabled:
        y=(tip-pts[0])@lateral; peaks,_=find_peaks(y[mask],prominence=settings.width_m*.5)
        frequency=1/float(np.mean(np.diff(t[mask][peaks]))) if len(peaks)>=2 else 0.
        width=float(np.ptp(y[mask])); report.update(width_m=width,frequency_hz=frequency,
            width_relative_error=abs(width/settings.width_m-1),frequency_relative_error=abs(frequency/settings.frequency_hz-1))
        report['passed'] &= report['width_relative_error']<=.05 and report['frequency_relative_error']<=.05
    report['passed']=bool(report['passed']); return report

def check_isaac_hardware(components):
    if len(components)!=1 or components[0].get('name')!='ConstructIsaacSystem' or components[0].get('plugin_name')!='topic_based_ros2_control/TopicBasedSystem' or components[0].get('state',{}).get('id')!=3:
        raise ValueError('active Construct Isaac hardware required; fake/physical controllers are refused')
