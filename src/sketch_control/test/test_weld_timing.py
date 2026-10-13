import numpy as np
import pytest
from sketch_control.weld_trajectory import validate_weld_timing, tracking_metrics
from test_weld_geometry import S,P
from sketch_control.weld_contract import WeldSettings
from sketch_control.weld_geometry import build_weld_samples

def test_retiming_over_five_percent_is_rejected():
    s=build_weld_samples(S,WeldSettings(),P)
    with pytest.raises(ValueError): validate_weld_timing(s,s.time_s*1.06)
    assert validate_weld_timing(s,s.time_s*1.04)['speed_relative_error']<.05

def test_feedback_required_and_wrong_orientation_fails():
    s=build_weld_samples(S,WeldSettings(),P)
    with pytest.raises(ValueError): tracking_metrics(s,[])
    measured=[dict(time_s=float(t),xyz=list(p),quaternion=[0,0,0,1]) for t,p in zip(s.time_s,s.tcp_xyz_m)]
    assert tracking_metrics(s,measured)['max_angle_deg']>5

def test_travel_retiming_enforces_velocity_and_acceleration_without_changing_order():
    from sketch_control.weld_trajectory import retime_travel
    positions=np.array([[0.],[.1],[.11],[.3]])
    t=retime_travel(positions,[0.,.1,.2,.3])
    velocity=np.diff(positions[:,0])/np.diff(t)
    acceleration=np.diff(velocity)/((np.diff(t)[1:]+np.diff(t)[:-1])/2)
    assert np.max(abs(velocity))<=.1 and np.max(abs(acceleration))<=.5
    assert t[0]==0 and np.all(np.diff(t)>0)
