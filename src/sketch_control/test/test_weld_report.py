import numpy as np
import pytest
from sketch_control.weld_report import report_segment
from sketch_control.weld_contract import WeldSettings
from sketch_control.weld_geometry import build_weld_samples
from test_weld_geometry import S,P

def test_feedback_required_for_success():
    with pytest.raises(ValueError): report_segment(S,WeldSettings(),P,[])

def test_tracking_thresholds_and_retained_selection():
    settings=WeldSettings(); samples=build_weld_samples(S,settings,P)
    measured=[dict(time_s=float(t),xyz=list(x),quaternion=list(q)) for t,x,q in zip(samples.time_s,samples.tcp_xyz_m,samples.tcp_quat_xyzw)]
    report=report_segment(S,settings,P,measured)
    assert report['passed']; assert report['seam_id']=='S1'
    assert report['width_m']==pytest.approx(.004,abs=1e-4)
    assert report['frequency_hz']==pytest.approx(1,rel=.05)
    for point in measured: point['xyz'][1]+=.006
    assert not report_segment(S,settings,P,measured)['passed']

def test_wall_time_not_used_for_weave_frequency():
    settings=WeldSettings(); samples=build_weld_samples(S,settings,P)
    measured=[dict(time_s=float(t),wall_time=float(t*5),xyz=list(x),quaternion=list(q)) for t,x,q in zip(samples.time_s,samples.tcp_xyz_m,samples.tcp_quat_xyzw)]
    assert report_segment(S,settings,P,measured)['frequency_hz']==pytest.approx(1,rel=.05)

def test_checker_rejects_fake_or_physical_controller_hardware():
    from sketch_control.weld_report import check_isaac_hardware
    component=dict(name='ConstructIsaacSystem',plugin_name='topic_based_ros2_control/TopicBasedSystem',state={'id':3})
    check_isaac_hardware([component])
    for bad in ([],[{**component,'plugin_name':'mock_components/GenericSystem'}],[{**component,'state':{'id':1}}],[component,component]):
        with pytest.raises(ValueError):check_isaac_hardware(bad)
