from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sketch_control.weld_contract import Seam, WeldSettings, load_tool_profile
from sketch_control.weld_geometry import build_weld_samples

P=load_tool_profile(Path(__file__).parents[1]/'config/weld_sim_profile.json','construct_left')
S=Seam('S1',((0.,0.,0.),(.1,0.,0.)),pose_profile_id=P.profile_id)

def test_tip_to_tcp_rotation_translation():
    out=build_weld_samples(S,WeldSettings(weave_enabled=False),P)
    actual=out.tcp_xyz_m+Rotation.from_quat(out.tcp_quat_xyzw).apply(np.tile([0,0,.15],(len(out.time_s),1)))
    np.testing.assert_allclose(actual,out.tip_xyz_m,atol=1e-10)
    np.testing.assert_allclose(out.tip_xyz_m[0],[0,-.01,0],atol=1e-10)

def test_reverse_recomputes_travel_angle():
    f=build_weld_samples(S,WeldSettings(weave_enabled=False,travel_angle_deg=20),P)
    r=build_weld_samples(S,WeldSettings(weave_enabled=False,travel_angle_deg=20),P,reverse=True)
    assert Rotation.from_quat(f.tcp_quat_xyzw[0]).apply([0,0,1])[0] == pytest.approx(-Rotation.from_quat(r.tcp_quat_xyzw[-1]).apply([0,0,1])[0])

def test_q_sign_does_not_flip_torch():
    seam=replace(S,quaternions_xyzw=((0,0,0,1),(0,0,0,-1)))
    out=build_weld_samples(seam,WeldSettings(weave_enabled=False),P)
    assert np.max(Rotation.from_quat(out.tcp_quat_xyzw).magnitude()) < 1e-9

def test_parallel_reference_and_missing_pose_are_rejected():
    for seam in [replace(S,pose_profile_id=''),replace(S,reference_up_vectors=((1,0,0),))]:
        with pytest.raises(ValueError): build_weld_samples(seam,WeldSettings(),P)

def test_full_weave_width_dwell_and_ramp():
    out=build_weld_samples(S,WeldSettings(width_m=.004,frequency_hz=1,left_dwell_s=.1,right_dwell_s=.1),P)
    assert np.ptp(out.tip_xyz_m[:,2]) == pytest.approx(.004,abs=1e-6)
    assert out.tip_xyz_m[0,2] == pytest.approx(0)
    assert out.tip_xyz_m[-1,2] == pytest.approx(0)
    assert np.all(np.diff(out.center_distance_m)>0)
    assert np.all(np.diff(out.time_s)>0)

def test_reverse_travel_angle_tracks_seam_direction_with_explicit_quaternion():
    from dataclasses import replace
    base=build_weld_samples(S,WeldSettings(weave_enabled=False),P).tcp_quat_xyzw[0]
    seam=replace(S,reference_up_vectors=(),quaternions_xyzw=(tuple(base),),pose_profile_id='')
    settings=WeldSettings(weave_enabled=False,travel_angle_deg=10.)
    forward=build_weld_samples(seam,settings,P)
    backward=build_weld_samples(seam,settings,P,reverse=True)
    zf=Rotation.from_quat(forward.tcp_quat_xyzw[0]).apply([0,0,1])
    zb=Rotation.from_quat(backward.tcp_quat_xyzw[0]).apply([0,0,1])
    tangent=np.array(S.points_xyz[-1])-S.points_xyz[0]
    assert (zf@tangent)*(zb@tangent)<0

def test_xyz_only_requires_explicit_settings_profile():
    from sketch_control.weld_contract import validate_settings
    seam=replace(S,pose_profile_id='')
    with pytest.raises(ValueError,match='pose'): build_weld_samples(seam,WeldSettings(),P)
    settings=validate_settings({'pose_profile_id':P.profile_id},P)
    out=build_weld_samples(seam,settings,P)
    assert len(out.time_s)>2
    with pytest.raises(ValueError,match='profile'): validate_settings({'pose_profile_id':'unregistered'},P)

def test_gentle_curve_has_continuous_local_frame_in_both_directions():
    theta=np.linspace(0,.2,11)
    seam=Seam('curve',tuple(map(tuple,np.column_stack([np.sin(theta),np.zeros(11),1-np.cos(theta)]))),pose_profile_id=P.profile_id)
    for reverse in (False,True):
        out=build_weld_samples(seam,WeldSettings(weave_enabled=False),P,reverse)
        r=Rotation.from_quat(out.tcp_quat_xyzw)
        angular_speed=(r[:-1].inv()*r[1:]).magnitude()/np.diff(out.time_s)
        assert np.max(angular_speed)<.012
        assert np.max(np.linalg.norm(np.diff(out.tip_xyz_m,axis=0),axis=1)/np.diff(out.time_s))<.012

def test_approach_axis_uses_tip_rotation_not_tcp_rotation():
    from sketch_control.weld_geometry import tool_outward_axes
    p=replace(P,tip_quaternion_xyzw=tuple(Rotation.from_euler('x',30,degrees=True).as_quat()))
    samples=build_weld_samples(S,WeldSettings(weave_enabled=False),p)
    normal=tool_outward_axes(samples.tcp_quat_xyzw,p)
    np.testing.assert_allclose(normal,np.tile(P.reference_up,(len(normal),1)),atol=1e-10)
