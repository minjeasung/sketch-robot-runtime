import numpy as np
import pytest
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import FrameBinding
from sketch_control.rgbd_weld_provider import project_seam, RGBDFrameStore
from sketch_control.weld_contract import validate_target


def inputs():
    return dict(polyline_px=[[0,2],[4,2]],depth_m=np.ones((5,5)),intrinsics=dict(fx=10.,fy=10.,cx=2.,cy=2.),frame_binding=FrameBinding('physical-1',3,100,10.,1000,0,'cal'),registration=CameraRegistration('zed','123','optical','link0',(1,2,3),(0,0,0,1),'cal','fixed_measured'),steady_now=10.2)


def test_backprojects_selected_pixels_and_measured_surface_normal():
    catalog=project_seam(**inputs())
    assert catalog.source=='external' and catalog.frame_id=='link0'
    assert len(catalog.seams)==1
    np.testing.assert_allclose(catalog.seams[0].points_xyz[0],[.8,2,4])
    np.testing.assert_allclose(catalog.seams[0].points_xyz[-1],[1.2,2,4])
    np.testing.assert_allclose(catalog.seams[0].reference_up_vectors,[[0,0,-1]])


@pytest.mark.parametrize('kind',['missing','jump','expired','calibration','degenerate'])
def test_invalid_depth_or_old_selection_is_rejected(kind):
    p=inputs()
    if kind=='missing': p['depth_m'][2,2]=np.nan
    if kind=='jump': p['depth_m'][:,2:]=2
    if kind=='expired': p['steady_now']=11.1
    if kind=='calibration': p['frame_binding']=FrameBinding('physical-1',3,100,10.,1000,0,'old')
    if kind=='degenerate': p['polyline_px']=[[1,1],[1,1]]
    with pytest.raises(ValueError): project_seam(**p)


def test_frame_store_requires_exact_components_and_invalidates_old_session():
    p=inputs(); store=RGBDFrameStore(p['registration'])
    store.add_depth(1000,p['depth_m']); store.add_intrinsics(1001,p['intrinsics'])
    identity=dict(frame_binding=p['frame_binding'].__dict__,stamp={'sec':0,'nanosec':1000},generation='physical-1',frame_id='optical',width=5,height=5,clock_domain='ros')
    store.add_identity(identity)
    with pytest.raises(ValueError,match='RGB-D'): store.selected(identity,10.2)
    store.add_intrinsics(1000,p['intrinsics']); store.add_rgb(1000,(5,5))
    assert store.selected(identity,10.2)[0].shape==(5,5)
    store.add_identity({**identity,'frame_binding':{**identity['frame_binding'],'source_session':'physical-2','source_frame_id':1}})
    with pytest.raises(ValueError): store.selected(identity,10.2)


def test_measured_roi_keeps_depth_and_does_not_bridge_a_missing_patch():
    from sketch_control.rgbd_weld_provider import project_surface_measurement
    p=inputs(); p['depth_m'][2,2]=np.nan
    m=project_surface_measurement([[0,0],[4,0],[4,4],[0,4]],p['depth_m'],p['intrinsics'],p['frame_binding'],'selected')
    vertices=np.asarray(m['vertices_m'])
    assert len(m['faces'])>0 and np.isfinite(vertices).all()
    assert not np.any(np.all(vertices[:,:2]==[0,0],axis=1))
    np.testing.assert_allclose(vertices[:,2],1.)


def test_pinned_capture_is_invalid_when_live_camera_stops():
    p=inputs(); store=RGBDFrameStore(p['registration'])
    identity=dict(frame_binding=p['frame_binding'].__dict__,stamp={'sec':0,'nanosec':1000},generation='physical-1',frame_id='optical',width=5,height=5,clock_domain='ros')
    store.add_depth(1000,p['depth_m']); store.add_intrinsics(1000,p['intrinsics']); store.add_rgb(1000,(5,5)); store.add_identity(identity)
    store.pin(identity,10.2)
    with pytest.raises(ValueError,match='live camera'): store.selected(identity,12.)


def test_visible_surface_normal_places_weld_standoff_on_camera_side():
    from sketch_control.weld_contract import ToolProfile,WeldSettings
    from sketch_control.weld_geometry import build_weld_samples
    from sketch_control.rgbd_weld_provider import project_surface_measurement
    p=inputs();seam=project_seam(**p).seams[0]
    profile=ToolProfile('test','r','rb10_1300e_u',(0,0,0),(0,0,0,1),(0,0,-1),.001)
    samples=build_weld_samples(seam,WeldSettings(weave_enabled=False,standoff_m=.01),profile)
    np.testing.assert_allclose(samples.tip_xyz_m[:,2],3.99)
    m=project_surface_measurement([[0,0],[4,0],[4,4],[0,4]],p['depth_m'],p['intrinsics'],p['frame_binding'],'s')
    normal=np.array(m['normal']);tri=np.array(m['vertices_m'])[np.array(m['faces'])]
    assert np.dot(normal,tri.mean((0,1)))<0
    assert np.all(np.einsum('ij,j->i',np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),normal)>0)
