from dataclasses import asdict
import numpy as np
import pytest
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import FrameBinding
from sketch_control.task_scene import SceneCoordinator, make_refined_surface


def geometry():
    registration = CameraRegistration('d405','42','physical_d405','World',(1.,2.,3.),(0.,0.,0.,1.),'cal','fixed_measured')
    binding = FrameBinding('session',2,200,10.,2000,0,'cal')
    corners = [[0.,0.,4.],[1.,0.,4.],[1.,1.,4.],[0.,1.,4.]]
    return registration,binding,corners


def test_refined_surface_uses_exact_projected_work_area_and_new_dual_ack():
    r,b,corners=geometry()
    surface=make_refined_surface('work-area',corners,[0.,0.,4.02],[0.,0.,-1.],'World',b,r)
    np.testing.assert_allclose(surface.vertices_m,np.array(corners)+[0,0,.02])
    assert surface.frame_binding==b and surface.normal==(0.,0.,-1.)
    coord=SceneCoordinator();coord.request(surface)
    coord.ack('moveit',surface.surface_id,surface.revision,True)
    assert not coord.ready
    assert not coord.ack('isaac',surface.surface_id,'old-revision',True)
    coord.ack('isaac',surface.surface_id,surface.revision,True)
    assert coord.ready
    updated=make_refined_surface('work-area',corners,[0.,0.,4.03],[0.,0.,-1.],'World',b,r)
    assert updated.revision!=surface.revision
    with pytest.raises(ValueError,match='active'):coord.request(updated,task_active=True)
    coord.request(updated);assert not coord.ready and not coord.acks


@pytest.mark.parametrize('corners,frame',[(None,'World'),([], 'World'),([[0.,0.,4.]]*4,'World'),([[0.,0.,4.],[1.,0.,4.],[1.,1.,4.],[0.,1.,4.]],'virtual_tcp')])
def test_refined_surface_rejects_missing_support_or_wrong_frame(corners,frame):
    r,b,_=geometry()
    with pytest.raises(ValueError):make_refined_surface('work-area',corners,[0.,0.,4.02],[0.,0.,-1.],frame,b,r)


def test_refined_scene_request_requires_fresh_exact_d405_context_and_parent():
    from sketch_control.task_scene import validate_scene_request
    from sketch_control.mixed_frame_sync import PhysicalCameraReadiness,SelectionLease
    r,b,corners=geometry();readiness=PhysicalCameraReadiness({'d405':r});readiness.frames['d405']=b
    surface=make_refined_surface('work-area',corners,[0.,0.,4.02],[0.,0.,-1.],'World',b,r)
    coordinator=SceneCoordinator();coordinator.request(surface)
    for engine in ('moveit','isaac'):coordinator.ack(engine,surface.surface_id,surface.revision,True)
    measurement={'surface_id':surface.surface_id,'vertices_m':((np.asarray(surface.vertices_m)-r.translation_m)@r.rotation).tolist(),'faces':surface.faces,'normal':(r.rotation.T@surface.normal).tolist(),'frame_binding':asdict(b)}
    request={'camera_name':'d405','parent_revision':surface.revision,'measurement':measurement}
    validate_scene_request(request,'zed',{'d405':r},readiness,SelectionLease('zed-cal'),coordinator,10.1)
    with pytest.raises(ValueError,match='parent'):
        validate_scene_request({**request,'parent_revision':'old'},'zed',{'d405':r},readiness,SelectionLease('zed-cal'),coordinator,10.1)
    with pytest.raises(ValueError,match='expired'):
        validate_scene_request(request,'zed',{'d405':r},readiness,SelectionLease('zed-cal'),coordinator,12.)
    with pytest.raises(ValueError,match='parent'):
        validate_scene_request({k:v for k,v in request.items() if k!='parent_revision'},'zed',{'d405':r},readiness,SelectionLease('zed-cal'),coordinator,10.1)
