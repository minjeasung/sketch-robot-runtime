import numpy as np
import pytest
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import FrameBinding
from sketch_control.task_scene import make_surface, SceneCoordinator


def measurement():
    return dict(surface_id='selected',vertices_m=[[0,0,1],[1,0,1],[1,1,1],[0,1,1]],faces=[[0,1,2],[0,2,3]],normal=[0,0,1],frame_binding=FrameBinding('s',1,100,10.,1000,0,'cal'))


def registration():
    return CameraRegistration('zed','123','physical_optical','link0',(2,3,4),(0,0,0,1),'cal','fixed_measured')


def test_surface_preserves_measured_front_and_metric_transform():
    surface=make_surface(measurement(),registration())
    assert surface.frame_id=='link0'
    np.testing.assert_allclose(surface.vertices_m,[[2,3,5],[3,3,5],[3,4,5],[2,4,5]])
    assert surface.normal==(0.,0.,1.)
    assert surface.revision!=make_surface({**measurement(),'vertices_m':[[0,0,2],[1,0,2],[1,1,2],[0,1,2]]},registration()).revision


@pytest.mark.parametrize('change',[{'vertices_m':[]},{'normal':[0,0,-1]},{'vertices_m':[[0,0,float('nan')],[1,0,1],[1,1,1]]},{'faces':[[0,1,99]]}])
def test_invalid_or_flipped_surface_rejected(change):
    with pytest.raises(ValueError): make_surface({**measurement(),**change},registration())


def test_scene_requires_both_current_engine_ack_and_rejects_changes_during_work():
    c=SceneCoordinator(); first=make_surface(measurement(),registration()); c.request(first)
    assert not c.ready
    assert not c.ack('isaac',first.surface_id,'old',True)
    c.ack('isaac',first.surface_id,first.revision,True)
    assert not c.ready
    c.ack('moveit',first.surface_id,first.revision,False)
    assert not c.ready
    c.ack('moveit',first.surface_id,first.revision,True)
    assert c.ready
    with pytest.raises(ValueError,match='active'): c.request(first,task_active=True)
    c.invalidate('clock epoch changed')
    assert not c.ready
    assert not c.ack('moveit',first.surface_id,first.revision,True)
