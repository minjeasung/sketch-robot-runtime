import json
import time
from dataclasses import asdict
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('rclpy')
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray,Pose
from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import FrameBinding,PhysicalCameraReadiness
from sketch_control.plane_lifecycle import PlaneLifecycle


def node_fixture(monkeypatch):
    node=object.__new__(D405SurfaceRefinerNode)
    r=CameraRegistration('d405','42','physical_d405','World',(1.,2.,3.),(0.,0.,0.,1.),'cal','fixed_measured')
    b=FrameBinding('session',2,200,10.,2000,0,'cal')
    node.fit_config=SimpleNamespace(max_tf_age_s=.25)
    node.scene_registration=r;node._scene_camera=PhysicalCameraReadiness({'d405':r});node._scene_camera.frames['d405']=b
    node._scene_bindings={2000:b};node._scene_status={'ready':True,'revision':'old','surface_id':'selected'}
    node._scene_task_active=False;node._pending_scene_refinement=None
    node._lifecycles={'work_area':PlaneLifecycle('work_area')}
    node.latest_corners=PoseArray();node.latest_corners.header.frame_id='World'
    for x,y in ((0,0),(1,0),(1,1),(0,1)):
        pose=Pose();pose.position.x=float(x);pose.position.y=float(y);pose.position.z=4.;node.latest_corners.poses.append(pose)
    messages=[];node.scene_request_pub=SimpleNamespace(publish=lambda msg:messages.append(json.loads(msg.data)))
    accepted=[];rejected=[]
    node._accept_refinement=lambda *args:accepted.append(args)
    node._reject_capture=lambda *args,**kw:rejected.append((args,kw))
    node._publish_status=lambda *args,**kw:None
    monkeypatch.setattr('sketch_control.d405_surface_refiner_node.time.monotonic',lambda:10.1)
    stamp=SimpleNamespace(sec=0,nanosec=2000)
    cloud=SimpleNamespace(header=SimpleNamespace(stamp=stamp,frame_id='physical_d405'))
    result=SimpleNamespace(center=[0,0,4.02],normal=[0,0,-1.])
    return node,cloud,result,messages,accepted,rejected


def test_refinement_waits_for_exact_new_dual_ack_before_publishing_plane(monkeypatch):
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    node._stage_scene_refinement('work_area',cloud,'World',result,{})
    assert len(messages)==1 and not accepted
    request=messages[0];assert request['camera_name']=='d405' and request['parent_revision']=='old'
    revision=node._pending_scene_refinement['revision']
    node._scene_status={'ready':True,'revision':'old','acks':{'moveit':True,'isaac':True}}
    node._poll_scene_refinement();assert not accepted
    node._scene_status={'ready':True,'revision':revision,'acks':{'moveit':True}}
    node._poll_scene_refinement();assert not accepted
    node._scene_status={'ready':True,'revision':revision,'acks':{'moveit':True,'isaac':True}}
    node._poll_scene_refinement();assert len(accepted)==1 and not rejected
    assert accepted[0][-1]['task_scene_revision']==revision


@pytest.mark.parametrize('failure',['active','missing_binding','wrong_frame','generation_changed','expired_camera'])
def test_refinement_cannot_publish_on_unsafe_scene_or_capture(monkeypatch,failure):
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    if failure=='active':node._scene_task_active=True
    if failure=='missing_binding':node._scene_bindings={}
    if failure=='wrong_frame':cloud.header.frame_id='virtual_d405'
    if failure in ('active','missing_binding','wrong_frame'):
        with pytest.raises(ValueError):node._stage_scene_refinement('work_area',cloud,'World',result,{})
    else:
        node._stage_scene_refinement('work_area',cloud,'World',result,{})
        if failure=='generation_changed':node._lifecycles['work_area'].plane_generation_id='new'
        else:node._scene_camera.frames={}
        node._poll_scene_refinement();assert rejected and node._pending_scene_refinement is None
    assert not accepted


def test_refined_scene_converts_physical_zed_work_area_frame_using_measured_tf(monkeypatch):
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    node.latest_corners.header.frame_id='physical_zed'
    transform=SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=2.,y=3.,z=1.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    node._lookup_transform=lambda target,source,stamp:(transform,0.)
    node._stage_scene_refinement('work_area',cloud,'physical_zed',result,{})
    from sketch_control.task_scene import make_surface
    surface=make_surface(messages[0]['measurement'],node.scene_registration)
    np.testing.assert_allclose(surface.vertices_m,[[2,3,5.02],[3,3,5.02],[3,4,5.02],[2,4,5.02]])


def test_target_refinement_reuses_only_current_measured_mesh_support(monkeypatch):
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    from sketch_control.task_scene import make_refined_surface,make_surface
    node._lifecycles['target']=PlaneLifecycle('target')
    binding=node._scene_bindings[2000]
    original=make_refined_surface('selected',node._corner_points(node.latest_corners),[0.,0.,4.],[0.,0.,-1.],'World',binding,node.scene_registration)
    node._scene_surface=original;node._scene_status['revision']=original.revision
    node._stage_scene_refinement('target',cloud,'World',result,{})
    surface=make_surface(messages[0]['measurement'],node.scene_registration)
    assert surface.revision==node._pending_scene_refinement['revision']
    np.testing.assert_allclose(np.asarray(surface.vertices_m)[:,2],4.02)
    node._scene_surface=None
    with pytest.raises(ValueError,match='support'):
        node._stage_scene_refinement('target',cloud,'World',result,{})


def test_capture_wait_allows_scene_update_but_active_path_does_not(monkeypatch):
    from sketch_control.moveit_executor import MoveItExecutor
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    executor=SimpleNamespace(executing=False,_d405_prescan_active=True)
    node._on_scene_activity(String(data=json.dumps({'active':MoveItExecutor._task_scene_active(executor)})))
    node._stage_scene_refinement('work_area',cloud,'World',result,{})
    assert messages and not accepted
    executor._active_trajectory_goal_handle=object()
    node._on_scene_activity(String(data=json.dumps({'active':MoveItExecutor._task_scene_active(executor)})))
    assert rejected and node._pending_scene_refinement is None
    with pytest.raises(ValueError,match='active'):
        node._stage_scene_refinement('work_area',cloud,'World',result,{})


def test_second_capture_cannot_replace_pending_scene_transaction(monkeypatch):
    node,cloud,result,messages,accepted,rejected=node_fixture(monkeypatch)
    node._stage_scene_refinement('work_area',cloud,'World',result,{})
    pending=node._pending_scene_refinement
    assert node._start_capture('work_area') is False
    assert node._pending_scene_refinement is pending and len(messages)==1
