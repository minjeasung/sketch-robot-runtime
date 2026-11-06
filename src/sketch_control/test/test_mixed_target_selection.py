import json
from dataclasses import asdict,replace
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('rclpy')
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray,Pose
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import FrameBinding
from sketch_control.rgbd_weld_provider import RGBDFrameStore
from sketch_control.target_selector_node import TargetSelectorNode


def selected_node(monkeypatch):
    r=CameraRegistration('zed','123','physical','World',(0,0,0),(0,0,0,1),'r','fixed_measured')
    store=RGBDFrameStore(r);binding=FrameBinding('s',1,1,10.,1000000000,0,'r')
    identity=dict(frame_binding=asdict(binding),generation='s',frame_id='physical',width=32,height=32,clock_domain='ros',stamp={'sec':1,'nanosec':0})
    k=dict(fx=100.,fy=100.,cx=16.,cy=16.)
    store.add_depth(binding.sim_stamp,np.ones((32,32)));store.add_intrinsics(binding.sim_stamp,k);store.add_rgb(binding.sim_stamp,(32,32),rgb=np.full((32,32,3),37,np.uint8));store.add_identity(identity);store.pin(identity,10.2)
    newer=replace(binding,source_frame_id=2,receipt_monotonic=15.,sim_stamp=2000000000)
    store.add_identity({**identity,'stamp':{'sec':2,'nanosec':0},'frame_binding':asdict(newer)})
    node=object.__new__(TargetSelectorNode);node.physical_frames=store
    node.K=np.array([[100.,0,16],[0,100,16],[0,0,1]])
    node.latest_depth=np.full((32,32),4.);node.latest_depth_received_at=15.
    node.latest_depth_header=SimpleNamespace(frame_id='physical')
    node.get_logger=lambda:SimpleNamespace(warn=lambda *a,**k:None)
    published=[];node.catalog_pub=SimpleNamespace(publish=lambda msg:published.append(json.loads(msg.data)))
    captured=[]
    def extract(points,pixels,**kwargs):
        captured.append((points,kwargs['rgb']));return [{'center':points.mean(axis=0).tolist(),'normal':[0,0,-1]}]
    monkeypatch.setattr('sketch_control.target_selector_node.extract_planes',extract)
    monkeypatch.setattr('sketch_control.target_selector_node.time.monotonic',lambda:15.2)
    request={**identity,'polygon':[[0,0],[31,0],[31,31],[0,31]],'request_id':'123.456','target_revision':'123.456'}
    return node,request,published,captured


def test_mixed_planes_use_exact_pinned_rgbd_not_latest_depth(monkeypatch):
    node,request,published,captured=selected_node(monkeypatch)
    node._on_physical_selection(String(data=json.dumps(request)))
    assert len(captured)==1
    np.testing.assert_allclose(captured[0][0][:,2],1.)
    assert np.all(captured[0][1]==37)
    assert published[-1]['generation']=='123000000456'
    assert published[-1]['frame_binding']==request['frame_binding']


def test_missing_or_mismatched_pinned_frame_fails_without_latest_fallback(monkeypatch):
    node,request,published,captured=selected_node(monkeypatch)
    node.physical_frames.pinned=None;node.physical_frames.lease.clear()
    node._on_physical_selection(String(data=json.dumps(request)))
    assert not captured and published[-1]['planes']==[] and published[-1]['error']


def test_mixed_legacy_pixel_message_cannot_trigger_unbound_extraction(monkeypatch):
    node,request,published,captured=selected_node(monkeypatch)
    msg=PoseArray();msg.header.frame_id='zed_raw';msg.header.stamp.sec=123
    msg.poses=[Pose(),Pose(),Pose()]
    node._on_selection(msg)
    assert not captured and not published
