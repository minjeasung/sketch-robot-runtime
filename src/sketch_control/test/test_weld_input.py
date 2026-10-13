from dataclasses import replace
from pathlib import Path
import pytest
from test_weld_contract import target,catalog
from sketch_control.weld_input_node import load_fixture, check_frame_identity, normalize_result
F=Path(__file__).parents[1]/'config/construct_weld_fixture.json'

def test_target_uses_original_zed_frame_metadata():
    t=target(); metadata=dict(generation='g',width=1280,height=720,frame_id=t.frame_id,stamp={'sec':1,'nanosec':0},clock_domain='ros')
    check_frame_identity(t,metadata)
    with pytest.raises(ValueError): check_frame_identity(t,{**metadata,'generation':'old'})

def test_fixture_never_runs_for_external_or_real_mode():
    with pytest.raises(ValueError): load_fixture(F,replace(target(),provider='external'),'1')
    with pytest.raises(ValueError): load_fixture(F,target(),'1',simulation=False)
    assert len(load_fixture(F,target(),'1').seams)==3

def test_optical_frame_requires_transform_at_capture():
    p=catalog(); p['frame_id']='optical'
    with pytest.raises(ValueError): normalize_result(p,target(),'World',None)

def test_empty_result_stays_empty():
    p=catalog(); p['seams']=[]
    assert not normalize_result(p,target(),'World',None).seams

def test_external_transform_uses_the_targets_capture_stamp():
    import numpy as np
    p=catalog();p.update(frame_id='optical',clock_domain='ros',stamp={'sec':2,'nanosec':0})
    with pytest.raises(ValueError,match='capture stamp'):
        normalize_result(p,target(),'World',np.eye(4))
