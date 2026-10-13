import copy
from pathlib import Path
import pytest
from sketch_control.weld_contract import validate_target, validate_catalog, validate_settings, load_tool_profile

PROFILE = Path(__file__).parents[1] / 'config/weld_sim_profile.json'

def target():
    return validate_target(dict(request_id='r', target_revision='t', generation='g', frame_id='zed_left_camera_frame_optical',
        stamp={'sec':1,'nanosec':0}, clock_domain='ros', width=1280, height=720,
        polygon=[[10,10],[100,10],[100,100],[10,100]], provider='fixture'))

def catalog():
    return dict(schema_version=1, request_id='r', target_revision='t', generation='g', source='fixture',
                frame_id='World', length_unit='m', seams=[dict(seam_id='S1', points_xyz=[[0,0,0],[.1,0,0]])])

def test_xyz_only_seam_is_selectable():
    raw=catalog(); result=validate_catalog(raw,target()); raw['seams'][0]['points_xyz'][0][0]=99
    assert result.seams[0].points_xyz[0] == (0.,0.,0.)
    assert result.seams[0].reference_up_vectors == ()

@pytest.mark.parametrize('key,value',[('length_unit','mm'),('request_id','old'),('generation','old')])
def test_catalog_rejects_wrong_request_unit_duplicates_and_nonfinite(key,value):
    data=catalog(); data[key]=value
    with pytest.raises(ValueError): validate_catalog(data,target())

def test_duplicate_and_nonfinite_and_bad_target():
    data=catalog(); data['seams']*=2
    with pytest.raises(ValueError): validate_catalog(data,target())
    data=catalog(); data['seams'][0]['points_xyz'][0][0]=float('nan')
    with pytest.raises(ValueError): validate_catalog(data,target())
    data=catalog(); data['seams'][0]['points_xyz'][0][0]=True
    with pytest.raises(ValueError): validate_catalog(data,target())

def test_settings_reject_invalid_dwell_and_preserve_previous():
    profile=load_tool_profile(PROFILE,'construct_left')
    a=validate_settings({},profile)
    with pytest.raises(ValueError): validate_settings({'frequency_hz':1,'left_dwell_s':.5,'right_dwell_s':.5},profile)
    with pytest.raises(ValueError): validate_settings({'speed_mps':None},profile)
    assert validate_settings({'weave_enabled':False,'frequency_hz':0},profile).weave_enabled is False
    assert a.speed_mps > 0

def test_profile_rejects_wrong_model():
    with pytest.raises(ValueError): load_tool_profile(PROFILE,'rb10_1300e_u')

def test_malformed_objects_raise_validation_error():
    from sketch_control.weld_contract import validate_target,validate_catalog
    from dataclasses import asdict
    p=catalog();p['seams']=[None]
    with pytest.raises(ValueError):validate_catalog(p,target())
    p=asdict(target());p['stamp']=None
    with pytest.raises(ValueError):validate_target(p)
