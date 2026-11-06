from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest
from sketch_control.weld_overlay import project_catalog


def inputs():
    catalog=NS(generation='g',target_revision='t',request_id='r',source='fixture',
        seams=[NS(seam_id='s',points_xyz=[[0,0,2],[1,0,2]])])
    frame=dict(valid=True,generation='g',frame_id='optical',stamp=dict(sec=2,nanosec=3),width=640,height=480)
    info=dict(frame_id='optical',stamp=frame['stamp'],width=640,height=480,p=[400,0,320,0,0,400,240,0,0,0,1,0])
    return catalog,frame,info,np.eye(4)


def test_projects_real_3d_seam_with_capture_frame_and_intrinsics():
    c,f,i,t=inputs();t[0,3]=-.5
    result=project_catalog(c,f,i,t)
    assert result['target_revision']=='t' and result['generation']=='g'
    assert result['header']==dict(frame_id='optical',stamp=f['stamp'])
    assert result['seams'][0]['points_px']==[[220.,240.],[420.,240.]]


@pytest.mark.parametrize('change', ['generation','stamp','frame_id','width','invalid','intrinsics'])
def test_does_not_project_with_mismatched_or_invalid_camera_data(change):
    c,f,i,t=inputs()
    if change=='generation':f['generation']='old'
    elif change=='stamp':i['stamp']=dict(sec=1,nanosec=3)
    elif change=='frame_id':i['frame_id']='other'
    elif change=='width':i['width']=320
    elif change=='invalid':f['valid']=False
    else:i['p'][0]=0
    with pytest.raises(ValueError):project_catalog(c,f,i,t)


def test_behind_camera_points_break_line_instead_of_connecting_across_them():
    c,f,i,t=inputs();c.seams[0].points_xyz=[[0,0,2],[0,0,-1],[1,0,2]]
    assert project_catalog(c,f,i,t)['seams'][0]['points_px']==[[320.,240.],None,[520.,240.]]
