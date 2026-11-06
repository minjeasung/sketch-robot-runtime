from types import SimpleNamespace
import numpy as np
from geometry_msgs.msg import PoseArray,Pose
from sketch_control.wall_projector_node import WallProjectorNode


def test_front_selection_in_zed_view_retains_source_projection():
    msg=PoseArray();msg.header.frame_id='wall_front'
    for x,y in [(0,0),(399,0),(399,399),(0,399)]:
        p=Pose();p.position.x=float(x);p.position.y=float(y);msg.poses.append(p)
    node=SimpleNamespace(front_view_source='zed',front_view_size=(400,400),
        front_view_extent=np.array([[-.2,-.2,1],[.2,-.2,1],[.2,.2,1],[-.2,.2,1]]),
        K=np.array([[400,0,320],[0,400,240],[0,0,1]]),
        latest_surface=(np.array([0,0,1]),np.array([0,0,-1]),'camera','target'),
        _clear_work_area_geometry_only=lambda:None,_begin_work_area_selection=lambda *a:None,
        _work_area_id='test',_publish_corners=lambda *a:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:msg.header.stamp)),
        get_logger=lambda:SimpleNamespace(info=lambda *a:None,warn=lambda *a:None))
    WallProjectorNode._lock_work_area_from_wall_front(node,msg)
    np.testing.assert_allclose(node.locked_work_area['src_pts'],[[240,160],[400,160],[400,320],[240,320]])

def test_extent_labels_the_frame_used_for_its_geometry():
    output=[];msg=PoseArray()
    node=SimpleNamespace(latest_surface=(None,None,'new_frame',None),front_extent_pub=SimpleNamespace(publish=output.append))
    WallProjectorNode._publish_front_view_extent(node,np.zeros((4,3)),msg.header.stamp,frame_id='geometry_frame')
    assert output[0].header.frame_id=='geometry_frame'
