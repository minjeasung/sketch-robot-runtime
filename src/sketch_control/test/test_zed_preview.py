"""Camera-only workflow: real ROS node, isolated domain, no hardware clients."""
import json
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String


@pytest.fixture
def preview():
    from sketch_control.zed_preview_node import ZedPreviewNode
    context = Context()
    rclpy.init(context=context, domain_id=226)
    node = ZedPreviewNode(context=context)
    yield node
    node.destroy_node()
    rclpy.shutdown(context=context)


def msg(data):
    return String(data=json.dumps(data))


def select_target(node, polygon=False):
    selection = PoseArray()
    selection.header.stamp.sec = 12
    node.on_zed_target_selection(selection)
    node._multi_on_catalog(msg(dict(generation='12000000000', frame_id='link0', planes=[
        dict(id='plane-1', center=[0., 0., 1.], normal=[0., 0., -1.],
             corners=[[-.5,-.5,1.],[.5,-.5,1.],[.5,.5,1.],[-.5,.5,1.]],
             inlier_count=1000, rms_m=.002)])))
    node._multi_on_select(msg(dict(generation='12000000000', ids=['plane-1'])))
    assert node._zed_target_lock['accepted'] is True
    selection.header.frame_id = 'wall_front'
    selection.header.stamp.sec = 13
    selection.poses = [Pose() for _ in range(3 if polygon else 4)]
    node.on_zed_area_pixels(selection)
    lock = node._zed_target_lock
    area = dict(source='zed', mode='work_area', state='locked', accepted=True,
                selection_id='13000000000', work_area_id='area-1',
                plane_generation_id=lock['plane_generation_id'], target_stamp=lock['stamp'],
                frame_id='link0', position=[0.,0.,1.], orientation=[1.,0.,0.,0.],
                corners=[[-.2,-.2,1.],[.2,-.2,1.],[.2,.2,1.],[-.2,.2,1.]])
    if polygon:
        area.update(boundary_pixels=[[0,0],[100,0],[50,100]],
                    front_extent=area['corners'], view_width=101, view_height=101)
    node.on_zed_surface_status(msg(area))
    assert node._zed_plane_accepted
    return area


@pytest.mark.parametrize('polygon', [False, True])
def test_preview_can_select_and_generate_without_joint_state_or_force(preview, polygon):
    area = select_target(preview, polygon)
    plan = dict(state='generated', process_mode='spray', path_id='path-1', plan_hash='hash-1',
                work_area_id=area['work_area_id'], plane_generation_id=area['plane_generation_id'])
    preview._on_plan_status(msg(plan))
    state = preview.readiness_payload()
    assert state['planning_only'] is True
    assert state['checks']['current_plan_generated'] is True
    assert state['checks']['current_plan_validated'] is False
    assert state['ready'] is False and state['running'] is False
    assert state['path_id'] == 'path-1'
    assert not hasattr(preview, 'traj_action_client')
    publishers = {name for name, _ in preview.get_publisher_names_and_types_by_node(preview.get_name(), '/')}
    assert not publishers & {'/motion_abort', '/spray_gun/command', '/sketch_execute',
                             '/joint_trajectory_controller/joint_trajectory', '/d405/refine_target_request'}


def test_reselection_discards_preview_and_late_old_plan(preview):
    area = select_target(preview)
    plan = dict(state='generated', process_mode='spray', path_id='path-1', plan_hash='hash-1',
                work_area_id=area['work_area_id'], plane_generation_id=area['plane_generation_id'])
    preview._on_plan_status(msg(plan))
    new_selection = PoseArray()
    new_selection.header.stamp.sec = 14
    preview.on_zed_target_selection(new_selection)
    preview._on_plan_status(msg(plan))
    state = preview.readiness_payload()
    assert not state['checks']['current_plan_generated']
    assert not state['path_id']


def test_preview_rejects_paint_mode_and_ignores_malformed_plan(preview):
    preview._on_set_mode(String(data='paint'))
    assert preview.process_mode == 'spray'
    for value in ('null', '[]', '{bad'):
        preview._on_plan_status(String(data=value))
    assert preview.readiness_payload()['ready'] is False
