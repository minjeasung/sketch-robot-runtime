from types import SimpleNamespace
import json,time
import numpy as np
import pytest
from sketch_control.moveit_executor import MoveItExecutor,_load_executor_objects_config


@pytest.mark.parametrize('source,name',[('task_scene','measured_task_surface'),('perception','perceived_task_surface')])
def test_explicit_dynamic_registry_starts_without_invented_geometry(tmp_path,source,name):
    path=tmp_path/'objects.yaml';path.write_text(f'active_target: {name}\nactive_target_source: {source}\nobjects: []\n')
    cfg=_load_executor_objects_config(str(path))
    assert cfg['objects']==[]
    node=SimpleNamespace(cfg=cfg,active_target_name=name,require_task_scene_ack=source=='task_scene',process_mode='paint',dynamic_surface_point=None,dynamic_surface_normal=None)
    target=MoveItExecutor._active_target(node)
    assert target=={'name':name,'source':source}
    with pytest.raises(ValueError,match='observed'):MoveItExecutor._active_surface_plane(node,target)
    assert MoveItExecutor._task_scene_blocker(node)
    node.dynamic_surface_point=np.array([0.,1.,2.]);node.dynamic_surface_normal=np.array([0.,-1.,0.])
    node._task_scene_status=dict(ready=True,surface_id='selected',revision='r',acks={'isaac':True,'moveit':True});node._task_scene_received_at=time.monotonic()
    assert not MoveItExecutor._task_scene_blocker(node)
    p,n=MoveItExecutor._active_surface_plane(node,target)
    np.testing.assert_allclose(p,[0,1,2]);np.testing.assert_allclose(n,[0,-1,0])
    assert cfg['objects']==[]


def test_dynamic_registry_rejects_missing_or_unrecognized_contract(tmp_path):
    for text in ('active_target: x\nobjects: []\n','active_target: x\nactive_target_source: imaginary\nobjects: []\n','active_target: wrong\nactive_target_source: task_scene\nobjects: []\n'):
        path=tmp_path/'objects.yaml';path.write_text(text)
        with pytest.raises(ValueError):_load_executor_objects_config(str(path))


def test_dynamic_target_never_reconstructed_from_commanded_waypoints():
    node=SimpleNamespace(cfg={'active_target':'measured_task_surface','active_target_source':'task_scene','objects':[]},process_mode='paint',dynamic_surface_point=None)
    MoveItExecutor._sync_active_target_plane_from_waypoints(node,[object()])
    assert node.dynamic_surface_point is None and node.cfg['objects']==[]


def scene_node(cfg):
    from concurrent.futures import Future
    from moveit_msgs.msg import AttachedCollisionObject
    from geometry_msgs.msg import Pose
    requests=[]
    def send(request):requests.append(request);return Future()
    def pose(position,orientation,transform):
        result=Pose()
        result.position.x,result.position.y,result.position.z=map(float,position)
        result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=map(float,orientation)
        return result
    logger=SimpleNamespace(info=lambda *a,**k:None,warn=lambda *a,**k:None)
    node=SimpleNamespace(cfg=cfg,active_target_name=cfg['active_target'],process_mode='weld',
        _joint_command_timer=None,scene_confirmed=False,scene_initialized=False,_stale_dynamic_obstacle_ids=set(),
        _lookup_transform_to_base=lambda *a,**k:np.eye(4),base_frame='World',dynamic_obstacles=[],
        _paint_eoat_collision_object=lambda:AttachedCollisionObject(),get_logger=lambda:logger,
        _transform_xyz_quat_to_pose=pose,
        apply_scene_client=SimpleNamespace(wait_for_service=lambda **k:True,call_async=send),
        dynamic_surface_point=np.array([-.04,-1.62,1.12]),dynamic_surface_normal=np.array([0.,1.,0.]),
        dynamic_work_area_corners=None,dynamic_surface_source='zed',
        _matched_perception_plane_extent=lambda *a:None,
        _dynamic_work_area_basis=lambda:(np.array([-.04,-1.62,1.12]),np.array([1.,0.,0.]),np.array([0.,0.,-1.]),.37,.396),
        _project_point_to_active_surface=lambda point,normal:point)
    node._dynamic_target_collision_object=lambda obj:MoveItExecutor._dynamic_target_collision_object(node,obj)
    return node,requests


def test_known_static_wall_keeps_full_edges_before_and_after_roi_refinement():
    from sketch_control.moveit_executor import WORLD_COLLISION_PADDING
    wall=dict(name='platform_painting_wall',shape='box',position=[0.,-1.65,1.1],size=[1.,.06,1.2],sketch_face='+y')
    node,requests=scene_node(dict(active_target=wall['name'],active_target_source='static',objects=[wall]))
    for source in ('zed','d405_refined'):
        node.dynamic_surface_source=source
        node._scene_apply_inflight_revision=None
        MoveItExecutor.publish_scene_periodic(node)
        collision=requests[-1].scene.world.collision_objects[0]
        assert collision.id==wall['name']
        np.testing.assert_allclose(collision.primitives[0].dimensions,np.asarray(wall['size'])+2*WORLD_COLLISION_PADDING)
        center=collision.primitive_poses[0].position
        np.testing.assert_allclose([center.x,center.y,center.z],wall['position'])
        # The measured ROI remains usable but cannot delete the known wall edge x=-.5.
        np.testing.assert_allclose(node.dynamic_surface_point,[-.04,-1.62,1.12])
        assert center.x-collision.primitives[0].dimensions[0]/2<=-.5


@pytest.mark.parametrize('source,name',[('task_scene','measured_task_surface'),('perception','perceived_task_surface')])
def test_unknown_surface_registry_never_adds_nominal_wall_collision(source,name):
    node,requests=scene_node(dict(active_target=name,active_target_source=source,objects=[]))
    MoveItExecutor.publish_scene_periodic(node)
    assert not requests[-1].scene.world.collision_objects


def test_legacy_nominal_registry_still_uses_observed_surface():
    wall=dict(name='wall',position=[0.,-1.65,1.1],size=[1.,.06,1.2],sketch_face='+y')
    node,requests=scene_node(dict(active_target='wall',objects=[wall]))
    MoveItExecutor.publish_scene_periodic(node)
    collision=requests[-1].scene.world.collision_objects[0]
    assert collision.primitive_poses[0].position.x==pytest.approx(-.04)


def test_commanded_waypoints_cannot_move_known_static_fixture_without_perception():
    from geometry_msgs.msg import Pose
    wall=dict(name='wall',position=[0.,-1.65,1.1],size=[1.,.06,1.2],sketch_face='+y')
    node,requests=scene_node(dict(active_target='wall',active_target_source='static',objects=[wall]))
    node.process_mode='paint';node.dynamic_surface_point=None
    node._infer_surface_normal_from_waypoints=lambda *args:np.array([0.,1.,0.])
    commanded=Pose();commanded.position.y=-.5
    MoveItExecutor._sync_active_target_plane_from_waypoints(node,[commanded])
    assert wall['position']==[0.,-1.65,1.1]
    assert node.dynamic_surface_point is None


@pytest.mark.parametrize('padding',[0.,.003])
def test_static_object_explicit_padding_preserves_full_pose_and_geometry(padding):
    wall=dict(name='wall',position=[0.,-1.65,1.1],size=[1.,.06,1.2],sketch_face='+y',collision_padding_m=padding)
    node,requests=scene_node(dict(active_target='wall',active_target_source='static',objects=[wall]))
    MoveItExecutor.publish_scene_periodic(node)
    collision=requests[-1].scene.world.collision_objects[0]
    np.testing.assert_allclose(collision.primitives[0].dimensions,np.array(wall['size'])+2*padding)
    center=collision.primitive_poses[0].position
    np.testing.assert_allclose([center.x,center.y,center.z],wall['position'])


@pytest.mark.parametrize('padding',[-.1,float('nan'),float('inf'),None,True,'invalid'])
def test_invalid_object_padding_rejected_before_scene_application(tmp_path,padding):
    import yaml
    path=tmp_path/'objects.yaml'
    path.write_text(yaml.safe_dump(dict(active_target='wall',active_target_source='static',objects=[
        dict(name='wall',position=[0.,-1.65,1.1],size=[1.,.06,1.2],sketch_face='+y',collision_padding_m=padding)])))
    with pytest.raises(ValueError,match='padding'):_load_executor_objects_config(str(path))


@pytest.mark.parametrize('platform,arm',[('commercial','left'),('commercial','right'),('rb10','single')])
def test_generated_simulator_contact_target_planning_geometry_matches_physics(tmp_path,platform,arm):
    import yaml
    from pathlib import Path
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    root=Path(__file__).resolve().parents[2]
    generate_urdf_assets(root,normalize_selection(dict(platform=platform,active_arm=arm,process='paint')),tmp_path)
    cfg=_load_executor_objects_config(str(tmp_path/'objects.yaml'))
    wall=json.loads((tmp_path/'scene.json').read_text())['wall']
    assert cfg['objects'][0]['collision_padding_m']==0.
    node,requests=scene_node(cfg)
    MoveItExecutor.publish_scene_periodic(node)
    collision=requests[-1].scene.world.collision_objects[0]
    assert collision.id==wall['id']
    np.testing.assert_allclose(collision.primitives[0].dimensions,wall['size'])
    center=collision.primitive_poses[0].position
    np.testing.assert_allclose([center.x,center.y,center.z],wall['center'])
