"""Selected-arm measured sensor wiring; these fixtures do not claim Isaac validation."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import rb20_painting_sensors as sensors


def arm(side='left'):
    return dict(tool_id='roller',joints=[f'{side}_joint{i}' for i in range(6)],
                sensor_joint=f'{side}_roller_ft_sensor_joint',wrench_frame=f'{side}_roller_ft_link',
                contact_link=f'{side}_roller_contact',base_frame='World',tcp_link=f'{side}_tcp',
                tool_revision='tool-1')


def contract(side='left'):
    assert hasattr(sensors,'RollerSensorContract'), 'Selected-arm sensor contract is not implemented'
    return sensors.RollerSensorContract.from_arm_contract(
        arm(side),session_id=f'run:{side}',scene_revision='assets:scene',
        topic_namespace=f'/platform/{side}/isaac_painting')


def test_selected_joints_reorder_without_accepting_other_arm():
    selected=contract()
    assert selected.joint_indices(['head']+arm('right')['joints']+list(reversed(arm()['joints'])))==[12,11,10,9,8,7]
    with pytest.raises(ValueError,match='joint'):
        selected.joint_indices(arm('right')['joints'])
    with pytest.raises(ValueError,match='joint'):
        selected.joint_indices(arm()['joints']+['left_joint0'])
    assert selected.topic('state')=='/platform/left/isaac_painting/state'
    assert contract('right').topic('command')=='/platform/right/isaac_painting/command'


def test_physics_boundary_and_measured_reaction_use_selected_sensor_frame():
    selected=contract()
    metadata=SimpleNamespace(joint_indices={'right_roller_ft_sensor_joint':2,'left_roller_ft_sensor_joint':7},
                             link_indices={'right_roller_ft_link':3,'left_roller_ft_link':9})
    assert selected.physics_indices(metadata)==(8,9)
    tf=np.eye(4);tf[:3,3]=[1,0,0]
    raw,gravity=selected.measure_wrench([0,-2,0,0,0,0],tf,np.eye(3),
                                      {'mass_kg':2.,'cog_ft_m':[1,0,0]})
    np.testing.assert_allclose(raw,[0,2,0,0,0,2])
    np.testing.assert_allclose(gravity,[0,0,-19.62,0,19.62,0])
    with pytest.raises(ValueError):
        selected.measure_wrench([float('nan')]*6,tf,np.eye(3),{'mass_kg':2.,'cog_ft_m':[1,0,0]})
    with pytest.raises(ValueError):
        selected.physics_indices(SimpleNamespace(joint_indices={},link_indices={}))


def test_arm_session_and_scene_revision_cannot_reuse_old_command():
    selected=contract()
    lease=selected.command_lease()
    lease.record_state(1,0,10.)
    cmd=SimpleNamespace(session_id='run:right',scene_revision='assets:scene',sequence=1,
                        enabled=True,positions=[.5]*6,header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)))
    assert not lease.accept(cmd,10.01)
    cmd.session_id='run:left';cmd.scene_revision='old-assets:scene'
    assert not lease.accept(cmd,10.01)
    cmd.scene_revision='assets:scene'
    assert lease.accept(cmd,10.01)
    np.testing.assert_allclose(lease.target,[.5]*6)
    assert not lease.active(10.201)
    cmd.sequence=2
    assert not lease.accept(cmd,10.201)


def test_nonroller_or_incomplete_arm_is_not_force_capable():
    selected=contract()
    assert selected.wrench_frame=='left_roller_ft_link'
    for changed in ({'tool_id':'spray_v2'},{'sensor_joint':''},{'joints':['same']*6},{'tool_revision':''}):
        with pytest.raises(ValueError):
            sensors.RollerSensorContract.from_arm_contract({**arm(),**changed},
                session_id='run:left',scene_revision='rev',topic_namespace='/platform/left/isaac_painting')


def test_only_active_roller_target_pair_is_allowed():
    pytest.importorskip('rclpy')
    sys.path.insert(0,str(ROOT/'src/sketch_control/test'))
    from test_contact_collision_acm import _executor,_full_acm,_get_response,_apply_response,moveit_executor
    node,_,pairs=_executor()
    node.roller_contact_link='left_roller_contact'
    baseline=_full_acm(pairs,target_pair=('right_roller_contact','wall'))
    node._set_contact_collision_allowed(True)
    node.get_planning_scene_client.futures[0].resolve(_get_response(baseline))
    allowed=node.apply_scene_client.requests[0].scene.allowed_collision_matrix
    executor=moveit_executor.MoveItExecutor
    assert executor._acm_effective_pair_value(allowed,'left_roller_contact','wall')
    assert not executor._acm_effective_pair_value(allowed,'right_roller_contact','wall')
    assert not executor._acm_effective_pair_value(allowed,moveit_executor.ROLLER_CONTACT_LINK,'wall')
    node.apply_scene_client.futures[0].resolve(_apply_response())
    node.get_planning_scene_client.futures[1].resolve(_get_response(allowed))
    node._set_contact_collision_allowed(False)
    restored=node.apply_scene_client.requests[1].scene.allowed_collision_matrix
    assert executor._acm_equal(restored,baseline)


def test_rendered_camera_selection_omits_unused_d405_and_uses_mounted_frame(monkeypatch):
    # Isaac Camera creation is the external boundary; exercise selection/pose math.
    class Camera:
        def __init__(self,*args,**kwargs): self.pose=None
        def set_focal_length(self,*args): pass
        def set_horizontal_aperture(self,*args): pass
        def set_clipping_range(self,*args): pass
        def initialize(self): pass
        def add_distance_to_image_plane_to_frame(self): pass
        def set_world_pose(self,**kwargs): self.pose=kwargs
    monkeypatch.setitem(sys.modules,'isaacsim.sensors.camera',SimpleNamespace(Camera=Camera))
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(Image=object,CameraInfo=object))
    node=SimpleNamespace(create_publisher=lambda *args:None)
    scene={'camera_resolution':[64,48],'zed_world_optical':np.eye(4),'camera_names':['zed']}
    cameras=sensors.RenderedCameras(node,{},scene)
    assert set(cameras.cameras)=={'zed'}
    cameras.update_poses(None)
    np.testing.assert_allclose(cameras.cameras['zed'][0].pose['position'],[0,0,0])
    scene.update(camera_names=['d405'],camera_frames={'d405':'left_d405_optical'})
    mounted=np.eye(4);mounted[:3,3]=[.1,.2,.3]
    cameras=sensors.RenderedCameras(node,{'camera_ft_transforms':{'left_d405_optical':mounted}},scene)
    world=np.eye(4);world[:3,3]=[1,2,3]
    cameras.update_poses(world)
    assert cameras.cameras['d405'][1]=='left_d405_optical'
    np.testing.assert_allclose(cameras.cameras['d405'][0].pose['position'],[1.1,2.2,3.3])


def test_roller_collider_selection_is_exact_and_arm_scoped():
    from rb20_painting_scene import select_roller_collision
    name='left_roller_contact_collision'
    selected=f'/World/Robot/left_ft/collisions/{name}'
    decoys=[f'/World/RobotOther/left_ft/collisions/{name}',
            f'/World/Robot/right_ft/collisions/{name}',selected+'_other']
    assert select_roller_collision(decoys+[selected],robot_root='/World/Robot',
        collision_name=name,body_name='left_ft')==selected
    with pytest.raises(ValueError,match='exactly one'):
        select_roller_collision(decoys,robot_root='/World/Robot',collision_name=name,body_name='left_ft')


@pytest.mark.parametrize('session',['session-1',None])
def test_rendered_camera_preview_identity_matches_actual_frame(monkeypatch,session):
    import json
    class Message:
        def __init__(self,**kwargs):
            self.header=SimpleNamespace()
            self.__dict__.update(kwargs)
    class Camera:
        def __init__(self,*a,**kw): pass
        def __getattr__(self,name): return lambda *a,**kw:None
        def get_current_frame(self): return dict(rendering_frame=7,rendering_time=2.25,distance_to_image_plane=np.ones((2,3)))
        def get_rgba(self): return np.full((2,3,4),125,dtype=np.uint8)
        def get_intrinsics_matrix(self): return np.eye(3)
    published={}; previews=[]
    def publisher(kind,topic,qos):
        published[topic]=[]
        return SimpleNamespace(publish=published[topic].append)
    class Preview:
        def __init__(self,node,topic): self.topic=topic
        def publish(self,rgb,header): previews.append((self.topic,rgb.copy(),header))
    monkeypatch.setitem(sys.modules,'isaacsim.sensors.camera',SimpleNamespace(Camera=Camera))
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(Image=Message,CameraInfo=Message))
    monkeypatch.setitem(sys.modules,'builtin_interfaces.msg',SimpleNamespace(Time=Message))
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=Message))
    monkeypatch.setitem(sys.modules,'sketch_control.image_preview',SimpleNamespace(ImagePreview=Preview))
    scene=dict(camera_resolution=[3,2],camera_names=['zed'],zed_world_optical=np.eye(4))
    cameras=sensors.RenderedCameras(SimpleNamespace(create_publisher=publisher),
        {'session_id':session} if session else {},scene)
    cameras.publish(10.)
    topic='/perception/zed_frame_identity'
    if session:
        assert json.loads(published[topic][0].data)==dict(generation=session,
            frame_id='zed_left_camera_frame_optical',stamp=dict(sec=2,nanosec=250000000),
            clock_domain='ros',width=3,height=2,valid=True,source='simulation',camera_id='zed')
        assert previews[0][0]==sensors.CAMERA_TOPICS['zed'][0]
        assert previews[0][1].shape==(2,3,3)
        assert previews[0][2] is published[sensors.CAMERA_TOPICS['zed'][0]][0].header
        cameras.publish(10.1)
        assert len(previews)==1 and len(published[topic])==1
        cameras.invalidate()
        assert json.loads(published[topic][-1].data)['valid'] is False
    else:
        assert topic not in published and not previews


def test_generated_limits_select_active_arm_from_full_platform(tmp_path):
    pytest.importorskip('rclpy')
    from sketch_control.moveit_executor import _generated_joint_limits
    selected=tuple(f'left_joint{i}' for i in range(6))
    joints=''.join(f'<joint name="{name}" type="revolute"><limit lower="-2" upper="2"/></joint>' for name in selected)
    path=tmp_path/'platform.urdf'
    path.write_text('<robot>'+joints+'<joint name="inactive_joint" type="continuous"/></robot>')
    assert _generated_joint_limits(str(path),selected)==dict.fromkeys(selected,(-2.,2.))
    for extra in ('<joint name="left_joint0" type="fixed"/>',
                  '<joint name="left_joint0" type="revolute"><limit lower="-2" upper="2"/></joint>'):
        path.write_text('<robot>'+joints+extra+'</robot>')
        with pytest.raises(ValueError,match='joint names'):
            _generated_joint_limits(str(path),selected)
    path.write_text('<robot>'+joints.replace('name="left_joint5"','name="right_joint5"')+'</robot>')
    with pytest.raises(ValueError,match='joint names'):
        _generated_joint_limits(str(path),selected)
    path.write_text('<robot>'+joints.replace('lower="-2"','lower="nan"',1)+'</robot>')
    with pytest.raises(ValueError,match='joint limits'):
        _generated_joint_limits(str(path),selected)


def test_generic_roller_material_applies_measured_spring_only_to_selected_collider(monkeypatch):
    from rb20_painting_scene import configure_roller_contact
    writes=[]
    class Prim:
        def __init__(self,path): self.path=path
        def GetPath(self): return self.path
        def HasAPI(self,api): return True
        def IsInstanceProxy(self): return False
    selected=Prim('/World/Robot/left_ft/collisions/left_roller_collision')
    other=Prim('/World/Robot/right_ft/collisions/right_roller_collision')
    stage=SimpleNamespace(GetPrimAtPath=lambda path:next(p for p in (selected,other) if p.path==path))
    class API:
        def __init__(self,prim): self.prim=prim
        @classmethod
        def Apply(cls,prim): return cls(prim)
        def __getattr__(self,name):
            return lambda *values:writes.append((self.prim.path,name,values))
    class Material:
        @staticmethod
        def Define(stage,path):
            return SimpleNamespace(GetPrim=lambda:Prim(path),GetPath=lambda:path)
    pxr=SimpleNamespace(Usd=SimpleNamespace(PrimRange=SimpleNamespace(Stage=lambda *a:[other,selected]),
                                           TraverseInstanceProxies=lambda:None),
        UsdPhysics=SimpleNamespace(CollisionAPI=object,MaterialAPI=API),
        PhysxSchema=SimpleNamespace(PhysxMaterialAPI=API),
        UsdShade=SimpleNamespace(Material=Material,MaterialBindingAPI=API,
                                Tokens=SimpleNamespace(strongerThanDescendants='strong')))
    monkeypatch.setitem(sys.modules,'pxr',pxr)
    result=configure_roller_contact(stage,dict(acceleration_spring=False,stiffness_n_per_m=5000.,damping_ns_per_m=50.),
        dict(static_friction=.4,dynamic_friction=.3,restitution=0.),robot_root='/World/Robot',
        collision_name='left_roller_collision',body_name='left_ft',material_path='/World/LeftMaterial')
    assert result['collider']==selected.path and result['material']=='/World/LeftMaterial'
    assert ('/World/LeftMaterial','CreateCompliantContactStiffnessAttr',(5000.,)) in writes
    assert ('/World/LeftMaterial','CreateCompliantContactAccelerationSpringAttr',(False,)) in writes
    bindings=[entry for entry in writes if entry[1]=='Bind']
    assert len(bindings)==1 and bindings[0][0]==selected.path
    assert bindings[0][2][1:]==('strong','physics')


@pytest.mark.parametrize('platform,arm',[('rb10','single'),('commercial','left'),('commercial','right')])
def test_generated_native_spray_uses_existing_collision_only(tmp_path,platform,arm):
    pytest.importorskip('rclpy')
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from sketch_control.moveit_executor import _generated_native_spray_tool,MoveItExecutor
    b=generate_urdf_assets(ROOT,normalize_selection(dict(platform=platform,active_arm=arm,process='spray')),tmp_path)
    a=b['arms'][arm]
    assert _generated_native_spray_tool(b['urdf'],a['model_id']) is True
    node=SimpleNamespace(model_id=a['model_id'],_generated_native_spray_tool=True)
    assert MoveItExecutor._spray_eoat_collision_object(node,None) is None


def test_generated_native_spray_rejects_wrong_hardware_and_detached_tool(tmp_path):
    pytest.importorskip('rclpy')
    import xml.etree.ElementTree as ET
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from sketch_control.moveit_executor import _generated_native_spray_tool
    b=generate_urdf_assets(ROOT,normalize_selection(dict(platform='rb10',process='spray')),tmp_path)
    path=Path(b['urdf']);original=path.read_text()
    assert _generated_native_spray_tool('', 'rb10_1300e_u') is False
    root=ET.fromstring(original);root.find('ros2_control/hardware/plugin').text='rbpodo_hardware/RBSystem'
    path.write_text(ET.tostring(root,encoding='unicode'))
    with pytest.raises(ValueError,match='simulation hardware'):
        _generated_native_spray_tool(str(path),'rb10_1300e_u')
    root=ET.fromstring(original)
    tool=root.find("link[@name='single_spray_v2_left_manipulator_link_ee']")
    tool.remove(tool.find('collision'))
    path.write_text(ET.tostring(root,encoding='unicode'))
    with pytest.raises(ValueError,match='native spray'):
        _generated_native_spray_tool(str(path),'rb10_1300e_u')
    root=ET.fromstring(original)
    joint=next(j for j in root.findall('joint') if j.find('child').get('link')=='single_spray_v2_left_manipulator_link_ee')
    joint.set('type','revolute');path.write_text(ET.tostring(root,encoding='unicode'))
    with pytest.raises(ValueError,match='native spray'):
        _generated_native_spray_tool(str(path),'rb10_1300e_u')

    root=ET.fromstring(original)
    joint=next(j for j in root.findall('joint') if j.find('child').get('link')=='tcp')
    joint.find('parent').set('link','link6');path.write_text(ET.tostring(root,encoding='unicode'))
    with pytest.raises(ValueError,match='native spray'):
        _generated_native_spray_tool(str(path),'rb10_1300e_u')


@pytest.mark.parametrize('tcp',['left_manipulator_ee_point','right_manipulator_ee_point','tcp'])
def test_d405_mount_lookup_uses_selected_tcp(tcp):
    pytest.importorskip('rclpy')
    import rclpy.time
    from sketch_control.d405_scan_selection import D405ScanSelectionMixin
    requested=[]
    transform=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=.1,y=.2,z=.3),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=2.)))
    def lookup(target,source,*args,**kwargs):
        requested.append((target,source))
        if target!=tcp:raise LookupError('wrong arm TCP')
        return transform
    node=SimpleNamespace(ee_link=tcp,tf_buffer=SimpleNamespace(lookup_transform=lookup))
    position,rotation=D405ScanSelectionMixin._d405_mount_transform(node)
    assert requested==[(tcp,'d405_color_optical_frame')]
    np.testing.assert_allclose(position,[.1,.2,.3])
    np.testing.assert_allclose(rotation,[0,0,0,1])


@pytest.mark.parametrize('side',['left','right'])
def test_d405_face_order_ik_targets_selected_arm(side):
    pytest.importorskip('rclpy')
    import time
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose
    from sketch_control.d405_scan_selection import D405ScanSelectionMixin
    requests=[]
    node=D405ScanSelectionMixin();context=dict(revision=1,started=time.monotonic(),phase='scene',candidates=[])
    node._multi_order_context=context;node._motion_abort_requested=False;node._scene_revision=1
    node.current_joint_state_time=time.monotonic();node.current_joint_state=JointState(name=[side+'_joint1'],position=[0.])
    node._d405_scene_revision_confirmed=lambda revision:True
    node._joint_state_within_limits=lambda *args:True;node._current_tcp_pose_np=lambda:([0,0,0],[0,0,0,1])
    node._d405_mount_transform=lambda:([0,0,0],[0,0,0,1]);node._multi_queue=[{}]
    node._d405_face_candidates=lambda *args,**kw:[dict(pose=Pose())]
    node._multi_cancel=lambda reason:pytest.fail(reason)
    node.ik_client=SimpleNamespace(wait_for_service=lambda **kw:True,
        call_async=lambda request:(requests.append(request) or SimpleNamespace(add_done_callback=lambda cb:None)))
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:JointState().header.stamp))
    node.planning_group=side+'_manipulator';node.ee_link=side+'_manipulator_ee_point';node.base_frame='World'
    node._multi_order_tick(context)
    request=requests[0].ik_request
    assert request.group_name==node.planning_group and request.ik_link_name==node.ee_link
    assert request.pose_stamped.header.frame_id=='World'
    assert request.avoid_collisions is True
    assert request.robot_state.joint_state.name==[side+'_joint1']
