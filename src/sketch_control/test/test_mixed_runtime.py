"""The mixed process graph must use physical input and simulated actuation."""
import importlib.util
from pathlib import Path
import os
import time
import json
import pytest


def launch_module():
    path=Path(__file__).parents[1]/'launch/platform_simulation.launch.py'
    spec=importlib.util.spec_from_file_location('platform_launch',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def make_bundle(tmp_path,platform,arm,process,mode='mixed'):
    model='construct_'+arm if platform=='commercial' else 'rb10_1300e_u'
    base='World' if platform=='commercial' else 'link0'
    camera=dict(camera_id='zed',serial='123',optical_frame='physical_zed_optical',target_frame=base,translation_m=[0,0,0],quaternion_xyzw=[0,0,0,1],revision='cal',pose_source='fixed_measured')
    calibration=tmp_path/'cal.json'; calibration.write_text(json.dumps({'cameras':{'zed':camera,'d405':{**camera,'camera_id':'d405','optical_frame':'physical_d405_optical'}}}))
    (tmp_path/'robot.urdf').write_text('<robot name="test"><ros2_control name="sim" type="system"><hardware><plugin>isaac_painting_hardware/IsaacPaintingSystem</plugin></hardware></ros2_control></robot>')
    (tmp_path/'robot.srdf').write_text('<robot name="test"/>')
    bundle=dict(revision='rev',selection=dict(platform=platform,active_arm=arm,process=process,execution_mode=mode),connections=dict(zed_calibration_file=str(calibration),d405_calibration_file=str(calibration),outpost_zed_hw_id='zed-hw',outpost_zed_serial='123',outpost_d405_hw_id='d-hw',outpost_d405_serial='123'),arms={arm:dict(model_id=model,base_frame=base,joints=['j'+str(i) for i in range(6)],group='arm',tcp_link='tcp',controller='jtc',wrench_frame='ft',contact_link='roller',camera_link='camera')})
    (tmp_path/'bundle.json').write_text(json.dumps(bundle))
    (tmp_path/'simulation-ready.json').write_text(json.dumps(dict(session_id='run',scene_revision='rev',joints_valid=True,wrench_valid=True,cameras_ready=True,updated_monotonic=time.monotonic(),pid=os.getpid())))
    return bundle


@pytest.mark.parametrize('platform,arm',[('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('process',['spray','weld','paint'])
def test_mixed_launch_uses_physical_cameras_and_selected_simulated_arm(tmp_path,platform,arm,process):
    make_bundle(tmp_path,platform,arm,process)
    contract=launch_module().build_contract(tmp_path,active_arm=arm,execution_mode='mixed',process_mode=process)
    assert contract['active']['joints']==['j0','j1','j2','j3','j4','j5']
    assert contract['camera_source']=='physical' and contract['weld_provider']=='external'
    assert contract['require_task_scene_ack'] is True
    assert contract['force_pipeline'] is (process=='paint')
    assert contract['camera_names']==(['zed','d405'] if process=='paint' else ['zed'])
    assert 'session_id' in contract['robot_description'] and 'run' in contract['robot_description']
    assert contract['base_frame']==('World' if platform=='commercial' else 'link0')


def test_launch_rejects_physical_hardware_and_mismatched_readiness(tmp_path):
    make_bundle(tmp_path,'rb10','single','paint')
    module=launch_module()
    p=tmp_path/'robot.urdf'; p.write_text(p.read_text().replace('isaac_painting_hardware/IsaacPaintingSystem','rbpodo/RBHardware'))
    with pytest.raises(ValueError,match='hardware'): module.build_contract(tmp_path)


def test_launch_rejects_wrong_revision_or_virtual_camera_registration(tmp_path):
    make_bundle(tmp_path,'commercial','left','spray')
    module=launch_module(); ready_path=tmp_path/'simulation-ready.json'; ready=json.loads(ready_path.read_text()); ready['scene_revision']='old'; ready_path.write_text(json.dumps(ready))
    with pytest.raises(ValueError,match='identity'): module.build_contract(tmp_path)
    ready['scene_revision']='rev'; ready_path.write_text(json.dumps(ready))
    calibration=tmp_path/'cal.json'; p=json.loads(calibration.read_text()); p['cameras']['zed']['serial']='stream://virtual'; calibration.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='physical'): module.build_contract(tmp_path)


def test_ros_graph_builds_with_actual_launch_nodes(tmp_path):
    pytest.importorskip('launch_ros')
    from launch_ros.actions import Node
    make_bundle(tmp_path,'commercial','left','paint')
    (tmp_path/'painting.yaml').write_text('{}')
    (tmp_path/'controllers.yaml').write_text('{}')
    actions=launch_module().build_actions(tmp_path)
    nodes=[a for a in actions if isinstance(a,Node)]
    assert len(nodes)==4  # state publisher, MoveIt, manager, gated spawner


def test_position_readiness_observes_only_selected_controller():
    from types import SimpleNamespace
    path=Path(__file__).resolve().parents[3]/'scripts/construct_position_state.py'
    spec=importlib.util.spec_from_file_location('position_state',path); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    active=SimpleNamespace(name='left_manipulator_controller',state='active',type='joint_trajectory_controller/JointTrajectoryController')
    assert module.observed_compliance([active],required_controllers=['left_manipulator_controller']) is False
    assert module.observed_compliance([active],required_controllers=['right_manipulator_controller']) is None


def test_generated_empty_arrays_are_loaded_as_yaml_not_untyped_launch_substitutions(tmp_path):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.utilities import normalize_parameters,evaluate_parameters
    config=tmp_path/'painting.yaml';config.write_text('moveit_executor:\n  ros__parameters:\n    roller_tool_links: []\n')
    layers=launch_module().executor_parameter_layers(config,{'process_mode':'spray'})
    evaluated=evaluate_parameters(LaunchContext(),normalize_parameters(layers))
    assert len(evaluated)==1 and isinstance(evaluated[0],Path)
    import yaml
    values=yaml.safe_load(evaluated[0].read_text())['moveit_executor']['ros__parameters']
    assert values['roller_tool_links']==[] and values['process_mode']=='spray'


def test_paint_controller_and_force_yaml_resolve_in_active_arm_namespace(tmp_path):
    pytest.importorskip('rclpy')
    from rclpy.parameter import parameter_dict_from_yaml_file
    source=tmp_path/'controllers.yaml'
    source.write_text('controller_manager:\n  ros__parameters:\n    update_rate: 100\njtc:\n  ros__parameters:\n    joints: [j0, j1]\n')
    scoped=launch_module().namespaced_parameter_file(source,'/platform/left')
    params=parameter_dict_from_yaml_file(str(scoped),target_nodes=['/platform/left/controller_manager'])
    assert params['update_rate'].value.integer_value==100
    params=parameter_dict_from_yaml_file(str(scoped),target_nodes=['/platform/left/jtc'])
    assert params['joints'].value.string_array_value==['j0','j1']


def test_mixed_d405_uses_physical_hash_and_capture_without_virtual_arrival(tmp_path):
    import yaml
    bundle=make_bundle(tmp_path,'commercial','left','paint')
    source=tmp_path/'painting.yaml';source.write_text('d405_surface_refiner:\n  ros__parameters:\n    calibration_file: virtual_hand_eye.json\n    max_tf_age_s: 0.25\nwall_projector:\n  ros__parameters:\n    d405_optical_frame: d405_color_optical_frame\n')
    module=launch_module();contract=module.build_contract(tmp_path)
    configured=module.perception_parameter_file(source,contract)
    parameters=yaml.safe_load(configured.read_text())
    refiner=parameters['d405_surface_refiner']['ros__parameters']
    assert refiner['calibration_file']==bundle['connections']['d405_calibration_file']
    assert refiner['scene_registration_file']==bundle['connections']['d405_calibration_file']
    selector=parameters['target_selector']['ros__parameters']
    assert selector['require_bound_frame'] is True
    assert selector['camera_registration_file']==bundle['connections']['zed_calibration_file']
    assert refiner['defer_target_capture_until_arrival'] is False
    assert refiner['max_tf_age_s']==.25
    assert parameters['wall_projector']['ros__parameters']['d405_optical_frame']=='physical_d405_optical'


def test_fixed_physical_d405_refinement_tf_ignores_virtual_wrist_motion():
    pytest.importorskip('rclpy')
    from tf2_ros import Buffer
    from geometry_msgs.msg import TransformStamped
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace
    from sketch_control.d405_surface_refiner_node import D405SurfaceRefinerNode
    buffer=Buffer()
    def transform(child,x):
        value=TransformStamped();value.header.frame_id='World';value.child_frame_id=child
        value.transform.translation.x=float(x);value.transform.rotation.w=1.;return value
    buffer.set_transform_static(transform('physical_zed_optical',1),'measured')
    buffer.set_transform_static(transform('physical_d405_optical',2),'measured')
    node=SimpleNamespace(tf_buffer=buffer)
    for wrist in (100.,1000.):
        buffer.set_transform_static(transform('virtual_wrist',wrist),'simulator')
        result,age=D405SurfaceRefinerNode._lookup_transform(node,'physical_zed_optical','physical_d405_optical',Time(sec=42))
        assert result.transform.translation.x==1. and age==0.


def test_scoped_controller_yaml_parses_in_ros_c_parameter_parser_without_aliases(tmp_path):
    pytest.importorskip('rclpy')
    from rclpy.context import Context
    source=tmp_path/'controllers.yaml'
    source.write_text('first:\n  ros__parameters:\n    joints: &joints [j0,j1]\nsecond:\n  ros__parameters:\n    joints: *joints\n')
    scoped=launch_module().namespaced_parameter_file(source,'/platform/left')
    context=Context()
    try: context.init(args=['--ros-args','--params-file',str(scoped)])
    finally:
        if context.ok(): context.shutdown()


def test_executor_specific_yaml_cannot_override_runtime_selection(tmp_path):
    pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.node import Node
    source=tmp_path/'painting.yaml'
    source.write_text('moveit_executor:\n  ros__parameters:\n    spray_tool_axis: ""\n    process_mode: paint\n    follow_joint_trajectory_action: /joint_trajectory_controller/follow_joint_trajectory\n')
    layers=launch_module().executor_parameter_layers(source,{'spray_tool_axis':'+z','process_mode':'spray','follow_joint_trajectory_action':'/platform/left/left_controller/follow_joint_trajectory'})
    context=Context();context.init(args=[]);node=None
    try:
        args=[]
        for layer in layers:
            if isinstance(layer,str):args.extend(['--params-file',layer])
        # Match launch's actual separate wildcard override file behavior.
        import yaml
        for index,layer in enumerate(layers):
            if isinstance(layer,dict):
                path=tmp_path/f'override-{index}.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':layer}}));args.extend(['--params-file',str(path)])
        node=Node('moveit_executor',context=context,cli_args=['--ros-args',*args],automatically_declare_parameters_from_overrides=True)
        assert node.get_parameter('spray_tool_axis').value=='+z'
        assert node.get_parameter('process_mode').value=='spray'
        assert node.get_parameter('follow_joint_trajectory_action').value=='/platform/left/left_controller/follow_joint_trajectory'
    finally:
        if node:node.destroy_node()
        context.shutdown()


def test_simulation_spray_perception_file_resolves_actual_process_and_zed(tmp_path):
    import yaml
    make_bundle(tmp_path,'commercial','left','spray',mode='simulation')
    source=tmp_path/'painting.yaml';source.write_text('wall_projector:\n  ros__parameters:\n    process_mode: paint\n    front_view_source: d405\nsketch_to_waypoints:\n  ros__parameters:\n    spray_tool_axis: ""\n')
    path=launch_module().perception_parameter_file(source,launch_module().build_contract(tmp_path))
    data=yaml.safe_load(path.read_text())
    assert data['wall_projector']['ros__parameters']['process_mode']=='spray'
    assert data['wall_projector']['ros__parameters']['front_view_source']=='zed'
    assert data['sketch_to_waypoints']['ros__parameters']['spray_tool_axis']=='+z'


def test_startup_waits_for_fresh_sensor_snapshot_without_accepting_old_identity(tmp_path,monkeypatch):
    make_bundle(tmp_path,'commercial','left','paint',mode='simulation');m=launch_module()
    path=tmp_path/'simulation-ready.json';ready=json.loads(path.read_text());ready['cameras_ready']=False;path.write_text(json.dumps(ready))
    sleeps=[]
    def next_frame(delay):
        sleeps.append(delay);ready['cameras_ready']=True;ready['updated_monotonic']=time.monotonic();path.write_text(json.dumps(ready))
    monkeypatch.setattr(m.time,'sleep',next_frame)
    assert m.build_contract(tmp_path,readiness_timeout=1.)['ready']['cameras_ready'] is True
    assert len(sleeps)==1
    ready['scene_revision']='old';path.write_text(json.dumps(ready))
    with pytest.raises(ValueError,match='identity'):m.build_contract(tmp_path,readiness_timeout=1.)
    assert len(sleeps)==1


@pytest.mark.parametrize('mode', ['simulation', 'mixed'])
@pytest.mark.parametrize('process', ['paint', 'spray'])
@pytest.mark.parametrize('platform,arm', [('commercial', 'left'), ('commercial', 'right'), ('rb10', 'single')])
def test_only_platform_paint_executor_subscribes_to_selected_nominal_controller(tmp_path, monkeypatch, mode, process, platform, arm):
    pytest.importorskip('launch_ros')
    from launch_ros.actions import Node
    make_bundle(tmp_path, platform, arm, process, mode)
    (tmp_path/'calibration').mkdir()
    for camera, key in [('zed', 'T_world_zed_optical'), ('d405', 'T_d405_optical_to_tcp')]:
        (tmp_path/'calibration'/f'{camera}.json').write_text(json.dumps({key: dict(translation=[0, 0, 0], rotation_xyzw=[0, 0, 0, 1])}))
    (tmp_path/'painting.yaml').write_text('{}')
    (tmp_path/'controllers.yaml').write_text('{}')
    captured = []
    original_init = Node.__init__

    def capture(self, *args, **kwargs):
        if kwargs.get('executable') == 'moveit_executor':
            captured.append(kwargs['parameters'])
        original_init(self, *args, **kwargs)

    monkeypatch.setattr(Node, '__init__', capture)
    launch_module().build_actions(tmp_path)
    assert len(captured) == 1
    import yaml
    params = yaml.safe_load(Path(captured[0][0]).read_text())['moveit_executor']['ros__parameters']
    if process == 'paint':
        assert params['paint_nominal_reference_topic'] == '/platform/' + arm + '/jtc/controller_state'
    else:
        assert not params.get('paint_nominal_reference_topic')
