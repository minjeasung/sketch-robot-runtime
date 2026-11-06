"""Generated platform ROS graph. Actuation is exclusively simulator hardware."""
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def build_contract(assets,*,active_arm=None,execution_mode=None,process_mode=None,readiness_timeout=0.):
    from sketch_control.camera_registration import load_registration
    assets=Path(assets).resolve(); bundle=json.loads((assets/'bundle.json').read_text())
    selection=bundle['selection']
    for name,value in (('active_arm',active_arm),('execution_mode',execution_mode),('process',process_mode)):
        if value and value!=selection[name]: raise ValueError('launch selection differs from generated assets: '+name)
    mode=selection['execution_mode']; process=selection['process']; arm=selection['active_arm']
    if mode not in ('simulation','mixed'): raise ValueError('platform launcher supports simulated actuation only')
    active=bundle['arms'][arm]
    robot=ET.fromstring((assets/'robot.urdf').read_text())
    allowed={'topic_based_ros2_control/TopicBasedSystem','isaac_painting_hardware/IsaacPaintingSystem'}
    plugins=robot.findall('./ros2_control/hardware/plugin')
    if not plugins or any(p.text not in allowed for p in plugins): raise ValueError('physical or unknown hardware plugin forbidden')
    deadline=time.monotonic()+readiness_timeout;source_session=None
    while True:
        ready=json.loads((assets/'simulation-ready.json').read_text())
        if ready['scene_revision']!=bundle['revision'] or not ready['session_id']: raise ValueError('simulator readiness identity mismatch')
        if source_session is not None and source_session!=ready['session_id']: raise ValueError('simulator readiness identity changed')
        source_session=ready['session_id'];os.kill(int(ready['pid']),0)
        error=None
        if not 0<=time.monotonic()-float(ready['updated_monotonic'])<=2.:error='simulator readiness expired'
        elif ready['joints_valid'] is not True:error='simulator joints unavailable'
        elif process=='paint' and ready.get('wrench_valid') is not True:error='measured simulator wrench unavailable'
        elif mode=='simulation' and ready.get('cameras_ready') is not True:error='simulation cameras unavailable'
        if error is None:break
        if time.monotonic()>=deadline:raise ValueError(error)
        time.sleep(.05)
    for hardware in robot.findall('./ros2_control/hardware'):
        if hardware.findtext('plugin')=='isaac_painting_hardware/IsaacPaintingSystem':
            for key in ('session_id','scene_revision'):
                param=hardware.find("param[@name='%s']"%key)
                if param is None: param=ET.SubElement(hardware,'param',name=key)
                param.text=str(ready[key])
    cameras=['zed','d405'] if process=='paint' else ['zed']; registrations={}; connections=bundle.get('connections',{})
    if mode=='mixed':
        for camera in cameras:
            path=connections.get(camera+'_calibration_file','')
            if not path: raise ValueError('physical '+camera+' measured registration required')
            registration=load_registration(path,camera)
            if registration.target_frame!=active['base_frame']: raise ValueError('camera registration target differs from selected robot base')
            if registration.serial!=connections.get('outpost_'+camera+'_serial'): raise ValueError('camera registration serial mismatch')
            if not connections.get('outpost_'+camera+'_hw_id'): raise ValueError('physical camera hardware ID required')
            registrations[camera]=registration
    return dict(assets=assets,bundle=bundle,ready=ready,active=active,base_frame=active['base_frame'],robot_description=ET.tostring(robot,encoding='unicode'),camera_source='physical' if mode=='mixed' else 'simulation',weld_provider='external' if mode=='mixed' else 'fixture',require_task_scene_ack=mode=='mixed',force_pipeline=process=='paint',camera_names=cameras,registrations=registrations,control_namespace='/platform/'+arm if process=='paint' else '')


def moveit_parameters(contract):
    from ament_index_python.packages import get_package_share_directory
    import yaml
    assets=contract['assets']; bundle=contract['bundle']; active=contract['active']
    root=ET.fromstring(contract['robot_description']); limits={}
    for joint in root.findall('joint'):
        limit=joint.find('limit')
        if joint.get('type')=='fixed' or limit is None: continue
        limits[joint.get('name')]=dict(has_position_limits=True,min_position=float(limit.get('lower',-3.14)),max_position=float(limit.get('upper',3.14)),has_velocity_limits=True,max_velocity=float(limit.get('velocity',1.)),has_acceleration_limits=True,max_acceleration=1.)
    defaults=Path(get_package_share_directory('moveit_configs_utils'))/'default_configs/ompl_planning.yaml'
    pipeline=yaml.safe_load(defaults.read_text())
    pipeline['planner_configs']={'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.}}
    # Jazzy's response adapter reads ompl.totg.path_tolerance (default
    # 0.1 rad). All processes, including paint's D405 prescan, need bounded
    # corner rounding and dense edge checks near the original tool/workpiece.
    # Retain ValidateSolution after retiming and all original collisions.
    # These bounds match the existing Construct weld planning contract.
    pipeline['totg']={'path_tolerance':.001}
    pipeline['planner_configs']['RRTConnectkConfigDefault']['longest_valid_segment_fraction']=.0001
    kinematics={}
    for arm in bundle['arms'].values():
        pipeline[arm['group']]={'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':.0001}
        kinematics[arm['group']]={'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_search_resolution':.005,'kinematics_solver_timeout':.1}
    controller=contract['control_namespace']+'/'+active['controller']
    return dict(robot_description=contract['robot_description'],robot_description_semantic=(assets/'robot.srdf').read_text(),robot_description_kinematics=kinematics,robot_description_planning={'joint_limits':limits},planning_pipelines=['ompl'],default_planning_pipeline='ompl',ompl=pipeline,moveit_controller_manager='moveit_simple_controller_manager/MoveItSimpleControllerManager',moveit_simple_controller_manager={'controller_names':[controller],controller:{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':active['joints']}},moveit_manage_controllers=False,publish_robot_description=True,publish_robot_description_semantic=True,publish_planning_scene=True,publish_geometry_updates=True,publish_state_updates=True,publish_transforms_updates=True,use_sim_time=True)


def ros_yaml_dump(data):
    import yaml
    class NoAliases(yaml.SafeDumper):
        def ignore_aliases(self,_value): return True
    return yaml.dump(data,Dumper=NoAliases)


def perception_parameter_file(source,contract):
    import yaml
    source=Path(source)
    data=yaml.safe_load(source.read_text()) or {}
    selected=contract['bundle']['selection'];active=contract['active'];process=selected['process'];mode=selected['execution_mode']
    def values(name):return data.setdefault(name,{}).setdefault('ros__parameters',{})
    front_source='zed' if mode=='mixed' or process=='spray' else values('wall_projector').get('front_view_source','d405')
    if front_source not in ('zed','d405'):raise ValueError('Unknown front-view camera')
    values('wall_projector').update(process_mode=process,front_view_source=front_source,zed_image_reliable=True)
    values('sketch_to_waypoints').update(process_mode=process,model_id=active['model_id'],real_painting_enabled=process=='paint',dry_run=False,eoat_segment_frame=active['base_frame'],spray_tool_axis='+z',spray_eoat_profile=str(contract['assets']/(selected['active_arm']+'_eoat.json')))
    values('d405_surface_refiner')['process_mode']=process
    values('target_selector').update(require_bound_frame=mode=='mixed',zed_image_reliable=True)
    if mode=='mixed':values('target_selector')['camera_registration_file']=contract['bundle']['connections']['zed_calibration_file']
    if 'd405' in contract['registrations']:
        registration=contract['registrations']['d405']
        # The refiner hashes calibration_file for invalidation; it never parses
        # a hand-eye transform from it. Actual geometry uses capture-time TF.
        refiner=data.setdefault('d405_surface_refiner',{}).setdefault('ros__parameters',{})
        refiner.update(calibration_file=contract['bundle']['connections']['d405_calibration_file'],scene_registration_file=contract['bundle']['connections']['d405_calibration_file'],defer_target_capture_until_arrival=False)
        data.setdefault('wall_projector',{}).setdefault('ros__parameters',{})['d405_optical_frame']=registration.optical_frame
    target=source.with_name(source.stem+'-perception'+source.suffix)
    temporary=target.with_suffix('.tmp');temporary.write_text(ros_yaml_dump(data));temporary.replace(target)
    return target


def namespaced_parameter_file(source,namespace):
    import yaml
    source=Path(source)
    if not namespace: return source
    namespace=namespace.rstrip('/')
    data=yaml.safe_load(source.read_text()) or {}
    scoped={namespace+'/'+name.lstrip('/'):value for name,value in data.items() if name!='/**'}
    if '/**' in data: scoped['/**']=data['/**']
    target=source.with_name(source.stem+'-'+namespace.strip('/').replace('/','-')+source.suffix)
    temporary=target.with_suffix('.tmp')
    temporary.write_text(ros_yaml_dump(scoped));temporary.replace(target)
    return target


def executor_parameter_layers(config_path,overrides):
    # Resolve defaults and runtime overrides before ROS sees the YAML. Node
    # selectors must not compete with launch-generated wildcard dictionaries.
    import yaml
    source=Path(config_path);data=yaml.safe_load(source.read_text()) or {}
    params=dict(data.get('/**',{}).get('ros__parameters',{}))
    params.update(data.get('moveit_executor',{}).get('ros__parameters',{}))
    params.update(overrides);params['use_sim_time']=True
    target=source.with_name(source.stem+'-executor'+source.suffix)
    temporary=target.with_suffix('.tmp')
    temporary.write_text(ros_yaml_dump({'moveit_executor':{'ros__parameters':params}}));temporary.replace(target)
    return [str(target)]


def spray_perception_arguments(config_path):
    """Keep generated cone geometry authoritative through legacy launch defaults."""
    import yaml
    data=yaml.safe_load(Path(config_path).read_text())
    params=data['sketch_to_waypoints']['ros__parameters']
    keys=('spray_footprint_width_m','spray_overlap','spray_speed_mps','spray_standoff_m')
    if params.get('spray_envelope_profile') and not all(key in params for key in keys):
        raise ValueError('Cone profile requires explicit spray geometry and speed')
    return {key:str(params[key]) for key in keys if key in params}


def build_actions(assets,**selection):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, ExecuteProcess
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node, SetParameter
    from rbpodo_painting_control.runtime_topics import painting_control_remappings
    import yaml
    c=build_contract(assets,readiness_timeout=10.,**selection); assets=c['assets']; bundle=c['bundle']; active=c['active']; selected=bundle['selection']
    mode=selected['execution_mode']; process=selected['process']; arm=selected['active_arm']; namespace=c['control_namespace']
    painting=assets/'painting.yaml'
    if not painting.is_file(): raise ValueError('generated painting.yaml missing')
    painting=perception_parameter_file(painting,c)
    profile=yaml.safe_load(painting.read_text()) or {}
    def params(name): return dict(profile.get('/**',{}).get('ros__parameters',{}),**profile.get(name,{}).get('ros__parameters',{}))
    def node(package,executable,name=None,parameters=None,**kwargs):
        return Node(package=package,executable=executable,name=name or executable,parameters=[*(parameters or []),{'use_sim_time':True}],output='screen',**kwargs)
    def include(package,filename,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/filename)),launch_arguments=args.items())
    remappings=painting_control_remappings(namespace)
    manager=namespace+'/controller_manager'
    controller_config=namespaced_parameter_file(assets/'controllers.yaml',namespace)
    force_config=namespaced_parameter_file(painting,namespace)
    chain=['joint_state_broadcaster','force_torque_sensor_broadcaster','force_torque_sensor_broadcaster_raw','admittance_controller',active['controller']] if process=='paint' else [active['controller']]
    spawner=node('controller_manager','spawner','platform_controller_spawner',arguments=[*chain,'-c',manager,'--controller-manager-timeout','60','--switch-timeout','30','--activate-as-group'])
    actions=[SetParameter(name='use_sim_time',value=True),node('robot_state_publisher','robot_state_publisher',parameters=[{'robot_description':c['robot_description']}]),node('moveit_ros_move_group','move_group',parameters=[moveit_parameters(c)]),node('controller_manager','ros2_control_node','controller_manager',namespace=namespace,parameters=[{'robot_description':c['robot_description']},str(controller_config)],remappings=remappings+[('/platform/'+arm+'/robot_description','/robot_description')])]
    downstream=[]
    if c['base_frame']=='link0':
        downstream.append(node('tf2_ros','static_transform_publisher','platform_world_alias',arguments=['--frame-id','world','--child-frame-id','World','--x','0','--y','0','--z','0','--qx','0','--qy','0','--qz','0','--qw','1']))
        # The generated RB model root remains link0, placed at the simulator origin.
        downstream.append(node('tf2_ros','static_transform_publisher','platform_base_alias',arguments=['--frame-id','World','--child-frame-id','link0','--x','0','--y','0','--z','0','--qx','0','--qy','0','--qz','0','--qw','1']))
    if mode=='mixed':
        for camera in c['camera_names']:
            connection=bundle['connections']
            downstream.append(node('sketch_control','outpost_bridge','platform_'+camera+'_bridge',parameters=[{'camera_name':camera,'execution_mode':'mixed','camera_registration_file':connection[camera+'_calibration_file'],'stamp_mode':'ros_clock','outpost_http':connection.get('outpost_http','http://127.0.0.1:8100'),'outpost_'+camera+'_hw_id':connection['outpost_'+camera+'_hw_id'],'outpost_'+camera+'_serial':connection['outpost_'+camera+'_serial'],'point_stride':1 if camera=='d405' else 2}]))
        common={'camera_registration_file':bundle['connections']['zed_calibration_file'],'camera_name':'zed'}
        downstream.extend([node('sketch_control','task_scene',parameters=[{**common,'required_camera_names':c['camera_names'],'d405_registration_file':bundle['connections'].get('d405_calibration_file',common['camera_registration_file'])}]),node('sketch_control','rgbd_weld_provider',parameters=[common])])
    else:
        for camera in c['camera_names']:
            calibration=json.loads((assets/'calibration'/f'{camera}.json').read_text())
            pose=(calibration.get('T_parent_optical') or calibration.get('T_world_zed_optical')) if camera=='zed' else calibration.get('T_d405_optical_to_tcp')
            if not pose: raise ValueError('generated simulation camera calibration missing')
            parent=calibration.get('parent_frame',c['base_frame']) if camera=='zed' else active['tcp_link']
            child='zed_left_camera_frame_optical' if camera=='zed' else 'd405_color_optical_frame'
            xyz=pose['translation']; q=pose['rotation_xyzw']
            downstream.append(node('tf2_ros','static_transform_publisher','platform_'+camera+'_tf',arguments=['--frame-id',parent,'--child-frame-id',child,'--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),'--qx',str(q[0]),'--qy',str(q[1]),'--qz',str(q[2]),'--qw',str(q[3])]))
    if process=='weld':
        downstream.append(node('sketch_control','weld_input',parameters=[{'execution_mode':mode,'simulation':mode=='simulation','base_frame':active['base_frame'],'weld_input_provider':c['weld_provider'],'weld_fixture_file':str(assets/'weld_fixture.json'),'weld_scene_revision':bundle['revision']}]))
    else:
        downstream.append(include('sketch_control','rb10_perception_sketch.launch.py',{'painting_config_file':str(painting),'model_id':active['model_id'],'process_mode':process,'eoat_segment_frame':active['base_frame'],'spray_tool_axis':'+z','spray_eoat_profile':str(assets/(arm+'_eoat.json')),'real_painting_enabled':str(process=='paint').lower(),'dry_run':'false','front_view_source':params('wall_projector').get('front_view_source','d405'),'zed_image_reliable':'true','use_sim_depth_pointcloud':str(mode=='simulation').lower(),'use_sim_d405_depth_pointcloud':str(mode=='simulation' and process=='paint').lower(),'launch_wall_detector':'false','launch_environment_scanner':'false','use_d405_refinement':str(process=='paint').lower(),'use_d405_mount_tf':'false','use_d405_optical_tf':'false','use_zed_calibration_file':'false','use_d405_calibration_file':'false','publish_zed_static_tf':'false','use_ft_normal_controller':'false',**spray_perception_arguments(painting)}))
    workspace=Path(os.environ.get('SKETCH_WORKSPACE',Path(__file__).resolve().parents[3]))
    if process!='paint':
        downstream.append(ExecuteProcess(cmd=['/usr/bin/python3',str(workspace/'scripts/construct_position_state.py'),'--controller',active['controller'],'--controller-manager',manager],output='screen'))
    downstream.append(ExecuteProcess(cmd=['/usr/bin/python3',str(workspace/'scripts/platform_environment.py'),'--assets',str(assets)],output='screen'))
    if process=='paint':
        lifecycle_remappings=painting_control_remappings(namespace)+[('/isaac/rb20_painting/state',namespace+'/isaac_painting/state'),('/isaac/rb20_painting/lifecycle_status',namespace+'/isaac_painting/lifecycle_status'),('/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal',namespace+'/'+active['controller']+'/follow_joint_trajectory/_action/cancel_goal')]
        downstream.append(node('sketch_control','isaac_painting_lifecycle',parameters=[{'session_id':c['ready']['session_id'],'scene_revision':c['ready']['scene_revision']}],remappings=lifecycle_remappings))
        downstream.append(include('rbpodo_painting_control','painting_admittance_control.launch.py',{'config_file':str(force_config),'namespace':namespace,'ft_frame':active['wrench_frame'],'tcp_frame':active['tcp_link'],'real_painting_enabled':'true','public_motion_abort_requires_restart':'true','dry_run':'false','enable_force':'false','desired_contact_force_n':'20.0','start_segment_mode_node':'false'}))
    executor={}
    executor.update(model_id=active['model_id'],process_mode=process,generated_description_file=str(assets/'robot.urdf'),collision_srdf_path=str(assets/'robot.srdf'),execution_backend='follow_joint_trajectory',real_painting_enabled=process=='paint',dry_run=False,painting_force_enabled=process=='paint',require_task_scene_ack=mode=='mixed',spray_motion_test=process=='spray',spray_tool_axis='+z',spray_eoat_profile=str(assets/(arm+'_eoat.json')),weld_input_provider=c['weld_provider'],weld_tool_profile=str(assets/'weld_tool.json'))
    executor['follow_joint_trajectory_action']=namespace+'/'+active['controller']+'/follow_joint_trajectory'
    if process=='paint': executor.update(paint_nominal_reference_topic=namespace+'/'+active['controller']+'/controller_state',roller_contact_link=active['contact_link'],roller_tool_links=active.get('tool_links',[active['contact_link']]),objects_config_file=str(assets/'objects.yaml'))
    remappings += [('/controller_manager/list_controllers',manager+'/list_controllers'),('/controller_manager/switch_controller',manager+'/switch_controller')]
    remappings += [('/'+active['controller']+'/follow_joint_trajectory',namespace+'/'+active['controller']+'/follow_joint_trajectory')]
    downstream.append(node('sketch_control','moveit_executor',parameters=executor_parameter_layers(painting,executor),remappings=remappings))
    def after(event,context):
        return downstream if event.returncode==0 else [EmitEvent(event=Shutdown(reason='platform controller activation failed'))]
    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner,on_exit=after))); actions.append(spawner)
    return actions


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration
    def setup(context):
        return build_actions(LaunchConfiguration('assets').perform(context),**{key:LaunchConfiguration(key).perform(context) for key in ('active_arm','execution_mode','process_mode')})
    return LaunchDescription([DeclareLaunchArgument('assets'),DeclareLaunchArgument('active_arm'),DeclareLaunchArgument('execution_mode',choices=['simulation','mixed']),DeclareLaunchArgument('process_mode',choices=['spray','weld','paint']),OpaqueFunction(function=setup)])
