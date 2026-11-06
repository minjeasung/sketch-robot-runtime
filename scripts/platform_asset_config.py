"""Generate controller, camera and process files bound to an asset bundle."""
import json
from copy import deepcopy
from pathlib import Path
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import trimesh
from sketch_control.eoat_catalog import origin_tf


class RosSafeDumper(yaml.SafeDumper):
    def ignore_aliases(self,data):
        return True

def dump_ros_yaml(value):
    return yaml.dump(value,Dumper=RosSafeDumper)


def merge_parameters(base,overrides):
    """Retain hardware/frame settings while applying nested process dynamics."""
    result=deepcopy(base)
    for key,value in overrides.items():
        result[key]=(merge_parameters(result[key],value)
                     if isinstance(value,dict) and isinstance(result.get(key),dict)
                     else deepcopy(value))
    return result


def write_runtime_files(root,output,bundle,robot,tfs):
    selection=bundle['selection'];active=selection['active_arm'];arm=bundle['arms'][active]
    paint=selection['process']=='paint';commercial=selection['platform']=='commercial'
    def dump(name,value): (output/name).write_text(json.dumps(value,indent=2))
    # All tool meshes are expressed in exactly the generated TCP frame.
    for side,contract in bundle['arms'].items():
        if contract['tool_id']=='none':
            for suffix in ('_eoat.stl','_eoat.json'):
                (output/(side+suffix)).unlink(missing_ok=True)
            continue
        meshes=[];inv=np.linalg.inv(tfs[contract['tcp_link']])
        for name in contract['tool_links']:
            link=robot.find(f"link[@name='{name}']")
            for item in link.findall('collision'):
                element=item.find('geometry/mesh')
                if element is None:continue
                mesh=trimesh.load_mesh(element.get('filename'),process=False)
                mesh.apply_scale(np.fromstring(element.get('scale','1 1 1'),sep=' '))
                mesh.apply_transform(inv @ tfs[name] @ origin_tf(item.find('origin')));meshes.append(mesh)
        if not meshes:raise ValueError('Tool collision meshes missing')
        combined=trimesh.util.concatenate(meshes);combined.vertices=combined.vertices.astype(np.float32).astype(float)
        v=combined.triangles;combined.update_faces(np.linalg.norm(np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0]),axis=1)>1e-18)
        combined.export(output/(side+'_eoat.stl'))
        dump(side+'_eoat.json',dict(schema_version=1,model_id=contract['model_id'],spray_tool_axis='+z',mesh_file=side+'_eoat.stl',mesh_scale_to_m=1.,
             mesh_to_tcp=dict(translation_m=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.]),endpoint_confirmed=True,endpoint_tcp_m=[0.,0.,0.]))
    weld_links=[name for name in arm['tool_links'] if name.endswith('hongik_eoat_base_link')]
    if selection['process']=='weld':
        dump('weld_tool.json',dict(profile_id='hongik_v4',revision=bundle['revision'],models=[arm['model_id']],simulation_only=True,
             tip_translation_m=[0.,0.,0.],tip_quaternion_xyzw=[0.,0.,0.,1.],reference_up=[0.,-1.,0.],radius_m=.006,
             collision_mode='native_urdf',native_collision_links=weld_links,robot_description_file=bundle['urdf'],
             robot_description_sha256=hashlib.sha256(Path(bundle['urdf']).read_bytes()).hexdigest()))
    bundle['camera_ft_transforms']={}
    for side,c in bundle['arms'].items():
        if c['tool_id']=='roller':
            bundle['camera_ft_transforms'][side]={kind:(np.linalg.inv(tfs[c['wrench_frame']]) @ tfs[next(n for n in c['tool_links'] if n.endswith('paint_'+kind))]).tolist()
                     for kind in ('d405_color_optical_frame','d405_depth_optical_frame')}
    eye=np.array(([.3,-.85,1.45] if paint else [1.4,-.3,1.8]) if commercial else ([.3,-.55,1.45] if paint else [.2,-1.8,1.7]));target=np.array(([0.,-1.62,1.1] if paint else [0.,-1.2,.9]) if commercial else ([1.12,0.,.9] if paint else [.9,0.,.9]))
    # Expose the left column's outward CAD face to the left spray arm. A shared
    # right-side view otherwise selects its opposite face across the structure.
    if commercial and selection['process']=='spray' and active=='left':eye[0]=-eye[0];eye[1]=-.9
    if not commercial and selection['process']=='spray' and selection['execution_mode']=='simulation':
        # View the real lower column face inside the floor-mounted arm's
        # native nozzle workspace; the complete workpiece stays in place.
        eye=np.array([.2,-1.4,.5]);target=np.array([.9,0.,.2])
    rb10_weld_fixture=(not commercial and selection['process']=='weld' and selection['execution_mode']=='simulation')
    if rb10_weld_fixture:
        # Move the complete fixture and its camera together, placing seam004
        # on the robot-facing column without reaching across the structure.
        eye=np.array([-1.8,-1.4,1.7]);target=np.array([-1.1,.4,.9])
    forward=target-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(forward,right)
    zed=np.eye(4);zed[:3,:3]=np.column_stack((right,down,forward));zed[:3,3]=eye
    wall=dict(center=([0.,-1.65,1.1] if paint else [0.,-1.25,.9]) if commercial else [1.15,0.,.9],size=[1.,.06,1.2] if commercial else [.06,1.,1.2],id='platform_painting_wall')
    scene=dict(wall=wall,zed_world_optical=zed.tolist(),camera_names=['zed','d405'] if paint else ['zed'],camera_resolution=[640,480],
               physics_hz=500,measured_state_hz=100,render_hz=15,h_steel_translation=[-.515,-1.7,0.] if commercial else [.5,-.4,0.])
    if commercial and selection['execution_mode']=='simulation' and ((active=='left' and selection['process']=='spray') or (active=='right' and selection['process']=='weld')):
        # User-requested demo layout: move the complete fixture 0.60 m
        # farther from the platform (-World Y) from the former -1.30 m pose.
        scene.update(h_steel_translation=[-.515,-1.9,0.],
                     h_steel_placement_source='simulation_fixture_rigid_translation',
                     h_steel_placement_delta_m=[0.,-.2,0.])
    if rb10_weld_fixture:
        scene.update(h_steel_translation=[-1.5,0.,0.],
                     h_steel_placement_source='simulation_fixture_rigid_translation',
                     h_steel_placement_delta_m=[-2.,.4,0.],
                     rb10_weld_fixture_setup=dict(seam_id='IfcFastener_WELD_004',
                         work_angle_deg=60.,travel_angle_deg=0.,reverse=False,
                         verification='offline_native_geometry_samples; live_process_validation_required'))
    zed_mount=None
    # Both welding arms use the same physical head camera.
    if commercial and selection['execution_mode']=='simulation' and (selection['process']=='weld' or (selection['process']=='spray' and active=='left')):
        from platform_camera_mount import nominal_zed_mount,link_world_transform
        zed_mount=nominal_zed_mount()
        zed=link_world_transform(robot,zed_mount['parent_link'],bundle['initial_positions'])@np.array(zed_mount['parent_from_optical'])
        scene.update(zed_world_optical=zed.tolist(),zed_mount=zed_mount,
            camera_pose_source='measured robot head with CAD nominal left lens reference')
    if commercial and active=='right' and selection['process']=='weld' and selection['execution_mode']=='simulation':
        scene['weld_demo']=dict(seam_id='horizontal_lower_front_fillet',standoff_m=.012,work_angle_deg=0.,travel_angle_deg=0.,tool_roll_deg=90.)
    dump('scene.json',scene)
    calibration=output/'calibration';calibration.mkdir(exist_ok=True)
    def pose(tf):return dict(translation=tf[:3,3].tolist(),rotation_xyzw=Rotation.from_matrix(tf[:3,:3]).as_quat().tolist())
    if zed_mount:
        dump('calibration/zed.json',dict(parent_frame=zed_mount['parent_link'],
            T_parent_optical=pose(np.array(zed_mount['parent_from_optical'])),simulation=True,
            calibrated=False,basis=zed_mount['basis']))
    else:
        dump('calibration/zed.json',{'T_world_zed_optical':pose(zed),'simulation':True})
    if paint:
        camera=next(n for n in arm['tool_links'] if n.endswith('paint_d405_color_optical_frame'))
        dump('calibration/d405.json',{'T_d405_optical_to_tcp':pose(np.linalg.inv(tfs[arm['tcp_link']]) @ tfs[camera]),'simulation':True})
    source='task_scene' if selection['execution_mode']=='mixed' else ('static' if paint else 'perception')
    target_name={'task_scene':'measured_task_surface','perception':'perceived_task_surface','static':wall['id']}[source]
    objects=dict(active_target=target_name,active_target_source=source,objects=[])
    if paint and selection['execution_mode']=='simulation':
        objects['objects'].append(dict(name=wall['id'],shape='box',position=wall['center'],size=wall['size'],sketch_face='+y' if commercial else '-x',enabled=True,collision_padding_m=0.,collision_geometry_source='shared_simulation_scene_wall'))
    (output/'objects.yaml').write_text(dump_ros_yaml(objects))
    source=yaml.safe_load((root/'src/sketch_control/config/painting_system_real.yaml').read_text())
    replacements={'tcp':arm['tcp_link'],'ft_link':arm['wrench_frame'] or arm['tcp_link'],'link0':arm['base_frame'],
                  'rb10_1300e_u':arm['model_id']}
    def replace(v):
        if isinstance(v,str):return replacements.get(v,v)
        if isinstance(v,list):return [replace(x) for x in v]
        if isinstance(v,dict):return {k:replace(x) for k,x in v.items()}
        return v
    source=replace(source)
    for section in source.values():
        if isinstance(section,dict) and 'ros__parameters' in section:
            parameters=section['ros__parameters'];parameters['use_sim_time']=True
            if 'process_mode' in parameters:parameters['process_mode']=selection['process']
    for name in ('moveit_executor','sketch_to_waypoints'):
        source[name]['ros__parameters'].update(model_id=arm['model_id'],process_mode=selection['process'],real_painting_enabled=paint,dry_run=False,
             spray_tool_axis='+z',spray_eoat_profile=str(output/(active+'_eoat.json')))
    source['moveit_executor']['ros__parameters'].update(painting_force_enabled=paint,generated_description_file=bundle['urdf'],
       objects_config_file=str(output/'objects.yaml'),collision_srdf_path=bundle['srdf'],require_task_scene_ack=selection['execution_mode']=='mixed',
       follow_joint_trajectory_action='/'+arm['controller']+'/follow_joint_trajectory')
    if paint:
        source['moveit_executor']['ros__parameters'].update(roller_contact_link=arm['contact_link'],roller_tool_links=arm['tool_links'])
    if selection['process']=='weld':
        source['moveit_executor']['ros__parameters'].update(weld_tool_profile=str(output/'weld_tool.json'),
            weld_input_provider='external' if selection['execution_mode']=='mixed' else 'fixture')
    source['sketch_to_waypoints']['ros__parameters']['eoat_segment_frame']=arm['base_frame']
    source['wall_projector']['ros__parameters']['front_view_source']='zed' if selection['execution_mode']=='mixed' or not paint else 'd405'
    source['painting_wrench_reference']['ros__parameters'].update(dry_run=False,enable_force=False)
    source['painting_wrench_guard']['ros__parameters']['real_painting_enabled']=paint
    if selection['process']=='spray' and arm['tool_id']=='spray_v2':
        from rbpodo_painting_control.spray_envelope import load_spray_envelope
        profile_path=root/'assets/construct/spray_v2/envelope.json'
        envelope=load_spray_envelope(profile_path)
        (output/'spray_envelope.json').write_bytes(profile_path.read_bytes())
        parameters=source['sketch_to_waypoints']['ros__parameters']
        parameters.update(spray_envelope_profile=str(output/'spray_envelope.json'),
                          spray_footprint_width_m=envelope.diameter_at(parameters['spray_standoff_m']),
                          spray_overlap=.10)
        bundle['spray_envelope_profile']=str(output/'spray_envelope.json')
    source['painting_force_monitor']['ros__parameters']['startup_readiness_grace_s']=5.
    # A physical registration is supplied by the mixed launch overlay. Never
    # leave the generated simulated calibration as an implicit mixed fallback.
    source['d405_surface_refiner']['ros__parameters']['calibration_file']=str(calibration/'d405.json') if paint and selection['execution_mode']=='simulation' else ''
    (output/'painting.yaml').write_text(dump_ros_yaml(source))
    if paint:
        controllers=yaml.safe_load((root/'src/rbpodo_ros2/rbpodo_bringup/config/controllers_admittance.yaml').read_text())
        controllers['controller_manager']['ros__parameters'].pop('ft_sensor_command_controller',None);controllers.pop('ft_sensor_command_controller',None)
        manager=controllers['controller_manager']['ros__parameters'];manager['use_sim_time']=True
        manager[arm['controller']]=manager.pop('joint_trajectory_controller')
        trajectory=controllers.pop('joint_trajectory_controller');controllers[arm['controller']]=trajectory
        # The chained JTC commands nominal positions; measured feedback also
        # includes admittance displacement. Preserve nominal interpolation.
        trajectory['ros__parameters'].update(joints=arm['joints'],command_joints=['admittance_controller/'+n for n in arm['joints']],open_loop_control=False,interpolate_from_desired_state=True)
        # ros2_control loads controllers.yaml, while painting.yaml is also
        # consumed by the separate force nodes. Apply the process dynamics to
        # the actual controller, before binding authoritative selected frames.
        adm=merge_parameters(controllers['admittance_controller']['ros__parameters'],
                             source['admittance_controller']['ros__parameters'])
        controllers['admittance_controller']['ros__parameters']=adm
        adm['joints']=arm['joints']
        adm['kinematics'].update(base=arm['base_frame'],tip=arm['tcp_link'],group_name=arm['group'])
        adm['ft_sensor']['frame']['id']=arm['wrench_frame'];adm['control']['frame']['id']=arm['tcp_link'];adm['fixed_world_frame']['frame']['id']=arm['base_frame'];adm['gravity_compensation']['frame']['id']=arm['tcp_link']
        adm['gravity_compensation']['CoG']={'pos':[0.,0.,0.],'force':0.}
        for name in ('force_torque_sensor_broadcaster','force_torque_sensor_broadcaster_raw'):controllers[name]['ros__parameters']['frame_id']=arm['wrench_frame']
        for value in controllers.values():value['ros__parameters']['use_sim_time']=True
    else:
        controllers=yaml.safe_load((output/'controllers.yaml').read_text())
    # Controller success must require measured tracking, including on the
    # admittance chain. No per-joint tolerances means positional error can go
    # unchecked even when the command never reaches the simulator drive.
    controllers[arm['controller']]['ros__parameters']['constraints']={
        'goal_time':2.0,'stopped_velocity_tolerance':0.01,
        **{name:{'goal':0.02,'trajectory':0.1} for name in arm['joints']}}
    (output/'controllers.yaml').write_text(dump_ros_yaml(controllers))
    return bundle
