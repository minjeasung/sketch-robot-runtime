#!/usr/bin/env python3
"""Generate one mechanical RB20 source for ROS and a mass-preserving physics reduction.

Only the generated physics file aggregates fixed tool bodies. All geometry origins,
full inertia tensors, mass and centre of mass are transformed into ft_link. This
keeps ft_sensor_joint physical without inventing mass for URDF coordinate frames.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import yaml

JOINT_NAMES = ['base','shoulder','elbow','wrist1','wrist2','wrist3']
ROLLER = 'paint_eoat_no_camera_roller_contact_link'


def expand_source(root, model_id="rb20_1900es"):
    ros_python = Path('/opt/ros/jazzy/lib/python3.12/site-packages')
    if str(ros_python) not in sys.path:
        sys.path.append(str(ros_python))
    import xacro
    import xacro.substitution_args
    packages = {'sketch_control':root/'src/sketch_control', 'eoat_description':root/'src/eoat_description',
                'rbpodo_description':root/'src/rbpodo_ros2/rbpodo_description',
                'rbpodo_moveit_config':root/'src/rbpodo_ros2/rbpodo_moveit_config',
                'realsense2_description':root/'src/vendor/realsense-ros/realsense2_description'}
    old = xacro.substitution_args._eval_find
    xacro.substitution_args._eval_find = lambda name: str(packages[name])
    try:
        doc = xacro.process_file(str(root/'src/sketch_control/urdf/rbpodo_with_eoat.urdf.xacro'),
                                 mappings={'model_id':model_id,'robot_ip':'','use_fake_hardware':'false'})
    finally:
        xacro.substitution_args._eval_find = old
    return ET.fromstring(doc.toxml())


def origin_matrix(origin):
    tf = np.eye(4)
    if origin is not None:
        tf[:3,3] = np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        tf[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return tf


def link_transforms(robot, positions=None):
    positions = positions or {}
    transforms = {'link0':np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'),joint.find('child').get('link')
            if parent not in transforms:
                continue
            local = origin_matrix(joint.find('origin'))
            if joint.get('type') in ('revolute','continuous'):
                axis = np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                rot = np.eye(4)
                rot[:3,:3] = Rotation.from_rotvec(axis*positions.get(joint.get('name'),0.)).as_matrix()
                local = local @ rot
            transforms[child] = transforms[parent] @ local
            pending.remove(joint); progressed = True
        if not progressed:
            raise ValueError('Disconnected robot joint tree')
    return transforms


def descendants(robot, start):
    names = {start}
    while True:
        added = {j.find('child').get('link') for j in robot.findall('joint') if j.find('parent').get('link') in names}
        if added <= names:
            return sorted(names)
        names.update(added)


def physics_model(robot):
    """Reduce fixed frames; retain named sensor joint and the exact combined inertia."""
    physics = ET.Element('robot',name='rb20_painting')
    for name in ['link0','link1','link2','link3','link4','link5','link6']:
        physics.append(copy.deepcopy(robot.find(f"link[@name='{name}']")))
    for name in JOINT_NAMES + ['ft_sensor_joint']:
        physics.append(copy.deepcopy(robot.find(f"joint[@name='{name}']")))
    tool = ET.SubElement(physics,'link',name='ft_link')
    tfs = link_transforms(robot)
    inv_ft = np.linalg.inv(tfs['ft_link'])
    down = descendants(robot,'ft_link')
    mass = 0.; weighted = np.zeros(3); inertials = []
    for name in down:
        link = robot.find(f"link[@name='{name}']")
        tf = inv_ft @ tfs[name]
        for kind in ('visual','collision'):
            for item in link.findall(kind):
                item = copy.deepcopy(item)
                item_tf = tf @ origin_matrix(item.find('origin'))
                origin = item.find('origin')
                if origin is None:
                    origin = ET.SubElement(item,'origin')
                origin.set('xyz',' '.join(map(str,item_tf[:3,3])))
                origin.set('rpy',' '.join(map(str,Rotation.from_matrix(item_tf[:3,:3]).as_euler('xyz'))))
                item.set('name',name+'_'+item.get('name',kind))
                tool.append(item)
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        inert_tf = tf @ origin_matrix(inertial.find('origin'))
        i = inertial.find('inertia')
        tensor = np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],
                           [float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],
                           [float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]])
        r = inert_tf[:3,:3]; c = inert_tf[:3,3]
        mass += m; weighted += m*c; inertials.append((m,c,r@tensor@r.T))
    cog = weighted/mass
    tensor = sum(i+m*(np.dot(c-cog,c-cog)*np.eye(3)-np.outer(c-cog,c-cog)) for m,c,i in inertials)
    node = ET.SubElement(tool,'inertial')
    ET.SubElement(node,'mass',value=str(mass))
    ET.SubElement(node,'origin',xyz=' '.join(map(str,cog)),rpy='0 0 0')
    ET.SubElement(node,'inertia',**{k:str(tensor[a,b]) for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
    return physics, {'joint':'ft_sensor_joint','child':'ft_link','downstream_links':down,
                     'mass_kg':mass,'cog_ft_m':cog.tolist(),'inertia_ft_kg_m2':tensor.tolist()}


def dump_xml(robot,path):
    ET.ElementTree(robot).write(path,encoding='unicode',xml_declaration=True)


def prepare(root: Path, output: Path) -> dict:
    root = root.resolve(); output.mkdir(parents=True,exist_ok=True)
    robot = expand_source(root)
    joints = [j for j in robot.findall('joint') if j.get('type') == 'revolute']
    if [j.get('name') for j in joints] != JOINT_NAMES:
        raise ValueError('RB20 controlled joints changed')
    # Reproducible front-wall viewing pose, independently solved from RB20 geometry.
    target = np.eye(4); target[:3,:3] = Rotation.from_euler('z',np.pi/2).as_matrix(); target[:3,3] = [1.05,0.,1.05]
    def error(q):
        actual = link_transforms(robot,dict(zip(JOINT_NAMES,q)))['tcp']
        return np.r_[actual[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
    solution = least_squares(error,[0.,.4,1.4,0.,-.2,1.57],bounds=(-3.13,3.13),xtol=1e-12,gtol=1e-12,ftol=1e-12)
    if np.linalg.norm(error(solution.x)) > 1e-7:
        raise ValueError('RB20 start-pose IK failed')
    initial = dict(zip(JOINT_NAMES,map(float,solution.x)))
    limits = {j.get('name'):{k:float(v) for k,v in j.find('limit').attrib.items()} for j in joints}
    for control in list(robot.findall('ros2_control')):
        robot.remove(control)
    control = ET.SubElement(robot,'ros2_control',name='RB20PaintingIsaacSystem',type='system')
    hardware = ET.SubElement(control,'hardware')
    ET.SubElement(hardware,'plugin').text='isaac_painting_hardware/IsaacPaintingSystem'
    for name,value in {'state_topic':'/isaac/rb20_painting/state','command_topic':'/isaac/rb20_painting/command',
                       'readiness_file':str(output.parent/'managed-ready.json'),'state_timeout_s':'0.20'}.items():
        ET.SubElement(hardware,'param',name=name).text=value
    for name in JOINT_NAMES:
        j = ET.SubElement(control,'joint',name=name)
        ET.SubElement(j,'command_interface',name='position')
        for interface in ('position','velocity','effort'):
            ET.SubElement(j,'state_interface',name=interface)
    for sensor in ('ft_sensor','ft_sensor_raw'):
        s = ET.SubElement(control,'sensor',name=sensor)
        for component in ('force.x','force.y','force.z','torque.x','torque.y','torque.z'):
            ET.SubElement(s,'state_interface',name=component)
    package_paths = {'rbpodo_description':root/'src/rbpodo_ros2/rbpodo_description','eoat_description':root/'src/eoat_description',
                     'realsense2_description':root/'src/vendor/realsense-ros/realsense2_description'}
    for mesh in robot.findall('.//mesh'):
        uri=mesh.get('filename')
        pkg,relative=uri.removeprefix('package://').split('/',1)
        path=package_paths[pkg]/relative
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename',path.resolve().as_uri())
    dump_xml(robot,output/'robot.urdf')
    physics,boundary=physics_model(robot)
    for mesh in physics.findall('.//mesh'):
        mesh.set('filename',mesh.get('filename').removeprefix('file://'))
    dump_xml(physics,output/'robot_physics.urdf')
    sys.path.insert(0,str(root/'src/sketch_control'))
    from sketch_control.robot_models import model_srdf
    srdf=ET.fromstring(model_srdf('rb20_1900es',root/'src/sketch_control/config/rbpodo_named.srdf'))
    for item in list(srdf.findall('group_state')):
        srdf.remove(item)
    # Preserve the exact RB20 allow policy checked by the original executor.
    # Extra fixed-frame entries would make its strict ACM baseline unsafe.
    dump_xml(srdf,output/'robot.srdf')
    controllers=yaml.safe_load((root/'src/rbpodo_ros2/rbpodo_bringup/config/controllers_admittance.yaml').read_text())
    controllers['controller_manager']['ros__parameters'].pop('ft_sensor_command_controller',None)
    controllers.pop('ft_sensor_command_controller',None)
    for value in controllers.values():
        value.setdefault('ros__parameters',{})['use_sim_time']=True
    controllers['joint_trajectory_controller']['ros__parameters']['open_loop_control']=False
    painting=yaml.safe_load((root/'src/sketch_control/config/painting_system_real.yaml').read_text())
    for value in painting.values():
        if isinstance(value,dict) and 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time']=True
            if 'model_id' in value['ros__parameters']:
                value['ros__parameters']['model_id']='rb20_1900es'
    # ROS named sections override wildcard launch dictionaries. Set authorized
    # strict simulation execution flags here, retaining initial force output OFF.
    for name in ('moveit_executor','sketch_to_waypoints'):
        painting[name]['ros__parameters'].update(real_painting_enabled=True,dry_run=False)
    painting['moveit_executor']['ros__parameters']['painting_force_enabled']=True
    painting['painting_wrench_guard']['ros__parameters']['real_painting_enabled']=True
    painting['painting_wrench_reference']['ros__parameters'].update(dry_run=False,enable_force=False)
    painting['painting_force_monitor']['ros__parameters']['startup_readiness_grace_s']=5.0
    painting['d405_surface_refiner']['ros__parameters']['calibration_file']=str(output/'calibration/d405.json')
    painting['admittance_controller']['ros__parameters'].setdefault('gravity_compensation',{})['CoG']={'pos':[0.,0.,0.],'force':0.}
    (output/'controllers.yaml').write_text(yaml.safe_dump(controllers,sort_keys=False))
    (output/'painting.yaml').write_text(yaml.safe_dump(painting,sort_keys=False))
    camera_position=np.array([.45,-1.8,1.5]); camera_target=np.array([1.45,0.,1.1])
    forward=(camera_target-camera_position); forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0,0,1]); right/=np.linalg.norm(right)
    down=np.cross(forward,right)
    zed_tf=np.eye(4); zed_tf[:3,:3]=np.column_stack((right,down,forward)); zed_tf[:3,3]=camera_position
    scene={'wall':{'center':[1.48,0.,1.10],'size':[.06,1.2,1.4],'surface_x':1.45,'id':'rb20_painting_wall'},
           'ground_z':-.025,'zed_world_optical':zed_tf.tolist(),'physics_hz':500,'controller_hz':100,'render_hz':15,
           'camera_resolution':[640,480],
           'drive':{'stiffness_nm_per_rad':300000.,'damping_nms_per_rad':30000.,'max_torque_nm':2000.},
           'roller_contact':{'stiffness_n_per_m':5000.,'damping_ns_per_m':50.,'acceleration_spring':False},
           'material':{'static_friction':.4,'dynamic_friction':.3,'restitution':0.},
           'initial_positions':initial}
    (output/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    objects={'active_target':scene['wall']['id'],'objects':[{'name':wall_id,'shape':'box','position':scene['wall']['center'],'size':scene['wall']['size'],'sketch_face':'-x','enabled':True} for wall_id in [scene['wall']['id']]]}
    objects['objects'].append({'name':'rb20_painting_ground','shape':'box','position':[0.,0.,scene['ground_z']-.025],'size':[8.,8.,.05],'enabled':True})
    (output/'objects.yaml').write_text(yaml.safe_dump(objects,sort_keys=False))
    tfs=link_transforms(robot)
    calibration=output/'calibration'; calibration.mkdir(exist_ok=True)
    def pose(tf):
        return {'translation':tf[:3,3].tolist(),'rotation_xyzw':Rotation.from_matrix(tf[:3,:3]).as_quat().tolist()}
    (calibration/'zed.json').write_text(json.dumps({'T_world_zed_optical':pose(zed_tf),'simulation':True},indent=2)+'\n')
    d405_tf=np.linalg.inv(tfs['tcp'])@tfs['paint_d405_color_optical_frame']
    (calibration/'d405.json').write_text(json.dumps({'T_d405_optical_to_tcp':pose(d405_tf),'simulation':True},indent=2)+'\n')
    generated=['robot.urdf','robot_physics.urdf','robot.srdf','controllers.yaml','painting.yaml','scene.json','objects.yaml','calibration/zed.json','calibration/d405.json']
    hashes={name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in generated}
    inputs=sorted(set(root.glob('src/sketch_control/urdf/*.xacro'))|set(root.glob('src/eoat_description/urdf/*.xacro'))|
                  set(root.glob('src/rbpodo_ros2/rbpodo_description/robots/**/*.yaml'))|
                  set(root.glob('src/rbpodo_ros2/rbpodo_description/robots/*.xacro'))|
                  {root/'src/vendor/realsense-ros/realsense2_description/urdf/_d405.urdf.xacro',root/'src/sketch_control/config/painting_system_real.yaml'})
    inputs=sorted(set(inputs)|{Path(mesh.get('filename').removeprefix('file://')) for mesh in robot.findall('.//mesh')})
    manifest={'model_id':'rb20_1900es','joint_names':JOINT_NAMES,'initial_positions':initial,'limits':limits,
              'world_to_link0':{'translation':[0,0,0],'rotation_xyzw':[0,0,0,1]},'ft_boundary':boundary,
              'inertia_overrides':[{'reason':'Aggregate fixed downstream tool bodies into ft_link, preserving mass, centre of mass and inertia; eliminate massless physics frames','mass_kg':boundary['mass_kg']}],
              'calibration':{'zed':'calibration/zed.json','d405':'calibration/d405.json'},
              'camera_ft_transforms':{n:(np.linalg.inv(tfs['ft_link'])@tfs['paint_'+n]).tolist() for n in ['d405_color_optical_frame','d405_depth_optical_frame']},
              'input_hashes':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'generated_hashes':hashes}
    manifest['scene_revision']=scene_revision(manifest)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

def scene_revision(manifest):
    # A source mesh can change without changing its URDF filename. Bind both
    # sets of bytes so re-preparing assets cannot bless an older imported USD.
    identity={key:manifest[key] for key in ('input_hashes','generated_hashes')}
    return hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()


def verify_assets(root, output, require_usd=True):
    manifest=json.loads((output/'manifest.json').read_text())
    for name,digest in manifest['generated_hashes'].items():
        if not (output/name).is_file() or hashlib.sha256((output/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Generated digest mismatch: '+name)
    for name,digest in manifest['input_hashes'].items():
        if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Source digest mismatch: '+name)
    if scene_revision(manifest)!=manifest['scene_revision']:
        raise ValueError('Manifest scene revision mismatch')
    if require_usd:
        evidence_file=output/'usd-verification.json'
        if not evidence_file.is_file():
            raise ValueError('USD verification missing')
        evidence=json.loads(evidence_file.read_text())
        if evidence.get('scene_revision')!=manifest['scene_revision']:
            raise ValueError('USD scene revision mismatch')
        if evidence.get('physics_urdf_sha256')!=manifest['generated_hashes']['robot_physics.urdf']:
            raise ValueError('USD source digest mismatch')
        if not evidence.get('usd_hashes') or 'robot.usd' not in evidence['usd_hashes']:
            raise ValueError('USD digest verification missing')
        for name,digest in evidence['usd_hashes'].items():
            if not (output/name).is_file() or hashlib.sha256((output/name).read_bytes()).hexdigest()!=digest:
                raise ValueError('USD digest mismatch: '+name)
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path)
    parser.add_argument('--verify',action='store_true')
    args=parser.parse_args(); root=Path(__file__).resolve().parents[1]
    manifest=(verify_assets if args.verify else prepare)(root,args.output or root/'.runtime/rb20_painting/assets')
    print(json.dumps({'scene_revision':manifest['scene_revision'],'joint_names':manifest['joint_names'],'ft_boundary':manifest['ft_boundary']},indent=2))
