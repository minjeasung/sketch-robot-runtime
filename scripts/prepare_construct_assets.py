#!/usr/bin/env python3
"""Derive simulation assets from the unmodified Construct description and MoveIt files."""
import copy
import json
import struct
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def mount_left_spray_v2(robot, root, config, output):
    """Use the CAD flange frame, with +Z along the nominal spray direction."""
    assets = root / 'assets/construct/spray_v2'
    mount = json.loads((assets / 'mount.json').read_text())
    tool = robot.find("link[@name='left_manipulator_link_ee']")
    for kind in ('visual', 'collision'):
        filename = assets / f'spray_v2_{kind}.stl'
        if not filename.is_file():
            raise FileNotFoundError(filename)
        node = tool.find(kind)
        node.find('origin').set('xyz', '0 0 0')
        node.find('origin').set('rpy', '0 0 0')
        node.find('geometry/mesh').set('filename', str(filename.resolve()))
    # No physical mass properties were supplied: explicitly nominal simulation inertia.
    lower, upper = mount['bounds_m']
    size = [b-a for a,b in zip(lower, upper)]
    center = [(a+b)/2 for a,b in zip(lower, upper)]
    mass = mount['simulation_mass_kg']
    inertial = tool.find('inertial')
    inertial.find('origin').set('xyz', ' '.join(map(str, center)))
    inertial.find('origin').set('rpy', '0 0 0')
    inertial.find('mass').set('value', str(mass))
    inertial.find('inertia').attrib.update(
        ixx=str(mass*(size[1]**2+size[2]**2)/12),
        iyy=str(mass*(size[0]**2+size[2]**2)/12),
        izz=str(mass*(size[0]**2+size[1]**2)/12), ixy='0', ixz='0', iyz='0')
    for joint_name, xyz in (
        ('left_manipulator_joint_ee', mount['flange_in_link6_m']),
        ('left_ee_point_joint', mount['tcp_in_flange_m']),
    ):
        origin = robot.find(f"joint[@name='{joint_name}']/origin")
        origin.set('xyz', ' '.join(map(str, xyz)))
        origin.set('rpy', '0 0 0')
    robot.remove(robot.find("joint[@name='left_D435_joint']"))
    robot.remove(robot.find("link[@name='left_D435_link']"))
    semantic = ET.parse(config / 'construct_robot_0528.srdf').getroot()
    for pair in list(semantic.findall('disable_collisions')):
        names = {pair.get('link1'), pair.get('link2')}
        if ('left_D435_link' in names or
            ('left_manipulator_link_ee' in names and names !=
             {'left_manipulator_link6', 'left_manipulator_link_ee'})):
            semantic.remove(pair)
    ET.ElementTree(semantic).write(output / 'construct.srdf', encoding='unicode', xml_declaration=True)
    return {'left': {'model': 'spray_v2', 'mount': mount,
                     'mass_calibrated': False, 'tcp_calibrated': False}}


def prepare(root: Path, output: Path) -> dict:
    description = root / 'src/construct_description'
    config = root / 'src/construct_moveit_config/config'
    robot = ET.parse(description / 'urdf_0528/construct_robot_0528.urdf').getroot()
    initial = yaml.safe_load((description / 'config/initial_positions.yaml').read_text())['initial_positions']
    joints = [j for j in robot.findall('joint') if j.get('type') == 'revolute']
    names = {j.get('name') for j in joints}
    if len(joints) != 14 or names != set(initial):
        raise ValueError('Expected 14 Construct joints with complete initial positions')
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename')
        prefix = 'package://construct_description/'
        if not filename.startswith(prefix):
            raise ValueError(f'Unexpected mesh URI: {filename}')
        resolved = (description / filename[len(prefix):]).resolve()
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        mesh.set('filename', str(resolved))
    # The supplied right D435 CAD inertia describes an entire assembly (236.7 kg).
    # Its vertices match the left D435 within 1e-12 m, so use the same local inertia.
    right = robot.find("link[@name='right_D435_link']")
    left = robot.find("link[@name='left_D435_link']")
    overrides = []
    original_mass = float(right.find('inertial/mass').get('value'))
    if original_mass > 200:
        geometry = []
        for link in (left, right):
            data = Path(link.find('visual/geometry/mesh').get('filename')).read_bytes()
            records = struct.iter_unpack('<12fH', data[84:])
            geometry.append([v for record in records for v in record[3:12]])
        if len(geometry[0]) != len(geometry[1]) or any(abs(a-b)>1e-8 for a,b in zip(*geometry)):
            raise ValueError('D435 geometry changed; revalidate the right-camera inertia correction')
        right.remove(right.find('inertial'))
        right.insert(0, copy.deepcopy(left.find('inertial')))
        overrides.append({'link':'right_D435_link','original_mass_kg':original_mass,
                          'replacement_mass_kg':float(left.find('inertial/mass').get('value')),
                          'reason':'Identical left/right camera meshes; copy left local inertia in generated assets only'})
    limits = {}
    for joint in joints:
        limit = joint.find('limit')
        limits[joint.get('name')] = {k: float(v) for k,v in limit.attrib.items()}
        if not limits[joint.get('name')]['lower'] <= initial[joint.get('name')] <= limits[joint.get('name')]['upper']:
            raise ValueError(f'Initial position outside limits: {joint.get("name")}')
    output.mkdir(parents=True, exist_ok=True)
    equipment = mount_left_spray_v2(robot, root, config, output)
    ET.ElementTree(robot).write(output / 'construct.urdf', encoding='unicode', xml_declaration=True)
    controlled = copy.deepcopy(robot)
    # ROS resource_retriever expects a URI, while Isaac importer uses local paths.
    for mesh in controlled.findall('.//mesh'):
        mesh.set('filename',Path(mesh.get('filename')).as_uri())
    system = ET.SubElement(controlled, 'ros2_control', name='ConstructIsaacSystem', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'topic_based_ros2_control/TopicBasedSystem'
    for name,value in {'joint_commands_topic':'/isaac_joint_command','joint_states_topic':'/joint_states','trigger_joint_command_threshold':'-1'}.items():
        ET.SubElement(hardware,'param',name=name).text = value
    for name, position in initial.items():
        joint = ET.SubElement(system,'joint',name=name)
        ET.SubElement(joint,'command_interface',name='position')
        state = ET.SubElement(joint,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text = str(position)
        ET.SubElement(joint,'state_interface',name='velocity')
    ET.ElementTree(controlled).write(output / 'construct_control.urdf', encoding='unicode', xml_declaration=True)
    moveit_controllers = yaml.safe_load((config / 'moveit_controllers.yaml').read_text())
    entries = moveit_controllers['moveit_simple_controller_manager']
    groups = {name:entries[name]['joints'] for name in entries['controller_names']}
    if set(sum(groups.values(),[])) != names:
        raise ValueError('Controller joints do not match robot')
    controllers = {'controller_manager': {'ros__parameters': {'update_rate':100, 'use_sim_time':True}}}
    for name, joint_names in groups.items():
        controllers['controller_manager']['ros__parameters'][name] = {'type':'joint_trajectory_controller/JointTrajectoryController'}
        controllers[name] = {'ros__parameters': {
            'joints':joint_names,'command_interfaces':['position'], 'state_interfaces':['position','velocity'],
            'allow_partial_joints_goal':False,
            'constraints':{'goal_time':5.0,'stopped_velocity_tolerance':0.05, **{j:{'trajectory':0.3,'goal':0.03} for j in joint_names}},
        }}
    (output / 'controllers.yaml').write_text(yaml.safe_dump(controllers, sort_keys=False))
    planning_limits = yaml.safe_load((config/'joint_limits.yaml').read_text())
    head_motion_overrides = {}
    for name in groups['robot_head_controller']:
        limit = planning_limits['joint_limits'][name]
        if not limit.get('has_acceleration_limits') or limit.get('max_acceleration', 0) <= 0:
            # Original file disables head acceleration limits; Jazzy TOTG requires them.
            head_motion_overrides[name] = {'has_acceleration_limits':True,'max_acceleration':0.5,
                                           'has_jerk_limits':True,'max_jerk':2.0}
            limit.update(head_motion_overrides[name])
    (output/'moveit_joint_limits.yaml').write_text(yaml.safe_dump(planning_limits,sort_keys=False))
    manifest = {'equipment':equipment, 'head_motion_overrides':head_motion_overrides, 'inertia_overrides':overrides, 'name':robot.get('name'),'joint_names':[j.get('name') for j in joints], 'initial_positions':initial, 'limits':limits, 'groups':groups}
    (output / 'construct_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    print(json.dumps(prepare(root, root/'isaac_assets'),indent=2))
