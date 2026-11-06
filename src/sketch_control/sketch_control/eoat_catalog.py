"""Source-preserving fixed EOAT assemblies in a mechanical flange frame."""
from dataclasses import dataclass, field
from copy import deepcopy
from pathlib import Path
import math
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class ToolDefinition:
    tool_id: str
    links: list
    joints: list
    anchor: str
    flange_transform: np.ndarray
    tcp: str
    tcp_basis: str
    sensor_joint: str = ''
    wrench_frame: str = ''
    contact_link: str = ''
    camera_link: str = ''
    tcp_calibrated: bool = False
    allowed_collisions: list = field(default_factory=list)


def transform(xyz=(0,0,0), rpy=(0,0,0)):
    result=np.eye(4); result[:3,3]=xyz
    result[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
    return result


def origin_tf(node):
    if node is None: return np.eye(4)
    return transform(np.fromstring(node.get('xyz','0 0 0'),sep=' '), np.fromstring(node.get('rpy','0 0 0'),sep=' '))


def fixed_descendants(robot, anchor):
    names={anchor}
    while True:
        children={j.find('child').get('link') for j in robot.findall('joint')
                  if j.get('type')=='fixed' and j.find('parent').get('link') in names}
        if children <= names: return names-{anchor}
        names |= children


def load_tool(tool_id, root):
    root=Path(root)
    if tool_id == 'none':
        # Keep a semantic flange TCP, with no EOAT geometry, mass or sensors.
        link=ET.Element('link',name='tcp')
        joint=ET.Element('joint',name='bare_flange_tcp',type='fixed')
        ET.SubElement(joint,'parent',link='flange')
        ET.SubElement(joint,'child',link='tcp')
        return ToolDefinition('none',[link],[joint],'flange',np.eye(4),'tcp','mechanical_flange')
    if tool_id not in ('spray_v2','hongik_v4','roller'): raise ValueError('Unknown tool')
    # Import source generators, not generated runtime state.
    script_dir=str(root/'scripts')
    if script_dir not in sys.path: sys.path.insert(0,script_dir)
    if tool_id=='roller':
        from prepare_rb20_painting_assets import expand_source, ROLLER
        robot=expand_source(root)
        semantic=ET.parse(root/'src/sketch_control/config/rbpodo_named.srdf').getroot()
        anchor='link6'; flange=transform((0,0,.13)); tcp='tcp'
        sensor='ft_sensor_joint'; wrench='ft_link'; contact=ROLLER; camera='paint_d405_link'
        basis='roller_contact_reference'
    else:
        from prepare_construct_assets import prepare
        with tempfile.TemporaryDirectory() as temp:
            prepare(root,Path(temp),source_model='hongik_v4')
            robot=ET.parse(Path(temp)/'construct.urdf').getroot()
            semantic=ET.parse(Path(temp)/'construct.srdf').getroot()
        side='left' if tool_id=='spray_v2' else 'right'
        anchor=f'{side}_manipulator_link6'; tcp=f'{side}_manipulator_ee_point'
        flange=transform((0,0,.0543)) if side=='left' else transform((0,0,.0605),(0,0,-2.0943951024))
        sensor=wrench=contact=camera=''
        basis='nozzle_face_center' if side=='left' else 'contact_tip_face_center'
    names=fixed_descendants(robot,anchor)
    links=[deepcopy(l) for l in robot.findall('link') if l.get('name') in names]
    joints=[deepcopy(j) for j in robot.findall('joint') if j.find('child').get('link') in names]
    packages={'rbpodo_description':root/'src/rbpodo_ros2/rbpodo_description',
              'eoat_description':root/'src/eoat_description','realsense2_description':root/'src/vendor/realsense-ros/realsense2_description'}
    for link in links:
        for mesh in link.findall('.//mesh'):
            filename=mesh.get('filename')
            if filename.startswith('package://'):
                package,relative=filename[10:].split('/',1)
                filename=str((packages[package]/relative).resolve())
            if filename.startswith('file://'): filename=filename[7:]
            if not Path(filename).is_file(): raise FileNotFoundError(filename)
            mesh.set('filename',filename)
    pairs=[(p.get('link1'),p.get('link2'),p.get('reason','Fixed assembly')) for p in semantic.findall('disable_collisions')
           if {p.get('link1'),p.get('link2')} <= names|{anchor}]
    return ToolDefinition(tool_id,links,joints,anchor,flange,tcp,basis,sensor,wrench,contact,camera,allowed_collisions=pairs)


def weld_tip_offset(stickout_m=None):
    if stickout_m is None: return [0.,0.,0.]
    if isinstance(stickout_m,bool) or not math.isfinite(stickout_m) or not 0 <= stickout_m <= .1:
        raise ValueError('Wire stickout must be finite and between 0 and 0.1 m')
    return [0.,0.,.0012+stickout_m]


def mount_tool(robot, arm, tool):
    if arm not in ('left','right','single'): raise ValueError('Unknown arm')
    anchor='link6' if arm=='single' else f'{arm}_manipulator_link6'
    tcp='tcp' if arm=='single' else f'{arm}_manipulator_ee_point'
    if robot.find(f"link[@name='{anchor}']") is None: raise ValueError('Missing wrist')
    # Mechanical flange clocking is explicit, never a mirrored mesh.
    flange=(transform((0,0,.0543)) if arm=='left' else
            transform((0,0,.0605),(0,0,-2.0943951024)) if arm=='right' else
            transform((0,-.1153,0),(math.pi/2,0,0)))
    old=fixed_descendants(robot,anchor)
    for element in list(robot):
        if ((element.tag=='link' and element.get('name') in old) or
            (element.tag=='joint' and element.find('child').get('link') in old)):
            robot.remove(element)
    mapping={l.get('name'):f'{arm}_{tool.tool_id}_{l.get("name")}' for l in tool.links}
    mapping[tool.tcp]=tcp; mapping[tool.anchor]=anchor
    for link in tool.links:
        new=deepcopy(link); new.set('name',mapping[link.get('name')]); robot.append(new)
    joint_mapping={}
    for joint in tool.joints:
        new=deepcopy(joint); name=f'{arm}_{tool.tool_id}_{joint.get("name")}'
        joint_mapping[joint.get('name')]=name; new.set('name',name)
        parent=joint.find('parent').get('link'); child=joint.find('child').get('link')
        new.find('parent').set('link',mapping[parent]);new.find('child').set('link',mapping[child])
        if parent==tool.anchor:
            tf=flange @ np.linalg.inv(tool.flange_transform) @ origin_tf(joint.find('origin'))
            origin=new.find('origin')
            if origin is None: origin=ET.SubElement(new,'origin')
            origin.set('xyz',' '.join(map(str,tf[:3,3])))
            origin.set('rpy',' '.join(map(str,Rotation.from_matrix(tf[:3,:3]).as_euler('xyz'))))
        robot.append(new)
    from .robot_models import model_execution_contract
    model='rb10_1300e_u' if arm=='single' else 'construct_'+arm
    contract=model_execution_contract(model)
    return dict(model_id=model,tool_id=tool.tool_id,joints=list(contract['arm_joint_names']),
                group=contract['planning_group'],base_frame=contract['base_frame'],flange_link=anchor,
                tcp_link=tcp,controller=contract['trajectory_controller'],tcp_basis=tool.tcp_basis,
                sensor_joint=joint_mapping.get(tool.sensor_joint,''),wrench_frame=mapping.get(tool.wrench_frame,''),
                contact_link=mapping.get(tool.contact_link,''),camera_link=mapping.get(tool.camera_link,''),
                tool_links=list(mapping[n.get('name')] for n in tool.links),tcp_calibrated=False,
                allowed_collisions=[(mapping[a],mapping[b],reason) for a,b,reason in tool.allowed_collisions])
