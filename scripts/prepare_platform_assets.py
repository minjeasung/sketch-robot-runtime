#!/usr/bin/env python3
"""Generate all robot descriptions from the same arm/tool selection."""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/sketch_control'))
from sketch_control.eoat_catalog import load_tool, mount_tool
from prepare_construct_assets import prepare as prepare_construct
from prepare_rb20_painting_assets import expand_source


def write_xml(robot,path):
    ET.ElementTree(robot).write(path,encoding='unicode',xml_declaration=True)


def tree_transforms(robot):
    import numpy as np
    from sketch_control.eoat_catalog import origin_tf
    children={j.find('child').get('link') for j in robot.findall('joint')}
    roots={l.get('name') for l in robot.findall('link')}-children
    if len(roots)!=1:raise ValueError('Robot must have one root')
    result={roots.pop():np.eye(4)};pending=list(robot.findall('joint'))
    while pending:
        advanced=False
        for j in pending[:]:
            parent=j.find('parent').get('link');child=j.find('child').get('link')
            if parent in result:
                result[child]=result[parent] @ origin_tf(j.find('origin'));pending.remove(j);advanced=True
        if not advanced:raise ValueError('Disconnected robot')
    return result


def reduce_roller_bodies(robot,arms):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from sketch_control.eoat_catalog import fixed_descendants,origin_tf
    physical=deepcopy(robot);boundaries={};tfs=tree_transforms(robot)
    for side,arm in arms.items():
        if arm['tool_id']!='roller':continue
        frame=arm['wrench_frame'];down=fixed_descendants(robot,frame)|{frame}
        aggregate=ET.Element('link',name=frame);inv=np.linalg.inv(tfs[frame]);parts=[]
        for link in robot.findall('link'):
            if link.get('name') not in down:continue
            tf=inv @ tfs[link.get('name')]
            for kind in ('visual','collision'):
                for item in link.findall(kind):
                    item=deepcopy(item);pose=tf @ origin_tf(item.find('origin'))
                    origin=item.find('origin')
                    if origin is None:origin=ET.SubElement(item,'origin')
                    origin.set('xyz',' '.join(map(str,pose[:3,3])))
                    origin.set('rpy',' '.join(map(str,Rotation.from_matrix(pose[:3,:3]).as_euler('xyz'))))
                    item.set('name',link.get('name')+'_'+item.get('name',kind));aggregate.append(item)
            inertial=link.find('inertial')
            if inertial is not None:
                m=float(inertial.find('mass').get('value'));pose=tf @ origin_tf(inertial.find('origin'));i=inertial.find('inertia')
                tensor=np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],[float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],[float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]])
                parts.append((m,pose[:3,3],pose[:3,:3] @ tensor @ pose[:3,:3].T))
        mass=sum(m for m,c,i in parts)
        if mass<=0:raise ValueError('Missing physical roller mass')
        cog=sum(m*c for m,c,i in parts)/mass
        tensor=sum(i+m*(np.dot(c-cog,c-cog)*np.eye(3)-np.outer(c-cog,c-cog)) for m,c,i in parts)
        inertial=ET.SubElement(aggregate,'inertial');ET.SubElement(inertial,'mass',value=str(mass))
        ET.SubElement(inertial,'origin',xyz=' '.join(map(str,cog)),rpy='0 0 0')
        ET.SubElement(inertial,'inertia',**{k:str(tensor[a,b]) for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
        for element in list(physical):
            if ((element.tag=='link' and element.get('name') in down) or (element.tag=='joint' and element.find('child').get('link') in down-{frame})):physical.remove(element)
        physical.append(aggregate)
        boundaries[side]=dict(joint=arm['sensor_joint'],child=frame,downstream_links=sorted(down),mass_kg=mass,cog_ft_m=cog.tolist(),inertia_ft_kg_m2=tensor.tolist())
    return physical,boundaries


def generate_urdf_assets(root,selection,output):
    root=Path(root).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    if selection.execution_mode=='real': raise ValueError('Simulation asset generator cannot generate physical drivers')
    if selection.platform=='commercial':
        manifest=prepare_construct(root,output,source_model='hongik_v4')
        robot=ET.parse(output/'construct.urdf').getroot()
        semantic=ET.parse(output/'construct.srdf').getroot()
        initial=dict(manifest['initial_positions'])
        if selection.execution_mode=='simulation' and selection.process=='spray' and selection.active_arm=='left':
            # Aim the actual two-axis head at the horizontal member's top
            # at World [0, -1.5, 1.016], after the 0.60 m fixture relocation.
            # These are explicit simulation setup angles, not a free camera pose.
            initial.update(robot_head_rev_joint1=-.05929017,robot_head_rev_joint2=.83027438)
            # Output-OFF standby at World [0.35, -1.5, 1.65] m: 0.174 m
            # from the top-face entry, with the complete 81-ray ZED patch clear.
            # Verified against full-robot collisions and the entry/stroke path.
            initial.update(zip((f'left_manipulator_joint{i}' for i in range(1,7)),
                [1.61565899,-.11542290,-1.91754763,.00000366,-1.10862947,.10417195]))
        if selection.execution_mode=='simulation' and selection.process=='weld' and selection.active_arm=='right':
            # Mounted ZED observes the front lower web/flange joint. Park the
            # spray arm aside and the torch 120 mm before the welding entry.
            initial.update(robot_head_rev_joint1=-.0595405049,robot_head_rev_joint2=.9712363144,
                           left_manipulator_joint1=.5)
            initial.update(zip((f'right_manipulator_joint{i}' for i in range(1,7)),
                [-0.8030074806203249, 1.122929514678968, 2.10950079059963, -2.559371292058832, 0.3330450827246061, -1.3768865253200249]))
    else:
        robot=expand_source(root,'rb10_1300e_u')
        semantic=ET.parse(root/'src/sketch_control/config/rbpodo_named.srdf').getroot()
        for item in list(robot.findall('ros2_control')): robot.remove(item)
        initial={j.get('name'):float(j.get('value')) for j in semantic.findall("group_state[@name='ready']/joint")}
        packages={'rbpodo_description':root/'src/rbpodo_ros2/rbpodo_description',
                  'eoat_description':root/'src/eoat_description','realsense2_description':root/'src/vendor/realsense-ros/realsense2_description'}
        for mesh in robot.findall('.//mesh'):
            pkg,relative=mesh.get('filename').removeprefix('package://').split('/',1)
            mesh.set('filename',str((packages[pkg]/relative).resolve()))
    semantic.set('name',robot.get('name'))
    arms={arm:mount_tool(robot,arm,load_tool(tool,root)) for arm,tool in selection.tools.items()}
    names={l.get('name') for l in robot.findall('link')}
    tool_links={n for a in arms.values() for n in a['tool_links']}
    for node in list(semantic):
        if node.tag=='group_state': semantic.remove(node)
        elif node.tag=='disable_collisions' and (
            not {node.get('link1'),node.get('link2')} <= names or
            {node.get('link1'),node.get('link2')} & tool_links): semantic.remove(node)
    for joint in robot.findall('joint'):
        parent=joint.find('parent').get('link');child=joint.find('child').get('link')
        if child in tool_links:
            ET.SubElement(semantic,'disable_collisions',link1=parent,link2=child,reason='Adjacent')
    existing={frozenset((p.get('link1'),p.get('link2'))) for p in semantic.findall('disable_collisions')}
    for arm in arms.values():
        for first,second,reason in arm['allowed_collisions']:
            if frozenset((first,second)) not in existing:
                ET.SubElement(semantic,'disable_collisions',link1=first,link2=second,reason=reason)
                existing.add(frozenset((first,second)))
    physical,boundaries=reduce_roller_bodies(robot,arms)
    write_xml(physical,output/'robot_physics.urdf')
    control=deepcopy(robot)
    system=ET.SubElement(control,'ros2_control',name='PlatformIsaacSystem',type='system')
    hardware=ET.SubElement(system,'hardware')
    ET.SubElement(hardware,'plugin').text='topic_based_ros2_control/TopicBasedSystem'
    for key,value in dict(joint_commands_topic='/isaac_joint_command',joint_states_topic='/joint_states',trigger_joint_command_threshold='-1').items():
        ET.SubElement(hardware,'param',name=key).text=value
    active=arms[selection.active_arm]
    paint_joints=set(active['joints']) if selection.process=='paint' else set()
    for name,value in initial.items():
        if name in paint_joints:continue
        joint=ET.SubElement(system,'joint',name=name)
        ET.SubElement(joint,'command_interface',name='position')
        state=ET.SubElement(joint,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text=str(value)
        ET.SubElement(joint,'state_interface',name='velocity')
    if paint_joints:
        if not system.findall('joint'):control.remove(system)
        force=ET.SubElement(control,'ros2_control',name='PlatformRollerSystem',type='system')
        hardware=ET.SubElement(force,'hardware');ET.SubElement(hardware,'plugin').text='isaac_painting_hardware/IsaacPaintingSystem'
        for key,value in dict(topic_namespace=f'/platform/{selection.active_arm}/isaac_painting',control_namespace=f'/platform/{selection.active_arm}',wrench_frame=active['wrench_frame'],base_frame=active['base_frame'],readiness_file=str(output/'simulation-ready.json')).items():
            ET.SubElement(hardware,'param',name=key).text=value
        for name in active['joints']:
            joint=ET.SubElement(force,'joint',name=name);ET.SubElement(joint,'command_interface',name='position')
            for interface in ('position','velocity','effort'):ET.SubElement(joint,'state_interface',name=interface)
        for name in ('ft_sensor','ft_sensor_raw'):
            sensor=ET.SubElement(force,'sensor',name=name)
            for component in ('force.x','force.y','force.z','torque.x','torque.y','torque.z'):ET.SubElement(sensor,'state_interface',name=component)
    for mesh in control.findall('.//mesh'):mesh.set('filename',Path(mesh.get('filename')).as_uri())
    write_xml(control,output/'robot.urdf'); write_xml(semantic,output/'robot.srdf')
    controllers={'controller_manager':{'ros__parameters':{'update_rate':100,'use_sim_time':True}}}
    groups={a['controller']:a['joints'] for a in arms.values()}
    if selection.platform=='commercial':groups['robot_head_controller']=manifest['groups']['robot_head_controller']
    for name,joints in groups.items():
        controllers['controller_manager']['ros__parameters'][name]={'type':'joint_trajectory_controller/JointTrajectoryController'}
        controllers[name]={'ros__parameters':{'joints':joints,'command_interfaces':['position'],'state_interfaces':['position','velocity'],'allow_partial_joints_goal':False}}
    (output/'controllers.yaml').write_text(yaml.safe_dump(controllers))
    result=dict(selection=selection.to_dict(),directory=str(output),urdf=str(output/'robot.urdf'),physics_urdf=str(output/'robot_physics.urdf'),
                ft_boundaries=boundaries,srdf=str(output/'robot.srdf'),controllers=str(output/'controllers.yaml'),arms=arms,initial_positions=initial,
                joint_names=list(initial),generation_stage='urdf_only')
    source_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('assets/construct/**/source.json')}
    inputs={Path(mesh.get('filename')) for mesh in robot.findall('.//mesh')}
    inputs.update(root/name for name in ('scripts/prepare_platform_assets.py','scripts/platform_asset_config.py','scripts/platform_camera_mount.py','scripts/platform_weld_fixture.py','scripts/platform_collision.py','scripts/import_platform_usd.py','scripts/prepare_h_steel.py','scripts/prepare_construct_assets.py','scripts/prepare_rb20_painting_assets.py','src/sketch_control/sketch_control/eoat_catalog.py','src/sketch_control/config/painting_system_real.yaml','src/rbpodo_ros2/rbpodo_bringup/config/controllers_admittance.yaml'))
    inputs.update(root/name for name in ('assets/construct/h_steel/source/Steel Structure.usdc','src/sketch_control/config/rbpodo_named.srdf'))
    if selection.process=='spray':
        inputs.add(root/'assets/construct/spray_v2/envelope.json')
    inputs.update(root.glob('src/eoat_description/urdf/*.xacro'))
    inputs.update(root.glob('src/rbpodo_ros2/rbpodo_description/urdf/*.xacro'))
    source_hashes.update({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
    result['source_hashes']=source_hashes
    result['revision']=hashlib.sha256((json.dumps(selection.to_dict(),sort_keys=True)+ET.tostring(robot).decode()+json.dumps(source_hashes,sort_keys=True)).encode()).hexdigest()
    for arm in arms.values():arm['tool_revision']=result['revision']
    from platform_asset_config import write_runtime_files
    write_runtime_files(root,output,result,robot,tree_transforms(robot))
    (output/'bundle.json').write_text(json.dumps(result,indent=2))
    return result


def generate_assets(root,selection,output,importer=None):
    import os
    import subprocess
    result=generate_urdf_assets(root,selection,output)
    output=Path(output).resolve()
    usd=output/'robot.usd'
    if importer is None:
        isaac=os.environ.get('ISAACSIM_VENV',str(Path(root).resolve().parent/'.venvs/isaacsim-6.1'))
        def importer(urdf,target):
            subprocess.run([str(Path(isaac)/'bin/python'),str(Path(root)/'scripts/import_platform_usd.py'),
                            '--urdf',str(urdf),'--output',str(target)],check=True,timeout=600,
                           env={**os.environ,'OMNI_KIT_ACCEPT_EULA':'YES'})
    importer(Path(result['physics_urdf']),usd)
    if not usd.is_file() or usd.stat().st_size==0: raise RuntimeError('USD importer did not produce robot.usd')
    result.update(usd=str(usd),generation_stage='usd_complete')
    (output/'bundle.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    import argparse
    from sketch_control.runtime_selection import normalize_selection
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--urdf-only',action='store_true')
    args=parser.parse_args()
    selection=normalize_selection(json.loads(args.selection.read_text()))
    function=generate_urdf_assets if args.urdf_only else generate_assets
    print(json.dumps(function(ROOT,selection,args.output),indent=2))
