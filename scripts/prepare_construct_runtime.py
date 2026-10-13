#!/usr/bin/env python3
"""Derive simulator equipment profiles from the existing Construct URDF.

These describe the virtual TCP already defined by that model. They are not
physical nozzle calibration and must not be selected for a physical robot.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


def transform(origin):
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, 3] = [float(x) for x in origin.get('xyz', '0 0 0').split()]
        matrix[:3, :3] = Rotation.from_euler('xyz', [float(x) for x in origin.get('rpy', '0 0 0').split()]).as_matrix()
    return matrix


def prepare(root, output=None):
    root = Path(root)
    output = Path(output or root / 'isaac_assets')
    output.mkdir(parents=True, exist_ok=True)
    urdf = root / 'isaac_assets/construct_control.urdf'
    robot = ET.parse(urdf).getroot()
    links = {node.get('name'): node for node in robot.findall('link')}
    fixed = [node for node in robot.findall('joint') if node.get('type') == 'fixed']
    for side in ('left', 'right'):
        model = 'construct_' + side
        anchor = side + '_manipulator_link_ee'
        tcp = side + '_manipulator_ee_point'
        poses = {anchor: np.eye(4)}
        for _ in fixed:
            for joint in fixed:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent in poses:
                    poses[child] = poses[parent] @ transform(joint.find('origin'))
        meshes = []
        for name, pose in poses.items():
            for collision in links[name].findall('collision'):
                element = collision.find('geometry/mesh')
                if element is None:
                    continue
                filename = element.get('filename')
                if not filename.startswith('file://'):
                    raise ValueError('Expected resolved Construct mesh resource')
                mesh = trimesh.load_mesh(filename[7:], process=False)
                mesh.apply_scale([float(v) for v in element.get('scale', '1 1 1').split()])
                mesh.apply_transform(np.linalg.inv(poses[tcp]) @ pose @ transform(collision.find('origin')))
                meshes.append(mesh)
        if not meshes:
            raise ValueError('Construct equipment collision geometry missing')
        mesh_path = output / (model + '_eoat.stl')
        combined = trimesh.util.concatenate(meshes)
        # STL stores float32: translating tiny CAD facets to TCP coordinates can
        # collapse distinct vertices after quantization. Validate that exact precision.
        combined.vertices = combined.vertices.astype(np.float32).astype(float)
        triangles = combined.triangles
        area2 = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                        triangles[:, 2]-triangles[:, 0]), axis=1)
        combined.update_faces(area2 > 1e-18)
        combined.export(mesh_path)
        profile = dict(schema_version=1, model_id=model, spray_tool_axis='+z',
                       mesh_file=mesh_path.name, mesh_scale_to_m=1.,
                       mesh_to_tcp=dict(translation_m=[0., 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]),
                       endpoint_confirmed=True, endpoint_tcp_m=[0., 0., 0.])
        (output / (model + '_eoat.json')).write_text(json.dumps(profile, indent=2) + '\n')
        (output / (model + '_equipment.json')).write_text(json.dumps(dict(
            source='Construct URDF collision geometry; simulation only',
            urdf=str(urdf), tcp=tcp, links=list(poses),
            equipment_model='spray_v2' if side == 'left' else 'original',
            tcp_calibrated=False,
            purpose='CAD nominal nozzle TCP; simulated motion only, no fluid/force output'
                    if side == 'left' else 'Non-contact path motion at the modeled TCP; no spray gun or force output'), indent=2) + '\n')


if __name__ == '__main__':
    prepare(Path(__file__).resolve().parents[1])
