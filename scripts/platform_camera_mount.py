"""CAD nominal optical references attached to measured robot bodies.

These are simulation pinhole references, not measured optical calibration.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from sketch_control.eoat_catalog import origin_tf


def nominal_zed_mount():
    transform=np.eye(4);transform[0,3]=-.06
    return dict(parent_link='zed2i_link',parent_from_optical=transform.tolist(),
                calibrated=False,basis='Hongik v4 STL nominal left lens center; optical projection-center depth uncalibrated',
                mesh_sha256='7bc1999bda1355f01fa8663fcdba759b4e64b87e435b80f175c58b2479ad3fba')


def link_world_transform(robot,link,positions):
    parents={j.find('child').get('link'):j for j in robot.findall('joint')}
    if robot.find(f"link[@name='{link}']") is None:raise ValueError('Missing camera mount link')
    result=np.eye(4);visited=set()
    while link in parents:
        if link in visited:raise ValueError('Cyclic camera mount tree')
        visited.add(link);joint=parents[link];motion=np.eye(4)
        if joint.get('type')!='fixed':
            name=joint.get('name')
            if name not in positions:raise ValueError('Missing joint position: '+name)
            value=float(positions[name]);axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            if not np.isfinite(value):raise ValueError('Invalid camera mount joint')
            if joint.get('type') in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
            elif joint.get('type')=='prismatic':motion[:3,3]=axis*value
            else:raise ValueError('Unsupported camera mount joint')
        result=origin_tf(joint.find('origin'))@motion@result
        link=joint.find('parent').get('link')
    return result
