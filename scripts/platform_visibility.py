"""Offline visibility checks of generated URDF geometry at measured joints."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from sketch_control.eoat_catalog import origin_tf
from platform_weld_fixture import occluded


def robot_occluders(robot, positions):
    children={j.find('child').get('link') for j in robot.findall('joint')}
    roots={link.get('name') for link in robot.findall('link')}-children
    if len(roots)!=1:raise ValueError('single generated robot root required')
    transforms={roots.pop():np.eye(4)};pending=list(robot.findall('joint'))
    while pending:
        progressed=False
        for joint in pending[:]:
            parent=joint.find('parent').get('link')
            if parent not in transforms:continue
            motion=np.eye(4);kind=joint.get('type')
            if kind!='fixed':
                value=positions.get(joint.get('name'))
                if value is None or not np.isfinite(value):raise ValueError('measured robot joints incomplete')
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                if kind in ('continuous','revolute'):motion[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
                elif kind=='prismatic':motion[:3,3]=axis*value
                else:raise ValueError('unsupported generated joint type')
            transforms[joint.find('child').get('link')]=transforms[parent]@origin_tf(joint.find('origin'))@motion
            pending.remove(joint);progressed=True
        if not progressed:raise ValueError('disconnected generated robot')
    result=[]
    for link in robot.findall('link'):
        for kind in ('visual','collision'):
            for item in link.findall(kind):
                geometry=item.find('geometry');mesh=geometry.find('mesh')
                box=geometry.find('box');cylinder=geometry.find('cylinder');sphere=geometry.find('sphere')
                if mesh is not None:
                    path=Path(mesh.get('filename').removeprefix('file://'))
                    if not path.is_absolute():raise ValueError('absolute generated mesh resource required')
                    shape=trimesh.load_mesh(path,process=False)
                    shape.apply_scale(np.fromstring(mesh.get('scale','1 1 1'),sep=' '))
                elif box is not None:shape=trimesh.creation.box(extents=np.fromstring(box.get('size'),sep=' '))
                elif cylinder is not None:shape=trimesh.creation.cylinder(radius=float(cylinder.get('radius')),height=float(cylinder.get('length')))
                elif sphere is not None:shape=trimesh.creation.icosphere(radius=float(sphere.get('radius')))
                else:raise ValueError('unsupported robot occlusion geometry')
                shape.apply_transform(transforms[link.get('name')]@origin_tf(item.find('origin')))
                if not np.isfinite(shape.triangles).all():raise ValueError('invalid robot occlusion geometry')
                result.append((link.get('name')+'/'+kind,shape.bounds,shape.triangles))
    return result


def patch_visibility(corners,view,occluders):
    """Check an 81-ray interior grid, including edges, against every robot shape."""
    a,b,c,d=np.asarray(corners);view=np.asarray(view);blocked=0;names=set()
    for u in np.linspace(0.,1.,9):
        for v in np.linspace(0.,1.,9):
            point=a+u*(b-a)+v*(d-a);low=np.minimum(point,view);high=np.maximum(point,view);hits=[]
            for name,bounds,triangles in occluders:
                if np.all(bounds[1]>=low) and np.all(bounds[0]<=high) and occluded(point,view,triangles):hits.append(name)
            blocked+=bool(hits);names.update(hits)
    return dict(rays=81,blocked_rays=blocked,occluders=sorted(names),clear=blocked==0)
