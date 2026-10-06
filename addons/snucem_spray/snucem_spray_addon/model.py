"""Bind the add-on to the live stack model, including collision resources."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

JOINTS = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
MODELS = {'rb10_spray': 'rb10_1300e_u', 'rb20_spray': 'rb20_1900es'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def resolve_mesh(uri, resource_roots):
    if not uri.startswith('package://'):
        raise ValueError('model mesh must use an installed package:// resource')
    package, relative = uri[10:].split('/', 1)
    if package not in resource_roots:
        raise ValueError('unresolved model mesh package: ' + package)
    root = Path(resource_roots[package]).resolve()
    path = (root/relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('mesh resource missing or outside package')
    return path


def parse_stack_model(urdf, srdf, joint_limits, resource_roots):
    tree, semantic = ET.fromstring(urdf), ET.fromstring(srdf)
    name = tree.get('name')
    if name not in MODELS or semantic.get('name') != name:
        raise ValueError('expected matching RB10/RB20 spray URDF/SRDF')
    links = {e.get('name') for e in tree.findall('link')}
    joints = {e.get('name'): e for e in tree.findall('joint')}
    if not {'link0', 'tcp', 'spray_eoat'} <= links or not set(JOINTS) | {'tcp_joint', 'spray_mount_joint'} <= joints.keys():
        raise ValueError('incomplete spray model')
    for fixed in ('tcp_joint', 'spray_mount_joint'):
        if joints[fixed].get('type') != 'fixed':
            raise ValueError('tool mounting must be fixed')
    if joints['tcp_joint'].find('child').get('link') != 'tcp':
        raise ValueError('tcp_joint must terminate at the nozzle TCP')
    chain = semantic.find("group[@name='manipulator']/chain")
    if chain is None or chain.get('base_link') != 'link0' or chain.get('tip_link') != 'tcp':
        raise ValueError('expected link0/tcp manipulator group')
    limits = {}
    for joint in JOINTS:
        limit = joints[joint].find('limit')
        if limit is None or joint not in joint_limits:
            raise ValueError('missing live joint limits: ' + joint)
        extra = joint_limits[joint]
        low, high = float(limit.get('lower')), float(limit.get('upper'))
        velocity = float(limit.get('velocity'))
        if extra.get('has_position_limits') is True:
            low, high = max(low, float(extra['min_position'])), min(high, float(extra['max_position']))
        if extra.get('has_velocity_limits') is not True or extra.get('has_acceleration_limits') is not True:
            raise ValueError('complete velocity/acceleration limits required')
        velocity = min(velocity, float(extra['max_velocity']))
        acceleration = float(extra['max_acceleration'])
        if not all(math.isfinite(v) for v in (low, high, velocity, acceleration)) or low >= high or min(velocity, acceleration) <= 0:
            raise ValueError('invalid joint limits')
        limits[joint] = dict(lower=low, upper=high, velocity=velocity, acceleration=acceleration)
    pairs = []
    for e in semantic.findall('disable_collisions'):
        pair = (e.get('link1'), e.get('link2'))
        if not set(pair) <= links or pair[0] == pair[1]:
            raise ValueError('foreign collision exclusion')
        pairs.append(sorted(pair))
    meshes = {}
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename', '')
        path = resolve_mesh(uri, resource_roots)
        meshes[uri] = dict(path=str(path), sha256=file_hash(path))
    data = dict(schema_version=1, kind='snucem_external', model_id=MODELS[name],
                urdf=urdf, srdf=srdf, joint_limits=limits, collision_pairs=sorted(pairs),
                meshes=meshes, endpoint_tcp_m=[0., 0., 0.], spray_tool_axis='+z',
                base_frame='link0', tcp_frame='tcp')
    return dict(data, fingerprint=digest(data))


def load_descriptor(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('invalid external model descriptor')
    expected = data.pop('fingerprint', '')
    if digest(data) != expected or data.get('kind') != 'snucem_external' or data.get('schema_version') != 1:
        raise ValueError('external model fingerprint mismatch')
    data['fingerprint'] = expected
    return data


def save_descriptor(model, state_root):
    from .config import owned_path
    directory = owned_path(state_root, 'models')
    directory.mkdir(parents=True, exist_ok=True)
    path = owned_path(state_root, 'models/'+model['fingerprint']+'.json')
    if not path.exists():
        temporary = owned_path(state_root, 'models/'+model['fingerprint']+'.tmp')
        temporary.write_text(json.dumps(model, sort_keys=True, allow_nan=False), encoding='utf-8')
        temporary.replace(path)
    return path


def external_profile(path, model_id, tool_axis):
    import numpy as np
    from rbpodo_painting_control.spray_eoat import SprayEoatProfile
    data = load_descriptor(path)
    if data['model_id'] != model_id or data['spray_tool_axis'] != tool_axis or data['endpoint_tcp_m'] != [0., 0., 0.]:
        raise ValueError('external model/tool identity mismatch')
    # The complete tool is already part of robot_description, at its real TCP.
    # Collision data stays in MoveIt; no second attached mesh is fabricated.
    return SprayEoatProfile(data['fingerprint'], np.zeros(3), np.empty((0, 3)), np.empty((0, 3), dtype=int))
