"""Explicit arm selection; task geometry remains in the common TCP frame."""
from pathlib import Path
import math
import json
import xml.etree.ElementTree as ET

DEFAULT_MODEL = 'rb10_1300e_u'
MODEL_LABELS = {DEFAULT_MODEL: 'RB10-1300E', 'rb20_1900es': 'RB20-1900ES'}
JOINT_NAMES = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')


def validate_model(model_id):
    if not isinstance(model_id, str) or model_id not in MODEL_LABELS:
        raise ValueError('model_id must be rb10_1300e_u or rb20_1900es')
    return model_id


def model_calibration_files(workspace, model_id):
    validate_model(model_id)
    root = Path(workspace).expanduser().resolve()
    if model_id != DEFAULT_MODEL:
        root = root / 'calibration' / model_id
    return dict(zed_calibration_file=str(root / 'zed_d405_apriltag_calibration.json'),
                d405_calibration_file=str(root / 'd405_eyeinhand_charuco_calibration.json'))


def validate_calibration_files(paths):
    for name, key in (('zed_calibration_file', 'T_world_zed_optical'),
                      ('d405_calibration_file', 'T_d405_optical_to_tcp')):
        path = paths[name]
        try:
            pose = json.loads(Path(path).read_text())[key]
            translation = [float(v) for v in pose['translation']]
            rotation = [float(v) for v in pose['rotation_xyzw']]
            if (len(translation) != 3 or len(rotation) != 4
                    or not all(math.isfinite(v) for v in translation + rotation)
                    or abs(sum(v*v for v in rotation) - 1) > .01):
                raise ValueError('Invalid rigid transform')
        except (OSError, ValueError, TypeError, KeyError) as exc:
            raise ValueError(f'Robot-specific camera calibration required: {path} ({exc})') from exc


def model_srdf(model_id=DEFAULT_MODEL, path=None):
    """Do not transfer RB10 taught poses or sampled collision exclusions to RB20."""
    validate_model(model_id)
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('sketch_control')) / 'config/rbpodo_named.srdf'
    text = Path(path).read_text()
    if model_id == DEFAULT_MODEL:
        return text
    root = ET.fromstring(text)
    for element in list(root):
        if element.tag == 'group_state' or (
            element.tag == 'disable_collisions' and element.get('reason') == 'Never'
        ):
            root.remove(element)
    return ET.tostring(root, encoding='unicode')


def model_joint_limits(model_id, description_share=None, moveit_share=None):
    """Use the selected arm YAML and the same MoveIt position overrides."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    validate_model(model_id)
    description_share = Path(description_share or get_package_share_directory('rbpodo_description'))
    moveit_share = Path(moveit_share or get_package_share_directory('rbpodo_moveit_config'))
    joints = yaml.safe_load((description_share / 'robots' / model_id / 'joint.yaml').read_text())
    overrides = yaml.safe_load((moveit_share / 'config/joint_limits.yaml').read_text())['joint_limits']
    limits = {}
    for name in JOINT_NAMES:
        low, high = (float(joints[name]['limit'][key]) for key in ('lower', 'upper'))
        override = overrides.get(name, {})
        if override.get('has_position_limits') is True:
            low, high = float(override['min_position']), float(override['max_position'])
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError(f'Invalid joint limits: {model_id}/{name}')
        limits[name] = (low, high)
    return limits
