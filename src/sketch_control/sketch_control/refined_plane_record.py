"""Decode geometry carried atomically by its accepted D405 generation."""
import math
from geometry_msgs.msg import PoseStamped


def make_refined_plane_record(frame_id, position, orientation, stamp):
    return {'frame_id': str(frame_id),
            'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
            'position': [float(value) for value in position],
            'orientation': [float(value) for value in orientation]}


def refined_pose_from_status(payload):
    """Return an identity-bound pose, or None for a metadata-only old status.

    A bare PoseStamped cannot establish this identity. Missing geometry stays
    unready; malformed geometry never falls back to a later unbound pose.
    """
    if 'refined_plane' not in payload:
        return None
    if (payload.get('mode') != 'work_area' or payload.get('accepted') is not True
            or not str(payload.get('work_area_id', '')).strip()
            or not str(payload.get('plane_generation_id', '')).strip()):
        raise ValueError('refined geometry requires accepted work-area identity')
    record = payload['refined_plane']
    if not isinstance(record, dict):
        raise ValueError('refined geometry must be an object')
    frame = record.get('frame_id')
    if not isinstance(frame, str) or not frame.strip() or frame != payload.get('target_frame'):
        raise ValueError('refined frame does not match accepted target frame')
    stamp = record.get('stamp')
    if not isinstance(stamp, dict) or stamp != payload.get('capture_stamp'):
        raise ValueError('refined stamp does not match accepted capture stamp')
    sec, nanosec = stamp.get('sec'), stamp.get('nanosec')
    if (type(sec) is not int or type(nanosec) is not int or sec < 0
            or not 0 <= nanosec < 1000000000 or sec == nanosec == 0):
        raise ValueError('refined capture stamp must be nonzero and valid')
    def vector(key, count):
        value = record.get(key)
        if not isinstance(value, (list, tuple)) or len(value) != count:
            raise ValueError(f'refined {key} has invalid dimensions')
        if any(type(item) not in (int, float) or not math.isfinite(item) for item in value):
            raise ValueError(f'refined {key} must be finite numeric values')
        return [float(item) for item in value]
    position, orientation = vector('position', 3), vector('orientation', 4)
    if abs(sum(value * value for value in orientation) - 1.0) > 1e-6:
        raise ValueError('refined orientation must be a unit quaternion')
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec, pose.header.stamp.nanosec = sec, nanosec
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = orientation
    return pose
