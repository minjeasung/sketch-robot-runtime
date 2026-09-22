"""Read-only Outpost raw IPC contract. No camera SDK or control requests.

Wire contract: Airsave Outpost/michelo-library 0.87.0, as documented by
SNUCEM_Robot at 342b284be828a07c1afd266380087d18c157e998.
"""
import json
import math
import os
from pathlib import Path
import stat
import time
from urllib.parse import quote, urlsplit
from urllib.request import urlopen

import numpy as np


def validate_origin(value):
    url = urlsplit(value)
    if (url.scheme != 'http' or url.hostname not in ('localhost', '127.0.0.1', '::1')
            or url.username or url.password or url.path not in ('', '/')
            or url.query or url.fragment):
        raise ValueError('Outpost must be a loopback HTTP origin on this robot PC')
    if url.port is not None and not 1 <= url.port <= 65535:
        raise ValueError('Invalid Outpost port')
    return value.rstrip('/')


def get_json(origin, path):
    with urlopen(validate_origin(origin) + path, timeout=2) as response:
        return json.load(response)


def camera_status(origin, hw_id, serial, kind, *, check_access=True):
    if not hw_id or not serial:
        raise ValueError(f'{kind}: configure both Outpost hardware ID and calibrated camera serial')
    status = get_json(origin, '/cameras/' + quote(hw_id, safe=''))
    validate_status(status, hw_id, serial, kind, check_access=check_access)
    return status


def validate_status(status, hw_id, serial, kind, *, check_access=True):
    if (status.get('hw_id') != hw_id or str(status.get('camera_id')) != serial
            or status.get('camera_type') != kind):
        raise ValueError(f'{kind}: camera identity/serial mismatch; check calibration')
    if status.get('state') != 'streaming':
        raise ValueError(f'{kind}: start camera streaming in Michelo first')
    if type(status.get('generation')) is not int or status['generation'] < 1:
        raise ValueError('Invalid camera generation')
    intrinsics(status.get('intrinsics', {}))
    resolution = status.get('resolution')
    if (not isinstance(resolution, list) or len(resolution) != 2
            or any(type(n) is not int or n <= 0 for n in resolution)):
        raise ValueError('Invalid camera resolution')
    endpoint = status.get('local_raw_endpoint', '')
    if not endpoint.startswith('ipc:///') or '\x00' in endpoint:
        raise ValueError('Outpost must provide an absolute raw IPC endpoint')
    path = Path(endpoint[6:])
    if check_access:
        try:
            mode = path.stat().st_mode
            accessible = stat.S_ISSOCK(mode) and os.access(path, os.W_OK)
        except OSError:
            accessible = False
        if not accessible:
            raise ValueError(f'Raw IPC is not accessible: {path}. Run the ROS bridge under '
                             'the Outpost account, or have its owner grant socket access.')


def intrinsics(values):
    result = {key: float(values[key]) for key in ('fx', 'fy', 'cx', 'cy')}
    if not all(math.isfinite(v) for v in result.values()) or min(result['fx'], result['fy']) <= 0:
        raise ValueError('Invalid camera intrinsics')
    return result


def decode_frame(parts, status, kind):
    """Return timestamp, sequence, RGB, depth metres and optical XYZ metres.

    D405 retains SDK sub-mm XYZ; never reconstruct it from PNG/uint16 depth.
    RGB must be aligned to the same depth grid. Header indices include topic.
    """
    if len(parts) < 4 or parts[0] != status['hw_id'].encode():
        raise ValueError('Wrong raw camera topic')
    header = json.loads(parts[1])
    if header.get('hw_id') != status['hw_id'] or header.get('generation') != status['generation']:
        raise ValueError('Camera generation/identity changed; restart perception')
    for key in ('capture_timestamp_ns', 'seq'):
        if type(header.get(key)) is not int or header[key] < (1 if key == 'capture_timestamp_ns' else 0):
            raise ValueError('Invalid frame timestamp/sequence')
    arrays, indices = {}, set()
    wanted = {'rgb', 'depth'} | ({'point_cloud'} if kind == 'realsense' else set())
    for ch in header['channels']:
        key = ch.get('kind')
        if key not in wanted:
            continue
        index, shape = ch.get('part'), ch.get('shape')
        if key in arrays or type(index) is not int or not 2 <= index < len(parts) or index in indices:
            raise ValueError('Duplicate/invalid raw channel')
        if (not isinstance(shape, list) or not shape
                or any(type(n) is not int or n < 0 for n in shape)):
            raise ValueError('Invalid raw array shape')
        dtype = np.dtype(ch['dtype'])
        allowed = {'rgb': (np.dtype('uint8'),), 'depth': (np.dtype('uint16'), np.dtype('float32')),
                   'point_cloud': (np.dtype('float32'),)}[key]
        if ch.get('encoding') != 'raw' or dtype not in allowed:
            raise ValueError('Unsupported raw encoding/dtype')
        if len(parts[index]) != math.prod(shape) * dtype.itemsize:
            raise ValueError('Raw payload size mismatch')
        arrays[key] = np.frombuffer(parts[index], dtype=dtype).reshape(shape)
        indices.add(index)
    if set(arrays) != wanted:
        raise ValueError('Enable RGB and depth streams (D405 also requires SDK point_cloud)')
    depth, rgb = arrays['depth'], arrays['rgb']
    if (depth.ndim != 2 or min(depth.shape) < 1 or rgb.shape != (*depth.shape, 3)
            or list(depth.shape[::-1]) != status['resolution']
            or header.get('frame_size') != status['resolution']):
        raise ValueError('RGB/depth calibration grid mismatch; align RGB to depth')
    metres = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
    valid = np.isfinite(metres) & (metres > 0)
    if kind == 'realsense':
        points = arrays['point_cloud']
        if (depth.dtype != np.uint16 or points.shape != (int(valid.sum()), 3)
                or not np.isfinite(points).all() or np.any(points[:, 2] <= 0)
                or np.any(np.abs(points[:, 2] - depth[valid]) > 1.0)):
            raise ValueError('D405 SDK XYZ does not match depth pixels/units')
        cloud = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
        cloud[valid] = points * np.float32(0.001)
    else:
        k = intrinsics(status['intrinsics'])
        v, u = np.indices(depth.shape, dtype=np.float32)
        cloud = np.stack(((u-k['cx'])*metres/k['fx'], (v-k['cy'])*metres/k['fy'], metres), -1)
        cloud[~valid] = np.nan
    metres[~valid] = np.nan
    return header['capture_timestamp_ns'], header['seq'], rgb, metres, cloud


class FrameGuard:
    def __init__(self, max_age=1.0):
        self.max_age, self.sequence, self.stamp = max_age, -1, 0

    def accept(self, stamp, sequence, now_ns=None):
        age = ((time.time_ns() if now_ns is None else now_ns) - stamp) / 1e9
        if not -0.1 <= age <= self.max_age or sequence <= self.sequence or stamp <= self.stamp:
            return False
        self.sequence, self.stamp = sequence, stamp
        return True
