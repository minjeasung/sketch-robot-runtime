#!/usr/bin/env python3
"""Register/connect Isaac ZED through official Michelo APIs (no server or SDK owner)."""
import json
from pathlib import Path
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
BACKEND = 'http://127.0.0.1:8101'
IDENTITY = 'stream://127.0.0.1:30000'
NAME = 'Construct ZED 2i (Isaac)'


def request(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BACKEND + path, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=45) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f'Michelo {path}: HTTP {exc.code}: {exc.read().decode()}') from exc


def main():
    # Do not open SDK until this launch's local stereo producer is advancing.
    previous = None
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        try:
            timing = json.loads((ROOT / '.runtime/construct/zed-timing.json').read_text())
            stamp = timing['latest_capture_sim_ns']
            fresh = 0 <= time.time_ns() - timing['capture_wall_ns'] < 2_000_000_000
            if fresh and previous is not None and stamp > previous:
                break
            previous = stamp
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(.2)
    else:
        raise TimeoutError('Isaac stereo producer is not advancing')

    existing = [entry for entry in request('/hw-interface/list')['interfaces'] if entry.get('name') == NAME]
    if len(existing) > 1:
        raise RuntimeError('Multiple Construct camera entries; select/remove duplicates in official Michelo UI')
    if existing:
        entry = request('/hw-interface/' + existing[0]['id'])
        # Official detail response wraps the entire HW entry in config.
        entry = entry['config']
        if (entry.get('camera_id') != IDENTITY or entry.get('type') != 'remote_camera'
                or entry.get('camera_type') != 'zed'
                or entry.get('remote') != {'host': '127.0.0.1', 'port': 8100}):
            raise RuntimeError('Existing Construct camera targets a different device; refusing to connect it')
        hw_id = existing[0]['id']
    else:
        created = request('/hw-interface/create', {
            'type': 'remote_camera', 'name': NAME, 'camera_type': 'zed', 'camera_id': IDENTITY,
            'manufacturer': 'stereolabs', 'remote': {'host': '127.0.0.1', 'port': 8100},
            'sdk_prewarm': False,
            'config': {
                'resolution': 'HD720', 'depth_mode': 'NEURAL_LIGHT', 'depth_minimum_distance_m': .2,
                'depth_maximum_distance_m': 10., 'capture_timeout': 2., 'enable_right_rgb': True,
                'stream_fps': 10, 'stream_jpeg_quality': 90, 'stream_channels': ['rgb', 'depth', 'rgb_right'],
                'reconnect_policies': {'task': {'mode': 'manual'}, 'vision_inspection': {'mode': 'manual'}},
            },
        })
        hw_id = created['id']
    result = request(f'/camera/{hw_id}/connect', {'trigger_mode': 'software'})
    path = ROOT / '.runtime/construct/zed-camera.json'
    path.write_text(json.dumps({'hw_id': hw_id, 'camera_id': IDENTITY, 'connect': result}, indent=2) + '\n')
    print(f'CONSTRUCT_ZED_CONNECTED hw_id={hw_id} camera_id={IDENTITY}', flush=True)


if __name__ == '__main__':
    main()
