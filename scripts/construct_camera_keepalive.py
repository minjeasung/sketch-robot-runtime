#!/usr/bin/env python3
"""Hold the official Michelo local-delivery lease for the original ROS bridge.

No SDK, image processing, HTTP server, or robot commands live here. Closing this
client returns its lease; Michelo owns stream lifetime and other subscribers.
"""
import argparse
import asyncio
import json
from pathlib import Path
from urllib.parse import urlsplit

import websockets


async def keepalive(camera_config, uri='ws://127.0.0.1:8101/ws/camera-stream'):
    camera = json.loads(Path(camera_config).read_text())
    if camera.get('camera_id') != 'stream://127.0.0.1:30000' or not camera.get('hw_id'):
        raise ValueError('Expected the Construct simulation camera')
    address = urlsplit(uri)
    if (address.scheme != 'ws' or address.hostname not in ('127.0.0.1', 'localhost', '::1')
            or address.username or address.password or address.path != '/ws/camera-stream'
            or address.query or address.fragment):
        raise ValueError('Michelo WebSocket must be a local /ws/camera-stream endpoint')
    hw_id = camera['hw_id']
    async with websockets.connect(uri, max_size=1024 * 1024, open_timeout=10) as ws:
        await ws.send(json.dumps({'op': 'subscribe', 'hw_ids': [hw_id], 'delivery': 'local'}))
        ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=45))
        if (ack.get('type') != 'subscribed' or ack.get('hw_id') != hw_id
                or not str(ack.get('local_raw_endpoint', '')).startswith('ipc:///')):
            raise RuntimeError(f'Official camera subscription failed: {ack}')
        print(f'CONSTRUCT_CAMERA_STREAMING hw_id={hw_id}', flush=True)
        async for message in ws:
            # Local delivery carries state only. Any later notice invalidates
            # the current perception session; let the supervisor expose failure.
            raise RuntimeError(f'Official camera lease changed: {message}')
        raise RuntimeError('Official camera lease disconnected')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-config', type=Path,
                        default=Path(__file__).resolve().parents[1] / '.runtime/construct/zed-camera.json')
    args = parser.parse_args()
    try:
        asyncio.run(keepalive(args.camera_config))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
