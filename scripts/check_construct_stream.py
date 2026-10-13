#!/usr/bin/env python3
"""Verify actual SDK -> official Outpost -> official Michelo frames."""
import asyncio
import json
from pathlib import Path
import struct
import time
import urllib.request

import cv2
import numpy as np
import websockets
import zmq
import zmq.asyncio

ROOT = Path(__file__).resolve().parents[1]


def decode_message(data):
    parts = []
    offset = 0
    while offset < len(data):
        length = struct.unpack_from('>I', data, offset)[0]
        offset += 4
        parts.append(data[offset:offset + length])
        offset += length
    assert offset == len(data)
    return json.loads(parts[0]), parts


async def main():
    hw_id = json.loads((ROOT / '.runtime/construct/zed-camera.json').read_text())['hw_id']
    report = {'hw_id': hw_id, 'samples': []}
    async with websockets.connect('ws://127.0.0.1:8101/ws/camera-stream', max_size=16*1024*1024) as ws:
        await ws.send(json.dumps({'op': 'subscribe', 'hw_ids': [hw_id]}))
        ack = json.loads(await asyncio.wait_for(ws.recv(), 15))
        assert ack['type'] == 'subscribed', ack
        report['michelo_ack'] = ack
        with urllib.request.urlopen(f'http://127.0.0.1:8100/cameras/{hw_id}', timeout=5) as response:
            status = json.load(response)
        report['outpost_status'] = status
        context = zmq.asyncio.Context()
        sock = context.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, hw_id.encode())
        sock.connect(status['local_raw_endpoint'])
        try:
            raw = await asyncio.wait_for(sock.recv_multipart(), 10)
            header = json.loads(raw[1])
            report['raw_header'] = header
            raw_arrays = {}
            for channel in header['channels']:
                arr = np.frombuffer(raw[channel['part']], dtype=channel['dtype']).reshape(channel['shape'])
                raw_arrays[channel['kind']] = arr
            assert raw_arrays['rgb'].shape == raw_arrays['rgb_right'].shape == (720, 1280, 3)
            assert raw_arrays['depth'].shape == (720, 1280) and raw_arrays['depth'].dtype == np.uint16
            assert np.count_nonzero(raw_arrays['depth']) > 10000
            start = time.monotonic()
            for index in range(30):
                message = await asyncio.wait_for(ws.recv(), 10)
                assert isinstance(message, bytes), message
                header, parts = decode_message(message)
                report['samples'].append({'seq': header['seq'], 'stamp_ns': header['capture_timestamp_ns']})
                assert header['frame_size'] == [1280, 720]
                assert 0 <= time.time_ns() - header['capture_timestamp_ns'] < 2_000_000_000
                channels = {channel['kind']: channel for channel in header['channels']}
                assert {'rgb', 'depth', 'rgb_right'} <= channels.keys()
                if index:
                    assert header['seq'] > report['samples'][index-1]['seq']
                    assert header['capture_timestamp_ns'] > report['samples'][index-1]['stamp_ns']
                if index == 29:
                    for kind, channel in channels.items():
                        arr = cv2.imdecode(np.frombuffer(parts[channel['part']-1], np.uint8), cv2.IMREAD_UNCHANGED)
                        assert arr is not None
                        if kind == 'depth':
                            assert arr.dtype == np.uint16 and np.count_nonzero(arr) > 10000
                            report['depth_valid_fraction'] = float((arr > 0).mean())
                            cv2.imwrite(str(ROOT / 'logs/official-zed-depth.png'), arr)
                            color = cv2.applyColorMap(np.clip(arr / 4000 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
                            cv2.imwrite(str(ROOT / 'logs/official-zed-depth-preview.jpg'), color)
                        else:
                            assert arr.std() > 3
                            cv2.imwrite(str(ROOT / f'logs/official-zed-{kind}.jpg'), arr)
            report['observed_fps'] = 29 / (time.monotonic() - start)
        finally:
            sock.close(0)
            context.term()
    (ROOT / 'logs/official-zed-stream-validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'OFFICIAL_STREAM_VERIFIED: 30 frames, {report["observed_fps"]:.1f} fps, '
          f'depth valid {report["depth_valid_fraction"]:.1%}; raw IPC + Michelo JPEG/PNG16', flush=True)


if __name__ == '__main__':
    asyncio.run(main())
