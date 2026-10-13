"""The official WS lease, without a camera SDK or replacement HTTP server."""
import asyncio
import importlib.util
import json
from pathlib import Path

import pytest
import websockets


def load_module():
    path = Path(__file__).resolve().parents[2] / 'scripts/construct_camera_keepalive.py'
    assert path.exists(), 'Construct needs an official camera stream lease'
    spec = importlib.util.spec_from_file_location('camera_keepalive', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('notice', [
    {'type': 'stream_stopped', 'hw_id': 'test-camera', 'reason': 'disconnected'},
    {'type': 'subscribe_rejected', 'hw_id': 'test-camera', 'reason': 'camera_not_connected'},
])
def test_local_lease_and_stream_loss_are_observable(tmp_path, capsys, notice):
    module = load_module()
    config = tmp_path / 'camera.json'
    config.write_text(json.dumps({'hw_id': 'test-camera', 'camera_id': 'stream://127.0.0.1:30000'}))
    requests = []

    async def scenario():
        async def server(ws, *_):
            requests.append(json.loads(await ws.recv()))
            if notice['type'] == 'stream_stopped':
                await ws.send(json.dumps({'type': 'subscribed', 'hw_id': 'test-camera',
                    'stream': {'fps': 10, 'jpeg_quality': 90},
                    'intrinsics': {'fx': 500, 'fy': 500, 'cx': 640, 'cy': 360},
                    'depth': {'dtype': 'uint16', 'unit': 'mm', 'invalid': 0},
                    'local_raw_endpoint': 'ipc:///tmp/test-camera.raw'}))
            await ws.send(json.dumps(notice))
            await ws.wait_closed()
        async with websockets.serve(server, '127.0.0.1', 0) as listener:
            port = listener.sockets[0].getsockname()[1]
            with pytest.raises(RuntimeError, match=notice['type']):
                await module.keepalive(config, f'ws://127.0.0.1:{port}/ws/camera-stream')
    asyncio.run(scenario())
    assert requests == [{'op': 'subscribe', 'hw_ids': ['test-camera'], 'delivery': 'local'}]
    assert ('CONSTRUCT_CAMERA_STREAMING' in capsys.readouterr().out) == (notice['type'] == 'stream_stopped')


def test_real_camera_identity_is_rejected_before_connect(tmp_path):
    module = load_module()
    config = tmp_path / 'camera.json'
    config.write_text(json.dumps({'hw_id': 'test-camera', 'camera_id': '12345'}))
    with pytest.raises(ValueError, match='simulation camera'):
        asyncio.run(module.keepalive(config))
