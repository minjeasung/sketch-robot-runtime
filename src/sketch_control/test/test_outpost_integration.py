"""Camera contracts and launcher integration; no physical camera/robot commands."""
import base64
import json
from pathlib import Path
import socket

import numpy as np
import pytest

from sketch_control.outpost_camera import decode_frame, FrameGuard, validate_origin, validate_status


def sample(kind='zed'):
    status = dict(hw_id='cam', camera_id='serial', camera_type=kind, state='streaming',
                  generation=1, resolution=[2, 2], intrinsics=dict(fx=2, fy=2, cx=0, cy=0),
                  local_raw_endpoint='ipc:///tmp/test.raw')
    rgb = np.arange(12, dtype=np.uint8).reshape(2,2,3)
    depth = np.array([[1000,0],[2000,3000]], dtype=np.uint16)
    arrays = [('rgb', rgb), ('depth', depth)]
    if kind == 'realsense':
        arrays.append(('point_cloud',np.array([[0,0,1000.25],[0,1000,2000.5],[1500,1500,3000.75]],np.float32)))
    header = dict(hw_id='cam', generation=1, seq=2, capture_timestamp_ns=5_000_000_000,
                  frame_size=[2,2], channels=[dict(kind=k, encoding='raw', dtype=str(a.dtype),
                  shape=list(a.shape), part=i+2) for i,(k,a) in enumerate(arrays)])
    return status, [b'cam',json.dumps(header).encode()]+[a.tobytes() for _,a in arrays]


def test_zed_optical_units_and_pixels():
    status, parts = sample()
    stamp, seq, rgb, depth, xyz = decode_frame(parts, status, 'zed')
    assert stamp == 5_000_000_000 and seq == 2
    assert rgb.shape == (2,2,3)
    np.testing.assert_allclose(xyz[1,1], [1.5,1.5,3.0])
    assert np.isnan(depth[0,1]) and np.isnan(xyz[0,1]).all()


def test_d405_preserves_sdk_submillimetre_geometry():
    status, parts = sample('realsense')
    _, _, _, depth, cloud = decode_frame(parts, status, 'realsense')
    assert depth[0,0] == 1.0
    assert cloud[0,0,2] == pytest.approx(1.00025)
    assert cloud[1,1,2] == pytest.approx(3.00075)
    assert np.isnan(cloud[0,1]).all()


@pytest.mark.parametrize('change', ['generation','identity','size','index','duplicate','encoding','timestamp','rgb-grid','missing'])
def test_malformed_frames_fail_closed(change):
    status, parts = sample()
    h = json.loads(parts[1])
    if change == 'generation': h['generation'] = 0
    if change == 'identity': h['hw_id'] = 'another'
    if change == 'size': parts[-1] = parts[-1][:-1]
    if change == 'index': h['channels'][0]['part'] = -1
    if change == 'duplicate': h['channels'].append(h['channels'][0])
    if change == 'encoding': h['channels'][0]['encoding'] = 'jpeg'
    if change == 'timestamp': h['capture_timestamp_ns'] = 0
    if change == 'rgb-grid': h['channels'][0]['shape'] = [1,4,3]
    if change == 'missing': h['channels'] = h['channels'][1:]
    parts[1] = json.dumps(h).encode()
    with pytest.raises(ValueError): decode_frame(parts,status,'zed')


def test_d405_wrong_sdk_order_rejected():
    status, parts = sample('realsense')
    parts[-1] = np.frombuffer(parts[-1],np.float32).reshape(-1,3)[::-1].tobytes()
    with pytest.raises(ValueError,match='SDK XYZ'): decode_frame(parts,status,'realsense')


def test_freshness_monotonicity():
    guard = FrameGuard()
    assert guard.accept(2_000_000_000, 2, 2_100_000_000)
    assert not guard.accept(2_000_000_000, 3, 2_100_000_000)
    assert not guard.accept(2_100_000_000, 2, 2_100_000_000)
    assert not guard.accept(3_000_000_000, 3, 5_000_000_000)
    assert not guard.accept(7_000_000_000, 4, 5_000_000_000)
    assert guard.accept(5_000_000_000, 5, 5_100_000_000)


@pytest.mark.parametrize('origin', ['https://127.0.0.1:8100','http://example.com','http://127.0.0.1/path',
    'http://user:secret@127.0.0.1','http://127.0.0.1?url=x'])
def test_only_local_outpost_origin(origin):
    with pytest.raises(ValueError): validate_origin(origin)


def test_status_identity_and_access(tmp_path):
    status,_ = sample()
    path = tmp_path/'camera.raw'
    status['local_raw_endpoint']='ipc://'+str(path)
    with socket.socket(socket.AF_UNIX) as sock:
        sock.bind(str(path))
        validate_status(status,'cam','serial','zed')
        with pytest.raises(ValueError,match='identity'): validate_status(status,'cam','wrong','zed')
    path.unlink()
    with pytest.raises(ValueError,match='not accessible'): validate_status(status,'cam','serial','zed')


def test_gateway_http_injection_and_auth():
    pytest.importorskip('fastapi')
    import httpx
    from fastapi.testclient import TestClient
    from sketch_control.michelo_gateway import create_gateway
    requests=[]
    def upstream(request):
        requests.append(request)
        if request.url.path == '/console/':
            return httpx.Response(200,headers={'content-type':'text/html','etag':'old'},text='<body><header>Michelo</header></body>')
        return httpx.Response(200,json={'ok':True})
    app=create_gateway(transport=httpx.MockTransport(upstream),token='test-password')
    with TestClient(app,base_url='http://localhost') as client:
        assert client.get('/console/').status_code==401
        auth=('sketch','test-password')
        page=client.get('/console/',auth=auth)
        assert 'Michelo' in page.text and '/__sketch/launcher.js' in page.text
        assert 'etag' not in page.headers
        assert 'authorization' not in requests[-1].headers
        script=client.get('/__sketch/launcher.js',auth=auth).text
        assert "link.target = '_blank'" in script and '8081' in script
        reply=client.post('/cameras/cam/stream/start',json={'fps':15},auth=auth)
        assert reply.json()=={'ok':True}
        assert json.loads(requests[-1].content)=={'fps':15}
        assert client.post('/admin/restart',auth=auth,headers={'Origin':'http://evil.test'}).status_code==403


def test_outpost_prepare_checks_before_robot_start(tmp_path, monkeypatch):
    pytest.importorskip('fastapi')
    from sketch_control.process_supervisor import Supervisor, SupervisorError, build_specs
    from test_system_api import Monitor
    import asyncio
    options,specs=build_specs(tmp_path)
    assert options['camera_backend']=='outpost' and not options['launch_zed_driver']
    assert all('camera_backend:=outpost' in s.command for s in specs)
    supervisor=Supervisor(tmp_path,Monitor())
    with pytest.raises(SupervisorError,match='hardware ID'):
        asyncio.run(supervisor.prepare())
    assert all(r['process'] is None for r in supervisor.records.values())
    with pytest.raises(SupervisorError,match='owns cameras'):
        build_specs(tmp_path,{'camera_backend':'outpost','launch_zed_driver':True})
