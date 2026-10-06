import json
import sys
from pathlib import Path
import pytest
from test_contracts import module, ROOT


def config(tmp_path):
    upstream = tmp_path/'upstream'
    upstream.mkdir()
    return module('config').AddonConfig(str(upstream), str(tmp_path/'install'), str(tmp_path/'state'))


def test_runtime_stops_only_owned_children_and_prepare_has_no_execution_command(tmp_path):
    m = module('runtime')
    cfg = config(tmp_path)
    commands = m.component_commands(cfg, 'config.json', 'profile.json')
    assert set(commands) == {'perception', 'bridge', 'model', 'projector', 'generator', 'preview'}
    assert not any('SNUCEM_Robot' in ' '.join(v) or 'rb20_spray.executor' in ' '.join(v) for v in commands.values())
    with pytest.raises(ValueError):
        m.Runtime(cfg, tmp_path/'config.json').start('external_stack')


def test_api_auth_blocks_mutations_and_status_does_not_expose_token(tmp_path):
    from fastapi.testclient import TestClient
    m = module('api')
    cfg = config(tmp_path)
    from dataclasses import replace
    cfg = replace(cfg, api_token='x'*32)
    class Runtime:
        def status(self):
            return {'owned': {}, 'external': {'stack': 'not managed'}}
        def prepare(self):
            self.prepared = True
            return self.status()
        def shutdown(self):
            self.stopped = True
            return self.status()
    runtime = Runtime()
    client = TestClient(m.create_app(cfg, runtime, ROOT/'web'))
    assert client.post('/prepare').status_code == 401
    assert client.post('/prepare', headers={'Authorization': 'Bearer '+'x'*32}).status_code == 200
    assert runtime.prepared
    assert 'api_token' not in client.get('/status').text
    assert client.get('/addon/capabilities').json()['process_modes'] == ['spray']
    assert 'SNUCEM' in client.get('/').text
    assert 'value="paint"' not in client.get('/sketch/').text
    assert client.get('/sketch/js/app.js').status_code == 200


def test_child_failure_rolls_back_only_new_children(tmp_path, monkeypatch):
    m = module('runtime')
    cfg = config(tmp_path)
    r = m.Runtime(cfg, tmp_path/'config.json')
    calls = []
    def start(name, profile=''):
        calls.append(('start', name))
        if name == 'bridge':
            raise RuntimeError('child failed')
        r.children[name] = object()
    monkeypatch.setattr(r, 'start', start)
    monkeypatch.setattr(r, 'stop', lambda name: calls.append(('stop', name)))
    monkeypatch.setattr(r, 'preflight', lambda: None)
    monkeypatch.setattr(r, 'wait_model', lambda: 'profile.json')
    with pytest.raises(RuntimeError):
        r.prepare()
    assert ('stop', 'model') in calls
    assert all(name != 'external_stack' for _, name in calls)
