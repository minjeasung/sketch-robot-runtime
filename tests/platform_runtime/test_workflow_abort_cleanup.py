"""Abort cleanup may stop an unready session without targeting a replacement."""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import check_platform_workflow as m


def observer(tmp_path,monkeypatch):
    node=object.__new__(m.PlatformObserver)
    node.assets=tmp_path;node.session='session';node.bundle=dict(revision='revision',runtime_revision=2,selection=dict(execution_mode='simulation'))
    node.latest={};node.publishers={};node.sent=[]
    node.node=NS(create_publisher=lambda *a:NS(get_subscription_count=lambda:1,publish=node.sent.append))
    node.guard=lambda:(_ for _ in ()).throw(ValueError('runtime readiness identity mismatch'))
    monkeypatch.setattr(m.time,'monotonic',lambda:10.)
    monkeypatch.setattr(m.Observer,'wait',lambda self,predicate,seconds,label:predicate())
    (tmp_path/'bundle.json').write_text(json.dumps(node.bundle))
    marker=dict(pid=os.getpid(),session_id='session',asset_revision='revision',scene_revision='revision',runtime_revision=2,updated_monotonic=9.99,ready=False)
    (tmp_path/'simulation-ready.json').write_text(json.dumps(marker))
    return node,marker


def test_abort_does_not_recurse_into_normal_readiness_gate(tmp_path,monkeypatch):
    node,_=observer(tmp_path,monkeypatch);message=NS(data=True)
    node.publish('/motion_abort',message)
    assert node.sent==[message]


@pytest.mark.parametrize('field,value',[('session_id','replacement'),('asset_revision','other'),
    ('runtime_revision',3),('updated_monotonic',7.)])
def test_abort_refuses_replacement_or_expired_destination(tmp_path,monkeypatch,field,value):
    node,marker=observer(tmp_path,monkeypatch);marker[field]=value
    (tmp_path/'simulation-ready.json').write_text(json.dumps(marker))
    with pytest.raises(ValueError):node.publish('/motion_abort',NS(data=True))
    assert not node.sent


def test_other_workflow_commands_retain_normal_readiness_gate(tmp_path,monkeypatch):
    node,_=observer(tmp_path,monkeypatch);node.execute_authorized=True
    with pytest.raises(ValueError,match='runtime readiness identity mismatch'):
        node.publish('/sketch_execute',NS(data=True))
    assert not node.sent


def test_abort_rechecks_identity_after_subscriber_discovery(tmp_path,monkeypatch):
    node,marker=observer(tmp_path,monkeypatch)
    counts=iter([0,1])
    node.node.create_publisher=lambda *a:NS(get_subscription_count=lambda:next(counts),publish=node.sent.append)
    def replacement(self,predicate,seconds,label):
        assert seconds==2. and predicate()
        (tmp_path/'simulation-ready.json').write_text(json.dumps({**marker,'session_id':'replacement'}))
    monkeypatch.setattr(m.Observer,'wait',replacement)
    with pytest.raises(ValueError,match='identity'):node.publish('/motion_abort',NS(data=True))
    assert not node.sent


def test_abort_preparation_discovers_without_publishing_and_hot_send_rechecks_identity(tmp_path,monkeypatch):
    node,marker=observer(tmp_path,monkeypatch)
    node.prepare_abort()
    assert '/motion_abort' in node.publishers and node.sent==[]
    publisher=node.publishers['/motion_abort']
    def no_discovery(*args):
        raise AssertionError('already discovered stop must not wait again')
    monkeypatch.setattr(m.Observer,'wait',no_discovery)
    node.publish('/motion_abort',NS(data=True))
    assert len(node.sent)==1 and node.publishers['/motion_abort'] is publisher
    (tmp_path/'simulation-ready.json').write_text(json.dumps({**marker,'session_id':'replacement'}))
    with pytest.raises(ValueError,match='identity'):node.publish('/motion_abort',NS(data=True))
    assert len(node.sent)==1


def test_workflow_guard_records_exact_rejected_marker(tmp_path,monkeypatch):
    node,marker=observer(tmp_path,monkeypatch)
    (tmp_path/'robot.urdf').write_text('<robot><ros2_control><hardware><plugin>isaac_painting_hardware/IsaacPaintingSystem</plugin></hardware></ros2_control></robot>')
    (tmp_path/'managed-ready.json').write_text(json.dumps(marker))
    with pytest.raises(ValueError,match='runtime readiness false: simulation-ready.json'):
        m.PlatformObserver.guard(node)
    (tmp_path/'simulation-ready.json').write_text(json.dumps({**marker,'ready':True}))
    assert node.readiness_rejection['markers']['simulation-ready.json']['ready'] is False
    assert node.readiness_rejection['checked_monotonic']==10.
