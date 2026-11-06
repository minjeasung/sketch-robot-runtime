"""Observation contracts tested without constructing a ROS context."""
import json
import threading
from types import SimpleNamespace
import pytest
from sketch_control.system_ros_monitor import RosMonitor


def monitor():
    result=RosMonitor.__new__(RosMonitor)
    result.lock=threading.Lock()
    result.nodes=[]
    result._motion_sources={}
    result.updated=result.readiness_at=result.execution_at=result.welding_at=0.
    result.readiness=result.execution=result.welding=None
    return result


@pytest.mark.parametrize('source', ['readiness','welding'])
def test_busy_observation_remains_inhibited_after_heartbeat_expires(source, monkeypatch):
    import sketch_control.system_ros_monitor as module
    now=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    observer=monitor()
    observer._receive(source,SimpleNamespace(data=json.dumps({'running':True,'state':'RUNNING'})))
    assert observer.snapshot().get('motion_busy') is True
    now[0]=20.
    snapshot=observer.snapshot()
    assert snapshot.get(source) is None
    assert snapshot.get('motion_busy') is True
    observer._receive(source,SimpleNamespace(data=json.dumps({'running':False,'state':'STOPPED'})))
    assert observer.snapshot().get('motion_busy') is False


def test_idle_from_other_process_does_not_clear_welding_motion():
    observer=monitor()
    observer._receive('welding',SimpleNamespace(data='{"running": true}'))
    observer._receive('readiness',SimpleNamespace(data='{"running": false}'))
    assert observer.snapshot().get('motion_busy') is True


def test_missing_running_field_cannot_clear_reported_motion():
    observer=monitor()
    observer._receive('welding',SimpleNamespace(data='{"running": true}'))
    observer._receive('welding',SimpleNamespace(data='{"error": "feedback lost"}'))
    assert observer.snapshot().get('motion_busy') is True


def test_stopping_stays_latched_until_explicit_idle_for_both_flags():
    observer=monitor()
    observer._receive('readiness',SimpleNamespace(data='{"running": false, "stopping": true}'))
    assert observer.snapshot()['motion_busy'] is True
    observer._receive('readiness',SimpleNamespace(data='{"running": false}'))
    assert observer.snapshot()['motion_busy'] is True
    observer._receive('readiness',SimpleNamespace(data='{"stopping": false, "running": false}'))
    assert observer.snapshot()['motion_busy'] is False
