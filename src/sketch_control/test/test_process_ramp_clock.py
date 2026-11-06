"""Force progress uses physics time; pause never freezes steady safety checks."""
from types import SimpleNamespace
import pytest
from test_moveit_executor_fail_closed import _ramp_executor, _set_guard_sample, _FakeTimer
from sketch_control import moveit_executor as module
M=module.MoveItExecutor


def ramp(monkeypatch,mode='RAMP_UP',simulation=True):
    wall={'now':20.};ros={'now':100.};clocks=[]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall['now'])
    step=SimpleNamespace(mode=mode,force_n=20. if mode=='RAMP_UP' else 0.)
    node,scheduled,aborts,rejections=_ramp_executor(wall,step,'CONTACT_SEARCH' if mode=='RAMP_UP' else 'PAINT',initial_active=mode!='RAMP_UP')
    clock=SimpleNamespace(ros_time_is_active=simulation,now=lambda:SimpleNamespace(nanoseconds=int(ros['now']*1e9)))
    node.get_clock=lambda:clock
    def timer(period,callback,**kw):
        clocks.append(kw.get('clock'))
        return _FakeTimer(callback)
    node.create_timer=timer
    M._start_process_ramp(node,step)
    return node,wall,ros,clock,scheduled,aborts,rejections,clocks


def fresh(node,wall,mode,done=False):
    _set_guard_sample(node,now=wall['now'],sequence=node._guard_status_sequence+1,mode=mode,active=True)
    node._painting_ramp_status={'mode':mode,'force_enable':True,'ramp_complete':done}
    node._painting_ramp_status_sequence+=1
    node._painting_ramp_status_time=node._painting_ramp_status_source_time=wall['now']
    node._painting_contact_feedback_time=wall['now']


@pytest.mark.parametrize('mode',['RAMP_UP','RAMP_DOWN'])
def test_slow_ramp_budget_follows_ros_progress_then_expires(monkeypatch,mode):
    n,w,r,c,scheduled,aborts,rejections,clocks=ramp(monkeypatch,mode)
    w['now']=26.1;r['now']=102.4;fresh(n,w,mode)
    n._process_timer.callback()
    assert not scheduled and not aborts and not rejections
    w['now']=33.;r['now']=105.01;fresh(n,w,mode)
    n._process_timer.callback()
    assert len(rejections)==1 and 'timeout' in rejections[0]


def test_physical_ramp_retains_wall_deadline(monkeypatch):
    n,w,r,c,s,a,reject,clocks=ramp(monkeypatch,simulation=False)
    w['now']=25.1;fresh(n,w,'RAMP_UP');n._process_timer.callback()
    assert len(reject)==1 and 'timeout' in reject[0]


@pytest.mark.parametrize('change',['zero','reset','domain'])
def test_ramp_clock_discontinuity_aborts_before_completion(monkeypatch,change):
    n,w,r,c,s,a,reject,clocks=ramp(monkeypatch)
    w['now']=20.1;r['now']=101.;fresh(n,w,'RAMP_UP');n._process_timer.callback()
    if change=='domain':c.ros_time_is_active=False
    else:r['now']=0. if change=='zero' else 100.5
    w['now']=20.2;fresh(n,w,'RAMP_UP',done=True);n._process_timer.callback()
    assert not s and len(a)==1 and a[0].startswith('RAMP_UP_CLOCK_')


def test_frozen_ramp_clock_keeps_guard_receipt_deadline_and_steady_dispatch(monkeypatch):
    n,w,r,c,s,a,reject,clocks=ramp(monkeypatch)
    w['now']=20.1;fresh(n,w,'RAMP_UP');n._process_timer.callback()
    w['now']=20.7;n._process_timer.callback()
    assert not s and len(a)==1 and 'GUARD' in a[0]
    assert len(clocks)==2 and all(clock.clock_type==module.ClockType.STEADY_TIME for clock in clocks)


def hold(monkeypatch):
    wall={'now':20.};ros={'now':100.};scheduled=[];aborts=[];done=[]
    monkeypatch.setattr(module.time,'monotonic',lambda:wall['now'])
    c=SimpleNamespace(ros_time_is_active=True,now=lambda:SimpleNamespace(nanoseconds=int(ros['now']*1e9)))
    n=SimpleNamespace(get_clock=lambda:c,stationary_paint_hold_s=5.,_motion_abort_requested=False,
      _safety_status_time=20.,_guard_status_time=20.,_painting_contact_feedback_time=20.,ft_required_timeout_s=.2,
      _safety_status={'abort_latched':False,'ft_valid':True,'tf_valid':True,'filtered_wrench':[0.,20.,0.,0.,0.,0.]},
      _guard_status={'forwarding':True,'compliance_enabled':True,'compliance_active':True,'controller_fault':False},
      _painting_contact_confirmed=True,_transition_execution_state=lambda *a:None,
      _request_motion_abort=aborts.append,_publish_execution_status=lambda *a,**kw:None,
      _process_motion_done=lambda pose:done.append(pose),
      _schedule_process_once=lambda dt,cb,**kw:scheduled.append(cb))
    M._start_stationary_paint_hold(n,SimpleNamespace(force_n=20.),'pose')
    return n,wall,ros,scheduled,aborts,done


def test_hold_duration_is_simulation_time_but_receipts_stay_steady(monkeypatch):
    n,w,r,s,a,d=hold(monkeypatch)
    w['now']=32.;r['now']=104.
    n._safety_status_time=n._guard_status_time=n._painting_contact_feedback_time=w['now']
    s.pop()();assert not a and not d
    w['now']=35.;r['now']=105.
    n._safety_status_time=n._guard_status_time=n._painting_contact_feedback_time=w['now']
    s.pop()();assert d==['pose'] and not a


def test_paused_hold_cannot_complete_with_expired_sensor_receipts(monkeypatch):
    n,w,r,s,a,d=hold(monkeypatch)
    w['now']=20.3;s.pop()()
    assert not d and a


def test_hold_clock_reset_aborts(monkeypatch):
    n,w,r,s,a,d=hold(monkeypatch)
    r['now']=99.;s.pop()()
    assert not d and a==['PAINT_HOLD_CLOCK_RESET']
