"""Contact dynamics use simulation time; all safety ages retain steady time."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from sketch_control.moveit_executor import MoveItExecutor,ContactSearchConfig


def executor(monkeypatch,simulation=True):
    wall={'now':200.};ros={'now':100.};aborts=[];steps=[];contact_times=[]
    monkeypatch.setattr('sketch_control.moveit_executor.time.monotonic',lambda:wall['now'])
    clock=SimpleNamespace(ros_time_is_active=simulation,now=lambda:SimpleNamespace(nanoseconds=int(ros['now']*1e9)))
    node=SimpleNamespace(get_clock=lambda:clock,_motion_abort_requested=False,_contact_search_step_active=False,
        _contact_search_mode_ack_blockers=lambda:(),_safety_status_time=200.,ft_required_timeout_s=.2,
        _safety_status={'ft_valid':True,'tf_valid':True,'abort_latched':False},_controller_fault=False,
        _current_tcp_pose_np=lambda:object(),_contact_search_distance_m=.0095,
        contact_search_config=ContactSearchConfig(step_m=.0005,max_distance_m=.02,timeout_s=30.),
        _contact_search_contact_sensed=lambda now:contact_times.append(now) or False,
        _plan_contact_search_step=lambda step,distance:steps.append(distance),
        _set_contact_collision_allowed=lambda *args:None,_request_motion_abort=aborts.append,
        _fail_workflow_known_safe=aborts.append)
    MoveItExecutor._begin_contact_search_progress(node,wall['now'])
    return node,wall,ros,clock,aborts,steps,contact_times


def test_slow_simulator_does_not_exhaust_search_before_configured_progress(monkeypatch):
    node,wall,ros,clock,aborts,steps,contact_times=executor(monkeypatch)
    wall['now']=230.474;ros['now']=112.450;node._safety_status_time=wall['now']
    MoveItExecutor._contact_search_iteration(node,object())
    assert steps==[.0005] and not aborts
    assert contact_times==[230.474], 'Contact/sensor freshness must still receive steady time'
    wall['now']=275.;ros['now']=130.;node._safety_status_time=wall['now']
    MoveItExecutor._contact_search_iteration(node,object())
    assert aborts==['CONTACT_NOT_FOUND']


def test_real_execution_retains_thirty_second_wall_deadline(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch,False)
    wall['now']=230.474;ros['now']=100.;node._safety_status_time=wall['now']
    MoveItExecutor._contact_search_iteration(node,object())
    assert not steps and aborts==['CONTACT_NOT_FOUND']


def test_paused_simulation_cannot_freeze_sensor_freshness(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    wall['now']=200.3  # /clock remains 100, safety receipt remains 200.
    MoveItExecutor._contact_search_iteration(node,object())
    assert not steps and aborts==['FT_STALE']


@pytest.mark.parametrize('change',['reset','clock_domain'])
def test_search_rejects_reset_or_clock_domain_change(monkeypatch,change):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    if change=='reset':ros['now']=99.
    else:clock.ros_time_is_active=False
    MoveItExecutor._contact_search_iteration(node,object())
    assert not steps and len(aborts)==1 and 'CONTACT_SEARCH_CLOCK' in aborts[0]


def test_distance_bound_is_independent_of_clock_rate(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    node._contact_search_distance_m=.02
    MoveItExecutor._contact_search_iteration(node,object())
    assert not steps and aborts==['CONTACT_NOT_FOUND']


def test_mode_ack_timeout_stays_on_steady_clock_during_pause(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    node._contact_search_mode_ack_blockers=lambda:('MODE_NOT_ACKNOWLEDGED',)
    node._contact_search_mode_published_at=200.;node.contact_search_mode_ack_timeout_s=1.
    node._schedule_process_once=lambda *args:pytest.fail('Expired wall ACK deadline cannot be rescheduled')
    wall['now']=201.1
    MoveItExecutor._wait_for_contact_search_mode_ack(node,object())
    assert not steps and 'mode ACK timeout' in aborts[0]


def test_start_search_initializes_progress_basis_before_motion(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    node._contact_search_progress_context=None
    node.painting_force_enabled=False;node.real_painting_enabled=True
    node._publish_painting_command=lambda *args,**kwargs:{'published_at_s':wall['now']}
    node._transition_execution_state=lambda *args,**kwargs:None
    MoveItExecutor._start_contact_search(node,object())
    assert node._contact_search_progress_context==('ros',100.,100.) and not aborts


def test_backward_jump_is_rejected_even_when_after_search_start(monkeypatch):
    node,wall,ros,clock,aborts,steps,_=executor(monkeypatch)
    ros['now']=110.
    assert MoveItExecutor._contact_search_progress_elapsed(node,wall['now'])==10.
    ros['now']=109.
    MoveItExecutor._contact_search_iteration(node,object())
    assert not steps and aborts==['CONTACT_SEARCH_CLOCK_RESET']
