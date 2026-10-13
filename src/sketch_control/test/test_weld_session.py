from dataclasses import replace
import pytest
from sketch_control.weld_contract import WeldPlan, validate_catalog
from sketch_control.weld_session import WeldSession
from sketch_control.weld_output import SimWeldOutput
from test_weld_geometry import P
from test_weld_contract import target,catalog

def prepared():
    s=WeldSession(P); s.request_target(target()); s.accept_catalog(validate_catalog(catalog(),target()))
    s.select([{'seam_id':'S1','reverse':False}]); s.apply_settings({}); return s

def planned():
    s=prepared(); identity=s.begin_plan(); plan=WeldPlan('p',identity,(),())
    assert s.accept_plan(identity,plan); return s,plan

def test_changed_settings_reject_late_plan():
    s=prepared(); identity=s.begin_plan(); s.apply_settings({'speed_mps':.02})
    assert not s.accept_plan(identity,WeldPlan('old',identity,(),()))
    assert s.state!='READY'

def test_restart_rejects_old_ack_and_execute():
    s,p=planned(); new=prepared()
    with pytest.raises(ValueError): new.begin_execution(p.plan_id,p.identity)

def test_duplicate_execute_starts_once():
    s,p=planned(); assert s.begin_execution('p',p.identity)==p
    with pytest.raises(ValueError): s.begin_execution('p',p.identity)

def test_stop_waits_for_controller_confirmation():
    s,p=planned(); s.begin_execution('p',p.identity); s.request_stop()
    assert s.state=='STOPPING'
    with pytest.raises(ValueError): s.apply_settings({})
    s.confirm_stop(); assert s.state=='STOPPED' and s.plan is None

def test_output_off_during_transfer_and_failure():
    now=[0.]; out=SimWeldOutput(lambda:now[0]); out.configure(prepared().settings)
    out.start_segment('p','S1'); assert out.status()['on']
    out.stop_segment('TRANSFER'); assert not out.status()['on']
    out.start_segment('p','S1'); now[0]=2.; assert not out.status()['on']

def test_all_seams_can_be_deselected():
    from test_weld_contract import target,catalog
    from sketch_control.weld_contract import validate_catalog
    from test_weld_geometry import P
    s=WeldSession(P);s.request_target(target());s.accept_catalog(validate_catalog(catalog(),target()))
    s.select([{'seam_id':'S1'}]);s.select([])
    assert not s.selection and s.state=='SEAMS_READY' and s.plan is None

def test_stopped_plan_cannot_replace_identical_replan():
    s=prepared(); old=s.begin_plan()
    s.request_stop(); s.confirm_stop(); current=s.begin_plan()
    assert old!=current
    assert not s.accept_plan(old,WeldPlan('old',old,(),()))
    assert s.state=='PLANNING'
    assert s.accept_plan(current,WeldPlan('new',current,(),()))
