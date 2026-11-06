"""RB10 simulation must hold source ready geometry without redefining physical limits."""
import sys
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/sketch_control')]
import platform_simulation_app as app
from prepare_platform_assets import generate_urdf_assets
from prepare_rb20_painting_assets import link_transforms,origin_matrix
from sketch_control.runtime_selection import normalize_selection


def gravity_torques(robot,positions):
    def potential(q):
        transforms=link_transforms(robot,q)
        return sum(float(link.find('inertial/mass').get('value'))*9.81*
                   (transforms[link.get('name')]@origin_matrix(link.find('inertial/origin')))[2,3]
                   for link in robot.findall('link') if link.find('inertial') is not None)
    step=1e-6
    return {name:(potential({**positions,name:value+step})-potential({**positions,name:value-step}))/(2*step)
            for name,value in positions.items()}


@pytest.mark.parametrize('mode',['simulation','mixed'])
@pytest.mark.parametrize('process',['spray','weld','paint'])
def test_rb10_simulated_drive_can_hold_unchanged_source_ready_for_each_tool(tmp_path,mode,process):
    selection=normalize_selection(dict(platform='rb10',process=process,execution_mode=mode))
    bundle=generate_urdf_assets(ROOT,selection,tmp_path)
    source=ET.parse(ROOT/'src/sketch_control/config/rbpodo_named.srdf').getroot()
    ready={j.get('name'):float(j.get('value')) for j in source.findall("group_state[@name='ready']/joint")}
    assert bundle['initial_positions']==ready
    robot=ET.parse(bundle['physics_urdf']).getroot()
    limits={j.get('name'):{k:float(v) for k,v in j.find('limit').attrib.items()} for j in robot.findall("joint[@type='revolute']")}
    assert all(limit['effort']==10. for limit in limits.values()),'Original description limits must not be redefined as physical ratings'
    gravity=gravity_torques(robot,ready)
    assert 25<abs(gravity['shoulder'])<32
    assert 55<abs(gravity['elbow'])<62
    contract=app.simulator_drive_contract(bundle['selection'],limits)
    scene=json.loads((tmp_path/'scene.json').read_text())
    assert scene['physics_hz']==contract['timing']['physics_hz']
    assert scene['measured_state_hz']==contract['timing']['joint_state_hz']
    assert scene['render_hz']==contract['timing']['render_hz']
    for name,torque in gravity.items():
        assert contract['max_torque_nm'][name]>abs(torque)*2, 'Drive must hold modeled gravity with bounded dynamic margin'
        assert contract['max_torque_nm'][name]==2000.,'Use the established RB20 simulator force-drive contract'
    assert contract['source_effort_limit_nm']=={name:10. for name in ready}
    assert contract['simulation_only'] is True
    assert 'RB20' in contract['provenance']
    assert all(limit['effort']==10. for limit in limits.values())


def test_commercial_robot_preserves_its_source_effort_limits():
    limits={'left_joint1':{'effort':400.},'right_joint1':{'effort':250.},'head_joint1':{'effort':50.}}
    contract=app.simulator_drive_contract({'platform':'commercial','execution_mode':'simulation','process':'paint'},limits)
    assert contract['max_torque_nm']=={'left_joint1':400.,'right_joint1':250.,'head_joint1':50.}


def test_simulator_override_cannot_be_used_for_real_hardware():
    with pytest.raises(ValueError,match='simulation'):
        app.simulator_drive_contract({'platform':'rb10','execution_mode':'real'},{'base':{'effort':10.}})


def test_rb10_override_rejects_wrong_joint_identity():
    with pytest.raises(ValueError,match='joint'):
        app.simulator_drive_contract({'platform':'rb10','execution_mode':'simulation','process':'paint'},{'left_joint1':{'effort':10.}})


@pytest.mark.parametrize('effort',[0.,-1.,float('nan'),float('inf')])
def test_invalid_source_effort_is_not_hidden_by_simulator_override(effort):
    with pytest.raises(ValueError,match='effort'):
        app.simulator_drive_contract({'platform':'commercial','execution_mode':'simulation','process':'paint'},{'left_joint1':{'effort':effort}})


@pytest.mark.parametrize('process,damping',[('paint',30000.),('spray',6000.),('weld',6000.)])
def test_effective_usd_drive_limits_apply_only_to_the_selected_robot(process,damping):
    pytest.importorskip('pxr.UsdPhysics')
    from pxr import Usd,UsdPhysics
    stage=Usd.Stage.CreateInMemory()
    names=['base','shoulder','elbow','wrist1','wrist2','wrist3']
    for name in names:
        joint=UsdPhysics.RevoluteJoint.Define(stage,'/World/Robot/'+name)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular').CreateMaxForceAttr(10.)
    other=UsdPhysics.RevoluteJoint.Define(stage,'/World/Other/base')
    UsdPhysics.DriveAPI.Apply(other.GetPrim(),'angular').CreateMaxForceAttr(10.)
    app.apply_simulator_drive_efforts(stage,{'platform':'rb10','execution_mode':'simulation','process':process},
                                    {name:{'effort':10.} for name in names})
    for name in names:
        drive=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/World/Robot/'+name),'angular')
        assert drive.GetMaxForceAttr().Get()==2000.
        assert drive.GetTypeAttr().Get()=='force'
        assert drive.GetStiffnessAttr().Get()==pytest.approx(300000.*np.pi/180.)
        assert drive.GetDampingAttr().Get()==pytest.approx(damping*np.pi/180.)
    assert UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/World/Other/base'),'angular').GetMaxForceAttr().Get()==10.


@pytest.mark.parametrize('mode',['simulation','mixed'])
@pytest.mark.parametrize('process,damping',[('paint',30000.),('spray',6000.),('weld',6000.)])
def test_process_drive_contract_preserves_contact_gains_and_reduces_noncontact_lag(mode,process,damping):
    contract=app.simulator_drive_contract(dict(platform='commercial',execution_mode=mode,process=process),
                                        {'joint':{'effort':400.}})
    assert contract['process']==process
    assert contract['stiffness_nm_per_rad']==300000.
    assert contract['damping_nms_per_rad']==damping
    assert contract['max_torque_nm']=={'joint':400.}
    assert contract['simulation_only'] is True
    assert 'physical actuator' in contract['gain_provenance']


@pytest.mark.parametrize('process',[None,'unknown'])
def test_process_drive_contract_requires_explicit_supported_process(process):
    with pytest.raises(ValueError,match='process'):
        app.simulator_drive_contract(dict(platform='commercial',execution_mode='simulation',process=process),
                                     {'joint':{'effort':400.}})


@pytest.mark.parametrize('process',['paint','spray','weld'])
def test_stiff_virtual_drives_share_five_substeps_per_measured_state(process):
    rates=app.simulator_timing_contract(process)
    assert rates==dict(physics_hz=500,joint_state_hz=100,render_hz=15,state_stride=5)
    contract=app.simulator_drive_contract(dict(platform='commercial',execution_mode='simulation',process=process),
                                        {'joint':{'effort':400.}})
    assert contract['timing']==rates


def test_simulator_timing_contract_rejects_unknown_process():
    with pytest.raises(ValueError,match='process'):
        app.simulator_timing_contract('unknown')
