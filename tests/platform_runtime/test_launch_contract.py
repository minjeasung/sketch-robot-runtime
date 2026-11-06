from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/sketch_control'))
from sketch_control.runtime_selection import normalize_selection
from sketch_control.platform_runtime import launch_contract,build_platform_specs


@pytest.mark.parametrize('platform,arm',[('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('mode',['simulation','mixed'])
@pytest.mark.parametrize('process',['spray','weld','paint'])
def test_contracts_select_camera_arm_and_native_tool(platform,arm,mode,process):
    selection=normalize_selection(dict(platform=platform,active_arm=arm,execution_mode=mode,process=process))
    c=launch_contract(selection)
    assert c['hardware_plugin']=='isaac_painting_hardware/IsaacPaintingSystem' if process=='paint' else c['hardware_plugin']=='topic_based_ros2_control/TopicBasedSystem'
    assert len(c['joints'])==6
    assert c['virtual_cameras']==(mode=='simulation')
    assert c['weld_provider']=='external' if mode=='mixed' else c['weld_provider']=='fixture'
    assert c['base_frame']==('link0' if platform=='rb10' else 'World')
    assert c['native_weld_tool']


def test_stack_command_has_fixed_argument_boundaries():
    specs=build_platform_specs(ROOT,{'directory':'/tmp/assets with space'})
    assert specs[0].command[-1]=='/tmp/assets with space'
    assert specs[0].command[:2]==('bash','scripts/run_platform_stack.sh')


def test_reject_real_simulator():
    with pytest.raises(ValueError):launch_contract(normalize_selection({'execution_mode':'real'}))


def test_simulator_ignores_commands_for_inactive_arm():
    sys.path.insert(0,str(ROOT/'scripts'))
    from platform_simulation_app import command_positions
    assert command_positions(['left1','right1'],[.5,.9],['left1'],{'left1':(-1,1)})=={'left1':.5}
    with pytest.raises(ValueError):command_positions(['left1'],[float('nan')],['left1'],{'left1':(-1,1)})
    with pytest.raises(ValueError):command_positions(['left1'],[2],['left1'],{'left1':(-1,1)})
