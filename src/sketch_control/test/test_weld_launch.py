import pytest
from sketch_control.process_supervisor import build_specs,SupervisorError
from sketch_control.robot_models import process_cameras

def test_construct_weld_launches_zed_without_d405_or_spray_generator(tmp_path):
    options,specs=build_specs(str(tmp_path),{'profile':'isaac_construct','process_mode':'weld'})
    assert options['process_mode']=='weld'
    assert process_cameras('weld') == (('zed','zed'),)
    p=next(s for s in specs if s.name=='perception')
    assert 'weld_input' in p.nodes
    assert p.command[-1]=='weld'
    simulation=next(s for s in specs if s.name=='simulation')
    assert '--process-mode' in simulation.command and 'weld' in simulation.command

@pytest.mark.parametrize('profile',['work','fake','dry_run','zed_preview'])
def test_real_weld_fails_with_unsupported_driver(tmp_path,profile):
    with pytest.raises(SupervisorError): build_specs(str(tmp_path),{'profile':profile,'process_mode':'weld'})

def test_paint_spray_configuration_unchanged(tmp_path):
    assert build_specs(str(tmp_path),{'profile':'isaac_construct'})[0]['process_mode']=='spray'
