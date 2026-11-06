import importlib.util
from pathlib import Path
import yaml


def test_explicit_simulated_paint_zed_view_is_preserved(tmp_path):
    path=Path(__file__).parents[2]/'src/sketch_control/launch/platform_simulation.launch.py'
    spec=importlib.util.spec_from_file_location('platform_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source=tmp_path/'painting.yaml'
    source.write_text('wall_projector:\n  ros__parameters:\n    front_view_source: zed\n')
    contract=dict(bundle=dict(selection=dict(process='paint',execution_mode='simulation',active_arm='right')),
                  active=dict(model_id='construct_right',base_frame='World'),assets=tmp_path,registrations={})
    result=yaml.safe_load(module.perception_parameter_file(source,contract).read_text())
    assert result['wall_projector']['ros__parameters']['front_view_source']=='zed'
