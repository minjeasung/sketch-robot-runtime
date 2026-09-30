"""Profile configuration contracts, independent of the ROS runtime."""
import ast
import json
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCHES = [
    'core', 'sketch_control', 'phase1_python', 'phase2_unity',
    'rb10_painting_system', 'rb10_real_perception_sketch',
    'rb10_perception_sketch', 'zed_preview',
]


@pytest.mark.parametrize('launch', LAUNCHES)
def test_profile_path_defaults_empty_and_is_forwarded_as_string(launch):
    tree = ast.parse((PACKAGE / 'launch' / (launch + '.launch.py')).read_text(encoding='utf-8'))
    declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                    and n.args and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == 'spray_eoat_profile']
    assert len(declarations) == 1
    assert ast.literal_eval(next(k.value for k in declarations[0].keywords
                                 if k.arg == 'default_value')) == ''
    # Evaluate the actual forwarding expression with a path that YAML could misread.
    path = 'C:/EOAT profiles/nozzle: verified.json'
    namespace = dict(LaunchConfiguration=lambda key: path if key == 'spray_eoat_profile' else '',
                     ParameterValue=lambda value, value_type: value_type(value))
    forwarded = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == 'spray_eoat_profile':
                    forwarded.append(eval(compile(ast.Expression(value), '<launch>', 'eval'), namespace))
        elif isinstance(node, ast.DictComp) and any(
                isinstance(n, ast.Constant) and n.value == 'spray_eoat_profile'
                for n in ast.walk(node)):
            forwarded.append(eval(compile(ast.Expression(node), '<launch>', 'eval'), namespace)
                             ['spray_eoat_profile'])
    assert forwarded and all(value == path for value in forwarded)


def test_both_nodes_start_without_an_implicitly_approved_profile():
    config = yaml.safe_load((PACKAGE / 'config/painting_system_real.yaml').read_text(encoding='utf-8'))
    for name in ('moveit_executor', 'sketch_to_waypoints'):
        assert config[name]['ros__parameters']['spray_eoat_profile'] == ''
        assert config[name]['ros__parameters']['process_mode'] == 'paint'
    assert config['sketch_to_waypoints']['ros__parameters']['spray_standoff_m'] == 0.5


def test_example_cannot_be_mistaken_for_confirmed_geometry():
    profile = json.loads((PACKAGE / 'config/spray_eoat_profile.UNCONFIRMED.json').read_text())
    assert profile['endpoint_confirmed'] is False
    assert profile['mesh_file'] == 'REPLACE_WITH_VERIFIED_FULL_EOAT.stl'
    assert not (PACKAGE / 'config' / profile['mesh_file']).exists()
    assert 'endpoint_tcp_m' not in profile
