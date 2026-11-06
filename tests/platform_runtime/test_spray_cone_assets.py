import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))


def test_spray_bundle_uses_cad_cone_at_500mm_with_ten_percent_overlap(tmp_path):
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    generate_urdf_assets(ROOT, normalize_selection(dict(
        platform='commercial', active_arm='left', process='spray')), tmp_path)
    params = yaml.safe_load((tmp_path/'painting.yaml').read_text())['sketch_to_waypoints']['ros__parameters']
    assert params['spray_standoff_m'] == .5
    assert params['spray_footprint_width_m'] == pytest.approx(.602376)
    assert params['spray_overlap'] == .1
    profile = Path(params['spray_envelope_profile'])
    assert profile.parent == tmp_path
    envelope = json.loads(profile.read_text())
    assert envelope['source_sha256'] == 'b8c4d8ce18572932fde019faf4b55f5a9289acf5d324d5dcc1a68006abba73b9'
    bundle = json.loads((tmp_path/'bundle.json').read_text())
    assert 'assets/construct/spray_v2/envelope.json' in bundle['source_hashes']
    # The included legacy launch declares defaults; they must not overwrite
    # the generated cone width and overlap when the actual node starts.
    import importlib.util
    spec = importlib.util.spec_from_file_location('cone_launch', ROOT/'src/sketch_control/launch/platform_simulation.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    arguments = launch.spray_perception_arguments(tmp_path/'painting.yaml')
    assert float(arguments['spray_footprint_width_m']) == pytest.approx(.602376)
    assert float(arguments['spray_overlap']) == .1


def test_legacy_launch_may_use_defaults_but_cone_profile_requires_explicit_geometry(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location('cone_launch_defaults', ROOT/'src/sketch_control/launch/platform_simulation.launch.py')
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    config=tmp_path/'painting.yaml'
    config.write_text(yaml.safe_dump({'sketch_to_waypoints':{'ros__parameters':{}}}))
    assert launch.spray_perception_arguments(config)=={}
    config.write_text(yaml.safe_dump({'sketch_to_waypoints':{'ros__parameters':{'spray_envelope_profile':'profile.json'}}}))
    with pytest.raises(ValueError,match='geometry'):
        launch.spray_perception_arguments(config)
