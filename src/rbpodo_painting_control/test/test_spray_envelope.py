"""The CAD cone retains its original slope beyond the finite display cap."""
from dataclasses import FrozenInstanceError
import importlib
import json
from pathlib import Path

import pytest

PROFILE = Path(__file__).resolve().parents[3] / 'assets/construct/spray_v2/envelope.json'


def load(path=PROFILE):
    try:
        module = importlib.import_module('rbpodo_painting_control.spray_envelope')
    except ModuleNotFoundError:
        pytest.fail('cone envelope loader is missing')
    return module.load_spray_envelope(path)


def test_original_cone_is_extrapolated_from_nozzle_without_rescaling():
    envelope = load()
    assert envelope.diameter_at(0.) == pytest.approx(.008376)
    assert envelope.diameter_at(.498) == pytest.approx(.600)
    assert envelope.diameter_at(.5) == pytest.approx(.602376)
    assert envelope.diameter_at(1.) == pytest.approx(1.196376)
    assert envelope.metadata()['spray_envelope_source_sha256'] == 'b8c4d8ce18572932fde019faf4b55f5a9289acf5d324d5dcc1a68006abba73b9'
    with pytest.raises(FrozenInstanceError):
        envelope.length_m = .2


@pytest.mark.parametrize('field,value', [
    ('length_m', 0), ('length_m', -1), ('length_m', float('nan')),
    ('start_radius_m', -.1), ('end_radius_m', .001),
    ('axial_start_m', float('inf')), ('length_m', True),
    ('source_sha256', 'bad'), ('basis', 'world'),
])
def test_invalid_envelope_is_rejected(tmp_path, field, value):
    envelope = load()
    data = json.loads(PROFILE.read_text())
    data[field] = value
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load(path)


@pytest.mark.parametrize('distance', [-.001, float('inf'), float('nan'), True])
def test_invalid_standoff_is_rejected(distance):
    with pytest.raises(ValueError):
        load().diameter_at(distance)


def test_profile_hash_binds_geometry_and_source(tmp_path):
    before = load()
    data = json.loads(PROFILE.read_text())
    data['end_radius_m'] = .31
    path = tmp_path / 'changed.json'
    path.write_text(json.dumps(data))
    after = load(path)
    assert after.metadata()['spray_envelope_profile_sha256'] != before.metadata()['spray_envelope_profile_sha256']
    assert after.diameter_at(.5) > before.diameter_at(.5)
