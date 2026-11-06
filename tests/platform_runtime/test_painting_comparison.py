import importlib.util
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location('comparison', Path(__file__).parents[2] / 'scripts/painting_comparison.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def test_three_strokes_preserve_overlap_and_same_coverage():
    h = module.strokes('horizontal', .175, .30)
    v = module.strokes('vertical', .175, .30)
    assert h.shape == v.shape == (3, 2, 2)
    assert np.allclose(h[:, :, ::-1], v)
    assert np.allclose(np.diff(h[:, 0, 1]), .1225)
    assert np.allclose(np.abs(h[:, 1, 0] - h[:, 0, 0]), .420)

def test_cumulative_motion_counts_reversal_without_phase_bridges():
    result = module.motion_metrics([0, .1, .2, .3], [[0], [1], [0], [5]], ['paint', 'paint', 'paint', 'travel'])
    assert np.allclose(result['paint']['rotation_rad'], [2])
    assert result['paint']['duration_s'] == .2
    assert np.allclose(result['travel']['rotation_rad'], [0])

def test_force_and_phase_classification_does_not_call_contact_travel():
    assert module.contact_class('PAINT', 24, True) == 'contact'
    assert module.contact_class('RETRACT', 12, True) == 'transition'
    assert module.contact_class('TRAVEL', 0, False) == 'unknown'
    assert module.contact_class('TRAVEL', 0, True) == 'noncontact'

def test_force_holding_without_traversal_is_not_a_completed_stroke():
    assert not module.stroke_displacement([[0,0,0],[0,0,0]],'horizontal',.42)['passed']
    assert module.stroke_displacement([[0,0,0],[.42,0,.002]],'horizontal',.42)['passed']
