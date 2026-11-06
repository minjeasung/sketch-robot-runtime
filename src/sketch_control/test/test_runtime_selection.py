import pytest
from sketch_control.runtime_selection import normalize_selection, legacy_to_selection, capabilities


def test_process_selects_only_active_arm_tool():
    first = normalize_selection({})
    second = normalize_selection({'process': 'paint'}, first)
    assert second.tools == {'left': 'roller', 'right': 'none'}
    third = normalize_selection({'active_arm': 'right', 'process': 'spray'}, second)
    assert third.tools == {'left': 'none', 'right': 'spray_v2'}
    assert first.tools == {'left': 'spray_v2', 'right': 'none'}


@pytest.mark.parametrize('profile', ['fake', 'dry_run', 'zed_preview'])
def test_modes_are_not_inferred_from_dry_run(profile):
    assert legacy_to_selection({'profile': profile, 'dry_run': True}) is None


def test_rb20_keeps_legacy_identity():
    assert legacy_to_selection({'profile': 'isaac_rb20_paint', 'model_id': 'rb20_1900es'}) is None


@pytest.mark.parametrize('mode', ['simulation', 'mixed'])
@pytest.mark.parametrize('platform,arm', [('commercial','left'),('commercial','right'),('rb10','single')])
@pytest.mark.parametrize('process,tool', [('spray','spray_v2'),('weld','hongik_v4'),('paint','roller')])
def test_all_eighteen_selections(mode, platform, arm, process, tool):
    selection = normalize_selection(dict(execution_mode=mode, platform=platform, active_arm=arm, process=process))
    assert selection.tools[arm] == tool
    assert selection.to_dict()['schema_version'] == 2
    assert capabilities(selection)[0]['verification'] != 'passed'


@pytest.mark.parametrize('payload', [
    {'platform':'commercial','active_arm':'single'}, {'platform':'rb10','active_arm':'left'},
    {'execution_mode':'fake'}, {'process':'painting'}, {'schema_version':1},
    {'tools':{'left':'roller','right':'hongik_v4'}}, {'tools':{'left':'spray_v2','right':'unknown'}},
    {'dry_run':True}, {'process':None}, {'tools':[]},
])
def test_invalid_selection_rejected(payload):
    with pytest.raises(ValueError): normalize_selection(payload)


def test_platform_switch_and_unsupported_real():
    prior = normalize_selection({'process':'paint'})
    single = normalize_selection({'platform':'rb10'}, prior)
    assert single.active_arm == 'single' and single.tools == {'single':'roller'}
    unsupported = normalize_selection({'execution_mode':'real','process':'weld'})
    assert not capabilities(unsupported)[0]['supported']
    assert capabilities(unsupported)[0]['reason']


def test_saved_dual_tool_selection_is_normalized_to_active_arm_only():
    selection = normalize_selection(dict(active_arm='right', process='paint',
        tools={'left':'roller','right':'roller'}))
    assert selection.tools == {'left':'none','right':'roller'}
