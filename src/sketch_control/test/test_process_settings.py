from sketch_control.process_settings import settings_snapshot

def test_paint_spray_settings_are_current_and_read_only():
    status=settings_snapshot('spray',{'spray_speed_mps':.017,'spray_overlap':.3},'session')
    assert status['fields']['speed_mps']['value']==.017
    assert status['fields']['speed_mps']['editable'] is False
    assert status['server_session_id']=='session'
    assert settings_snapshot('paint',{},'session')['fields']['target_force_n']['value'] is None

def test_pose_profile_is_explicit_server_registered_choice():
    from dataclasses import asdict
    from sketch_control.weld_contract import WeldSettings
    from test_weld_geometry import P
    s=settings_snapshot('weld',asdict(WeldSettings()),'session',profile=P)
    f=s['fields']['pose_profile_id']
    assert f['value']==''
    assert [o['value'] for o in f['options']]==['',P.profile_id]
