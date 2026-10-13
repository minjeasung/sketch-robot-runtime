"""Common settings metadata, displaying only values supplied by the server."""
from dataclasses import asdict
from .weld_contract import WeldSettings

WELD_FIELDS={
    'speed_mps':('진행 속도','mm/s',1000), 'approach_m':('접근 거리','mm',1000),
    'retreat_m':('이탈 거리','mm',1000), 'standoff_m':('토치 끝 이격','mm',1000),
    'work_angle_deg':('작업각 보정','°',1),'travel_angle_deg':('진행각 보정','°',1),
    'width_m':('위빙 전체 폭','mm',1000),'frequency_hz':('위빙 주파수','Hz',1),
    'left_dwell_s':('왼쪽 횡방향 정지','s',1),'right_dwell_s':('오른쪽 횡방향 정지','s',1),
    'current_a':('시험 전류 설정','A',1),'weave_enabled':('위빙 사용','',1)}

def settings_snapshot(mode,values,server_session_id,revision=0,applied=False,profile=None):
    if mode=='weld':
        fields={k:dict(label=label,unit=unit,scale=scale,value=values.get(k),editable=True) for k,(label,unit,scale) in WELD_FIELDS.items()}
        choices=[dict(value='',label='선택 안 함 · 추출 자세 사용')]
        if profile: choices.append(dict(value=profile.profile_id,label=f'시험 기준 자세 · {profile.reference_up} ({profile.profile_id})'))
        fields['pose_profile_id']=dict(label='XYZ 심의 기준 자세',unit='',scale=1,value=values.get('pose_profile_id',''),editable=True,options=choices)
    else:
        names=({'target_force_n':('목표 접촉력','N','target_force_n',1)} if mode=='paint' else {
            'speed_mps':('도포 속도','mm/s','spray_speed_mps',1000),
            'standoff_m':('분사 이격','mm','spray_standoff_m',1000),
            'width_m':('도포 폭','mm','spray_footprint_width_m',1000),
            'overlap':('겹침 비율','%','spray_overlap',100)})
        fields={k:dict(label=label,unit=unit,scale=scale,value=values.get(param),editable=False) for k,(label,unit,param,scale) in names.items()}
    return dict(mode=mode,server_session_id=server_session_id,revision=revision,applied=applied,fields=fields,
                policy='apply' if mode=='weld' else 'restart',defaults=asdict(WeldSettings()) if mode=='weld' else {})
