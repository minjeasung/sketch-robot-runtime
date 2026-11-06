"""Versioned execution selection, independent of legacy dry-run flags."""
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

PROCESS_TO_TOOL = {'spray': 'spray_v2', 'weld': 'hongik_v4', 'paint': 'roller'}
PLATFORM_ARMS = {'commercial': ('left', 'right'), 'rb10': ('single',)}


@dataclass(frozen=True)
class RuntimeSelection:
    execution_mode: str
    platform: str
    active_arm: str
    process: str
    tools: Mapping[str, str]
    schema_version: int = 2

    def to_dict(self):
        return dict(schema_version=self.schema_version, execution_mode=self.execution_mode,
                    platform=self.platform, active_arm=self.active_arm, process=self.process,
                    tools=dict(self.tools))


def normalize_selection(payload, previous=None):
    if not isinstance(payload, dict) or set(payload) - {
        'schema_version', 'execution_mode', 'platform', 'active_arm', 'process', 'tools'
    }:
        raise ValueError('Unknown runtime selection fields')
    base = previous.to_dict() if previous else dict(execution_mode='simulation', platform='commercial',
                                                  active_arm='left', process='spray')
    mode = payload.get('execution_mode', base['execution_mode'])
    platform = payload.get('platform', base['platform'])
    process = payload.get('process', base['process'])
    if type(payload.get('schema_version', 2)) is not int or payload.get('schema_version', 2) != 2:
        raise ValueError('schema_version must be 2')
    if mode not in ('simulation', 'mixed', 'real'):
        raise ValueError('execution_mode must be simulation, mixed or real')
    if not isinstance(platform, str) or platform not in PLATFORM_ARMS:
        raise ValueError('platform must be commercial or rb10')
    if not isinstance(process, str) or process not in PROCESS_TO_TOOL:
        raise ValueError('process must be spray, weld or paint')
    changed_platform = platform != base['platform']
    arm = payload.get('active_arm', PLATFORM_ARMS[platform][0] if changed_platform else base['active_arm'])
    if arm not in PLATFORM_ARMS[platform]:
        raise ValueError('active_arm does not belong to platform')
    # Rebuild from bare wrists on every selection; inactive tools never persist.
    tools = {side: 'none' for side in PLATFORM_ARMS[platform]}
    if 'tools' in payload:
        supplied = payload['tools']
        if not isinstance(supplied, dict) or set(supplied) != set(PLATFORM_ARMS[platform]):
            raise ValueError('tools must define every platform arm')
        if any(not isinstance(t, str) or t not in (*PROCESS_TO_TOOL.values(), 'none') for t in supplied.values()):
            raise ValueError('Unknown tool')
        if supplied[arm] != PROCESS_TO_TOOL[process]:
            raise ValueError('Active arm tool conflicts with process')
    tools[arm] = PROCESS_TO_TOOL[process]
    return RuntimeSelection(mode, platform, arm, process, MappingProxyType(tools))


def capabilities(selection):
    supported = not (selection.execution_mode == 'real' and
                     (selection.platform == 'commercial' or selection.process == 'weld'))
    reason = '' if supported else '해당 플랫폼/공정의 실환경 드라이버가 연결되어 있지 않습니다.'
    return [dict(execution_mode=selection.execution_mode, platform=selection.platform,
                 active_arm=selection.active_arm, process=selection.process,
                 supported=supported, reason=reason, verification='not_run')]


def legacy_to_selection(options):
    """Only unambiguous existing profiles migrate; RB20 retains its own identity."""
    profile = options.get('profile')
    model = options.get('model_id', 'rb10_1300e_u')
    if model == 'rb20_1900es' or profile == 'isaac_rb20_paint':
        return None
    if profile == 'isaac_construct':
        if model not in ('construct_left', 'construct_right'):
            return None
        return normalize_selection(dict(platform='commercial', execution_mode='simulation',
                                        active_arm=model.removeprefix('construct_'),
                                        process=options.get('process_mode', 'spray')))
    if profile in ('work', 'spray_motion_test') and model == 'rb10_1300e_u':
        return normalize_selection(dict(platform='rb10', execution_mode='real', active_arm='single',
                                        process='spray' if profile == 'spray_motion_test' else options.get('process_mode','paint')))
    return None
