import ast
import json
from types import SimpleNamespace
import numpy as np
import pytest
from test_contracts import ROOT, module
from test_model_guard import robot, semantic, limits
from test_planes import entry


def test_external_descriptor_is_accepted_as_zero_offset_profile(tmp_path):
    model = module('model')
    data = model.parse_stack_model(robot(), semantic(), limits(), {})
    path = model.save_descriptor(data, tmp_path)
    from rbpodo_painting_control.spray_eoat import load_spray_eoat_profile
    profile = load_spray_eoat_profile(path, 'rb20_1900es', '+z')
    assert profile.sha256 == data['fingerprint']
    assert np.array_equal(profile.endpoint_tcp_m, [0, 0, 0])
    assert len(profile.faces) == 0
    with pytest.raises(ValueError):
        load_spray_eoat_profile(path, 'rb10_1300e_u', '+z')


def test_every_motion_dispatch_checks_external_interlock_first():
    path = ROOT/'src/sketch_control/sketch_control/moveit_executor.py'
    cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef) and n.name=='MoveItExecutor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='_motion_dispatch_inhibited_reason')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    subject = SimpleNamespace(_external_motion_blockers=lambda: ('EXTERNAL_MOTION_OWNER',))
    # If the external hook is missing, the old method will attempt MoveIt checks
    # instead of returning before any physical-dispatch eligibility.
    namespace['MoveItExecutor'] = SimpleNamespace(_acm_baseline_is_verified=lambda _: True)
    assert namespace[method.name](subject) == 'EXTERNAL_MOTION_OWNER'


def test_projected_work_area_enforces_measured_cells():
    from sketch_control import zed_spray_projection as projection
    assert hasattr(projection, 'validate_measured_work_boundary')
    target = dict(normal=[0, 0, -1], support_cells=entry()['cells'])
    extent = np.array([[-.3, -.2, 1], [.3, -.2, 1], [.3, .2, 1], [-.3, .2, 1]])
    with pytest.raises(ValueError, match='support'):
        projection.validate_measured_work_boundary(target, extent, (601, 401), [[10, 10], [590, 10], [590, 390], [10, 390]])
    projection.validate_measured_work_boundary(target, extent, (601, 401), [[10, 10], [190, 10], [190, 390], [10, 390]])
