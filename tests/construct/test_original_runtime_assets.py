import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/rbpodo_painting_control'))


def test_generated_equipment_uses_construct_tcp_and_urdf_meshes(tmp_path):
    from prepare_construct_runtime import prepare
    from rbpodo_painting_control.spray_eoat import load_spray_eoat_profile
    import json
    prepare(ROOT, tmp_path)
    for side in ('left', 'right'):
        name = 'construct_' + side
        path = tmp_path / (name + '_eoat.json')
        config = json.loads(path.read_text())
        assert config['model_id'] == name
        assert config['spray_tool_axis'] == '+z'
        assert config['endpoint_tcp_m'] == [0., 0., 0.]
        profile = load_spray_eoat_profile(path, name, '+z')
        assert np.isfinite(profile.vertices_tcp_m).all()
        assert len(profile.faces) > 100
        assert np.linalg.norm(profile.vertices_tcp_m.max(axis=0)-profile.vertices_tcp_m.min(axis=0)) > .1
        provenance = json.loads((tmp_path / (name + '_equipment.json')).read_text())
        assert provenance['tcp'] == side + '_manipulator_ee_point'
        assert (side + '_D435_link' in provenance['links']) == (side == 'right')
        assert provenance['equipment_model'] == ('spray_v2' if side == 'left' else 'original')
        assert provenance['source'] == 'Construct URDF collision geometry; simulation only'


def test_ready_requires_camera_and_initial_view(tmp_path):
    from run_construct_demo import managed_inputs_ready
    logs = tmp_path / 'logs'
    logs.mkdir()
    assert not managed_inputs_ready(tmp_path)
    (logs / 'construct-camera-keepalive.log').write_text('CONSTRUCT_CAMERA_STREAMING')
    assert not managed_inputs_ready(tmp_path)
    (logs / 'construct-environment.log').write_text('CONSTRUCT_CAMERA_VIEW_POSE_READY')
    assert managed_inputs_ready(tmp_path)


def test_compliance_status_is_observed_from_controller_inventory():
    from construct_position_state import observed_compliance
    from types import SimpleNamespace as C
    jt = lambda side: C(name=side+'_manipulator_controller', state='active', type='joint_trajectory_controller/JointTrajectoryController')
    assert observed_compliance([]) is None
    assert observed_compliance([jt('left')]) is None
    controllers = [jt('left'), jt('right')]
    assert observed_compliance(controllers) is False
    assert observed_compliance(controllers + [C(name='admittance_controller', state='active', type='admittance_controller/AdmittanceController')]) is True


def test_compliance_poll_uses_steady_clock_when_simulation_paused():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.clock import ClockType
    from construct_position_state import create_poll_timer
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_construct_poll_clock', context=context)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    try:
        timer = create_poll_timer(node, lambda: None)
        assert timer.clock.clock_type == ClockType.STEADY_TIME
    finally:
        node.destroy_node()
        context.try_shutdown()
