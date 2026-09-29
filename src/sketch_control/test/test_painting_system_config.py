import ast
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "painting_system_real.yaml"
)
WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('profile', ['dry_run', 'work', 'fake'])
def test_process_startup_defaults_to_paint(profile, tmp_path):
    from sketch_control.process_supervisor import build_specs
    options, specs = build_specs(tmp_path, {'profile': profile})
    assert options['process_mode'] == 'paint'
    assert all('process_mode:=paint' in spec.command for spec in specs)
    assert 'd405_surface_refiner' in next(s for s in specs if s.name == 'perception').nodes


@pytest.mark.parametrize('profile', ['dry_run', 'work', 'fake', 'spray_motion_test'])
@pytest.mark.parametrize('backend', ['native', 'outpost'])
def test_spray_startup_has_no_d405_dependency(profile, backend, tmp_path):
    from sketch_control.process_supervisor import build_specs
    options, specs = build_specs(tmp_path, {
        'profile': profile, 'process_mode': 'spray', 'camera_backend': backend,
        'launch_d405_driver': True,
    })
    assert options['process_mode'] == 'spray'
    assert options['launch_d405_driver'] is False
    for spec in specs:
        assert 'process_mode:=spray' in spec.command
        assert 'launch_d405_driver:=false' in spec.command
        assert 'painting_force_enabled:=false' in spec.command
        assert not any(arg.startswith('d405_calibration_file:=') for arg in spec.command)
    perception = next(s for s in specs if s.name == 'perception')
    assert 'd405_surface_refiner' not in perception.nodes
    assert {'target_selector', 'wall_projector', 'sketch_to_waypoints'} <= set(perception.nodes)


def test_motion_test_selects_spray_and_rejects_explicit_paint(tmp_path):
    from sketch_control.process_supervisor import build_specs, SupervisorError
    options, _ = build_specs(tmp_path, {'profile': 'spray_motion_test'})
    assert options['process_mode'] == 'spray'
    with pytest.raises(SupervisorError, match='requires process_mode=spray'):
        build_specs(tmp_path, {'profile': 'spray_motion_test', 'process_mode': 'paint'})


@pytest.mark.parametrize('mode', ['unknown', '', True, None, ['spray']])
def test_invalid_startup_process_modes_rejected(mode, tmp_path):
    from sketch_control.process_supervisor import build_specs, SupervisorError
    with pytest.raises(SupervisorError, match='process_mode'):
        build_specs(tmp_path, {'process_mode': mode})


def test_rb20_spray_requires_zed_calibration_but_not_d405(tmp_path):
    from sketch_control.robot_models import model_calibration_files, validate_calibration_files
    paths = model_calibration_files(tmp_path, 'rb20_1900es', process_mode='spray')
    assert set(paths) == {'zed_calibration_file'}
    with pytest.raises(ValueError, match='calibration required'):
        validate_calibration_files(paths, process_mode='spray')
    path = Path(paths['zed_calibration_file'])
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({'T_world_zed_optical': {
        'translation': [0, 0, 0], 'rotation_xyzw': [0, 0, 0, 1],
    }}))
    validate_calibration_files(paths, process_mode='spray')
    with pytest.raises(ValueError, match='calibration required'):
        validate_calibration_files(model_calibration_files(tmp_path, 'rb20_1900es'))


@pytest.mark.parametrize('mode,expected', [('paint', ['zed', 'realsense']), ('spray', ['zed'])])
def test_supervisor_preflight_checks_only_process_cameras(mode, expected, tmp_path, monkeypatch):
    from sketch_control import process_supervisor
    calls = []
    monkeypatch.setattr(process_supervisor, 'camera_status',
                        lambda origin, hw_id, serial, kind: calls.append(kind))
    supervisor = process_supervisor.Supervisor(tmp_path, None, {'process_mode': mode})
    supervisor._validate_cameras()
    assert calls == expected


def test_spray_config_keeps_explicit_geometry_and_safe_defaults():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    for node in ('moveit_executor', 'sketch_to_waypoints', 'wall_projector', 'd405_surface_refiner'):
        assert config[node]['ros__parameters']['process_mode'] == 'paint'
    for node in ('moveit_executor', 'sketch_to_waypoints'):
        assert config[node]['ros__parameters']['model_id'] == 'rb10_1300e_u'
        assert config[node]['ros__parameters']['spray_tool_axis'] == ''
    generator = config['sketch_to_waypoints']['ros__parameters']
    assert generator['spray_footprint_width_m'] == .35
    assert generator['spray_overlap'] == .30
    assert generator['spray_speed_mps'] == .020
    assert generator['spray_standoff_m'] == .5


def test_rb20_spray_supervisor_validates_zed_only_before_start(tmp_path):
    from sketch_control.process_supervisor import Supervisor, SupervisorError
    from sketch_control.robot_models import model_calibration_files
    supervisor = Supervisor(tmp_path, None, {'model_id': 'rb20_1900es', 'process_mode': 'spray'})
    with pytest.raises(SupervisorError, match='calibration required'):
        supervisor._validate_model_calibration()
    paths = model_calibration_files(tmp_path, 'rb20_1900es')
    path = Path(paths['zed_calibration_file'])
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({'T_world_zed_optical': {
        'translation': [0, 0, 0], 'rotation_xyzw': [0, 0, 0, 1],
    }}))
    supervisor._validate_model_calibration()
    supervisor.options['process_mode'] = 'paint'
    with pytest.raises(SupervisorError, match='d405_eyeinhand'):
        supervisor._validate_model_calibration()


def test_spray_supervisor_still_rejects_missing_zed(tmp_path, monkeypatch):
    from sketch_control import process_supervisor
    def unavailable(origin, hw_id, serial, kind):
        assert kind == 'zed'
        raise ValueError('ZED unavailable')
    monkeypatch.setattr(process_supervisor, 'camera_status', unavailable)
    supervisor = process_supervisor.Supervisor(tmp_path, None, {'process_mode': 'spray'})
    with pytest.raises(process_supervisor.SupervisorError, match='ZED unavailable'):
        supervisor._validate_cameras()


def test_spray_duplicate_detection_ignores_d405_but_keeps_zed_and_projector(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from sketch_control import process_supervisor
    monkeypatch.setattr(process_supervisor, 'camera_status', lambda *args: None)
    graph = {'graph_fresh': True, 'nodes': ['/sketch_outpost_d405_bridge', '/d405_surface_refiner']}
    monitor = SimpleNamespace(snapshot=lambda: graph)
    supervisor = process_supervisor.Supervisor(tmp_path, monitor, {'process_mode': 'spray'})
    supervisor.preflight('perception')
    for name in ('sketch_outpost_zed_bridge', 'wall_projector', 'sketch_to_waypoints'):
        graph['nodes'] = ['/' + name]
        with pytest.raises(process_supervisor.SupervisorError, match='Already running'):
            supervisor.preflight('perception')
    supervisor = process_supervisor.Supervisor(tmp_path, monitor, {'process_mode': 'paint'})
    graph['nodes'] = ['/sketch_outpost_d405_bridge']
    with pytest.raises(process_supervisor.SupervisorError, match='Already running'):
        supervisor.preflight('perception')


def _launch_function(filename, function_name, **namespace):
    """Exercise Python launch guards without importing the ROS launch runtime."""
    path = WORKSPACE_SRC / 'sketch_control/launch' / filename
    tree = ast.parse(path.read_text(encoding='utf-8'))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == function_name)
    module = ast.Module(body=[function], type_ignores=[])
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace[function_name]


class _LaunchValue:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def test_spray_launch_disables_every_d405_source_and_skips_calibration_read():
    from sketch_control.robot_models import validate_process_mode
    configure = _launch_function('rb10_perception_sketch.launch.py', '_configure_process',
                                 LaunchConfiguration=_LaunchValue,
                                 SetLaunchConfiguration=lambda name, value: (name, value),
                                 validate_process_mode=validate_process_mode)
    assert configure({'process_mode': 'paint'}) == []
    updates = dict(configure({'process_mode': 'spray'}))
    assert updates['front_view_source'] == 'zed'
    for key in ('use_sim_d405_depth_pointcloud', 'use_d405_refinement', 'use_d405_mount_tf',
                'use_d405_optical_tf', 'use_d405_calibration_file', 'use_ft_normal_controller'):
        assert updates[key] == 'false'
    mount = _launch_function('rb10_perception_sketch.launch.py', '_make_d405_mount_static_tf',
                             LaunchConfiguration=_LaunchValue)
    # No camera settings or loader exist in this context: spray must exit first.
    assert mount({'process_mode': 'spray', 'use_d405_mount_tf': 'true'}) == []


def test_spray_launch_rejects_paint_and_force_bypasses_without_ros():
    from sketch_control.robot_models import validate_process_mode
    validate = _launch_function('rb10_painting_system.launch.py', '_validate_interlock_values',
                                validate_process_mode=validate_process_mode)
    flags = dict(use_fake_hardware=False, use_isaac_sim=False, real_painting_enabled=True,
                 dry_run=False, painting_force_enabled=False, process_mode='spray')
    validate(**flags)
    with pytest.raises(RuntimeError, match='painting_force_enabled=false'):
        validate(**(flags | {'painting_force_enabled': True}))
    with pytest.raises(RuntimeError, match='requires process_mode=spray'):
        validate(**(flags | {'spray_motion_test': True, 'process_mode': 'paint'}))
    with pytest.raises(RuntimeError, match='real_painting_enabled=true'):
        validate(**(flags | {'real_painting_enabled': False}))


@pytest.mark.parametrize('mode,expected', [('paint', ['zed', 'realsense']), ('spray', ['zed'])])
def test_real_perception_launch_validates_only_required_camera_descriptors(mode, expected):
    from sketch_control.robot_models import validate_process_mode, process_cameras
    calls = []
    validate = _launch_function('rb10_real_perception_sketch.launch.py', '_validate_camera_backend',
                                LaunchConfiguration=_LaunchValue,
                                SetLaunchConfiguration=lambda name, value: (name, value),
                                validate_process_mode=validate_process_mode, process_cameras=process_cameras,
                                camera_status=lambda origin, hw_id, serial, kind: calls.append(kind))
    context = {'process_mode': mode, 'camera_backend': 'outpost', 'outpost_http': 'http://127.0.0.1:8100'}
    for camera, _ in process_cameras(mode):
        context.update({f'launch_{camera}_driver': 'false', f'outpost_{camera}_hw_id': camera,
                        f'outpost_{camera}_serial': camera})
    updates = dict(validate(context))
    assert calls == expected
    if mode == 'spray':
        assert updates['launch_d405_driver'] == 'false'
        assert updates['front_view_source'] == 'zed'


def test_top_launch_rb20_spray_needs_only_zed_descriptors_and_calibration(tmp_path, monkeypatch):
    import os
    from sketch_control import robot_models
    monkeypatch.setenv('SKETCH_WORKSPACE', str(tmp_path))
    paths = robot_models.model_calibration_files(tmp_path, 'rb20_1900es', 'spray')
    path = Path(paths['zed_calibration_file'])
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({'T_world_zed_optical': {
        'translation': [0, 0, 0], 'rotation_xyzw': [0, 0, 0, 1],
    }}))
    calls = []
    validate_values = _launch_function('rb10_painting_system.launch.py', '_validate_interlock_values',
                                       validate_process_mode=robot_models.validate_process_mode)
    validate = _launch_function('rb10_painting_system.launch.py', '_validate_launch_interlocks',
                                os=os, LaunchConfiguration=_LaunchValue,
                                SetLaunchConfiguration=lambda name, value: (name, value),
                                _validate_interlock_values=validate_values,
                                camera_status=lambda origin, hw_id, serial, kind: calls.append(kind),
                                **{name: getattr(robot_models, name) for name in (
                                    'DEFAULT_MODEL', 'validate_model', 'validate_process_mode',
                                    'process_cameras', 'model_calibration_files', 'validate_calibration_files')})
    context = {
        'model_id': 'rb20_1900es', 'process_mode': 'spray', 'use_fake_hardware': 'false',
        'use_isaac_sim': 'false', 'real_painting_enabled': 'true', 'dry_run': 'false',
        'painting_force_enabled': 'false', 'spray_motion_test': 'true', 'launch_perception': 'true',
        'zed_calibration_file': '', 'camera_backend': 'outpost', 'launch_zed_driver': 'false',
        'outpost_http': 'http://127.0.0.1:8100', 'outpost_zed_hw_id': 'zed', 'outpost_zed_serial': '123',
    }
    updates = dict(validate(context))
    assert calls == ['zed']
    assert updates['zed_calibration_file'] == str(path)
    assert 'd405_calibration_file' not in updates
    assert updates['launch_d405_driver'] == 'false'
    path.unlink()
    with pytest.raises(ValueError, match='calibration required'):
        validate(context)


@pytest.mark.parametrize('filename', [
    'rb10_painting_system.launch.py', 'rb10_real_perception_sketch.launch.py',
    'rb10_perception_sketch.launch.py', 'core.launch.py', 'sketch_control.launch.py',
    'phase1_python.launch.py', 'phase2_unity.launch.py',
])
def test_all_active_launches_expose_process_and_spray_geometry(filename):
    tree = ast.parse((WORKSPACE_SRC / 'sketch_control/launch' / filename).read_text(encoding='utf-8'))
    declarations = {n.args[0].value: {k.arg: k.value for k in n.keywords}
                    for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                    and n.args and isinstance(n.args[0], ast.Constant)}
    assert 'process_mode' in declarations
    for key, expected in {'model_id': 'rb10_1300e_u', 'spray_tool_axis': '',
                          'spray_footprint_width_m': '0.35', 'spray_overlap': '0.30',
                          'spray_speed_mps': '0.020', 'spray_standoff_m': '0.5'}.items():
        assert ast.literal_eval(declarations[key]['default_value']) == expected


def test_common_config_is_accepted_by_ros_argument_parser():
    """Catch YAML features accepted by PyYAML but rejected by ROS 2."""

    script = """
import rclpy
import sys

rclpy.init(args=["--ros-args", "--params-file", sys.argv[1]])
rclpy.shutdown()
"""
    result = subprocess.run(
        [sys.executable, "-c", script, str(CONFIG_PATH)],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )

    assert result.returncode == 0, result.stderr or result.stdout


def test_runtime_tare_and_generated_precontact_use_same_ten_mm_clearance():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    executor = config["moveit_executor"]["ros__parameters"]
    generator = config["sketch_to_waypoints"]["ros__parameters"]
    hardware_tare = config["rbpodo_ft_tare"]["ros__parameters"]

    assert hardware_tare["auto_tare_on_activate"] is False
    assert hardware_tare["supervised_tare"]["enabled"] is False
    assert hardware_tare["runtime_tare"]["min_recent_joint_samples"] == 50
    assert hardware_tare["runtime_tare"]["max_abs_post_tare_residual"] == [
        0.5, 0.5, 0.5, 0.05, 0.05, 0.05
    ]
    assert executor["runtime_tare_enabled"] is True
    assert executor["precontact_clearance_m"] == 0.010
    assert executor["runtime_tare_min_clearance_m"] == 0.010
    assert executor["runtime_tare_min_actual_clearance_m"] == 0.007
    assert executor["runtime_tare_max_tcp_position_error_m"] == 0.003
    assert executor["runtime_tare_max_tcp_orientation_error_deg"] == 3.0
    assert executor["runtime_tare_max_tcp_tf_age_s"] == 0.20
    assert executor["runtime_tare_quiet_s"] == 3.0
    assert generator["precontact_clearance_m"] == 0.010
    assert executor["contact_search_max_distance_m"] == 0.020
    assert executor["contact_search_timeout_s"] == 30.0
    assert generator["contact_search_max_distance_m"] == 0.020
    assert generator["contact_search_timeout_s"] == 30.0


def test_contact_detection_and_controller_use_absolute_normal_reaction():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    reference = config["painting_wrench_reference"]["ros__parameters"]
    monitor = config["painting_force_monitor"]["ros__parameters"]
    controller = config["admittance_controller"]["ros__parameters"]

    assert reference["target_wrench_sign"] in (-1.0, 1.0)
    assert reference["target_wrench_sign"] == -1.0
    assert monitor["absolute_normal_force"] is True
    assert controller["admittance"]["absolute_normal_force"] is True

    search = monitor["limits"]["contact_search"]
    # The bounded first-touch search deliberately has no software force/torque
    # envelope. Contact detection and sensor validity/saturation remain global.
    assert monitor["contact_detect_threshold_n"] == 3.5
    assert search["force_axis_n"] == [0.0, 0.0, 0.0]
    assert search["force_norm_n"] == 0.0
    assert search["torque_axis_nm"] == [0.0, 0.0, 0.0]
    assert search["torque_norm_nm"] == 0.0
    assert search["raw_force_axis_n"] == [0.0, 0.0, 0.0]
    assert search["raw_force_norm_n"] == 0.0
    assert search["raw_torque_axis_nm"] == [0.0, 0.0, 0.0]
    assert search["raw_torque_norm_nm"] == 0.0
    assert search["contact_opposite_force_n"] == 0.0
    assert search["contact_off_axis_force_n"] == 0.0


def test_real_profile_uses_twenty_to_thirty_newton_band_with_guard_headroom():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    generator = config["sketch_to_waypoints"]["ros__parameters"]
    reference = config["painting_wrench_reference"]["ros__parameters"]
    monitor = config["painting_force_monitor"]["ros__parameters"]
    controller = config["admittance_controller"]["ros__parameters"]

    band_activation_n = 20.0
    assert generator["default_paint_force_n"] == band_activation_n
    assert reference["desired_contact_force_n"] == band_activation_n
    assert generator["paint_speed_mps"] == 0.020
    assert reference["force_ramp_up_duration_s"] == 3.0
    assert reference["force_ramp_down_duration_s"] == 3.0
    assert reference["max_command_force_n"] == 45.0
    assert controller["admittance"]["min_normal_drive_force_n"] == 32.0
    assert controller["admittance"]["normal_force_hold_lower_n"] == 20.0
    assert controller["admittance"]["normal_force_hold_upper_n"] == 30.0
    assert controller["admittance"]["normal_force_hold_hysteresis_n"] == 1.0
    assert controller["max_normal_velocity_mps"] == 0.003
    assert controller["max_normal_acceleration_mps2"] == 0.05

    assert monitor["over_force_warn_n"] == 60.0
    assert monitor["over_force_abort_n"] == 100.0
    assert monitor["sensor_force_saturation_n"] == 190.0
    assert monitor["sensor_torque_saturation_nm"] == 14.0
    assert monitor["contact_detect_threshold_n"] == 3.5
    assert monitor["contact_release_threshold_n"] == 2.0
    assert monitor["contact_confirm_duration_s"] == 0.04
    for mode in ("paint", "ramp_up", "ramp_down"):
        limits = monitor["limits"][mode]
        assert limits["force_axis_n"] == [35.0, 90.0, 35.0]
        assert limits["force_norm_n"] == 110.0
        assert limits["torque_axis_nm"] == [6.0, 6.0, 6.0]
        assert limits["torque_norm_nm"] == 8.0
        assert limits["raw_force_axis_n"] == [60.0, 150.0, 60.0]
        assert limits["raw_force_norm_n"] == 170.0
        assert limits["raw_torque_axis_nm"] == [10.0, 10.0, 10.0]
        assert limits["raw_torque_norm_nm"] == 13.0

    launch_path = (
        WORKSPACE_SRC
        / "sketch_control"
        / "launch"
        / "rb10_painting_system.launch.py"
    )
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    defaults = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant):
            continue
        if node.args[0].value != "desired_contact_force_n":
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                defaults.append(keyword.value.value)
    assert defaults == ["20.0"]


def test_roller_balance_is_bounded_force_scaled_and_commissions_fail_closed():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    controller = config["admittance_controller"]["ros__parameters"]
    balance = controller["roller_balance"]
    monitor = config["painting_force_monitor"]["ros__parameters"]

    assert balance["monitoring_enabled"] is True
    assert balance["control_enabled"] is False
    assert controller["admittance"]["selected_axes"] == [
        False, True, False, False, False, False
    ]
    assert balance["rotation_axis_index"] == 2
    assert balance["contact_center_offset_m"] == [0.0, -0.27020, 0.0]
    assert balance["roller_length_m"] == 0.175
    assert balance["minimum_normal_force_n"] == 8.0
    assert balance["cop_enter_m"] == 0.008
    assert balance["cop_exit_m"] == 0.005
    assert balance["filter_time_constant_s"] == 0.10
    assert balance["torque_to_cop_sign"] in (-1.0, 1.0)
    assert balance["rotation_feedback_sign"] in (-1.0, 1.0)
    assert math.degrees(balance["max_rotation_trim_rad"]) == pytest.approx(0.5)
    assert balance["soft_limit_ratio"] == 0.9
    assert math.degrees(balance["max_rotation_trim_rad"] * balance["soft_limit_ratio"]) == (
        pytest.approx(0.45)
    )
    assert math.degrees(balance["max_rotation_velocity_radps"]) == pytest.approx(0.2)
    assert math.degrees(balance["max_rotation_acceleration_radps2"]) == pytest.approx(1.0)
    assert controller["admittance"]["damping_absolute"][5] == 25.0

    # T = Fn*x: the torque band scales with the current 20..30 N force band.
    assert 20.0 * balance["cop_exit_m"] == pytest.approx(0.10)
    assert 20.0 * balance["cop_enter_m"] == pytest.approx(0.16)
    assert 30.0 * balance["cop_exit_m"] == pytest.approx(0.15)
    assert 30.0 * balance["cop_enter_m"] == pytest.approx(0.24)
    assert monitor["roller_balance_limit_reached_topic"] == (
        "/admittance_controller/roller_balance/limit_reached"
    )
    assert monitor["roller_balance_limit_abort_duration_s"] == 0.25


def test_real_d405_profile_uses_authoritative_two_stage_plane_validation():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    refiner = config["d405_surface_refiner"]["ros__parameters"]

    assert refiner["min_inlier_ratio"] == 0.65
    assert refiner["max_plane_shift_m"] == 0.25
    assert refiner["support_band_half_width_m"] == 0.006
    assert refiner["min_broad_inlier_ratio"] == 0.15
    assert refiner["min_support_points"] == 100
    assert refiner["support_span_quantile"] == 0.02
    assert refiner["min_support_span_m"] == 0.15
    assert refiner["max_target_support_offset_m"] == 0.32
    assert refiner["min_secondary_plane_inliers"] == 100
    assert refiner["max_secondary_plane_relative_inliers"] == 0.60
    assert refiner["min_secondary_plane_separation_m"] == 0.020
    assert refiner["min_secondary_plane_normal_delta_deg"] == 5.0


def test_top_launch_rejects_real_motion_and_force_interlock_bypasses():
    launch_module = _load_module(
        "rb10_painting_system_contract_test",
        WORKSPACE_SRC
        / "sketch_control"
        / "launch"
        / "rb10_painting_system.launch.py",
    )

    with pytest.raises(RuntimeError, match="real_painting_enabled=true"):
        launch_module._validate_interlock_values(
            use_fake_hardware=False,
            use_isaac_sim=False,
            real_painting_enabled=False,
            dry_run=False,
            painting_force_enabled=False,
        )
    with pytest.raises(RuntimeError, match="painting_force_enabled=true"):
        launch_module._validate_interlock_values(
            use_fake_hardware=False,
            use_isaac_sim=False,
            real_painting_enabled=True,
            dry_run=True,
            painting_force_enabled=True,
        )
    launch_module._validate_interlock_values(
        use_fake_hardware=False,
        use_isaac_sim=False,
        real_painting_enabled=True,
        dry_run=False,
        painting_force_enabled=True,
    )


def test_moveit_painting_admittance_requires_shared_runtime_tare_config():
    launch_module = _load_module(
        "rb10_moveit_full_contract_test",
        WORKSPACE_SRC
        / "sketch_control"
        / "launch"
        / "rb10_moveit_full.launch.py",
    )

    with pytest.raises(RuntimeError, match="requires painting_config_file"):
        launch_module._validate_painting_config_contract(
            use_admittance=True,
            admittance_use_case="painting",
            painting_config_file="",
        )
    launch_module._validate_painting_config_contract(
        use_admittance=True,
        admittance_use_case="painting",
        painting_config_file=str(CONFIG_PATH),
    )


def test_rviz_receives_planning_pipeline_parameters():
    launch_path = (
        WORKSPACE_SRC
        / "sketch_control"
        / "launch"
        / "rb10_moveit_full.launch.py"
    )
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    rviz_nodes = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {item.arg: item.value for item in node.keywords if item.arg}
        package = keywords.get("package")
        if isinstance(package, ast.Constant) and package.value == "rviz2":
            rviz_nodes.append(keywords)

    assert len(rviz_nodes) == 1
    parameters = rviz_nodes[0].get("parameters")
    assert isinstance(parameters, ast.List)
    assert any(
        isinstance(item, ast.Attribute)
        and isinstance(item.value, ast.Name)
        and item.value.id == "moveit_config"
        and item.attr == "planning_pipelines"
        for item in parameters.elts
    )


def test_ft_defaults_reject_non_object_json(tmp_path):
    launch_module = _load_module(
        "rb10_real_perception_sketch_test",
        WORKSPACE_SRC
        / "sketch_control"
        / "launch"
        / "rb10_real_perception_sketch.launch.py",
    )

    for index, payload in enumerate(("null", "42", "[]")):
        config_path = tmp_path / f"invalid_shape_{index}.json"
        config_path.write_text(payload, encoding="utf-8")
        launch_module.DEFAULT_FT_CONFIG_PATH = str(config_path)
        defaults = launch_module._load_ft_defaults()
        assert defaults["target_force_n"] == "1.6"
        assert defaults["abort_force_n"] == "5.0"


def test_force_launch_config_keeps_only_explicit_runtime_overrides():
    launch_module = _load_module(
        "painting_admittance_control_test",
        WORKSPACE_SRC
        / "rbpodo_painting_control"
        / "launch"
        / "painting_admittance_control.launch.py",
    )
    overrides = {
        "desired_contact_force_n": 1.6,
        "dry_run": True,
        "enable_force": False,
        "max_command_force_n": 99.0,
        "publish_rate_hz": 1.0,
        "requested_wrench_topic": "/wrong",
        "target_wrench_sign": -1.0,
        "contact_force_sign": 1.0,
    }

    layers = launch_module._parameter_layers("/tmp/painting.yaml", overrides)
    assert layers == [
        "/tmp/painting.yaml",
        {
            "desired_contact_force_n": 1.6,
            "dry_run": True,
            "enable_force": False,
        },
    ]
    assert launch_module._parameter_layers("", overrides) == [overrides]
