import ast
import importlib.util
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
