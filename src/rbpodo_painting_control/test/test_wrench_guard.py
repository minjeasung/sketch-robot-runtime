from dataclasses import replace

from rbpodo_painting_control.wrench_guard import (
    GuardConfig,
    GuardInputs,
    NONZERO_MODES,
    ZERO_WRENCH,
    evaluate_guard,
)


def config():
    return GuardConfig(
        requested_wrench_timeout_s=0.1,
        mode_timeout_s=0.1,
        enable_timeout_s=0.1,
        executor_heartbeat_timeout_s=0.1,
        ft_timeout_s=0.1,
        tf_timeout_s=0.1,
        safety_status_timeout_s=0.1,
        controller_status_timeout_s=0.1,
        max_command_force_n=15.0,
        max_command_torque_nm=0.0,
    )


def valid_inputs():
    return GuardInputs(
        now=10.0,
        real_painting_enabled=True,
        requested_wrench=(0.0, -1.6, 0.0, 0.0, 0.0, 0.0),
        requested_wrench_received_at=9.95,
        requested_frame_valid=True,
        mode="RAMP_UP",
        mode_received_at=9.95,
        force_enable=True,
        force_enable_received_at=9.95,
        executor_heartbeat=True,
        executor_heartbeat_received_at=9.95,
        ft_valid=True,
        ft_received_at=9.95,
        tf_valid=True,
        tf_checked_at=9.95,
        safety_status_valid=True,
        safety_status_received_at=9.95,
        abort_latched=False,
        controller_fault=False,
        controller_status_received_at=9.95,
    )


def test_forwards_only_when_every_guard_is_true_and_fresh():
    decision = evaluate_guard(config(), valid_inputs())
    assert decision.forwarding
    assert decision.compliance_enabled
    assert decision.output_wrench[1] == -1.6
    assert decision.blockers == ()


def test_ramp_down_remains_force_capable_for_a_commanded_zero_ramp():
    assert "RAMP_DOWN" in NONZERO_MODES
    decision = evaluate_guard(config(), replace(valid_inputs(), mode="RAMP_DOWN"))
    assert decision.forwarding


def test_any_stale_or_disallowed_mode_forces_exact_zero():
    inputs = replace(valid_inputs(), mode="CONTACT_SEARCH", mode_received_at=9.0)
    decision = evaluate_guard(config(), inputs)
    assert decision.output_wrench == ZERO_WRENCH
    assert not decision.compliance_enabled
    assert "MODE_STALE" in decision.blockers
    assert "MODE_NOT_FORCE_CAPABLE" in decision.blockers


def test_abort_controller_fault_and_disabled_real_mode_are_fail_closed():
    inputs = replace(
        valid_inputs(),
        abort_latched=True,
        controller_fault=True,
        real_painting_enabled=False,
    )
    decision = evaluate_guard(config(), inputs)
    assert decision.output_wrench == ZERO_WRENCH
    assert set(decision.blockers) >= {
        "ABORT_LATCHED",
        "CONTROLLER_FAULT",
        "REAL_PAINTING_DISABLED",
    }


def test_nonfinite_and_vector_norm_over_cap_are_rejected():
    nonfinite = replace(
        valid_inputs(),
        requested_wrench=(0.0, float("nan"), 0.0, 0.0, 0.0, 0.0),
    )
    assert "REQUESTED_WRENCH_INVALID" in evaluate_guard(config(), nonfinite).blockers

    over_cap = replace(
        valid_inputs(),
        requested_wrench=(12.0, 12.0, 0.0, 0.0, 0.0, 0.0),
    )
    decision = evaluate_guard(config(), over_cap)
    assert decision.output_wrench == ZERO_WRENCH
    assert "REQUESTED_FORCE_CAP" in decision.blockers
