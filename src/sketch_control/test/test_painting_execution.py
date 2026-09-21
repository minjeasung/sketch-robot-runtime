import pytest

from sketch_control.painting_execution import (
    ContactSearchConfig,
    evaluate_contact_search,
    evaluate_stationary_paint_hold,
    follow_joint_watchdog_reason,
    joint_velocities_allow_stationary,
    ramp_down_handshake_action,
    real_plan_gate_blockers,
)


def _healthy_stationary_hold(**overrides):
    values = dict(
        elapsed_s=1.0,
        duration_s=2.0,
        now_s=10.0,
        safety_status_received_at_s=9.95,
        guard_status_received_at_s=9.95,
        contact_received_at_s=9.95,
        safety_timeout_s=0.20,
        guard_timeout_s=0.50,
        contact_timeout_s=0.20,
        abort_requested=False,
        safety_abort_latched=False,
        ft_valid=True,
        tf_valid=True,
        guard_forwarding=True,
        guard_compliance_enabled=True,
        guard_compliance_active=True,
        guard_controller_fault=False,
        contact_confirmed=True,
    )
    values.update(overrides)
    return evaluate_stationary_paint_hold(**values)


def test_stationary_paint_hold_waits_then_completes_with_fresh_dependencies():
    assert _healthy_stationary_hold(elapsed_s=1.99).action == "WAIT"
    assert _healthy_stationary_hold(elapsed_s=2.0).action == "COMPLETE"


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"abort_requested": True}, "ABORT_LATCHED"),
        ({"safety_status_received_at_s": 9.0}, "SAFETY_STATUS_STALE"),
        ({"ft_valid": False}, "FT_STALE"),
        ({"tf_valid": False}, "TF_INVALID"),
        ({"guard_status_received_at_s": 9.0}, "WRENCH_GUARD_STALE"),
        ({"guard_controller_fault": True}, "CONTROLLER_FAULT"),
        ({"guard_forwarding": False}, "WRENCH_GUARD_NOT_FORWARDING"),
        ({"guard_compliance_enabled": False}, "ADMITTANCE_NOT_ACTIVE"),
        ({"guard_compliance_active": False}, "ADMITTANCE_NOT_ACTIVE"),
        ({"contact_received_at_s": 9.0}, "CONTACT_STATUS_STALE"),
        ({"contact_confirmed": False}, "CONTACT_LOST"),
    ],
)
def test_stationary_paint_hold_fails_closed(overrides, reason):
    decision = _healthy_stationary_hold(**overrides)
    assert decision.action == "FAULT"
    assert decision.fault_reason == reason


@pytest.mark.parametrize("velocities", [[], [float("nan")] * 6])
def test_unavailable_joint_velocities_allow_position_based_stationary_check(
    velocities,
):
    assert joint_velocities_allow_stationary(
        velocities,
        expected_count=6,
        max_abs_velocity_rad_s=0.01,
    )


@pytest.mark.parametrize(
    "velocities",
    [
        [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0],
        [0.0, 0.0, float("inf"), 0.0, 0.0, 0.0],
    ],
)
def test_partially_nonfinite_joint_velocities_fail_closed(velocities):
    assert not joint_velocities_allow_stationary(
        velocities,
        expected_count=6,
        max_abs_velocity_rad_s=0.01,
    )


def test_finite_joint_velocity_above_stationary_limit_is_moving():
    assert not joint_velocities_allow_stationary(
        [0.0, 0.0, 0.0101, 0.0, 0.0, 0.0],
        expected_count=6,
        max_abs_velocity_rad_s=0.01,
    )


def test_contact_search_steps_are_bounded_then_fail_closed():
    config = ContactSearchConfig(step_m=0.0005, max_distance_m=0.0012, timeout_s=8.0)
    decision = evaluate_contact_search(
        config,
        elapsed_s=1.0,
        cumulative_distance_m=0.0010,
        contact_confirmed=False,
        ft_valid=True,
        tf_valid=True,
        controller_fault=False,
        abort_latched=False,
    )
    assert decision.action == "STEP"
    assert decision.next_step_m == pytest.approx(0.0002)

    exhausted = evaluate_contact_search(
        config,
        elapsed_s=2.0,
        cumulative_distance_m=0.0012,
        contact_confirmed=False,
        ft_valid=True,
        tf_valid=True,
        controller_fault=False,
        abort_latched=False,
    )
    assert exhausted.action == "FAULT"
    assert exhausted.fault_reason == "CONTACT_NOT_FOUND"


def test_contact_search_sensor_or_tf_failure_never_requests_motion():
    config = ContactSearchConfig(step_m=0.0005, max_distance_m=0.010, timeout_s=8.0)
    assert evaluate_contact_search(
        config,
        elapsed_s=0.1,
        cumulative_distance_m=0.0,
        contact_confirmed=False,
        ft_valid=False,
        tf_valid=True,
        controller_fault=False,
        abort_latched=False,
    ).fault_reason == "FT_STALE"
    assert evaluate_contact_search(
        config,
        elapsed_s=0.1,
        cumulative_distance_m=0.0,
        contact_confirmed=False,
        ft_valid=True,
        tf_valid=False,
        controller_fault=False,
        abort_latched=False,
    ).fault_reason == "TF_INVALID"


def test_contact_search_simulated_contact_transitions_without_another_step():
    config = ContactSearchConfig(step_m=0.0005, max_distance_m=0.010, timeout_s=8.0)

    decision = evaluate_contact_search(
        config,
        elapsed_s=0.25,
        cumulative_distance_m=0.0005,
        contact_confirmed=True,
        ft_valid=True,
        tf_valid=True,
        controller_fault=False,
        abort_latched=False,
    )

    assert decision.action == "CONTACT_CONFIRMED"
    assert decision.next_step_m == 0.0


def test_real_plan_gate_rejects_every_identity_mismatch():
    blockers = real_plan_gate_blockers(
        segment_present=True,
        segment_version=2,
        segment_path_id="segment",
        waypoint_path_id="pose-array",
        segment_plan_hash="hash-a",
        accepted_plan_hash="hash-b",
        accepted_plan_path_id="accepted-path",
        segment_work_area_id="wa-a",
        current_work_area_id="wa-b",
        segment_plane_generation_id="plane-a",
        current_plane_generation_id="plane-b",
        d405_plane_accepted=False,
    )
    assert {
        "SEGMENT_SCHEMA_NOT_V3",
        "PATH_ID_MISMATCH",
        "PLAN_HASH_MISMATCH",
        "ACCEPTED_PLAN_PATH_ID_MISMATCH",
        "WORK_AREA_ID_MISMATCH",
        "PLANE_GENERATION_ID_MISMATCH",
        "D405_PLANE_NOT_ACCEPTED",
    }.issubset(blockers)


def test_follow_joint_watchdog_times_out_result_and_cancel_paths():
    assert follow_joint_watchdog_reason(
        now_s=10.0,
        result_deadline_s=9.0,
        cancel_requested=False,
        cancel_deadline_s=0.0,
        abort_latched=False,
    ) == "FJT_RESULT_TIMEOUT"
    assert follow_joint_watchdog_reason(
        now_s=10.0,
        result_deadline_s=9.0,
        cancel_requested=True,
        cancel_deadline_s=9.5,
        abort_latched=True,
    ) == "FJT_CANCEL_TIMEOUT"


def test_follow_joint_watchdog_does_not_raise_result_timeout_after_abort():
    assert follow_joint_watchdog_reason(
        now_s=10.0,
        result_deadline_s=9.0,
        cancel_requested=False,
        cancel_deadline_s=0.0,
        abort_latched=True,
    ) == ""


def _ramp_down_zero_ack(**overrides):
    values = dict(
        ramp_complete=False,
        disable_requested=True,
        now_s=10.2,
        disable_requested_at_s=10.0,
        guard_status_received_at_s=10.1,
        guard_forwarding=False,
        compliance_enabled=False,
        guard_compliance_active=False,
        guard_mode="RAMP_DOWN",
        guard_force_enable=False,
    )
    values.update(overrides)
    return ramp_down_handshake_action(**values)


def test_ramp_down_has_distinct_slew_disable_and_zero_ack_phases():
    assert ramp_down_handshake_action(
        ramp_complete=False,
        disable_requested=False,
        now_s=9.9,
        disable_requested_at_s=0.0,
        guard_status_received_at_s=9.9,
        guard_forwarding=True,
        compliance_enabled=True,
    ) == "WAIT_SLEW"
    assert ramp_down_handshake_action(
        ramp_complete=True,
        disable_requested=False,
        now_s=10.0,
        disable_requested_at_s=0.0,
        guard_status_received_at_s=9.9,
        guard_forwarding=True,
        compliance_enabled=True,
    ) == "DISABLE"
    assert _ramp_down_zero_ack(
        guard_status_received_at_s=10.0,
    ) == "WAIT_GUARD_ZERO_ACK"
    # A disable command may reset ramp_complete.  The transaction must stay in
    # the acknowledgement phase and may complete from the post-disable guard.
    assert _ramp_down_zero_ack(ramp_complete=False) == "COMPLETE"


@pytest.mark.parametrize(
    "overrides",
    [
        {"guard_status_received_at_s": 9.9},
        {"guard_status_received_at_s": 10.0},
        {"now_s": 10.7, "guard_status_received_at_s": 10.1},
        {"now_s": 10.0, "guard_status_received_at_s": 10.1},
        {"guard_status_received_at_s": float("nan")},
        {"guard_timeout_s": 0.0},
        {"disable_requested_at_s": 0.0},
    ],
)
def test_ramp_down_rejects_invalid_or_nonfresh_local_guard_edge(overrides):
    assert _ramp_down_zero_ack(**overrides) == "WAIT_GUARD_ZERO_ACK"


@pytest.mark.parametrize(
    "overrides",
    [
        {"guard_mode": "PAINT"},
        {"guard_mode": "ramp_down"},
        {"guard_mode": None},
        {"guard_force_enable": True},
        {"guard_force_enable": None},
        {"guard_forwarding": True},
        {"guard_forwarding": None},
        {"compliance_enabled": True},
        {"compliance_enabled": None},
        {"guard_compliance_active": True},
        {"guard_compliance_active": None},
    ],
)
def test_ramp_down_zero_ack_requires_exact_disabled_guard_state(overrides):
    assert _ramp_down_zero_ack(**overrides) == "WAIT_GUARD_ZERO_ACK"


def test_ramp_down_accepts_strictly_newer_optional_sequence_and_source_edges():
    assert _ramp_down_zero_ack(
        guard_status_sequence=42,
        disable_requested_guard_sequence=41,
        guard_source_timestamp_s=100.1,
        disable_requested_guard_source_timestamp_s=100.0,
    ) == "COMPLETE"


@pytest.mark.parametrize(
    "overrides",
    [
        {
            "guard_status_sequence": 41,
            "disable_requested_guard_sequence": 41,
        },
        {
            "guard_status_sequence": 40,
            "disable_requested_guard_sequence": 41,
        },
        {"guard_status_sequence": 42},
        {"disable_requested_guard_sequence": 41},
        {
            "guard_status_sequence": 42.0,
            "disable_requested_guard_sequence": 41,
        },
        {
            "guard_status_sequence": True,
            "disable_requested_guard_sequence": 0,
        },
        {
            "guard_source_timestamp_s": 100.0,
            "disable_requested_guard_source_timestamp_s": 100.0,
        },
        {
            "guard_source_timestamp_s": 99.9,
            "disable_requested_guard_source_timestamp_s": 100.0,
        },
        {"guard_source_timestamp_s": 100.1},
        {"disable_requested_guard_source_timestamp_s": 100.0},
        {
            "guard_source_timestamp_s": float("nan"),
            "disable_requested_guard_source_timestamp_s": 100.0,
        },
    ],
)
def test_ramp_down_rejects_nonnew_or_incomplete_optional_edges(overrides):
    assert _ramp_down_zero_ack(**overrides) == "WAIT_GUARD_ZERO_ACK"
