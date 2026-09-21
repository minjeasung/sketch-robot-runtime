import math

import pytest

from rbpodo_painting_control.force_safety import (
    BiasEstimator,
    CONTACT_DIRECTION_MISMATCH,
    ForceSafetyMonitor,
    ModeLimits,
    NORMAL_OVERFORCE,
    NONE,
    OFF_AXIS_CONTACT,
    RAW_IMPACT,
    ResetConditions,
    ROLLER_BALANCE_LIMIT,
    SafetyConfig,
    SustainedCondition,
    TANGENTIAL_FORCE_LIMIT,
    CONTROLLER_FAULT,
    FT_NONFINITE,
    FT_STALE,
    TF_INVALID,
    consume_free_space_confirmation,
    motion_abort_required,
)


def test_bias_ready_requires_complete_interlocked_window_and_invalidates_on_bad_sample():
    estimator = BiasEstimator(duration_s=1.0)
    sample = (0.2, -0.1, 0.3, 0.01, -0.02, 0.03)

    assert not estimator.observe(sample, 1.0, eligible=True)
    assert not estimator.ready
    assert not estimator.observe(sample, 1.5, eligible=True)
    assert not estimator.ready

    # Losing an interlock discards the partial window.
    assert not estimator.observe(sample, 1.6, eligible=False)
    assert not estimator.observe(sample, 2.0, eligible=True)
    assert not estimator.observe(sample, 2.9, eligible=True)
    assert estimator.observe(sample, 3.01, eligible=True)
    assert estimator.ready
    assert estimator.bias == pytest.approx(sample)

    # Starting motion only resets an in-progress adaptation window; the
    # completed calibration remains valid for that session.
    estimator.reset_window()
    assert estimator.ready

    bad = (float("nan"),) + sample[1:]
    assert not estimator.observe(bad, 3.1, eligible=True)
    assert not estimator.ready
    assert estimator.bias == [0.0] * 6


def test_bias_reset_requires_a_new_full_window():
    estimator = BiasEstimator(duration_s=0.5)
    sample = (0.0,) * 6
    assert not estimator.observe(sample, 0.0, eligible=True)
    assert estimator.observe(sample, 0.5, eligible=True)
    assert estimator.ready

    estimator.invalidate()
    assert not estimator.ready
    assert not estimator.observe(sample, 1.0, eligible=True)
    assert estimator.observe(sample, 1.5, eligible=True)
    assert estimator.ready


def test_free_space_confirmation_is_kept_in_reset_safe_inactive_modes():
    assert consume_free_space_confirmation(True, "IDLE", False)
    assert consume_free_space_confirmation(True, "ABORT", False)
    assert not consume_free_space_confirmation(True, "PAINT", False)
    assert not consume_free_space_confirmation(True, "IDLE", True)
    assert not consume_free_space_confirmation(True, "ABORT", True)
    assert not consume_free_space_confirmation(False, "IDLE", False)


@pytest.mark.parametrize("reason", [FT_NONFINITE, FT_STALE, TF_INVALID])
def test_pre_tare_sensor_invalidity_does_not_abort_geometry_approach(reason):
    assert not motion_abort_required(reason, "IDLE", False)
    assert not motion_abort_required(reason, "APPROACH_PRECONTACT", False)


@pytest.mark.parametrize("mode", ["CONTACT_SEARCH", "RAMP_UP", "PAINT", "RAMP_DOWN"])
@pytest.mark.parametrize("reason", [FT_NONFINITE, FT_STALE, TF_INVALID])
def test_sensor_invalidity_aborts_every_force_required_mode(reason, mode):
    assert motion_abort_required(reason, mode, False)


def test_force_enable_arms_sensor_invalidity_and_other_faults_never_defer():
    assert motion_abort_required(FT_STALE, "IDLE", True)
    assert motion_abort_required(RAW_IMPACT, "APPROACH_PRECONTACT", False)
    assert not motion_abort_required(NONE, "PAINT", True)
    assert motion_abort_required(ROLLER_BALANCE_LIMIT, "PAINT", True)


def test_sustained_condition_requires_continuous_true_in_force_context():
    qualifier = SustainedCondition(0.25)
    assert not qualifier.update(True, True, 1.0)
    assert not qualifier.update(True, True, 1.20)
    assert qualifier.pending
    assert qualifier.active_duration(1.20) == pytest.approx(0.20)

    # A false sample resets the continuous window.
    assert not qualifier.update(False, True, 1.21)
    assert not qualifier.pending
    assert not qualifier.update(True, True, 1.22)
    assert not qualifier.update(True, True, 1.40)

    # Leaving force context also resets, even while the signal stays true.
    assert not qualifier.update(True, False, 1.41)
    assert not qualifier.update(True, True, 2.0)
    assert qualifier.update(True, True, 2.26)


def limits(**changes):
    values = {
        "force_axis_n": (5.0, 10.0, 5.0),
        "force_norm_n": 20.0,
        "torque_axis_nm": (2.0, 2.0, 2.0),
        "torque_norm_nm": 3.0,
        "force_derivative_nps": 0.0,
        "torque_derivative_nmps": 0.0,
        "raw_force_axis_n": (20.0, 20.0, 20.0),
        "raw_force_norm_n": 25.0,
        "raw_torque_axis_nm": (4.0, 4.0, 4.0),
        "raw_torque_norm_nm": 5.0,
        "raw_force_derivative_nps": 0.0,
        "raw_torque_derivative_nmps": 0.0,
        "unexpected_contact_n": 0.0,
        "contact_opposite_force_n": 0.0,
        "contact_off_axis_force_n": 0.0,
        "filtered_debounce_s": 0.1,
        "raw_debounce_s": 0.0,
    }
    values.update(changes)
    return ModeLimits(**values)


def config(
    mode_limits=None, *, contact_force_sign=1.0, absolute_normal_force=True
):
    fallback = limits()
    return SafetyConfig(
        mode_limits=mode_limits or {"PAINT": fallback, "IDLE": fallback},
        fallback_limits=fallback,
        contact_force_sign=contact_force_sign,
        absolute_normal_force=absolute_normal_force,
        contact_detect_n=1.5,
        contact_release_n=0.8,
        contact_confirm_duration_s=0.1,
        force_saturation_n=100.0,
        torque_saturation_nm=10.0,
        filtered_timeout_s=0.2,
        raw_timeout_s=0.2,
        tf_timeout_s=0.2,
        max_message_age_s=0.2,
        reset_max_force_norm_n=1.0,
        reset_max_torque_norm_nm=0.2,
    )


def make_valid(monitor, now=1.0, wrench=(0.0,) * 6):
    monitor.observe_tf(True, now)
    monitor.process_filtered(wrench, now, "IDLE")
    monitor.process_raw(wrench, now, "IDLE")


def test_filtered_axis_limit_debounces_then_latches_until_explicit_reset():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.observe_tf(True, 1.0)
    wrench = (0.0, 11.0, 0.0, 0.0, 0.0, 0.0)
    monitor.process_filtered(wrench, 1.0, "PAINT")
    assert monitor.latched_reason == NONE
    monitor.process_filtered(wrench, 1.11, "PAINT")
    assert monitor.latched_reason == NORMAL_OVERFORCE

    monitor.process_filtered((0.0,) * 6, 1.12, "PAINT")
    assert monitor.latched_reason == NORMAL_OVERFORCE


def test_all_axis_tangential_limit_is_not_reduced_to_normal_force_only():
    immediate = limits(filtered_debounce_s=0.0)
    monitor = ForceSafetyMonitor(config({"PAINT": immediate}), started_at=0.0)
    monitor.observe_tf(True, 1.0)
    monitor.process_filtered((6.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    assert monitor.latched_reason == TANGENTIAL_FORCE_LIMIT


@pytest.mark.parametrize("force_x", [-30.0, 30.0])
def test_raw_fast_path_latches_impact_immediately(force_x):
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.process_raw((force_x, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    assert monitor.latched_reason == RAW_IMPACT


def test_deferred_startup_fault_escalates_to_raw_impact():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.process_filtered((float("nan"),) * 6, 0.1, "IDLE")
    assert monitor.latched_reason == FT_NONFINITE
    assert not motion_abort_required(monitor.latched_reason, "IDLE", False)

    monitor.process_raw((30.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.2, "IDLE")
    assert monitor.latched_reason == RAW_IMPACT
    assert motion_abort_required(monitor.latched_reason, "IDLE", False)


def test_deferred_startup_fault_escalates_to_controller_fault():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.tick(0.21)
    assert monitor.latched_reason == FT_STALE

    monitor.latch_external(CONTROLLER_FAULT, "controller fault", 0.3)
    assert monitor.latched_reason == CONTROLLER_FAULT
    assert motion_abort_required(monitor.latched_reason, "IDLE", False)


def test_reset_requires_all_interlocks_and_low_fresh_wrench():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.process_raw((30.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    make_valid(monitor, now=1.1)

    blocked = monitor.reset(
        ResetConditions(
            mode="IDLE",
            force_enabled=True,
            trajectory_active=False,
            robot_stationary=True,
            free_space=True,
        ),
        now=1.1,
    )
    assert not blocked.success
    assert "FORCE_ENABLED" in blocked.blockers
    assert monitor.latched_reason == RAW_IMPACT

    reset = monitor.reset(
        ResetConditions(
            mode="IDLE",
            force_enabled=False,
            trajectory_active=False,
            robot_stationary=True,
            free_space=True,
        ),
        now=1.1,
    )
    assert reset.success
    assert monitor.latched_reason == NONE


def test_abort_mode_can_reset_but_force_capable_mode_cannot():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.process_raw((30.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    make_valid(monitor, now=1.1)

    safe_conditions = {
        "force_enabled": False,
        "trajectory_active": False,
        "robot_stationary": True,
        "free_space": True,
    }
    reset = monitor.reset(
        ResetConditions(mode="ABORT", **safe_conditions),
        now=1.1,
    )
    assert reset.success
    assert monitor.latched_reason == NONE

    blocked = monitor.reset(
        ResetConditions(mode="PAINT", **safe_conditions),
        now=1.1,
    )
    assert not blocked.success


def test_contact_confirmation_uses_filtered_normal_force():
    monitor = ForceSafetyMonitor(config(), started_at=0.0)
    monitor.observe_tf(True, 1.0)
    monitor.process_filtered((0.0, 2.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    monitor.process_filtered((0.0, 2.0, 0.0, 0.0, 0.0, 0.0), 1.11, "PAINT")
    assert monitor.contact_confirmed
    monitor.process_filtered((0.0, 0.5, 0.0, 0.0, 0.0, 0.0), 1.12, "PAINT")
    assert not monitor.contact_confirmed


def test_absolute_contact_confirmation_accepts_either_normal_sign():
    monitor = ForceSafetyMonitor(
        config(contact_force_sign=-1.0), started_at=0.0
    )
    monitor.observe_tf(True, 1.0)

    # Both signs represent the same unilateral wall-reaction magnitude.
    contact_wrench = (0.0, -4.929, 0.0, 0.0, 0.0, 0.0)
    monitor.process_filtered(contact_wrench, 1.0, "CONTACT_SEARCH")
    monitor.process_filtered(contact_wrench, 1.11, "CONTACT_SEARCH")

    assert monitor.contact_confirmed
    status = monitor.status(1.11, "CONTACT_SEARCH", require_raw=False)
    assert status["contact_force_sign"] == -1.0
    assert status["absolute_normal_force"] is True
    assert status["normal_force_tcp_y_signed_n"] == pytest.approx(-4.929)
    assert status["contact_force_n"] == pytest.approx(4.929)

    # Reversing only the reported sensor sign keeps contact confirmed.
    monitor.process_filtered(
        (0.0, 4.929, 0.0, 0.0, 0.0, 0.0),
        1.12,
        "CONTACT_SEARCH",
    )
    assert monitor.contact_confirmed
    assert monitor.status(1.12, "CONTACT_SEARCH", require_raw=False)[
        "contact_force_n"
    ] == pytest.approx(4.929)


def test_negative_contact_direction_debounce_and_release_hysteresis():
    monitor = ForceSafetyMonitor(
        config(contact_force_sign=-1.0), started_at=0.0
    )
    monitor.observe_tf(True, 1.0)

    above_detect = (0.0, -2.0, 0.0, 0.0, 0.0, 0.0)
    between_thresholds = (0.0, -1.0, 0.0, 0.0, 0.0, 0.0)
    at_release = (0.0, -0.8, 0.0, 0.0, 0.0, 0.0)

    monitor.process_filtered(above_detect, 1.00, "CONTACT_SEARCH")
    monitor.process_filtered(between_thresholds, 1.05, "CONTACT_SEARCH")
    monitor.process_filtered(above_detect, 1.10, "CONTACT_SEARCH")
    monitor.process_filtered(above_detect, 1.19, "CONTACT_SEARCH")
    assert not monitor.contact_confirmed

    monitor.process_filtered(above_detect, 1.21, "CONTACT_SEARCH")
    assert monitor.contact_confirmed

    # Once confirmed, a force between release and detect retains contact.
    monitor.process_filtered(between_thresholds, 1.22, "CONTACT_SEARCH")
    assert monitor.contact_confirmed
    monitor.process_filtered(at_release, 1.23, "CONTACT_SEARCH")
    assert not monitor.contact_confirmed


@pytest.mark.parametrize("force_y", [-8.822, 8.822])
def test_negative_contact_direction_keeps_bidirectional_overforce_guard(force_y):
    search_limits = limits(
        force_axis_n=(6.0, 8.0, 6.0),
        filtered_debounce_s=0.0,
    )
    monitor = ForceSafetyMonitor(
        config(
            {"CONTACT_SEARCH": search_limits},
            contact_force_sign=-1.0,
        ),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)
    monitor.process_filtered(
        (0.0, force_y, 0.0, 0.0, 0.0, 0.0),
        1.0,
        "CONTACT_SEARCH",
    )

    assert monitor.latched_reason == NORMAL_OVERFORCE


def test_contact_search_rejects_opposite_direction_before_it_can_authorize_paint():
    search_limits = limits(
        contact_opposite_force_n=1.5,
        contact_off_axis_force_n=3.0,
        filtered_debounce_s=0.01,
    )
    monitor = ForceSafetyMonitor(
        config(
            {"CONTACT_SEARCH": search_limits},
            contact_force_sign=1.0,
            absolute_normal_force=False,
        ),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)

    # With sign=+1, negative TCP Fy is opposite the commissioned wall reaction
    # direction.  A sustained load must fail closed, never confirm contact.
    opposite = (0.0, -1.6, 0.0, 0.0, 0.0, 0.0)
    monitor.process_filtered(opposite, 1.00, "CONTACT_SEARCH")
    assert monitor.latched_reason == NONE
    monitor.process_filtered(opposite, 1.02, "CONTACT_SEARCH")

    assert monitor.latched_reason == CONTACT_DIRECTION_MISMATCH
    assert not monitor.contact_confirmed


def test_latest_off_axis_contact_signature_stops_before_torque_limit():
    search_limits = limits(
        torque_axis_nm=(1.0, 1.0, 1.0),
        contact_opposite_force_n=1.5,
        contact_off_axis_force_n=3.0,
        filtered_debounce_s=0.01,
    )
    monitor = ForceSafetyMonitor(
        config({"CONTACT_SEARCH": search_limits}, contact_force_sign=1.0),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)

    # Reproduce the 2026-08-13 pre-trip sample: force norm 3.362 N with
    # Fy=+0.728 N means about 3.282 N was tangential.  Tx was still below its
    # 1 Nm hard limit at this point, so the new diagnostic must win first.
    off_axis = (3.282, 0.728, 0.0, 0.794, 0.0, 0.0)
    monitor.process_filtered(off_axis, 1.00, "CONTACT_SEARCH")
    assert monitor.latched_reason == NONE
    monitor.process_filtered(off_axis, 1.02, "CONTACT_SEARCH")

    assert monitor.latched_reason == OFF_AXIS_CONTACT
    assert not monitor.contact_confirmed
    status = monitor.status(1.02, "CONTACT_SEARCH", require_raw=False)
    assert status["contact_force_n"] == pytest.approx(0.728)
    assert status["contact_tangential_force_n"] == pytest.approx(3.282)


def test_current_positive_wall_reaction_confirms_contact_without_off_axis_reject():
    search_limits = limits(
        contact_opposite_force_n=1.5,
        contact_off_axis_force_n=3.0,
        filtered_debounce_s=0.01,
    )
    monitor = ForceSafetyMonitor(
        config({"CONTACT_SEARCH": search_limits}, contact_force_sign=1.0),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)

    # Current run geometry approaches the wall along TCP -Y, so the measured
    # reaction is +Fy.  Tangential load remains below the 3 N diagnostic gate.
    successful = (1.744, 1.6, 0.0, 0.478, 0.0, 0.0)
    monitor.process_filtered(successful, 1.00, "CONTACT_SEARCH")
    monitor.process_filtered(successful, 1.11, "CONTACT_SEARCH")

    assert monitor.latched_reason == NONE
    assert monitor.contact_confirmed


def test_absolute_normal_force_disables_direction_mismatch_but_keeps_axis_limit():
    search_limits = limits(
        force_axis_n=(6.0, 8.0, 6.0),
        contact_opposite_force_n=1.5,
        filtered_debounce_s=0.0,
    )
    monitor = ForceSafetyMonitor(
        config({"CONTACT_SEARCH": search_limits}, absolute_normal_force=True),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)

    monitor.process_filtered(
        (0.0, -2.0, 0.0, 0.0, 0.0, 0.0), 1.0, "CONTACT_SEARCH"
    )
    monitor.process_filtered(
        (0.0, -2.0, 0.0, 0.0, 0.0, 0.0), 1.11, "CONTACT_SEARCH"
    )
    assert monitor.latched_reason == NONE
    assert monitor.contact_confirmed

    monitor.process_filtered(
        (0.0, -8.1, 0.0, 0.0, 0.0, 0.0), 1.12, "CONTACT_SEARCH"
    )
    assert monitor.latched_reason == NORMAL_OVERFORCE


def test_disabled_contact_search_envelope_keeps_detection_without_force_latch():
    disabled = limits(
        force_axis_n=(0.0, 0.0, 0.0),
        force_norm_n=0.0,
        torque_axis_nm=(0.0, 0.0, 0.0),
        torque_norm_nm=0.0,
        force_derivative_nps=0.0,
        torque_derivative_nmps=0.0,
        raw_force_axis_n=(0.0, 0.0, 0.0),
        raw_force_norm_n=0.0,
        raw_torque_axis_nm=(0.0, 0.0, 0.0),
        raw_torque_norm_nm=0.0,
        raw_force_derivative_nps=0.0,
        raw_torque_derivative_nmps=0.0,
        contact_opposite_force_n=0.0,
        contact_off_axis_force_n=0.0,
        filtered_debounce_s=0.0,
        raw_debounce_s=0.0,
    )
    monitor = ForceSafetyMonitor(
        config({"CONTACT_SEARCH": disabled}, absolute_normal_force=True),
        started_at=0.0,
    )
    monitor.observe_tf(True, 1.0)

    # Well above the former 15 N/6 Nm search limits, but still below the
    # independent 100 N/10 Nm sensor-saturation validity thresholds.
    sample = (0.0, -90.0, 0.0, 9.0, 0.0, 0.0)
    monitor.process_filtered(sample, 1.00, "CONTACT_SEARCH")
    monitor.process_raw(sample, 1.00, "CONTACT_SEARCH")
    monitor.process_filtered(sample, 1.11, "CONTACT_SEARCH")

    assert monitor.contact_confirmed
    assert monitor.latched_reason == NONE


@pytest.mark.parametrize("contact_force_sign", [0.0, 0.5, -0.5, math.nan])
def test_contact_force_sign_must_be_a_finite_unit_direction(contact_force_sign):
    with pytest.raises(ValueError, match="contact_force_sign"):
        ForceSafetyMonitor(
            config(contact_force_sign=contact_force_sign), started_at=0.0
        )


def test_derivative_dt_floor_rejects_jitter_amplified_rate():
    """Burst-delivered samples must not manufacture an impact-rate trip.

    The rate limit is evaluated on wall-clock arrival, so a pair of messages
    delivered 1 ms apart used to inflate the computed derivative tenfold and
    latch RAW_IMPACT on a force step well inside the sensor noise band.
    """
    rate_limited = limits(raw_force_derivative_nps=900.0)
    monitor = ForceSafetyMonitor(config({"PAINT": rate_limited}), started_at=0.0)
    monitor.observe_tf(True, 1.0)

    monitor.process_raw((0.0,) * 6, 1.000, "PAINT")
    # 1.0 N of noise arriving 1 ms later is 1000 N/s without the floor and
    # 200 N/s with it.
    monitor.process_raw((0.0, 1.0, 0.0, 0.0, 0.0, 0.0), 1.001, "PAINT")
    assert monitor.latched_reason == NONE

    # A genuine edge still trips: 10 N inside one nominal period is 1000 N/s
    # even after the floor is applied.
    monitor.process_raw((0.0, 11.0, 0.0, 0.0, 0.0, 0.0), 1.011, "PAINT")
    assert monitor.latched_reason == RAW_IMPACT


def test_filtered_and_raw_debounce_windows_are_independent():
    """Interleaved channels must not reset each other's debounce timer.

    Both detectors run at the sensor rate.  With one shared pending slot, a
    continuously asserted filtered candidate never accumulated its window
    because every intervening raw sample cleared it, so the latch was missed.
    """
    mode_limits = limits(filtered_debounce_s=0.1, raw_debounce_s=0.1)
    monitor = ForceSafetyMonitor(config({"PAINT": mode_limits}), started_at=0.0)
    monitor.observe_tf(True, 1.0)

    over_force = (0.0, 11.0, 0.0, 0.0, 0.0, 0.0)
    quiet = (0.0,) * 6
    # Raw stays quiet throughout and is interleaved on every filtered sample.
    for offset in (0.0, 0.04, 0.08):
        monitor.process_filtered(over_force, 1.0 + offset, "PAINT")
        monitor.process_raw(quiet, 1.0 + offset, "PAINT")
        assert monitor.latched_reason == NONE

    monitor.process_filtered(over_force, 1.11, "PAINT")
    assert monitor.latched_reason == NORMAL_OVERFORCE


def test_raw_debounce_survives_interleaved_quiet_filtered_samples():
    mode_limits = limits(
        raw_force_axis_n=(20.0, 20.0, 20.0), raw_debounce_s=0.1
    )
    monitor = ForceSafetyMonitor(config({"PAINT": mode_limits}), started_at=0.0)
    monitor.observe_tf(True, 1.0)

    impact = (25.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for offset in (0.0, 0.04, 0.08):
        monitor.process_raw(impact, 1.0 + offset, "PAINT")
        monitor.process_filtered((0.0,) * 6, 1.0 + offset, "PAINT")
        assert monitor.latched_reason == NONE

    monitor.process_raw(impact, 1.11, "PAINT")
    assert monitor.latched_reason == RAW_IMPACT


def test_raw_debounce_window_resets_when_the_impact_clears():
    mode_limits = limits(
        raw_force_axis_n=(20.0, 20.0, 20.0), raw_debounce_s=0.1
    )
    monitor = ForceSafetyMonitor(config({"PAINT": mode_limits}), started_at=0.0)
    monitor.observe_tf(True, 1.0)

    monitor.process_raw((25.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, "PAINT")
    monitor.process_raw((0.0,) * 6, 1.05, "PAINT")
    monitor.process_raw((25.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.09, "PAINT")
    assert monitor.latched_reason == NONE
