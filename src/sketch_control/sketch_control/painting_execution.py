"""ROS-independent execution gates for the painting state machine."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def joint_velocities_allow_stationary(
    velocities: Sequence[float],
    *,
    expected_count: int,
    max_abs_velocity_rad_s: float,
) -> bool:
    """Validate optional joint velocity feedback for a stationary decision.

    The RB10 ros2_control description exports position and effort only.  The
    joint-state broadcaster represents that entirely unavailable velocity
    interface as one NaN per joint.  Treat an empty vector or an all-NaN vector
    as unavailable and let the independent position-delta/hold-time check make
    the decision.  Mixed validity, infinity, a size mismatch, or an actual
    finite velocity above the limit remains fail-closed.
    """

    if expected_count <= 0 or not math.isfinite(float(max_abs_velocity_rad_s)):
        return False
    if max_abs_velocity_rad_s < 0.0:
        return False
    values = tuple(float(value) for value in velocities)
    if not values:
        return True
    if len(values) != expected_count:
        return False
    if all(math.isnan(value) for value in values):
        return True
    return all(
        math.isfinite(value) and abs(value) <= max_abs_velocity_rad_s
        for value in values
    )


@dataclass(frozen=True)
class ContactSearchConfig:
    step_m: float
    max_distance_m: float
    timeout_s: float

    def __post_init__(self) -> None:
        for name, value in (
            ("step_m", self.step_m),
            ("max_distance_m", self.max_distance_m),
            ("timeout_s", self.timeout_s),
        ):
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.step_m > self.max_distance_m:
            raise ValueError("step_m cannot exceed max_distance_m")


@dataclass(frozen=True)
class ContactSearchDecision:
    action: str
    next_step_m: float = 0.0
    fault_reason: str = ""


@dataclass(frozen=True)
class StationaryPaintHoldDecision:
    """Result of one fail-closed stationary PAINT hold check."""

    action: str
    fault_reason: str = ""


def evaluate_stationary_paint_hold(
    *,
    elapsed_s: float,
    duration_s: float,
    now_s: float,
    safety_status_received_at_s: float,
    guard_status_received_at_s: float,
    contact_received_at_s: float,
    safety_timeout_s: float,
    guard_timeout_s: float,
    contact_timeout_s: float,
    abort_requested: bool,
    safety_abort_latched: object,
    ft_valid: object,
    tf_valid: object,
    guard_forwarding: object,
    guard_compliance_enabled: object,
    guard_compliance_active: object,
    guard_controller_fault: object,
    contact_confirmed: object,
) -> StationaryPaintHoldDecision:
    """Check whether a zero-motion PAINT hold may continue or complete.

    The caller invokes this at a fixed rate while the ordinary PAINT command,
    executor heartbeat, safety monitor, and wrench guard remain active.  Every
    dependency uses locally received monotonic timestamps; missing, future, or
    stale feedback fails closed instead of allowing a timed hold to finish.
    """

    numeric = (
        elapsed_s,
        duration_s,
        now_s,
        safety_status_received_at_s,
        guard_status_received_at_s,
        contact_received_at_s,
        safety_timeout_s,
        guard_timeout_s,
        contact_timeout_s,
    )
    if not all(math.isfinite(float(value)) for value in numeric):
        return StationaryPaintHoldDecision("FAULT", "PAINT_HOLD_TIME_INVALID")
    if (
        duration_s <= 0.0
        or elapsed_s < 0.0
        or safety_timeout_s <= 0.0
        or guard_timeout_s <= 0.0
        or contact_timeout_s <= 0.0
    ):
        return StationaryPaintHoldDecision("FAULT", "PAINT_HOLD_TIME_INVALID")

    def fresh(received_at_s: float, timeout_s: float) -> bool:
        age_s = float(now_s) - float(received_at_s)
        return received_at_s > 0.0 and 0.0 <= age_s <= timeout_s

    if abort_requested or safety_abort_latched is not False:
        return StationaryPaintHoldDecision("FAULT", "ABORT_LATCHED")
    if not fresh(safety_status_received_at_s, safety_timeout_s):
        return StationaryPaintHoldDecision("FAULT", "SAFETY_STATUS_STALE")
    if ft_valid is not True:
        return StationaryPaintHoldDecision("FAULT", "FT_STALE")
    if tf_valid is not True:
        return StationaryPaintHoldDecision("FAULT", "TF_INVALID")
    if not fresh(guard_status_received_at_s, guard_timeout_s):
        return StationaryPaintHoldDecision("FAULT", "WRENCH_GUARD_STALE")
    if guard_controller_fault is not False:
        return StationaryPaintHoldDecision("FAULT", "CONTROLLER_FAULT")
    if guard_forwarding is not True:
        return StationaryPaintHoldDecision(
            "FAULT", "WRENCH_GUARD_NOT_FORWARDING"
        )
    if (
        guard_compliance_enabled is not True
        or guard_compliance_active is not True
    ):
        return StationaryPaintHoldDecision("FAULT", "ADMITTANCE_NOT_ACTIVE")
    if not fresh(contact_received_at_s, contact_timeout_s):
        return StationaryPaintHoldDecision("FAULT", "CONTACT_STATUS_STALE")
    if contact_confirmed is not True:
        return StationaryPaintHoldDecision("FAULT", "CONTACT_LOST")
    if elapsed_s >= duration_s:
        return StationaryPaintHoldDecision("COMPLETE")
    return StationaryPaintHoldDecision("WAIT")


def ramp_down_handshake_action(
    *,
    ramp_complete: bool,
    disable_requested: bool,
    now_s: float,
    disable_requested_at_s: float,
    guard_status_received_at_s: float,
    guard_forwarding: object,
    compliance_enabled: object,
    guard_compliance_active: object = None,
    guard_mode: object = None,
    guard_force_enable: object = None,
    guard_status_sequence: object = None,
    disable_requested_guard_sequence: object = None,
    guard_source_timestamp_s: object = None,
    disable_requested_guard_source_timestamp_s: object = None,
    guard_timeout_s: float = 0.5,
) -> str:
    """Drive the zero-slew -> disable -> guard-zero acknowledgement barrier.

    ``WAIT_SLEW`` is only possible before the disable command.  Once the
    caller records ``disable_requested=True``, a later ramp feedback sample
    cannot move the transaction back into the slew phase: only a guard status
    proven to be newer than the disable edge may complete it.

    The locally received monotonic timestamp is always required.  A guard
    source timestamp and/or receive sequence may additionally be supplied;
    each supplied ordering mechanism must have a matching edge value and be
    strictly newer than it.  Missing zero-state fields fail closed.
    """

    if disable_requested is not True:
        return "DISABLE" if ramp_complete is True else "WAIT_SLEW"

    def finite_float(value: object) -> float | None:
        try:
            converted = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return converted if math.isfinite(converted) else None

    now = finite_float(now_s)
    disable_at = finite_float(disable_requested_at_s)
    received_at = finite_float(guard_status_received_at_s)
    timeout = finite_float(guard_timeout_s)
    if (
        now is None
        or disable_at is None
        or received_at is None
        or timeout is None
        or disable_at <= 0.0
        or timeout <= 0.0
        or received_at <= disable_at
        or not 0.0 <= now - received_at <= timeout
    ):
        return "WAIT_GUARD_ZERO_ACK"

    sequence_values = (
        guard_status_sequence,
        disable_requested_guard_sequence,
    )
    if any(value is not None for value in sequence_values):
        if any(value is None for value in sequence_values):
            return "WAIT_GUARD_ZERO_ACK"
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in sequence_values
        ):
            return "WAIT_GUARD_ZERO_ACK"
        if guard_status_sequence <= disable_requested_guard_sequence:
            return "WAIT_GUARD_ZERO_ACK"

    source_timestamp_values = (
        guard_source_timestamp_s,
        disable_requested_guard_source_timestamp_s,
    )
    if any(value is not None for value in source_timestamp_values):
        if any(value is None for value in source_timestamp_values):
            return "WAIT_GUARD_ZERO_ACK"
        source_timestamp = finite_float(guard_source_timestamp_s)
        source_edge = finite_float(disable_requested_guard_source_timestamp_s)
        if (
            source_timestamp is None
            or source_edge is None
            or source_edge < 0.0
            or source_timestamp <= source_edge
        ):
            return "WAIT_GUARD_ZERO_ACK"

    if (
        guard_mode == "RAMP_DOWN"
        and guard_force_enable is False
        and guard_forwarding is False
        and compliance_enabled is False
        and guard_compliance_active is False
    ):
        return "COMPLETE"
    return "WAIT_GUARD_ZERO_ACK"


def evaluate_contact_search(
    config: ContactSearchConfig,
    *,
    elapsed_s: float,
    cumulative_distance_m: float,
    contact_confirmed: bool,
    ft_valid: bool,
    tf_valid: bool,
    controller_fault: bool,
    abort_latched: bool,
) -> ContactSearchDecision:
    """Return the next bounded contact-search action.

    The caller executes at most one small Cartesian step for ``STEP`` and calls
    this function again after the action result.  No trajectory is generated
    when a safety input is invalid.
    """

    if abort_latched:
        return ContactSearchDecision("FAULT", fault_reason="ABORT_LATCHED")
    if not ft_valid:
        return ContactSearchDecision("FAULT", fault_reason="FT_STALE")
    if not tf_valid:
        return ContactSearchDecision("FAULT", fault_reason="TF_INVALID")
    if controller_fault:
        return ContactSearchDecision("FAULT", fault_reason="CONTROLLER_FAULT")
    if contact_confirmed:
        return ContactSearchDecision("CONTACT_CONFIRMED")
    if not math.isfinite(float(elapsed_s)) or elapsed_s >= config.timeout_s:
        return ContactSearchDecision("FAULT", fault_reason="CONTACT_NOT_FOUND")
    remaining = config.max_distance_m - max(0.0, float(cumulative_distance_m))
    if remaining <= 1e-9:
        return ContactSearchDecision("FAULT", fault_reason="CONTACT_NOT_FOUND")
    return ContactSearchDecision("STEP", next_step_m=min(config.step_m, remaining))


def real_plan_gate_blockers(
    *,
    segment_present: bool,
    segment_version: int | None,
    segment_path_id: str,
    waypoint_path_id: str,
    segment_plan_hash: str,
    accepted_plan_hash: str,
    accepted_plan_path_id: str,
    segment_work_area_id: str,
    current_work_area_id: str,
    segment_plane_generation_id: str,
    current_plane_generation_id: str,
    d405_plane_accepted: bool,
) -> tuple[str, ...]:
    """Check runtime identities without any receive-time heuristic."""

    blockers: list[str] = []
    if not segment_present:
        blockers.append("SEGMENT_PATH_MISSING")
        return tuple(blockers)
    if segment_version != 3:
        blockers.append("SEGMENT_SCHEMA_NOT_V3")
    required = (
        ("SEGMENT_PATH_ID_MISSING", segment_path_id),
        ("WAYPOINT_PATH_ID_MISSING", waypoint_path_id),
        ("PLAN_HASH_MISSING", segment_plan_hash),
        ("ACCEPTED_PLAN_HASH_MISSING", accepted_plan_hash),
        ("ACCEPTED_PLAN_PATH_ID_MISSING", accepted_plan_path_id),
        ("WORK_AREA_ID_MISSING", segment_work_area_id),
        ("PLANE_GENERATION_ID_MISSING", segment_plane_generation_id),
    )
    blockers.extend(reason for reason, value in required if not str(value).strip())
    if segment_path_id and waypoint_path_id and segment_path_id != waypoint_path_id:
        blockers.append("PATH_ID_MISMATCH")
    if accepted_plan_hash and segment_plan_hash != accepted_plan_hash:
        blockers.append("PLAN_HASH_MISMATCH")
    if (
        accepted_plan_path_id
        and segment_path_id
        and accepted_plan_path_id != segment_path_id
    ):
        blockers.append("ACCEPTED_PLAN_PATH_ID_MISMATCH")
    if not d405_plane_accepted:
        blockers.append("D405_PLANE_NOT_ACCEPTED")
    if (
        segment_work_area_id
        and current_work_area_id
        and segment_work_area_id != current_work_area_id
    ):
        blockers.append("WORK_AREA_ID_MISMATCH")
    elif not current_work_area_id:
        blockers.append("CURRENT_WORK_AREA_ID_MISSING")
    if (
        segment_plane_generation_id
        and current_plane_generation_id
        and segment_plane_generation_id != current_plane_generation_id
    ):
        blockers.append("PLANE_GENERATION_ID_MISMATCH")
    elif not current_plane_generation_id:
        blockers.append("CURRENT_PLANE_GENERATION_ID_MISSING")
    return tuple(dict.fromkeys(blockers))


def follow_joint_watchdog_reason(
    *,
    now_s: float,
    result_deadline_s: float,
    cancel_requested: bool,
    cancel_deadline_s: float,
    abort_latched: bool,
) -> str:
    """Return a fail-closed action watchdog reason, or an empty string."""

    values = (now_s, result_deadline_s, cancel_deadline_s)
    if not all(math.isfinite(float(value)) for value in values):
        return "FJT_WATCHDOG_TIME_INVALID"
    if (
        cancel_requested
        and cancel_deadline_s > 0.0
        and now_s >= cancel_deadline_s
    ):
        return "FJT_CANCEL_TIMEOUT"
    if (
        not abort_latched
        and result_deadline_s > 0.0
        and now_s >= result_deadline_s
    ):
        return "FJT_RESULT_TIMEOUT"
    return ""
