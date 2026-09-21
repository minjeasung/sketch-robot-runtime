"""Pure fail-closed decision logic for the painting wrench guard.

The ROS node deliberately keeps all watchdog timestamps in monotonic time and
passes a snapshot into :func:`evaluate_guard`.  Keeping this module free of ROS
types makes the safety truth table small enough to unit-test exhaustively.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


WrenchTuple = Tuple[float, float, float, float, float, float]
ZERO_WRENCH: WrenchTuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
NONZERO_MODES = frozenset({"RAMP_UP", "PAINT", "RAMP_DOWN"})


@dataclass(frozen=True)
class GuardConfig:
    """Watchdog and command limits used for one guard evaluation."""

    requested_wrench_timeout_s: float
    mode_timeout_s: float
    enable_timeout_s: float
    executor_heartbeat_timeout_s: float
    ft_timeout_s: float
    tf_timeout_s: float
    safety_status_timeout_s: float
    controller_status_timeout_s: float
    max_command_force_n: float
    max_command_torque_nm: float = 0.0


@dataclass(frozen=True)
class GuardInputs:
    """Latest input values and their local monotonic receive/check times."""

    now: float
    real_painting_enabled: bool
    requested_wrench: Optional[WrenchTuple]
    requested_wrench_received_at: Optional[float]
    requested_frame_valid: bool
    mode: str
    mode_received_at: Optional[float]
    force_enable: bool
    force_enable_received_at: Optional[float]
    executor_heartbeat: bool
    executor_heartbeat_received_at: Optional[float]
    ft_valid: bool
    ft_received_at: Optional[float]
    tf_valid: bool
    tf_checked_at: Optional[float]
    safety_status_valid: bool
    safety_status_received_at: Optional[float]
    abort_latched: bool
    controller_fault: bool
    controller_status_received_at: Optional[float]


@dataclass(frozen=True)
class GuardDecision:
    """Guard output.  A non-empty blocker list always implies zero output."""

    output_wrench: WrenchTuple
    compliance_enabled: bool
    blockers: Tuple[str, ...]
    requested_force_norm_n: float
    requested_torque_norm_nm: float

    @property
    def forwarding(self) -> bool:
        return not self.blockers


def _fresh(now: float, received_at: Optional[float], timeout_s: float) -> bool:
    if received_at is None or timeout_s <= 0.0:
        return False
    age = now - received_at
    return -1.0e-6 <= age <= timeout_s


def _norm(values: WrenchTuple, start: int) -> float:
    return math.sqrt(sum(value * value for value in values[start : start + 3]))


def evaluate_guard(config: GuardConfig, inputs: GuardInputs) -> GuardDecision:
    """Evaluate every allow condition; any failed condition forces exact zero.

    All failures are returned, instead of stopping at the first one, so the ROS
    wrapper can publish actionable diagnostics while retaining a single simple
    rule for the controller output.
    """

    blockers = []
    requested = inputs.requested_wrench
    requested_force_norm = math.inf
    requested_torque_norm = math.inf

    if not inputs.real_painting_enabled:
        blockers.append("REAL_PAINTING_DISABLED")

    if not _fresh(
        inputs.now,
        inputs.requested_wrench_received_at,
        config.requested_wrench_timeout_s,
    ):
        blockers.append("REQUESTED_WRENCH_STALE")
    if requested is None or len(requested) != 6 or not all(
        math.isfinite(value) for value in (requested or ())
    ):
        blockers.append("REQUESTED_WRENCH_INVALID")
    else:
        requested_force_norm = _norm(requested, 0)
        requested_torque_norm = _norm(requested, 3)
        if requested_force_norm > config.max_command_force_n:
            blockers.append("REQUESTED_FORCE_CAP")
        if requested_torque_norm > config.max_command_torque_nm:
            blockers.append("REQUESTED_TORQUE_CAP")
    if not inputs.requested_frame_valid:
        blockers.append("REQUEST_FRAME_INVALID")

    if not _fresh(inputs.now, inputs.mode_received_at, config.mode_timeout_s):
        blockers.append("MODE_STALE")
    if inputs.mode not in NONZERO_MODES:
        blockers.append("MODE_NOT_FORCE_CAPABLE")

    if not _fresh(inputs.now, inputs.force_enable_received_at, config.enable_timeout_s):
        blockers.append("FORCE_ENABLE_STALE")
    if not inputs.force_enable:
        blockers.append("FORCE_DISABLED")

    if not _fresh(
        inputs.now,
        inputs.executor_heartbeat_received_at,
        config.executor_heartbeat_timeout_s,
    ):
        blockers.append("EXECUTOR_HEARTBEAT_STALE")
    if not inputs.executor_heartbeat:
        blockers.append("EXECUTOR_HEARTBEAT_FALSE")

    if not _fresh(inputs.now, inputs.ft_received_at, config.ft_timeout_s):
        blockers.append("FT_STALE")
    if not inputs.ft_valid:
        blockers.append("FT_INVALID")

    if not _fresh(inputs.now, inputs.tf_checked_at, config.tf_timeout_s):
        blockers.append("TF_STALE")
    if not inputs.tf_valid:
        blockers.append("TF_INVALID")

    if not _fresh(
        inputs.now,
        inputs.safety_status_received_at,
        config.safety_status_timeout_s,
    ):
        blockers.append("SAFETY_STATUS_STALE")
    if not inputs.safety_status_valid:
        blockers.append("SAFETY_STATUS_INVALID")
    if inputs.abort_latched:
        blockers.append("ABORT_LATCHED")

    if not _fresh(
        inputs.now,
        inputs.controller_status_received_at,
        config.controller_status_timeout_s,
    ):
        blockers.append("CONTROLLER_STATUS_STALE")
    if inputs.controller_fault:
        blockers.append("CONTROLLER_FAULT")

    unique_blockers = tuple(dict.fromkeys(blockers))
    if unique_blockers:
        output = ZERO_WRENCH
        compliance_enabled = False
    else:
        # requested cannot be None here because that condition is a blocker.
        output = requested if requested is not None else ZERO_WRENCH
        compliance_enabled = True

    return GuardDecision(
        output_wrench=output,
        compliance_enabled=compliance_enabled,
        blockers=unique_blockers,
        requested_force_norm_n=requested_force_norm,
        requested_torque_norm_nm=requested_torque_norm,
    )
