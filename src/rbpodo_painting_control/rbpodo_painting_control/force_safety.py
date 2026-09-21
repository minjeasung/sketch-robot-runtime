"""ROS-independent six-axis painting F/T safety and latch logic."""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional, Tuple


Vector3 = Tuple[float, float, float]
WrenchTuple = Tuple[float, float, float, float, float, float]

NONE = "NONE"
FT_STALE = "FT_STALE"
FT_NONFINITE = "FT_NONFINITE"
FT_SATURATION = "FT_SATURATION"
TF_INVALID = "TF_INVALID"
NORMAL_OVERFORCE = "NORMAL_OVERFORCE"
CONTACT_DIRECTION_MISMATCH = "CONTACT_DIRECTION_MISMATCH"
OFF_AXIS_CONTACT = "OFF_AXIS_CONTACT"
TANGENTIAL_FORCE_LIMIT = "TANGENTIAL_FORCE_LIMIT"
FORCE_NORM_LIMIT = "FORCE_NORM_LIMIT"
TORQUE_LIMIT = "TORQUE_LIMIT"
FORCE_DERIVATIVE_LIMIT = "FORCE_DERIVATIVE_LIMIT"
TORQUE_DERIVATIVE_LIMIT = "TORQUE_DERIVATIVE_LIMIT"
RAW_IMPACT = "RAW_IMPACT"
UNEXPECTED_CONTACT = "UNEXPECTED_CONTACT"
CONTROLLER_FAULT = "CONTROLLER_FAULT"
ADMITTANCE_TRIM_LIMIT = "ADMITTANCE_TRIM_LIMIT"
ROLLER_BALANCE_LIMIT = "ROLLER_BALANCE_LIMIT"

# Missing F/T data is expected before the execution-scoped pre-contact tare.
# It must keep compliance disabled, but it must not prevent the geometry-only
# safety/pre-contact approach that is required before that tare can run.
# Every other safety reason (impact, over-force, controller fault, ...) remains
# an immediate motion abort in every mode.
PRE_TARE_DEFERRED_ABORT_REASONS = frozenset(
    {FT_STALE, FT_NONFINITE, TF_INVALID}
)
FORCE_SENSOR_REQUIRED_MODES = frozenset(
    {"CONTACT_SEARCH", "RAMP_UP", "PAINT", "RAMP_DOWN"}
)

# Derivative limits are evaluated on wall-clock message arrival, so DDS
# delivery jitter can compress the interval between two samples far below the
# nominal publish period.  Without a floor a burst-delivered pair inflates the
# computed rate by the same factor and trips an impact limit on sensor noise.
# 5 ms is half the 100 Hz broadcaster period: short enough to preserve a real
# impact edge, long enough that jitter alone cannot manufacture one.
MIN_DERIVATIVE_DT_S = 0.005

# Independent debounce channels; see ForceSafetyMonitor._debounced_latch.
FILTERED_CHANNEL = "filtered"
RAW_CHANNEL = "raw"


@dataclass(frozen=True)
class ModeLimits:
    """Six-axis thresholds for one process mode; values <= 0 disable a test."""

    force_axis_n: Vector3
    force_norm_n: float
    torque_axis_nm: Vector3
    torque_norm_nm: float
    force_derivative_nps: float
    torque_derivative_nmps: float
    raw_force_axis_n: Vector3
    raw_force_norm_n: float
    raw_torque_axis_nm: Vector3
    raw_torque_norm_nm: float
    raw_force_derivative_nps: float
    raw_torque_derivative_nmps: float
    unexpected_contact_n: float = 0.0
    contact_opposite_force_n: float = 0.0
    contact_off_axis_force_n: float = 0.0
    filtered_debounce_s: float = 0.02
    raw_debounce_s: float = 0.0


@dataclass(frozen=True)
class SafetyConfig:
    mode_limits: Mapping[str, ModeLimits]
    fallback_limits: ModeLimits
    contact_force_sign: float
    absolute_normal_force: bool
    contact_detect_n: float
    contact_release_n: float
    contact_confirm_duration_s: float
    force_saturation_n: float
    torque_saturation_nm: float
    filtered_timeout_s: float
    raw_timeout_s: float
    tf_timeout_s: float
    max_message_age_s: float
    reset_max_force_norm_n: float
    reset_max_torque_norm_nm: float


@dataclass(frozen=True)
class ResetConditions:
    mode: str
    force_enabled: bool
    trajectory_active: bool
    robot_stationary: bool
    free_space: bool


@dataclass(frozen=True)
class ResetResult:
    success: bool
    blockers: Tuple[str, ...]


class BiasEstimator:
    """Interlocked finite-sample bias window with explicit readiness.

    Leaving the calibration posture resets only the in-progress window; a
    completed bias remains usable for the ensuing motion. Sensor/TF invalidity
    and an explicit safety reset call ``invalidate`` and require a new window.
    """

    def __init__(self, duration_s: float, dimensions: int = 6) -> None:
        if duration_s < 0.0:
            raise ValueError("duration_s must be non-negative")
        if dimensions <= 0:
            raise ValueError("dimensions must be positive")
        self.duration_s = float(duration_s)
        self.dimensions = int(dimensions)
        self.bias = [0.0] * self.dimensions
        self.ready = False
        self.started_at: Optional[float] = None
        self.accum = [0.0] * self.dimensions
        self.count = 0

    def reset_window(self) -> None:
        self.started_at = None
        self.accum = [0.0] * self.dimensions
        self.count = 0

    def invalidate(self, clear_bias: bool = True) -> None:
        self.ready = False
        if clear_bias:
            self.bias = [0.0] * self.dimensions
        self.reset_window()

    def observe(self, values, now: float, eligible: bool) -> bool:
        """Observe one sample and return True only when a window completes."""
        sample = tuple(float(value) for value in values)
        if len(sample) != self.dimensions or not all(
            math.isfinite(value) for value in sample
        ):
            self.invalidate()
            return False
        if not eligible:
            self.reset_window()
            return False
        if self.started_at is None:
            self.started_at = now
        self.accum = [total + value for total, value in zip(self.accum, sample)]
        self.count += 1
        if now - self.started_at < self.duration_s:
            return False

        average = [value / self.count for value in self.accum]
        if not self.ready:
            self.bias = average
        else:
            self.bias = [
                0.99 * old + 0.01 * new for old, new in zip(self.bias, average)
            ]
        self.ready = True
        self.reset_window()
        return True


class SustainedCondition:
    """Qualify a continuously asserted condition inside an explicit context."""

    def __init__(self, duration_s: float) -> None:
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        self.duration_s = float(duration_s)
        self.since: Optional[float] = None

    def reset(self) -> None:
        self.since = None

    def update(self, asserted: bool, in_context: bool, now: float) -> bool:
        if not asserted or not in_context:
            self.reset()
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.duration_s

    def active_duration(self, now: float) -> float:
        return 0.0 if self.since is None else max(0.0, now - self.since)

    @property
    def pending(self) -> bool:
        return self.since is not None


def consume_free_space_confirmation(
    current: bool, mode: str, trajectory_active: bool
) -> bool:
    """Keep an operator confirmation only in a motion-inhibited reset state."""
    reset_safe_mode = mode.strip().upper() in {"IDLE", "ABORT"}
    return bool(current) and reset_safe_mode and not trajectory_active


def motion_abort_required(
    reason: str, mode: str, force_enabled: bool, *, noncontact_spray: bool = False
) -> bool:
    """Return whether a latched safety reason must stop robot motion now.

    Before the pre-contact runtime tare, the hardware intentionally exports
    NaN and the force monitor reports the sensor as invalid.  Geometry-only
    approach is still allowed in that state, with compliance forced off.  As
    soon as contact search or another force-capable mode begins, the exact
    same invalidity becomes an immediate abort.  Non-sensor faults are never
    deferred.
    """

    normalized_reason = str(reason).strip().upper() or NONE
    if normalized_reason == NONE:
        return False
    ft_reasons = {FT_STALE, FT_NONFINITE, FT_SATURATION, TF_INVALID,
        NORMAL_OVERFORCE, CONTACT_DIRECTION_MISMATCH, OFF_AXIS_CONTACT,
        TANGENTIAL_FORCE_LIMIT, FORCE_NORM_LIMIT, TORQUE_LIMIT,
        FORCE_DERIVATIVE_LIMIT, TORQUE_DERIVATIVE_LIMIT, RAW_IMPACT, UNEXPECTED_CONTACT}
    if noncontact_spray and not force_enabled and normalized_reason in ft_reasons:
        return False
    normalized_mode = str(mode).strip().upper() or "UNKNOWN"
    if normalized_reason not in PRE_TARE_DEFERRED_ABORT_REASONS:
        return True
    return bool(
        force_enabled or normalized_mode in FORCE_SENSOR_REQUIRED_MODES
    )


def _finite(values: WrenchTuple) -> bool:
    return len(values) == 6 and all(math.isfinite(value) for value in values)


def _norm3(values) -> float:
    return math.sqrt(sum(value * value for value in values))


def _exceeds(value: float, limit: float) -> bool:
    return limit > 0.0 and abs(value) >= limit


def _axis_exceeded(values, limits: Vector3) -> Optional[int]:
    for index, (value, limit) in enumerate(zip(values, limits)):
        if _exceeds(value, limit):
            return index
    return None


class ForceSafetyMonitor:
    """Stateful six-axis safety monitor with an explicit reset-only latch."""

    def __init__(self, config: SafetyConfig, started_at: float = 0.0) -> None:
        if (
            not math.isfinite(float(config.contact_force_sign))
            or float(config.contact_force_sign) not in (-1.0, 1.0)
        ):
            raise ValueError("contact_force_sign must be exactly -1.0 or +1.0")
        self.config = config
        self.started_at = started_at
        self.latched_reason = NONE
        self.latched_detail = ""
        self.latched_at: Optional[float] = None
        self.filtered: WrenchTuple = (0.0,) * 6
        self.raw: WrenchTuple = (0.0,) * 6
        self.filtered_derivative: WrenchTuple = (0.0,) * 6
        self.raw_derivative: WrenchTuple = (0.0,) * 6
        self.last_filtered_at: Optional[float] = None
        self.last_raw_at: Optional[float] = None
        self.last_tf_ok_at: Optional[float] = None
        self.tf_valid = False
        self.tf_source_age_s: Optional[float] = None
        self.last_message_age_s: Optional[float] = None
        self.filtered_finite = False
        self.raw_finite = False
        self.nonfinite = True
        self.saturated = False
        self.contact_confirmed = False
        self.contact_since: Optional[float] = None
        self.last_contact_at: Optional[float] = None
        # The filtered and raw envelopes are independent detectors running at
        # the same rate.  A single shared debounce slot let each one reset the
        # other's timer on every interleaved sample, so a candidate that was
        # continuously asserted could never accumulate its debounce window and
        # the latch was silently missed.  Keep one slot per channel.
        self._pending_reason = {FILTERED_CHANNEL: NONE, RAW_CHANNEL: NONE}
        self._pending_since: Dict[str, Optional[float]] = {
            FILTERED_CHANNEL: None,
            RAW_CHANNEL: None,
        }

    def _contact_force(self, normal_force_n: float) -> float:
        """Return the normal reaction used for contact decisions.

        The real painting profile treats the measured TCP-normal channel as a
        magnitude because its upstream sign has reversed while physical wall
        contact was maintained.  Legacy profiles may retain directional sign
        conversion.  The original signed sample remains available in status.
        """
        if self.config.absolute_normal_force:
            return abs(normal_force_n)
        return self.config.contact_force_sign * normal_force_n

    def limits_for(self, mode: str) -> ModeLimits:
        return self.config.mode_limits.get(mode.upper(), self.config.fallback_limits)

    def _latch(self, reason: str, detail: str, now: float) -> None:
        # A startup F/T/TF invalidity is intentionally non-actionable while
        # the robot performs its geometry-only move to the pre-contact tare
        # pose.  It must not mask a later impact, over-force, or controller
        # fault.  Escalate exactly once from a deferred sensor reason to any
        # non-deferred reason; otherwise preserve the original first-fault
        # latch for diagnostics and explicit reset semantics.
        may_escalate = bool(
            self.latched_reason in PRE_TARE_DEFERRED_ABORT_REASONS
            and reason not in PRE_TARE_DEFERRED_ABORT_REASONS
            and reason != NONE
        )
        if self.latched_reason != NONE and not may_escalate:
            return
        self.latched_reason = reason
        self.latched_detail = detail
        self.latched_at = now

    def latch_external(self, reason: str, detail: str, now: float) -> None:
        """Latch a non-wrench safety input such as a controller fault."""
        self._latch(reason, detail, now)

    def _debounced_latch(
        self,
        reason: str,
        detail: str,
        now: float,
        debounce_s: float,
        channel: str = FILTERED_CHANNEL,
    ) -> None:
        may_escalate = bool(
            self.latched_reason in PRE_TARE_DEFERRED_ABORT_REASONS
            and reason not in PRE_TARE_DEFERRED_ABORT_REASONS
            and reason != NONE
        )
        if self.latched_reason != NONE and not may_escalate:
            return
        if debounce_s <= 0.0:
            self._latch(reason, detail, now)
            return
        if self._pending_reason[channel] != reason:
            self._pending_reason[channel] = reason
            self._pending_since[channel] = now
            return
        since = self._pending_since[channel]
        if since is not None and now - since >= debounce_s:
            self._latch(reason, detail, now)

    def _clear_pending(self, channel: Optional[str] = None) -> None:
        channels = (
            (channel,) if channel is not None else tuple(self._pending_reason)
        )
        for name in channels:
            self._pending_reason[name] = NONE
            self._pending_since[name] = None

    def observe_tf(
        self, valid: bool, now: float, source_age_s: Optional[float] = None
    ) -> None:
        self.tf_valid = bool(valid)
        self.tf_source_age_s = source_age_s
        if valid:
            self.last_tf_ok_at = now
        else:
            self._latch(TF_INVALID, "ft_link to tcp transform unavailable", now)

    def _update_contact(self, normal_force_n: float, now: float) -> None:
        contact_force_n = self._contact_force(normal_force_n)
        if contact_force_n >= self.config.contact_detect_n:
            if self.contact_since is None:
                self.contact_since = now
            if now - self.contact_since >= self.config.contact_confirm_duration_s:
                self.contact_confirmed = True
                self.last_contact_at = now
        else:
            self.contact_since = None
        if contact_force_n <= self.config.contact_release_n:
            self.contact_confirmed = False

    @staticmethod
    def _derivative(
        current: WrenchTuple,
        previous: WrenchTuple,
        now: float,
        previous_at: Optional[float],
    ) -> WrenchTuple:
        if previous_at is None or now <= previous_at:
            return (0.0,) * 6
        dt = max(now - previous_at, MIN_DERIVATIVE_DT_S)
        return tuple((value - old) / dt for value, old in zip(current, previous))

    def process_filtered(
        self,
        values: WrenchTuple,
        now: float,
        mode: str,
        message_age_s: float = 0.0,
    ) -> None:
        if not _finite(values):
            self.filtered_finite = False
            self.nonfinite = True
            self._latch(FT_NONFINITE, "filtered wrench contains NaN or infinity", now)
            return
        self.filtered_finite = True
        self.nonfinite = not (self.filtered_finite and self.raw_finite)
        if message_age_s < -0.05 or message_age_s > self.config.max_message_age_s:
            self._latch(
                FT_STALE,
                "filtered wrench header age %.6f s" % message_age_s,
                now,
            )
        previous = self.filtered
        previous_at = self.last_filtered_at
        self.filtered_derivative = self._derivative(values, previous, now, previous_at)
        self.filtered = tuple(float(value) for value in values)
        self.last_filtered_at = now
        self.last_message_age_s = message_age_s
        self._update_contact(self.filtered[1], now)

        limits = self.limits_for(mode)
        force = self.filtered[:3]
        torque = self.filtered[3:]
        contact_force_n = self._contact_force(force[1])
        tangential_force_n = math.hypot(force[0], force[2])
        force_derivative = _norm3(self.filtered_derivative[:3])
        torque_derivative = _norm3(self.filtered_derivative[3:])
        candidate = None

        force_axis = _axis_exceeded(force, limits.force_axis_n)
        if (
            mode.upper() == "CONTACT_SEARCH"
            and not self.config.absolute_normal_force
            and limits.contact_opposite_force_n > 0.0
            and contact_force_n <= -limits.contact_opposite_force_n
        ):
            candidate = (
                CONTACT_DIRECTION_MISMATCH,
                "directional contact force %.3f N is opposite the commissioned "
                "compression direction by at least %.3f N"
                % (contact_force_n, limits.contact_opposite_force_n),
            )
        elif (
            mode.upper() == "CONTACT_SEARCH"
            and not self.contact_confirmed
            and contact_force_n < self.config.contact_detect_n
            and limits.contact_off_axis_force_n > 0.0
            and tangential_force_n >= limits.contact_off_axis_force_n
        ):
            candidate = (
                OFF_AXIS_CONTACT,
                "TCP tangential force %.3f N reached %.3f N before directional "
                "contact was confirmed (directional=%.3f N)"
                % (
                    tangential_force_n,
                    limits.contact_off_axis_force_n,
                    contact_force_n,
                ),
            )
        elif (
            limits.unexpected_contact_n > 0.0
            and _norm3(force) >= limits.unexpected_contact_n
        ):
            candidate = (
                UNEXPECTED_CONTACT,
                "force norm %.3f N in %s exceeds %.3f N"
                % (_norm3(force), mode, limits.unexpected_contact_n),
            )
        elif force_axis == 1:
            candidate = (
                NORMAL_OVERFORCE,
                "TCP Fy %.3f N exceeds %.3f N"
                % (force[1], limits.force_axis_n[1]),
            )
        elif force_axis is not None:
            candidate = (
                TANGENTIAL_FORCE_LIMIT,
                "TCP F%s %.3f N exceeds %.3f N"
                % ("xyz"[force_axis], force[force_axis], limits.force_axis_n[force_axis]),
            )
        elif limits.force_norm_n > 0.0 and _norm3(force) >= limits.force_norm_n:
            candidate = (
                FORCE_NORM_LIMIT,
                "force norm %.3f N exceeds %.3f N"
                % (_norm3(force), limits.force_norm_n),
            )
        else:
            torque_axis = _axis_exceeded(torque, limits.torque_axis_nm)
            if torque_axis is not None:
                candidate = (
                    TORQUE_LIMIT,
                    "TCP T%s %.3f Nm exceeds %.3f Nm"
                    % (
                        "xyz"[torque_axis],
                        torque[torque_axis],
                        limits.torque_axis_nm[torque_axis],
                    ),
                )
            elif limits.torque_norm_nm > 0.0 and _norm3(torque) >= limits.torque_norm_nm:
                candidate = (
                    TORQUE_LIMIT,
                    "torque norm %.3f Nm exceeds %.3f Nm"
                    % (_norm3(torque), limits.torque_norm_nm),
                )
            elif (
                limits.force_derivative_nps > 0.0
                and force_derivative >= limits.force_derivative_nps
            ):
                candidate = (
                    FORCE_DERIVATIVE_LIMIT,
                    "force derivative %.3f N/s exceeds %.3f N/s"
                    % (force_derivative, limits.force_derivative_nps),
                )
            elif (
                limits.torque_derivative_nmps > 0.0
                and torque_derivative >= limits.torque_derivative_nmps
            ):
                candidate = (
                    TORQUE_DERIVATIVE_LIMIT,
                    "torque derivative %.3f Nm/s exceeds %.3f Nm/s"
                    % (torque_derivative, limits.torque_derivative_nmps),
                )
        if candidate is None:
            self._clear_pending(FILTERED_CHANNEL)
        else:
            self._debounced_latch(
                candidate[0],
                candidate[1],
                now,
                limits.filtered_debounce_s,
                channel=FILTERED_CHANNEL,
            )

    def process_raw(
        self,
        values: WrenchTuple,
        now: float,
        mode: str,
        message_age_s: float = 0.0,
    ) -> None:
        if not _finite(values):
            self.raw_finite = False
            self.nonfinite = True
            self._latch(FT_NONFINITE, "raw wrench contains NaN or infinity", now)
            return
        self.raw_finite = True
        self.nonfinite = not (self.filtered_finite and self.raw_finite)
        if message_age_s < -0.05 or message_age_s > self.config.max_message_age_s:
            self._latch(FT_STALE, "raw wrench header age %.6f s" % message_age_s, now)
        previous = self.raw
        previous_at = self.last_raw_at
        self.raw_derivative = self._derivative(values, previous, now, previous_at)
        self.raw = tuple(float(value) for value in values)
        self.last_raw_at = now

        force = self.raw[:3]
        torque = self.raw[3:]
        force_saturated = self.config.force_saturation_n > 0.0 and any(
            abs(value) >= self.config.force_saturation_n for value in force
        )
        torque_saturated = self.config.torque_saturation_nm > 0.0 and any(
            abs(value) >= self.config.torque_saturation_nm for value in torque
        )
        self.saturated = force_saturated or torque_saturated
        if self.saturated:
            self._latch(
                FT_SATURATION,
                "raw sensor saturation threshold reached",
                now,
            )
            return

        limits = self.limits_for(mode)
        force_derivative = _norm3(self.raw_derivative[:3])
        torque_derivative = _norm3(self.raw_derivative[3:])
        impact = (
            _axis_exceeded(force, limits.raw_force_axis_n) is not None
            or (limits.raw_force_norm_n > 0.0 and _norm3(force) >= limits.raw_force_norm_n)
            or _axis_exceeded(torque, limits.raw_torque_axis_nm) is not None
            or (
                limits.raw_torque_norm_nm > 0.0
                and _norm3(torque) >= limits.raw_torque_norm_nm
            )
            or (
                limits.raw_force_derivative_nps > 0.0
                and force_derivative >= limits.raw_force_derivative_nps
            )
            or (
                limits.raw_torque_derivative_nmps > 0.0
                and torque_derivative >= limits.raw_torque_derivative_nmps
            )
        )
        if impact:
            self._debounced_latch(
                RAW_IMPACT,
                "raw force/torque impact threshold reached",
                now,
                limits.raw_debounce_s,
                channel=RAW_CHANNEL,
            )
        else:
            self._clear_pending(RAW_CHANNEL)

    def tick(self, now: float, require_raw: bool = True) -> None:
        if self.last_filtered_at is None:
            filtered_age = now - self.started_at
        else:
            filtered_age = now - self.last_filtered_at
        if filtered_age > self.config.filtered_timeout_s:
            self._latch(FT_STALE, "filtered wrench age %.6f s" % filtered_age, now)

        if require_raw:
            raw_age = (
                now - self.started_at
                if self.last_raw_at is None
                else now - self.last_raw_at
            )
            if raw_age > self.config.raw_timeout_s:
                self._latch(FT_STALE, "raw wrench age %.6f s" % raw_age, now)

        tf_age = (
            now - self.started_at
            if self.last_tf_ok_at is None
            else now - self.last_tf_ok_at
        )
        if tf_age > self.config.tf_timeout_s:
            self._latch(TF_INVALID, "TF validity age %.6f s" % tf_age, now)

    def ft_valid(self, now: float, require_raw: bool = True) -> bool:
        filtered_fresh = (
            self.last_filtered_at is not None
            and now - self.last_filtered_at <= self.config.filtered_timeout_s
        )
        raw_fresh = (
            not require_raw
            or (
                self.last_raw_at is not None
                and now - self.last_raw_at <= self.config.raw_timeout_s
            )
        )
        finite = self.filtered_finite and (not require_raw or self.raw_finite)
        return filtered_fresh and raw_fresh and finite and not self.saturated

    def tf_is_valid(self, now: float) -> bool:
        return (
            self.tf_valid
            and self.last_tf_ok_at is not None
            and now - self.last_tf_ok_at <= self.config.tf_timeout_s
        )

    def reset(
        self,
        conditions: ResetConditions,
        now: float,
        require_raw: bool = True,
    ) -> ResetResult:
        blockers = []
        # ABORT keeps motion latched and force disabled.  Permitting reset in
        # this state breaks the circular dependency where the executor needs a
        # clear safety latch before publishing IDLE while this monitor used to
        # require IDLE first.  All remaining reset interlocks stay mandatory.
        if conditions.mode.upper() not in {"IDLE", "ABORT"}:
            blockers.append("MODE_NOT_IDLE")
        if conditions.force_enabled:
            blockers.append("FORCE_ENABLED")
        if conditions.trajectory_active:
            blockers.append("TRAJECTORY_ACTIVE")
        if not conditions.robot_stationary:
            blockers.append("ROBOT_NOT_STATIONARY")
        if not conditions.free_space:
            blockers.append("FREE_SPACE_UNCONFIRMED")
        if not self.ft_valid(now, require_raw=require_raw):
            blockers.append("FT_INVALID_OR_STALE")
        if not self.tf_is_valid(now):
            blockers.append("TF_INVALID_OR_STALE")
        if _norm3(self.filtered[:3]) > self.config.reset_max_force_norm_n:
            blockers.append("FORCE_NOT_LOW")
        if _norm3(self.filtered[3:]) > self.config.reset_max_torque_norm_nm:
            blockers.append("TORQUE_NOT_LOW")

        result = ResetResult(not blockers, tuple(blockers))
        if result.success:
            self.latched_reason = NONE
            self.latched_detail = ""
            self.latched_at = None
            self._clear_pending()
        return result

    def status(self, now: float, mode: str, require_raw: bool = True) -> Dict[str, object]:
        def finite_or_none(value):
            return value if value is not None and math.isfinite(value) else None

        filtered_force = self.filtered[:3]
        filtered_torque = self.filtered[3:]
        raw_force = self.raw[:3]
        raw_torque = self.raw[3:]
        return {
            "mode": mode,
            "abort_latched": self.latched_reason != NONE,
            "reason": self.latched_reason,
            "detail": self.latched_detail,
            "latched_at_monotonic": self.latched_at,
            "contact_confirmed": self.contact_confirmed,
            "contact_force_sign": self.config.contact_force_sign,
            "absolute_normal_force": self.config.absolute_normal_force,
            "normal_force_tcp_y_signed_n": self.filtered[1],
            "contact_force_n": self._contact_force(self.filtered[1]),
            "contact_tangential_force_n": math.hypot(
                self.filtered[0], self.filtered[2]
            ),
            "ft_valid": self.ft_valid(now, require_raw=require_raw),
            "tf_valid": self.tf_is_valid(now),
            "nonfinite": not self.filtered_finite or (
                require_raw and not self.raw_finite
            ),
            "saturated": self.saturated,
            "filtered_wrench": list(self.filtered),
            "raw_wrench": list(self.raw),
            "filtered_force_norm_n": _norm3(filtered_force),
            "filtered_torque_norm_nm": _norm3(filtered_torque),
            "raw_force_norm_n": _norm3(raw_force),
            "raw_torque_norm_nm": _norm3(raw_torque),
            "filtered_force_derivative_nps": _norm3(self.filtered_derivative[:3]),
            "filtered_torque_derivative_nmps": _norm3(self.filtered_derivative[3:]),
            "raw_force_derivative_nps": _norm3(self.raw_derivative[:3]),
            "raw_torque_derivative_nmps": _norm3(self.raw_derivative[3:]),
            "filtered_age_s": (
                None if self.last_filtered_at is None else now - self.last_filtered_at
            ),
            "raw_age_s": None if self.last_raw_at is None else now - self.last_raw_at,
            "message_age_s": finite_or_none(self.last_message_age_s),
            "tf_age_s": (
                None if self.last_tf_ok_at is None else now - self.last_tf_ok_at
            ),
            "tf_source_stamp_age_s": finite_or_none(self.tf_source_age_s),
        }
