/**
 * Copyright (c) 2026 Rainbow Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rbpodo_hardware {

constexpr size_t kHardwareWrenchSize{6};
using HardwareWrench = std::array<double, kHardwareWrenchSize>;
inline bool is_finite_ft_sample(const HardwareWrench& wrench);

constexpr size_t kHardwareJointCount{6};
using HardwareJointPositions = std::array<double, kHardwareJointCount>;

/// A bounded set of joint samples.  The hardware interface keeps one window
/// immediately before a supervised tare request and another over the tare
/// samples themselves.  Both must be stationary before a high raw F/T
/// baseline can be accepted as bias.
struct JointSampleWindow {
  HardwareJointPositions minimum{};
  HardwareJointPositions maximum{};
  size_t sample_count{0};
  bool source_valid{false};
};

/// Limits for the explicit, operator-confirmed high-baseline tare path.
///
/// expected_bias +/- max_abs_bias_delta is the commissioned free-space
/// envelope. expected_joint_positions +/- max_abs_joint_position_delta_rad
/// binds that envelope to the pose where it was commissioned. These are not
/// generic high force or workspace limits: a stable but unrelated load or a
/// different tool pose must not be silently baked into the bias.
struct SupervisedTareConfig {
  bool enabled{false};
  HardwareWrench expected_bias{};
  HardwareWrench max_abs_bias_delta{};
  HardwareWrench max_stddev{};
  HardwareJointPositions expected_joint_positions{};
  HardwareJointPositions max_abs_joint_position_delta_rad{};
  size_t min_recent_joint_samples{0};
  double max_recent_joint_excursion_rad{0.0};
  double max_tare_joint_excursion_rad{0.0};
};

/// Limits for the execution-scoped free-space tare performed at the
/// pre-contact pose.  Unlike SupervisedTareConfig, this deliberately has no
/// expected raw bias or commissioned joint pose: the whole point of the
/// runtime tare is to absorb the installation's run-to-run raw offset at the
/// current, explicitly confirmed, non-contact pose.  Safety comes from fresh
/// telemetry, motion/compliance interlocks, two stationary joint windows,
/// low wrench noise, and a post-bias residual check.
struct RuntimeTareConfig {
  HardwareWrench max_stddev{};
  HardwareWrench max_abs_post_tare_residual{};
  size_t min_recent_joint_samples{0};
  double max_recent_joint_excursion_rad{0.0};
  double max_tare_joint_excursion_rad{0.0};
};

struct RuntimeTareControlState {
  bool trajectory_status_fresh{false};
  bool trajectory_active{true};
  bool force_enable_status_fresh{false};
  bool force_enabled{true};
  bool compliance_enable_status_fresh{false};
  bool compliance_enabled{true};
  bool compliance_status_fresh{false};
  bool compliance_active{true};
};

enum class RuntimeTareRejection : uint8_t {
  None,
  InvalidConfiguration,
  CancelledOrTimedOut,
  InvalidOrStaleSource,
  SafetyInterlockActive,
  ControlStatusMissingOrStale,
  ControlInterlockViolated,
  TrajectoryActive,
  ForceEnabled,
  ComplianceEnabled,
  ComplianceActive,
  InsufficientRecentJointSamples,
  RecentJointMotion,
  TareWindowJointMotion,
  ExcessiveWrenchNoise,
  ExcessivePostTareResidual,
};

inline const char* runtime_tare_rejection_message(RuntimeTareRejection rejection) {
  switch (rejection) {
    case RuntimeTareRejection::None:
      return "accepted";
    case RuntimeTareRejection::InvalidConfiguration:
      return "invalid runtime-tare configuration";
    case RuntimeTareRejection::CancelledOrTimedOut:
      return "request was cancelled after its service deadline";
    case RuntimeTareRejection::InvalidOrStaleSource:
      return "F/T or joint source was stale/non-finite during validation";
    case RuntimeTareRejection::SafetyInterlockActive:
      return "robot initialization, collision, or emergency-stop interlock was not clear";
    case RuntimeTareRejection::ControlStatusMissingOrStale:
      return "trajectory, force-enable, or compliance status was missing/stale";
    case RuntimeTareRejection::ControlInterlockViolated:
      return "trajectory, force-enable, or compliance became active during the tare window";
    case RuntimeTareRejection::TrajectoryActive:
      return "a trajectory was active during the free-space tare";
    case RuntimeTareRejection::ForceEnabled:
      return "painting force output was enabled during the free-space tare";
    case RuntimeTareRejection::ComplianceEnabled:
      return "admittance compliance enable command was true during the free-space tare";
    case RuntimeTareRejection::ComplianceActive:
      return "admittance compliance was active during the free-space tare";
    case RuntimeTareRejection::InsufficientRecentJointSamples:
      return "not enough recent fresh stationary joint samples before the request";
    case RuntimeTareRejection::RecentJointMotion:
      return "joints moved immediately before the request";
    case RuntimeTareRejection::TareWindowJointMotion:
      return "joints moved during tare or post-tare validation";
    case RuntimeTareRejection::ExcessiveWrenchNoise:
      return "F/T sample standard deviation exceeded the runtime-tare limit";
    case RuntimeTareRejection::ExcessivePostTareResidual:
      return "post-tare corrected F/T residual exceeded the validation limit";
  }
  return "unknown runtime-tare rejection";
}

enum class SupervisedTareRejection : uint8_t {
  None,
  Disabled,
  InvalidConfiguration,
  CancelledOrTimedOut,
  InvalidOrStaleSource,
  SafetyInterlockActive,
  InsufficientRecentJointSamples,
  RecentJointMotion,
  TareWindowJointMotion,
  OutsideCommissionedJointPose,
  ExcessiveWrenchNoise,
  OutsideKnownBaselineEnvelope,
};

inline const char* supervised_tare_rejection_message(SupervisedTareRejection rejection) {
  switch (rejection) {
    case SupervisedTareRejection::None:
      return "accepted";
    case SupervisedTareRejection::Disabled:
      return "supervised tare is disabled until its installation profile is explicitly enabled";
    case SupervisedTareRejection::InvalidConfiguration:
      return "invalid supervised-tare configuration";
    case SupervisedTareRejection::CancelledOrTimedOut:
      return "request was cancelled after its service deadline";
    case SupervisedTareRejection::InvalidOrStaleSource:
      return "F/T or joint source was stale/non-finite during validation";
    case SupervisedTareRejection::SafetyInterlockActive:
      return "free-drive, initialization, collision, or emergency-stop interlock was not clear";
    case SupervisedTareRejection::InsufficientRecentJointSamples:
      return "not enough recent fresh joint samples before the request";
    case SupervisedTareRejection::RecentJointMotion:
      return "joints moved immediately before the request";
    case SupervisedTareRejection::TareWindowJointMotion:
      return "joints moved during the tare sample window";
    case SupervisedTareRejection::OutsideCommissionedJointPose:
      return "joint pose was outside the commissioned supervised-tare pose envelope";
    case SupervisedTareRejection::ExcessiveWrenchNoise:
      return "F/T sample standard deviation exceeded the supervised limit";
    case SupervisedTareRejection::OutsideKnownBaselineEnvelope:
      return "mean F/T was outside the commissioned free-space baseline envelope";
  }
  return "unknown supervised-tare rejection";
}

enum class StrictTareOutcome : uint8_t {
  NotCompleted,
  Succeeded,
  FailedInvalidSource,
  FailedNoise,
  FailedMeanEnvelopeOnly,
};

enum class ActivationTareAction : uint8_t {
  WaitForExplicitRequest,
  RequestStrict,
};

inline ActivationTareAction activation_tare_action(bool auto_tare_on_activate) {
  return auto_tare_on_activate ? ActivationTareAction::RequestStrict
                               : ActivationTareAction::WaitForExplicitRequest;
}

inline bool supervised_tare_may_follow(StrictTareOutcome strict_outcome) {
  return strict_outcome == StrictTareOutcome::FailedMeanEnvelopeOnly;
}

inline bool supervised_joint_history_sample_is_eligible(
    bool ft_source_valid, bool joint_source_valid, bool robot_safety_clear,
    bool freedrive_or_post_hold_active) {
  return ft_source_valid && joint_source_valid && robot_safety_clear &&
         !freedrive_or_post_hold_active;
}

inline bool supervised_joint_history_gap_is_contiguous(
    int64_t previous_sample_ns, int64_t current_sample_ns, int64_t max_gap_ns) {
  return previous_sample_ns > 0 && current_sample_ns >= previous_sample_ns && max_gap_ns >= 0 &&
         current_sample_ns - previous_sample_ns <= max_gap_ns;
}

/// Decode the controller's physical safety-board summary bits.
///
/// information_chunk_1:
///   bit 6  - arm DC power is on (must be 1)
///   bit 7  - direct-teach button is pressed (must be 0)
///   bit 12 - SOS is active (must be 0)
/// information_chunk_3:
///   bits 22..25 - Safety EMS2 / PRS / HSS / SSS pressed (all must be 0)
///
/// Cast to uint32_t before masking so a set sign bit in the SDK's int32 field
/// cannot affect the result through signed bit operations.
inline bool physical_safety_board_interlocks_clear(
    int32_t information_chunk_1, int32_t information_chunk_3) {
  constexpr uint32_t kArmDcPowerOn = uint32_t{1} << 6;
  constexpr uint32_t kDirectTeachPressed = uint32_t{1} << 7;
  constexpr uint32_t kSosActive = uint32_t{1} << 12;
  constexpr uint32_t kSafetyButtonsPressed = uint32_t{0xF} << 22;

  const uint32_t chunk_1 = static_cast<uint32_t>(information_chunk_1);
  const uint32_t chunk_3 = static_cast<uint32_t>(information_chunk_3);
  return (chunk_1 & kArmDcPowerOn) != 0 &&
         (chunk_1 & (kDirectTeachPressed | kSosActive)) == 0 &&
         (chunk_3 & kSafetyButtonsPressed) == 0;
}

/// The SDK collision fields are packed status words, not booleans.  In the
/// v6.10 schema only the lower two bits describe a current collision.  Upper
/// bits contain history/timezone information and must not make a healthy
/// robot look unsafe.
inline bool current_collision_status_clear(int32_t packed_status) {
  return (static_cast<uint32_t>(packed_status) & uint32_t{0x3}) == 0;
}

inline bool packed_status_bits_equal(int32_t packed_status, uint32_t valid_mask,
                                     uint32_t expected) {
  return (static_cast<uint32_t>(packed_status) & valid_mask) == expected;
}

/// Bit-for-bit explanation of `real_robot_motion_abort_interlock_triggered()`.
///
/// Stable mask: bit 0 collision, bit 1 self-collision, bit 2 SOS code,
/// bit 3 soft e-stop, bit 4 EMS code, bit 5 safety-board SOS, and bits
/// 6..9 safety EMS2/PRS/HSS/SSS respectively.
inline uint32_t real_robot_motion_abort_reason_mask(
    int32_t collision_status, int32_t sos_flag,
    int32_t self_collision_status, bool soft_estop_occurred,
    int32_t ems_flag, int32_t information_chunk_1,
    int32_t information_chunk_3) {
  constexpr uint32_t kOperationFlagValidMask{0x3F};
  constexpr uint32_t kSosActive = uint32_t{1} << 12;
  const uint32_t chunk_1 = static_cast<uint32_t>(information_chunk_1);
  const uint32_t chunk_3 = static_cast<uint32_t>(information_chunk_3);
  uint32_t reasons = 0;
  reasons |= current_collision_status_clear(collision_status) ? 0 : uint32_t{1} << 0;
  reasons |= current_collision_status_clear(self_collision_status) ? 0 : uint32_t{1} << 1;
  reasons |= packed_status_bits_equal(sos_flag, kOperationFlagValidMask, 0)
                 ? 0
                 : uint32_t{1} << 2;
  reasons |= soft_estop_occurred ? uint32_t{1} << 3 : 0;
  reasons |= packed_status_bits_equal(ems_flag, kOperationFlagValidMask, 0)
                 ? 0
                 : uint32_t{1} << 4;
  reasons |= (chunk_1 & kSosActive) != 0 ? uint32_t{1} << 5 : 0;
  for (uint32_t offset = 0; offset < 4; ++offset) {
    if ((chunk_3 & (uint32_t{1} << (22 + offset))) != 0) {
      reasons |= uint32_t{1} << (6 + offset);
    }
  }
  return reasons;
}

/// Complete pure predicate for a real robot sample.  Keeping this decision in
/// one helper prevents the supervised-tare path from accidentally checking
/// only software operation flags while omitting the physical safety board.
inline bool real_robot_safety_interlocks_clear(
    bool is_freedrive_mode, int32_t init_state_info, int32_t init_error,
    int32_t collision_status, int32_t sos_flag, int32_t self_collision_status,
    bool soft_estop_occurred, int32_t ems_flag,
    int32_t information_chunk_1, int32_t information_chunk_3) {
  constexpr uint32_t kInitStateValidMask{0x3F};
  constexpr uint32_t kInitErrorValidMask{0xFFF};
  constexpr uint32_t kOperationFlagValidMask{0x3F};
  return !is_freedrive_mode &&
         packed_status_bits_equal(init_state_info, kInitStateValidMask, 6) &&
         packed_status_bits_equal(init_error, kInitErrorValidMask, 0) &&
         current_collision_status_clear(collision_status) &&
         current_collision_status_clear(self_collision_status) &&
         packed_status_bits_equal(sos_flag, kOperationFlagValidMask, 0) &&
         !soft_estop_occurred &&
         packed_status_bits_equal(ems_flag, kOperationFlagValidMask, 0) &&
         physical_safety_board_interlocks_clear(information_chunk_1, information_chunk_3);
}

/// Irrecoverable-for-this-process motion-abort predicate.
///
/// This is deliberately narrower than the tare/readiness predicate above.
/// Free-drive, initialization, and arm-power transitions are recoverable and
/// must not permanently inhibit servo writes.  A current collision or an
/// emergency/safety stop, however, requires a full ros2_control relaunch
/// before this hardware plugin will accept another motion command.
inline bool real_robot_motion_abort_interlock_triggered(
    int32_t collision_status, int32_t sos_flag,
    int32_t self_collision_status, bool soft_estop_occurred,
    int32_t ems_flag, int32_t information_chunk_1,
    int32_t information_chunk_3) {
  return real_robot_motion_abort_reason_mask(
             collision_status, sos_flag, self_collision_status,
             soft_estop_occurred, ems_flag, information_chunk_1,
             information_chunk_3) != 0;
}

/// Decide whether a loss of the complete robot-ready predicate must become a
/// process-lifetime motion inhibit.
///
/// Before the first healthy sample, initialization and arm-power transitions
/// are expected and merely keep servo writes suppressed.  After the hardware
/// has once been ready, however, silently resuming an open-loop trajectory
/// after power/init/direct-teach recovers would jump to a later wall-clock
/// setpoint.  The only recoverable exception is a free-drive transition that
/// this hardware plugin initiated itself while all motion/compliance
/// heartbeats were already idle.
inline bool robot_safety_loss_requires_motion_inhibit(
    bool robot_safety_clear, bool has_observed_safety_clear,
    bool intentional_internal_freedrive_transition) {
  return has_observed_safety_clear && !robot_safety_clear &&
         !intentional_internal_freedrive_transition;
}

inline bool supervised_tare_config_is_valid(const SupervisedTareConfig& config) {
  if (!is_finite_ft_sample(config.expected_bias) ||
      !is_finite_ft_sample(config.max_abs_bias_delta) ||
      !is_finite_ft_sample(config.max_stddev) ||
      !std::all_of(config.expected_joint_positions.begin(),
                   config.expected_joint_positions.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !std::all_of(config.max_abs_joint_position_delta_rad.begin(),
                   config.max_abs_joint_position_delta_rad.end(),
                   [](double value) { return std::isfinite(value); }) ||
      config.min_recent_joint_samples == 0 ||
      !std::isfinite(config.max_recent_joint_excursion_rad) ||
      !std::isfinite(config.max_tare_joint_excursion_rad) ||
      config.max_recent_joint_excursion_rad < 0.0 ||
      config.max_tare_joint_excursion_rad < 0.0) {
    return false;
  }
  return std::all_of(config.max_abs_bias_delta.begin(), config.max_abs_bias_delta.end(),
                     [](double value) { return value >= 0.0; }) &&
         std::all_of(config.max_stddev.begin(), config.max_stddev.end(),
                     [](double value) { return value >= 0.0; }) &&
         std::all_of(config.max_abs_joint_position_delta_rad.begin(),
                     config.max_abs_joint_position_delta_rad.end(),
                     [](double value) { return value >= 0.0; });
}

inline bool joint_window_is_finite(const JointSampleWindow& window) {
  return std::all_of(window.minimum.begin(), window.minimum.end(), [](double value) {
           return std::isfinite(value);
         }) &&
         std::all_of(window.maximum.begin(), window.maximum.end(), [](double value) {
           return std::isfinite(value);
         });
}

inline bool joint_window_within_excursion(const JointSampleWindow& window,
                                          double max_excursion_rad) {
  if (!window.source_valid || window.sample_count == 0 || !joint_window_is_finite(window) ||
      !std::isfinite(max_excursion_rad) || max_excursion_rad < 0.0) {
    return false;
  }
  for (size_t i = 0; i < kHardwareJointCount; ++i) {
    if (window.maximum[i] < window.minimum[i] ||
        window.maximum[i] - window.minimum[i] > max_excursion_rad) {
      return false;
    }
  }
  return true;
}

inline bool joint_window_within_commissioned_pose(
    const JointSampleWindow& window,
    const HardwareJointPositions& expected_joint_positions,
    const HardwareJointPositions& max_abs_joint_position_delta_rad) {
  if (!window.source_valid || window.sample_count == 0 || !joint_window_is_finite(window)) {
    return false;
  }
  for (size_t i = 0; i < kHardwareJointCount; ++i) {
    const double expected = expected_joint_positions[i];
    const double delta = max_abs_joint_position_delta_rad[i];
    if (!std::isfinite(expected) || !std::isfinite(delta) || delta < 0.0 ||
        window.minimum[i] < expected - delta ||
        window.maximum[i] > expected + delta) {
      return false;
    }
  }
  return true;
}

inline bool runtime_tare_config_is_valid(const RuntimeTareConfig& config) {
  if (!is_finite_ft_sample(config.max_stddev) ||
      !is_finite_ft_sample(config.max_abs_post_tare_residual) ||
      config.min_recent_joint_samples == 0 ||
      !std::isfinite(config.max_recent_joint_excursion_rad) ||
      !std::isfinite(config.max_tare_joint_excursion_rad) ||
      config.max_recent_joint_excursion_rad < 0.0 ||
      config.max_tare_joint_excursion_rad < 0.0) {
    return false;
  }
  return std::all_of(config.max_stddev.begin(), config.max_stddev.end(),
                     [](double value) { return value >= 0.0; }) &&
         std::all_of(config.max_abs_post_tare_residual.begin(),
                     config.max_abs_post_tare_residual.end(),
                     [](double value) { return value >= 0.0; });
}

inline RuntimeTareRejection validate_runtime_tare_control_state(
    const RuntimeTareControlState& state) {
  if (!state.trajectory_status_fresh || !state.force_enable_status_fresh ||
      !state.compliance_enable_status_fresh || !state.compliance_status_fresh) {
    return RuntimeTareRejection::ControlStatusMissingOrStale;
  }
  if (state.trajectory_active) {
    return RuntimeTareRejection::TrajectoryActive;
  }
  if (state.force_enabled) {
    return RuntimeTareRejection::ForceEnabled;
  }
  if (state.compliance_enabled) {
    return RuntimeTareRejection::ComplianceEnabled;
  }
  if (state.compliance_active) {
    return RuntimeTareRejection::ComplianceActive;
  }
  return RuntimeTareRejection::None;
}

/// Validate the bias-collection window for the repeatable pre-contact tare.
/// The raw mean is checked only for finiteness, never against a fixed expected
/// value.  A stable high installation offset is therefore accepted, while a
/// stale source, moving arm, active controller path, or noisy signal is not.
inline RuntimeTareRejection validate_runtime_tare(
    const HardwareWrench& mean, const HardwareWrench& stddev,
    bool source_window_valid, bool safety_interlocks_clear,
    const RuntimeTareControlState& control_state,
    const JointSampleWindow& recent_joints, const JointSampleWindow& tare_joints,
    const RuntimeTareConfig& config) {
  if (!runtime_tare_config_is_valid(config)) {
    return RuntimeTareRejection::InvalidConfiguration;
  }
  if (!source_window_valid || !is_finite_ft_sample(mean) ||
      !is_finite_ft_sample(stddev) || !recent_joints.source_valid ||
      !tare_joints.source_valid || !joint_window_is_finite(recent_joints) ||
      !joint_window_is_finite(tare_joints)) {
    return RuntimeTareRejection::InvalidOrStaleSource;
  }
  if (!safety_interlocks_clear) {
    return RuntimeTareRejection::SafetyInterlockActive;
  }
  const auto control_rejection = validate_runtime_tare_control_state(control_state);
  if (control_rejection != RuntimeTareRejection::None) {
    return control_rejection;
  }
  if (recent_joints.sample_count < config.min_recent_joint_samples) {
    return RuntimeTareRejection::InsufficientRecentJointSamples;
  }
  if (!joint_window_within_excursion(recent_joints,
                                     config.max_recent_joint_excursion_rad)) {
    return RuntimeTareRejection::RecentJointMotion;
  }
  if (!joint_window_within_excursion(tare_joints,
                                     config.max_tare_joint_excursion_rad)) {
    return RuntimeTareRejection::TareWindowJointMotion;
  }
  for (size_t i = 0; i < kHardwareWrenchSize; ++i) {
    if (stddev[i] < 0.0 || stddev[i] > config.max_stddev[i]) {
      return RuntimeTareRejection::ExcessiveWrenchNoise;
    }
  }
  return RuntimeTareRejection::None;
}

inline RuntimeTareRejection validate_runtime_tare_post_residual(
    const HardwareWrench& corrected_mean, const HardwareWrench& stddev,
    const RuntimeTareConfig& config) {
  if (!runtime_tare_config_is_valid(config)) {
    return RuntimeTareRejection::InvalidConfiguration;
  }
  if (!is_finite_ft_sample(corrected_mean) || !is_finite_ft_sample(stddev)) {
    return RuntimeTareRejection::InvalidOrStaleSource;
  }
  for (size_t i = 0; i < kHardwareWrenchSize; ++i) {
    if (stddev[i] < 0.0 || stddev[i] > config.max_stddev[i]) {
      return RuntimeTareRejection::ExcessiveWrenchNoise;
    }
    if (std::abs(corrected_mean[i]) > config.max_abs_post_tare_residual[i]) {
      return RuntimeTareRejection::ExcessivePostTareResidual;
    }
  }
  return RuntimeTareRejection::None;
}

/// Pure safety decision used by the RT path and unit tests.
///
/// `source_window_valid` means every wrench sample in the tare window came
/// from fresh transport and was finite.  The caller must not adopt the mean as
/// bias unless this returns None.
inline SupervisedTareRejection validate_supervised_tare(
    const HardwareWrench& mean, const HardwareWrench& stddev, bool source_window_valid,
    bool safety_interlocks_clear,
    const JointSampleWindow& recent_joints, const JointSampleWindow& tare_joints,
    const SupervisedTareConfig& config) {
  if (!supervised_tare_config_is_valid(config)) {
    return SupervisedTareRejection::InvalidConfiguration;
  }
  if (!config.enabled) {
    return SupervisedTareRejection::Disabled;
  }
  if (!source_window_valid || !is_finite_ft_sample(mean) || !is_finite_ft_sample(stddev) ||
      !recent_joints.source_valid || !tare_joints.source_valid ||
      !joint_window_is_finite(recent_joints) || !joint_window_is_finite(tare_joints)) {
    return SupervisedTareRejection::InvalidOrStaleSource;
  }
  if (!safety_interlocks_clear) {
    return SupervisedTareRejection::SafetyInterlockActive;
  }
  if (recent_joints.sample_count < config.min_recent_joint_samples) {
    return SupervisedTareRejection::InsufficientRecentJointSamples;
  }
  if (!joint_window_within_excursion(recent_joints, config.max_recent_joint_excursion_rad)) {
    return SupervisedTareRejection::RecentJointMotion;
  }
  if (!joint_window_within_excursion(tare_joints, config.max_tare_joint_excursion_rad)) {
    return SupervisedTareRejection::TareWindowJointMotion;
  }
  if (!joint_window_within_commissioned_pose(
          recent_joints, config.expected_joint_positions,
          config.max_abs_joint_position_delta_rad) ||
      !joint_window_within_commissioned_pose(
          tare_joints, config.expected_joint_positions,
          config.max_abs_joint_position_delta_rad)) {
    return SupervisedTareRejection::OutsideCommissionedJointPose;
  }
  for (size_t i = 0; i < kHardwareWrenchSize; ++i) {
    if (stddev[i] < 0.0 || stddev[i] > config.max_stddev[i]) {
      return SupervisedTareRejection::ExcessiveWrenchNoise;
    }
  }
  for (size_t i = 0; i < kHardwareWrenchSize; ++i) {
    if (std::abs(mean[i] - config.expected_bias[i]) > config.max_abs_bias_delta[i]) {
      return SupervisedTareRejection::OutsideKnownBaselineEnvelope;
    }
  }
  return SupervisedTareRejection::None;
}

inline bool state_sample_is_fresh(bool transport_valid, double age_s, double max_age_s) {
  return transport_valid && std::isfinite(age_s) && age_s >= 0.0 && max_age_s >= 0.0 && age_s <= max_age_s;
}

inline bool is_finite_ft_sample(const HardwareWrench& wrench) {
  return std::all_of(wrench.begin(), wrench.end(), [](double value) { return std::isfinite(value); });
}

inline void invalidate_ft_states(HardwareWrench& filtered, HardwareWrench& raw) {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  filtered.fill(invalid);
  raw.fill(invalid);
}

/// Produce the two exported F/T states, or invalidate both atomically as a group.
///
/// NaN is deliberately used as an in-band validity signal: ros2_control state
/// interfaces have no companion validity bit, and zero is a physically valid
/// wrench that must never stand in for missing telemetry or an invalid tare.
inline bool update_ft_output_states(const HardwareWrench& sample, const HardwareWrench& bias,
                                    bool source_valid, bool tare_ready, bool human_collab,
                                    double deadband, double clamp, HardwareWrench& filtered,
                                    HardwareWrench& raw) {
  if (!source_valid || !tare_ready || !is_finite_ft_sample(sample) || !is_finite_ft_sample(bias)) {
    invalidate_ft_states(filtered, raw);
    return false;
  }

  HardwareWrench next_filtered{};
  HardwareWrench next_raw{};
  for (size_t i = 0; i < kHardwareWrenchSize; ++i) {
    const double value_raw = sample[i] - bias[i];
    if (!std::isfinite(value_raw)) {
      invalidate_ft_states(filtered, raw);
      return false;
    }
    next_raw[i] = value_raw;
    double value = value_raw;
    if (human_collab) {
      if (std::abs(value) < deadband) {
        value = 0.0;
      } else {
        value = std::clamp(value, -clamp, clamp);
      }
    }
    next_filtered[i] = value;
  }

  filtered = next_filtered;
  raw = next_raw;
  return true;
}

}  // namespace rbpodo_hardware
