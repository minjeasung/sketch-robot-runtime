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

#include "rbpodo_hardware/hardware_safety.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace rbpodo_hardware {
namespace {

bool all_nan(const HardwareWrench& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isnan(value); });
}

SupervisedTareConfig supervised_config() {
  SupervisedTareConfig config;
  config.enabled = true;
  config.expected_bias = {{65.77, 2.47, 276.00, -1.955, 0.997, -0.1155}};
  config.max_abs_bias_delta = {{1.0, 1.0, 1.0, 0.1, 0.1, 0.05}};
  config.max_stddev = {{0.5, 0.5, 0.5, 0.05, 0.05, 0.05}};
  config.expected_joint_positions = {{
      0.0003189465, -0.9519414306, 2.4623770714,
      -1.6286282539, 1.5665779114, -0.00009492455}};
  config.max_abs_joint_position_delta_rad = {{0.01, 0.01, 0.01, 0.01, 0.01, 0.01}};
  config.min_recent_joint_samples = 50;
  config.max_recent_joint_excursion_rad = 0.0005;
  config.max_tare_joint_excursion_rad = 0.001;
  return config;
}

JointSampleWindow stationary_joint_window(size_t sample_count) {
  JointSampleWindow window;
  window.minimum = supervised_config().expected_joint_positions;
  window.maximum = window.minimum;
  window.sample_count = sample_count;
  window.source_valid = true;
  return window;
}

RuntimeTareConfig runtime_config() {
  RuntimeTareConfig config;
  config.max_stddev = {{0.5, 0.5, 0.5, 0.05, 0.05, 0.05}};
  config.max_abs_post_tare_residual = {{0.5, 0.5, 0.5, 0.05, 0.05, 0.05}};
  config.min_recent_joint_samples = 50;
  config.max_recent_joint_excursion_rad = 0.0005;
  config.max_tare_joint_excursion_rad = 0.001;
  return config;
}

RuntimeTareControlState runtime_controls_off() {
  RuntimeTareControlState state;
  state.trajectory_status_fresh = true;
  state.trajectory_active = false;
  state.force_enable_status_fresh = true;
  state.force_enabled = false;
  state.compliance_enable_status_fresh = true;
  state.compliance_enabled = false;
  state.compliance_status_fresh = true;
  state.compliance_active = false;
  return state;
}

TEST(HardwareSafety, StateFreshnessRequiresTransportAndBoundedAge) {
  EXPECT_TRUE(state_sample_is_fresh(true, 0.05, 0.1));
  EXPECT_FALSE(state_sample_is_fresh(false, 0.05, 0.1));
  EXPECT_FALSE(state_sample_is_fresh(true, 0.11, 0.1));
  EXPECT_FALSE(state_sample_is_fresh(true, std::numeric_limits<double>::infinity(), 0.1));
}

TEST(HardwareSafety, ActivationTarePolicyPreservesGenericDefaultAndAllowsExplicitRuntimeWait) {
  EXPECT_EQ(activation_tare_action(true), ActivationTareAction::RequestStrict);
  EXPECT_EQ(activation_tare_action(false),
            ActivationTareAction::WaitForExplicitRequest);
}

TEST(HardwareSafety, InvalidSourceOrTareInvalidatesBothExports) {
  const HardwareWrench sample{{1.0, 2.0, 3.0, 0.1, 0.2, 0.3}};
  const HardwareWrench bias{};
  HardwareWrench filtered{};
  HardwareWrench raw{};

  EXPECT_FALSE(update_ft_output_states(sample, bias, false, true, false, 1.2, 30.0, filtered, raw));
  EXPECT_TRUE(all_nan(filtered));
  EXPECT_TRUE(all_nan(raw));

  EXPECT_FALSE(update_ft_output_states(sample, bias, true, false, false, 1.2, 30.0, filtered, raw));
  EXPECT_TRUE(all_nan(filtered));
  EXPECT_TRUE(all_nan(raw));
}

TEST(HardwareSafety, NonfiniteInputFailsClosedThenFiniteInputRecovers) {
  HardwareWrench sample{{1.0, 2.0, 3.0, 0.1, 0.2, 0.3}};
  const HardwareWrench bias{{0.5, 0.5, 0.5, 0.05, 0.05, 0.05}};
  HardwareWrench filtered{};
  HardwareWrench raw{};

  sample[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(update_ft_output_states(sample, bias, true, true, false, 1.2, 30.0, filtered, raw));
  EXPECT_TRUE(all_nan(filtered));

  sample[2] = 3.0;
  EXPECT_TRUE(update_ft_output_states(sample, bias, true, true, false, 1.2, 30.0, filtered, raw));
  EXPECT_DOUBLE_EQ(raw[0], 0.5);
  EXPECT_DOUBLE_EQ(raw[2], 2.5);
  EXPECT_DOUBLE_EQ(filtered[2], 2.5);
}

TEST(HardwareSafety, CollaborationDeadbandAndClampDoNotMaskRawState) {
  const HardwareWrench sample{{0.5, 40.0, -40.0, 0.5, 40.0, -40.0}};
  const HardwareWrench bias{};
  HardwareWrench filtered{};
  HardwareWrench raw{};

  ASSERT_TRUE(update_ft_output_states(sample, bias, true, true, true, 1.2, 30.0, filtered, raw));
  EXPECT_DOUBLE_EQ(filtered[0], 0.0);
  EXPECT_DOUBLE_EQ(filtered[1], 30.0);
  EXPECT_DOUBLE_EQ(filtered[2], -30.0);
  EXPECT_DOUBLE_EQ(raw[0], 0.5);
  EXPECT_DOUBLE_EQ(raw[1], 40.0);
}

TEST(HardwareSafety, RuntimeTareAcceptsStableArbitraryRawOffsetWithoutFixedBaseline) {
  const HardwareWrench observed_offset{{65.1, 2.8, 274.69, -1.9, 1.02, -0.12}};
  const HardwareWrench different_run_offset{{71.8, 1.4, 299.57, -1.87, 0.74, -0.15}};
  const HardwareWrench stddev{{0.20, 0.18, 0.22, 0.020, 0.015, 0.006}};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  EXPECT_EQ(validate_runtime_tare(observed_offset, stddev, true, true,
                                  runtime_controls_off(), recent, tare,
                                  runtime_config()),
            RuntimeTareRejection::None);
  EXPECT_EQ(validate_runtime_tare(different_run_offset, stddev, true, true,
                                  runtime_controls_off(), recent, tare,
                                  runtime_config()),
            RuntimeTareRejection::None);
}

TEST(HardwareSafety, RuntimeTareRequiresFreshInactiveMotionForceAndCompliance) {
  auto controls = runtime_controls_off();
  EXPECT_EQ(validate_runtime_tare_control_state(controls), RuntimeTareRejection::None);

  controls.trajectory_status_fresh = false;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::ControlStatusMissingOrStale);
  controls = runtime_controls_off();
  controls.trajectory_active = true;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::TrajectoryActive);
  controls = runtime_controls_off();
  controls.force_enabled = true;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::ForceEnabled);
  controls = runtime_controls_off();
  controls.compliance_enable_status_fresh = false;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::ControlStatusMissingOrStale);
  controls = runtime_controls_off();
  controls.compliance_enabled = true;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::ComplianceEnabled);
  controls = runtime_controls_off();
  controls.compliance_active = true;
  EXPECT_EQ(validate_runtime_tare_control_state(controls),
            RuntimeTareRejection::ComplianceActive);
}

TEST(HardwareSafety, RuntimeTareRejectsInvalidSourceSafetyMotionAndNoise) {
  const HardwareWrench mean{{65.1, 2.8, 274.69, -1.9, 1.02, -0.12}};
  HardwareWrench stddev{};
  const auto config = runtime_config();
  auto recent = stationary_joint_window(50);
  auto tare = stationary_joint_window(100);

  EXPECT_EQ(validate_runtime_tare(mean, stddev, false, true, runtime_controls_off(),
                                  recent, tare, config),
            RuntimeTareRejection::InvalidOrStaleSource);
  EXPECT_EQ(validate_runtime_tare(mean, stddev, true, false, runtime_controls_off(),
                                  recent, tare, config),
            RuntimeTareRejection::SafetyInterlockActive);

  recent.maximum[2] += 0.0006;
  EXPECT_EQ(validate_runtime_tare(mean, stddev, true, true, runtime_controls_off(),
                                  recent, tare, config),
            RuntimeTareRejection::RecentJointMotion);
  recent = stationary_joint_window(50);
  tare.maximum[4] += 0.0011;
  EXPECT_EQ(validate_runtime_tare(mean, stddev, true, true, runtime_controls_off(),
                                  recent, tare, config),
            RuntimeTareRejection::TareWindowJointMotion);
  tare = stationary_joint_window(100);
  stddev[0] = 0.51;
  EXPECT_EQ(validate_runtime_tare(mean, stddev, true, true, runtime_controls_off(),
                                  recent, tare, config),
            RuntimeTareRejection::ExcessiveWrenchNoise);
}

TEST(HardwareSafety, RuntimeTarePostWindowMustConfirmNearZeroFiniteResidual) {
  const auto config = runtime_config();
  HardwareWrench residual{{0.10, -0.15, 0.25, 0.01, -0.02, 0.01}};
  HardwareWrench stddev{{0.20, 0.18, 0.22, 0.020, 0.015, 0.006}};
  EXPECT_EQ(validate_runtime_tare_post_residual(residual, stddev, config),
            RuntimeTareRejection::None);

  residual[2] = 0.5001;
  EXPECT_EQ(validate_runtime_tare_post_residual(residual, stddev, config),
            RuntimeTareRejection::ExcessivePostTareResidual);
  residual[2] = 0.0;
  stddev[3] = 0.0501;
  EXPECT_EQ(validate_runtime_tare_post_residual(residual, stddev, config),
            RuntimeTareRejection::ExcessiveWrenchNoise);
  residual[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate_runtime_tare_post_residual(residual, stddev, config),
            RuntimeTareRejection::InvalidOrStaleSource);
}

TEST(HardwareSafety, SupervisedTareAcceptsOnlyCommissionedStableFreeSpaceProfile) {
  const HardwareWrench live_mean{{65.79, 2.49, 275.65, -1.957, 0.998, -0.116}};
  const HardwareWrench live_stddev{{0.25, 0.22, 0.19, 0.025, 0.017, 0.005}};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  EXPECT_EQ(validate_supervised_tare(live_mean, live_stddev, true, true, recent, tare,
                                     supervised_config()),
            SupervisedTareRejection::None);
}

TEST(HardwareSafety, SupervisedTareIsDisabledByDefaultProfileGate) {
  auto config = supervised_config();
  config.enabled = false;
  const HardwareWrench mean = config.expected_bias;
  const HardwareWrench stddev{};

  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true,
                                     stationary_joint_window(50), stationary_joint_window(100),
                                     config),
            SupervisedTareRejection::Disabled);
}

TEST(HardwareSafety, SupervisedTareRejectsStaleNonfiniteAndUnsafeSources) {
  const auto config = supervised_config();
  HardwareWrench mean = config.expected_bias;
  const HardwareWrench stddev{};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  EXPECT_EQ(validate_supervised_tare(mean, stddev, false, true, recent, tare, config),
            SupervisedTareRejection::InvalidOrStaleSource);
  mean[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::InvalidOrStaleSource);
  mean = config.expected_bias;
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, false, recent, tare, config),
            SupervisedTareRejection::SafetyInterlockActive);
}

TEST(HardwareSafety, SupervisedTareRequiresRecentAndInWindowJointStationarity) {
  const auto config = supervised_config();
  const HardwareWrench mean = config.expected_bias;
  const HardwareWrench stddev{};
  auto recent = stationary_joint_window(49);
  auto tare = stationary_joint_window(100);

  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::InsufficientRecentJointSamples);

  recent = stationary_joint_window(50);
  recent.maximum[1] += 0.0006;
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::RecentJointMotion);

  recent = stationary_joint_window(50);
  tare.maximum[4] += 0.0011;
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::TareWindowJointMotion);
}

TEST(HardwareSafety, SupervisedTareRequiresCommissionedJointPoseBeforeAndDuringTare) {
  const auto config = supervised_config();
  const HardwareWrench mean = config.expected_bias;
  const HardwareWrench stddev{};
  auto recent = stationary_joint_window(50);
  auto tare = stationary_joint_window(100);

  // The exact configured boundary is accepted.
  recent.minimum[0] = config.expected_joint_positions[0] - 0.01;
  recent.maximum[0] = recent.minimum[0];
  tare.minimum[5] = config.expected_joint_positions[5] + 0.01;
  tare.maximum[5] = tare.minimum[5];
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::None);

  recent = stationary_joint_window(50);
  tare = stationary_joint_window(100);
  recent.minimum[2] = config.expected_joint_positions[2] + 0.0101;
  recent.maximum[2] = recent.minimum[2];
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::OutsideCommissionedJointPose);

  recent = stationary_joint_window(50);
  tare.minimum[4] = config.expected_joint_positions[4] - 0.0101;
  tare.maximum[4] = tare.minimum[4];
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::OutsideCommissionedJointPose);
}

TEST(HardwareSafety, SupervisedTareRejectsInvalidCommissionedJointPoseConfiguration) {
  auto config = supervised_config();
  const HardwareWrench mean = config.expected_bias;
  const HardwareWrench stddev{};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  config.expected_joint_positions[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::InvalidConfiguration);

  config = supervised_config();
  config.max_abs_joint_position_delta_rad[3] = -0.001;
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::InvalidConfiguration);
}

TEST(HardwareSafety, SupervisedTareRejectsNoiseAndOtherStableHighBaselines) {
  const auto config = supervised_config();
  HardwareWrench mean = config.expected_bias;
  HardwareWrench stddev{};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  stddev[2] = 0.51;
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::ExcessiveWrenchNoise);

  stddev.fill(0.0);
  mean = {{71.7569258, 1.4077458, 299.566155, -1.868, 0.742, -0.150}};
  EXPECT_EQ(validate_supervised_tare(mean, stddev, true, true, recent, tare, config),
            SupervisedTareRejection::OutsideKnownBaselineEnvelope);
}

TEST(HardwareSafety, SupervisedTareCannotBakeNominalWallContactIntoBias) {
  const auto config = supervised_config();
  const HardwareWrench stddev{};
  const auto recent = stationary_joint_window(50);
  const auto tare = stationary_joint_window(100);

  // At the commissioned pose the wall normal maps almost entirely to the
  // sensor Z axis (about 0.993). Both reaction-force polarities for the 1.6 N
  // painting target must therefore fall outside the +/-1 N bias envelope,
  // including the worst cancellation direction at both observed free-space
  // anchors used to commission the midpoint profile.
  constexpr double nominal_wall_contact_sensor_z{1.6 * 0.993};
  const std::array<HardwareWrench, 2> free_space_anchors{{
      HardwareWrench{{65.75, 2.44, 276.36, -1.952, 0.996, -0.115}},
      HardwareWrench{{65.79, 2.49, 275.65, -1.957, 0.998, -0.116}},
  }};
  for (const auto& anchor : free_space_anchors) {
    for (const double sign : {-1.0, 1.0}) {
      HardwareWrench contacted = anchor;
      contacted[2] += sign * nominal_wall_contact_sensor_z;
      EXPECT_EQ(validate_supervised_tare(
                    contacted, stddev, true, true, recent, tare, config),
                SupervisedTareRejection::OutsideKnownBaselineEnvelope);
    }
  }
}

TEST(HardwareSafety, SupervisedTareMayOnlyFollowStrictStableMeanFailure) {
  EXPECT_TRUE(supervised_tare_may_follow(StrictTareOutcome::FailedMeanEnvelopeOnly));
  EXPECT_FALSE(supervised_tare_may_follow(StrictTareOutcome::NotCompleted));
  EXPECT_FALSE(supervised_tare_may_follow(StrictTareOutcome::Succeeded));
  EXPECT_FALSE(supervised_tare_may_follow(StrictTareOutcome::FailedInvalidSource));
  EXPECT_FALSE(supervised_tare_may_follow(StrictTareOutcome::FailedNoise));
}

TEST(HardwareSafety, SupervisedJointHistoryRejectsLongTelemetryGap) {
  constexpr int64_t previous_ns = 1000000000;
  constexpr int64_t max_gap_ns = 100000000;

  EXPECT_TRUE(supervised_joint_history_gap_is_contiguous(
      previous_ns, previous_ns + max_gap_ns, max_gap_ns));
  EXPECT_FALSE(supervised_joint_history_gap_is_contiguous(
      previous_ns, previous_ns + max_gap_ns + 1, max_gap_ns));
  EXPECT_FALSE(supervised_joint_history_gap_is_contiguous(0, previous_ns, max_gap_ns));
}

TEST(HardwareSafety, SupervisedJointHistoryRejectsAnyUnsafeInterval) {
  EXPECT_TRUE(supervised_joint_history_sample_is_eligible(true, true, true, false));
  EXPECT_FALSE(supervised_joint_history_sample_is_eligible(false, true, true, false));
  EXPECT_FALSE(supervised_joint_history_sample_is_eligible(true, false, true, false));
  EXPECT_FALSE(supervised_joint_history_sample_is_eligible(true, true, false, false));
  EXPECT_FALSE(supervised_joint_history_sample_is_eligible(true, true, true, true));
}

TEST(HardwareSafety, PhysicalSafetyBoardAcceptsNormalPoweredState) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  EXPECT_TRUE(physical_safety_board_interlocks_clear(arm_power_on, 0));
}

TEST(HardwareSafety, PhysicalSafetyBoardRejectsPowerOffDirectTeachAndSos) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  constexpr int32_t direct_teach_pressed = int32_t{1} << 7;
  constexpr int32_t sos_active = int32_t{1} << 12;

  EXPECT_FALSE(physical_safety_board_interlocks_clear(0, 0));
  EXPECT_FALSE(physical_safety_board_interlocks_clear(
      arm_power_on | direct_teach_pressed, 0));
  EXPECT_FALSE(physical_safety_board_interlocks_clear(arm_power_on | sos_active, 0));
}

TEST(HardwareSafety, PhysicalSafetyBoardRejectsEveryPressedSafetyButtonBit) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  for (uint32_t bit = 22; bit <= 25; ++bit) {
    const auto pressed = static_cast<int32_t>(uint32_t{1} << bit);
    EXPECT_FALSE(physical_safety_board_interlocks_clear(arm_power_on, pressed))
        << "information_chunk_3 bit " << bit << " must fail closed";
  }
}

TEST(HardwareSafety, RealRobotSafetyPredicateKeepsExistingSoftwareInterlocks) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  const auto check = [arm_power_on](bool freedrive, int init_state, int init_error,
                                    int32_t collision, int sos, int32_t self_collision,
                                    bool soft_estop, int ems) {
    return real_robot_safety_interlocks_clear(
        freedrive, init_state, init_error, collision, sos, self_collision,
        soft_estop, ems, arm_power_on, 0);
  };

  EXPECT_TRUE(check(false, 6, 0, false, 0, false, false, 0));
  EXPECT_FALSE(check(true, 6, 0, false, 0, false, false, 0));
  EXPECT_FALSE(check(false, 5, 0, false, 0, false, false, 0));
  EXPECT_FALSE(check(false, 6, 1, false, 0, false, false, 0));
  EXPECT_FALSE(check(false, 6, 0, true, 0, false, false, 0));
  EXPECT_FALSE(check(false, 6, 0, false, 1, false, false, 0));
  EXPECT_FALSE(check(false, 6, 0, false, 0, true, false, 0));
  EXPECT_FALSE(check(false, 6, 0, false, 0, false, true, 0));
  EXPECT_FALSE(check(false, 6, 0, false, 0, false, false, 1));
}

TEST(HardwareSafety, CollisionStatusIgnoresUpperHistoryAndTimezoneBits) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  constexpr int32_t upper_timezone_bits = static_cast<int32_t>(uint32_t{0x1234567} << 4);

  EXPECT_TRUE(current_collision_status_clear(upper_timezone_bits));
  EXPECT_TRUE(real_robot_safety_interlocks_clear(
      false, 6, 0, upper_timezone_bits, 0, upper_timezone_bits,
      false, 0, arm_power_on, 0));
}

TEST(HardwareSafety, RealRobotSafetyIgnoresUnrelatedUpperPackedBits) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  constexpr int32_t init_state_with_upper = 6 | (int32_t{1} << 6);
  constexpr int32_t init_error_upper = int32_t{1} << 12;
  constexpr int32_t collision_history_upper = int32_t{1} << 2;
  constexpr int32_t sos_upper = int32_t{1} << 6;
  constexpr int32_t timezone_upper = int32_t{1} << 4;
  constexpr int32_t ems_upper = int32_t{1} << 6;

  EXPECT_TRUE(real_robot_safety_interlocks_clear(
      false, init_state_with_upper, init_error_upper, collision_history_upper,
      sos_upper, timezone_upper, false, ems_upper, arm_power_on, 0));
}

TEST(HardwareSafety, RealRobotSafetyRejectsEveryValidLowerStatusField) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;

  for (int32_t init_state = 0; init_state < 64; ++init_state) {
    if (init_state != 6) {
      EXPECT_FALSE(real_robot_safety_interlocks_clear(
          false, init_state, 0, 0, 0, 0, false, 0, arm_power_on, 0));
    }
  }
  for (uint32_t bit = 0; bit < 12; ++bit) {
    EXPECT_FALSE(real_robot_safety_interlocks_clear(
        false, 6, static_cast<int32_t>(uint32_t{1} << bit), 0, 0, 0,
        false, 0, arm_power_on, 0));
  }
  for (uint32_t bit = 0; bit < 6; ++bit) {
    const auto active = static_cast<int32_t>(uint32_t{1} << bit);
    EXPECT_FALSE(real_robot_safety_interlocks_clear(
        false, 6, 0, 0, active, 0, false, 0, arm_power_on, 0));
    EXPECT_FALSE(real_robot_safety_interlocks_clear(
        false, 6, 0, 0, 0, 0, false, active, arm_power_on, 0));
  }
}

TEST(HardwareSafety, CollisionStatusRejectsEitherCurrentCollisionBit) {
  constexpr int32_t arm_power_on = int32_t{1} << 6;
  for (int32_t current_collision_bit : {int32_t{1}, int32_t{2}}) {
    EXPECT_FALSE(current_collision_status_clear(current_collision_bit));
    EXPECT_FALSE(real_robot_safety_interlocks_clear(
        false, 6, 0, current_collision_bit, 0, 0,
        false, 0, arm_power_on, 0));
    EXPECT_FALSE(real_robot_safety_interlocks_clear(
        false, 6, 0, 0, 0, current_collision_bit,
        false, 0, arm_power_on, 0));
  }
}

TEST(HardwareSafety, MotionAbortLatchIgnoresRecoverableStartupAndFreedriveBits) {
  // The irreversible predicate intentionally has no init-state, arm-power,
  // or direct-teach inputs.  With all collision/stop inputs clear it must not
  // latch merely because those recoverable states are handled elsewhere.
  EXPECT_FALSE(real_robot_motion_abort_interlock_triggered(
      0, 0, 0, false, 0, 0, 0));
}

TEST(HardwareSafety, RobotSafetyLossIsRecoverableOnlyBeforeFirstHealthySample) {
  EXPECT_FALSE(robot_safety_loss_requires_motion_inhibit(false, false, false));
  EXPECT_FALSE(robot_safety_loss_requires_motion_inhibit(true, false, false));
  EXPECT_FALSE(robot_safety_loss_requires_motion_inhibit(true, true, false));
  EXPECT_TRUE(robot_safety_loss_requires_motion_inhibit(false, true, false));
}

TEST(HardwareSafety, IntentionalInternalFreedriveTransitionDoesNotLatch) {
  EXPECT_FALSE(robot_safety_loss_requires_motion_inhibit(false, true, true));
  // A merely pending ON request has not stopped the robot or entered
  // free-drive yet, so callers must pass false and latch the safety loss.
  EXPECT_TRUE(robot_safety_loss_requires_motion_inhibit(false, true, false));
}

TEST(HardwareSafety, MotionAbortLatchRejectsCollisionAndEveryEmergencyStopSource) {
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      1, 0, 0, false, 0, 0, 0));
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      0, 1, 0, false, 0, 0, 0));
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      0, 0, 1, false, 0, 0, 0));
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      0, 0, 0, true, 0, 0, 0));
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      0, 0, 0, false, 1, 0, 0));
  EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
      0, 0, 0, false, 0, int32_t{1} << 12, 0));
  for (int bit = 22; bit <= 25; ++bit) {
    EXPECT_TRUE(real_robot_motion_abort_interlock_triggered(
        0, 0, 0, false, 0, 0, int32_t{1} << bit));
  }
}

TEST(HardwareSafety, MotionAbortReasonMaskIdentifiesEveryIndependentSource) {
  EXPECT_EQ(real_robot_motion_abort_reason_mask(1, 0, 0, false, 0, 0, 0),
            uint32_t{1} << 0);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(0, 0, 1, false, 0, 0, 0),
            uint32_t{1} << 1);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(0, 5, 0, false, 0, 0, 0),
            uint32_t{1} << 2);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(0, 0, 0, true, 0, 0, 0),
            uint32_t{1} << 3);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(0, 0, 0, false, 2, 0, 0),
            uint32_t{1} << 4);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(
                0, 0, 0, false, 0, int32_t{1} << 12, 0),
            uint32_t{1} << 5);
  for (int bit = 22; bit <= 25; ++bit) {
    EXPECT_EQ(real_robot_motion_abort_reason_mask(
                  0, 0, 0, false, 0, 0, int32_t{1} << bit),
              uint32_t{1} << (6 + bit - 22));
  }
}

TEST(HardwareSafety, MotionAbortReasonMaskPreservesSimultaneousCauses) {
  const uint32_t expected =
      (uint32_t{1} << 0) | (uint32_t{1} << 2) |
      (uint32_t{1} << 4) | (uint32_t{1} << 8);
  EXPECT_EQ(real_robot_motion_abort_reason_mask(
                2, 12, 0, false, 3, 0, int32_t{1} << 24),
            expected);
}

}  // namespace
}  // namespace rbpodo_hardware
