// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/// \author: Denis Stogl

#include "test_admittance_controller.hpp"

#include <memory>
#include <limits>
#include <vector>

TEST(AdmittanceNormalForceTest, absolute_mode_makes_both_measured_signs_equivalent)
{
  EXPECT_DOUBLE_EQ(
    admittance_controller::measured_normal_force_for_control(3.0, true), 3.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::measured_normal_force_for_control(-3.0, true), 3.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::measured_normal_force_for_control(-3.0, false), -3.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::measured_normal_force_for_control(3.0, true) - 3.0, 0.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::measured_normal_force_for_control(-3.0, true) - 3.0, 0.0);
}

TEST(AdmittanceNormalForceTest, hold_band_uses_inner_entry_and_outer_exit_boundaries)
{
  constexpr double lower = 8.0;
  constexpr double upper = 18.0;
  constexpr double hysteresis = 1.0;
  constexpr double commanded_press = 10.0;

  EXPECT_TRUE(admittance_controller::normal_force_hold_band_is_valid(
      lower, upper, hysteresis));
  EXPECT_FALSE(admittance_controller::next_normal_force_hold_state(
      false, 8.5, commanded_press, lower, upper, hysteresis));
  EXPECT_TRUE(admittance_controller::next_normal_force_hold_state(
      false, 9.0, commanded_press, lower, upper, hysteresis));
  EXPECT_TRUE(admittance_controller::next_normal_force_hold_state(
      true, 8.0, commanded_press, lower, upper, hysteresis));
  EXPECT_TRUE(admittance_controller::next_normal_force_hold_state(
      true, 18.0, commanded_press, lower, upper, hysteresis));
  EXPECT_FALSE(admittance_controller::next_normal_force_hold_state(
      true, 7.99, commanded_press, lower, upper, hysteresis));
  EXPECT_FALSE(admittance_controller::next_normal_force_hold_state(
      true, 18.01, commanded_press, lower, upper, hysteresis));
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      13.0, commanded_press, true, lower, upper, hysteresis),
    commanded_press);
  // Outside the band, target only the nearest inner boundary (9 or 17 N),
  // never a hidden 10 N midpoint/setpoint.
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      7.0, commanded_press, false, lower, upper, hysteresis),
    commanded_press + 7.0 - 9.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      19.0, commanded_press, false, lower, upper, hysteresis),
    commanded_press + 19.0 - 17.0);
  EXPECT_DOUBLE_EQ(admittance_controller::normal_stiffness_for_hold(100.0, true), 0.0);
  EXPECT_DOUBLE_EQ(admittance_controller::normal_stiffness_for_hold(100.0, false), 100.0);
}

TEST(AdmittanceNormalForceTest, hold_band_disables_without_press_or_with_invalid_configuration)
{
  EXPECT_FALSE(admittance_controller::next_normal_force_hold_state(
      true, 10.0, 0.0, 8.0, 18.0, 1.0));
  EXPECT_FALSE(admittance_controller::next_normal_force_hold_state(
      true, 10.0, 7.99, 8.0, 18.0, 1.0));
  EXPECT_FALSE(admittance_controller::normal_force_hold_band_is_valid(8.0, 8.0, 0.0));
  EXPECT_FALSE(admittance_controller::normal_force_hold_band_is_valid(8.0, 18.0, 5.0));
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      7.0, 7.99, false, 8.0, 18.0, 1.0),
    7.0);
}

TEST(AdmittanceNormalForceTest, thirty_to_forty_band_has_no_hidden_midpoint_target)
{
  constexpr double lower = 30.0;
  constexpr double upper = 40.0;
  constexpr double hysteresis = 1.0;
  constexpr double activation_command = 30.0;

  // Add the still-signed -30 N command to each returned component. Below the
  // band the net drive is toward 31 N, inside it is zero, and above it is
  // toward 39 N. No branch drives toward 35 N.
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      20.0, activation_command, false, lower, upper, hysteresis) - activation_command,
    -11.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      35.0, activation_command, true, lower, upper, hysteresis) - activation_command,
    0.0);
  EXPECT_DOUBLE_EQ(
    admittance_controller::normal_force_component_for_band(
      45.0, activation_command, false, lower, upper, hysteresis) - activation_command,
    6.0);
}

TEST(AdmittanceRollerBalanceTest, cop_band_uses_wide_entry_and_narrow_exit)
{
  constexpr double enter = 0.008;
  constexpr double exit = 0.005;
  constexpr double roller_length = 0.175;

  EXPECT_TRUE(admittance_controller::roller_balance_band_is_valid(
      enter, exit, roller_length));
  EXPECT_FALSE(admittance_controller::roller_balance_band_is_valid(
      exit, enter, roller_length));
  EXPECT_FALSE(admittance_controller::roller_balance_band_is_valid(
      0.09, exit, roller_length));

  EXPECT_FALSE(admittance_controller::next_roller_balance_correction_state(
      false, true, 0.0079, enter, exit, roller_length));
  EXPECT_TRUE(admittance_controller::next_roller_balance_correction_state(
      false, true, 0.008, enter, exit, roller_length));
  EXPECT_TRUE(admittance_controller::next_roller_balance_correction_state(
      true, true, 0.0051, enter, exit, roller_length));
  EXPECT_FALSE(admittance_controller::next_roller_balance_correction_state(
      true, true, 0.005, enter, exit, roller_length));
  EXPECT_FALSE(admittance_controller::next_roller_balance_correction_state(
      true, false, 0.02, enter, exit, roller_length));
}

TEST(AdmittanceRollerBalanceTest, torque_band_scales_with_normal_force)
{
  constexpr double exit = 0.005;

  // At 20 N, a 10 mm CoP error drives only the 5 mm outside the inner
  // boundary: 20 * (0.010 - 0.005) = 0.10 Nm.  At 30 N the same geometric
  // error becomes 0.15 Nm, so the torque threshold is a range, not a point.
  EXPECT_DOUBLE_EQ(admittance_controller::roller_balance_boundary_torque(
      0.010, 20.0, true, exit, 1.0), 0.10);
  EXPECT_DOUBLE_EQ(admittance_controller::roller_balance_boundary_torque(
      -0.010, 30.0, true, exit, 1.0), -0.15);
  EXPECT_DOUBLE_EQ(admittance_controller::roller_balance_boundary_torque(
      0.010, 20.0, false, exit, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(admittance_controller::roller_balance_boundary_torque(
      0.004, 20.0, true, exit, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(admittance_controller::roller_balance_boundary_torque(
      0.010, 20.0, true, exit, -1.0), -0.10);
}

TEST(AdmittanceRollerBalanceTest, sensor_lever_arm_is_removed_at_contact_center)
{
  const Eigen::Vector3d force(4.0, 20.0, -3.0);
  const Eigen::Vector3d sensor_to_contact(0.01, -0.38, 0.02);
  const Eigen::Vector3d true_contact_torque(0.2, -0.1, 0.35);
  const Eigen::Vector3d sensor_torque =
    true_contact_torque + sensor_to_contact.cross(force);

  const Eigen::Vector3d recovered =
    admittance_controller::shift_torque_to_contact_center(
    sensor_torque, force, sensor_to_contact);
  EXPECT_TRUE(recovered.isApprox(true_contact_torque, 1e-12));
}

// Test on_init returns ERROR when a required parameter is missing
TEST_P(AdmittanceControllerTestParameterizedMissingParameters, one_init_parameter_is_missing)
{
  ASSERT_EQ(SetUpController(GetParam()), controller_interface::return_type::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
  MissingMandatoryParameterDuringInit, AdmittanceControllerTestParameterizedMissingParameters,
  ::testing::Values(
    "admittance.mass", "admittance.selected_axes", "admittance.stiffness", "command_interfaces",
    "control.frame.id", "fixed_world_frame.frame.id", "ft_sensor.frame.id", "ft_sensor.name",
    "gravity_compensation.CoG.pos", "gravity_compensation.frame.id", "joints", "kinematics.base",
    "kinematics.plugin_name", "kinematics.plugin_package", "kinematics.tip", "state_interfaces"));

INSTANTIATE_TEST_SUITE_P(
  InvalidParameterDuringConfiguration, AdmittanceControllerTestParameterizedInvalidParameters,
  ::testing::Values(
    // wrong length COG
    std::make_tuple(
      std::string("gravity_compensation.CoG.pos"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3, 4})),
    // wrong length stiffness
    std::make_tuple(
      std::string("admittance.stiffness"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // negative stiffness
    std::make_tuple(
      std::string("admittance.stiffness"),
      rclcpp::ParameterValue(std::vector<double>() = {-1, -2, 3, 4, 5, 6})),
    // wrong length mass
    std::make_tuple(
      std::string("admittance.mass"), rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // negative mass
    std::make_tuple(
      std::string("admittance.mass"),
      rclcpp::ParameterValue(std::vector<double>() = {-1, -2, 3, 4, 5, 6})),
    // wrong length damping ratio
    std::make_tuple(
      std::string("admittance.damping_ratio"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // wrong length selected axes
    std::make_tuple(
      std::string("admittance.selected_axes"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // invalid robot description.
    std::make_tuple(
      std::string("robot_description"), rclcpp::ParameterValue(std::string() = "bad_robot"))));

// Test on_init returns ERROR when a parameter is invalid
TEST_P(AdmittanceControllerTestParameterizedInvalidParameters, invalid_parameters)
{
  ASSERT_EQ(SetUpController(), controller_interface::return_type::ERROR);
}

TEST_F(AdmittanceControllerTest, all_parameters_set_configure_success)
{
  auto result = SetUpController();

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(!controller_->admittance_->parameters_.joints.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.joints.size() == joint_names_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.joints.begin(),
      controller_->admittance_->parameters_.joints.end(), joint_names_.begin(),
      joint_names_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.command_interfaces.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.command_interfaces.size() ==
    command_interface_types_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.command_interfaces.begin(),
      controller_->admittance_->parameters_.command_interfaces.end(),
      command_interface_types_.begin(), command_interface_types_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.state_interfaces.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.state_interfaces.size() == state_interface_types_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.state_interfaces.begin(),
      controller_->admittance_->parameters_.state_interfaces.end(), state_interface_types_.begin(),
      state_interface_types_.end()));

  ASSERT_EQ(controller_->admittance_->parameters_.ft_sensor.name, ft_sensor_name_);
  ASSERT_EQ(controller_->admittance_->parameters_.kinematics.base, ik_base_frame_);
  ASSERT_EQ(controller_->admittance_->parameters_.ft_sensor.frame.id, sensor_frame_);

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.selected_axes.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.admittance.selected_axes.size() ==
    admittance_selected_axes_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.admittance.selected_axes.begin(),
      controller_->admittance_->parameters_.admittance.selected_axes.end(),
      admittance_selected_axes_.begin(), admittance_selected_axes_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.mass.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.admittance.mass.size() == admittance_mass_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.admittance.mass.begin(),
      controller_->admittance_->parameters_.admittance.mass.end(), admittance_mass_.begin(),
      admittance_mass_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.damping_ratio.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.admittance.damping_ratio.size() ==
    admittance_damping_ratio_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.admittance.damping_ratio.begin(),
      controller_->admittance_->parameters_.admittance.damping_ratio.end(),
      admittance_damping_ratio_.begin(), admittance_damping_ratio_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.stiffness.empty());
  ASSERT_TRUE(
    controller_->admittance_->parameters_.admittance.stiffness.size() ==
    admittance_stiffness_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->admittance_->parameters_.admittance.stiffness.begin(),
      controller_->admittance_->parameters_.admittance.stiffness.end(),
      admittance_stiffness_.begin(), admittance_stiffness_.end()));
}

TEST_F(AdmittanceControllerTest, check_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), joint_command_values_.size());
  EXPECT_EQ(
    command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  ASSERT_EQ(
    controller_->command_interfaces_.size(), command_interface_types_.size() * joint_names_.size());

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), joint_state_values_.size() + fts_state_values_.size());
  EXPECT_EQ(state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  ASSERT_EQ(
    controller_->state_interfaces_.size(),
    state_interface_types_.size() * joint_names_.size() + fts_state_values_.size());

  const auto reference_interfaces = controller_->ordered_exported_reference_interfaces_;
  ASSERT_EQ(reference_interfaces.size(), 2 * joint_names_.size());
  for (auto i = 0ul; i < joint_names_.size(); i++) {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + joint_names_[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(reference_interfaces[i]->get_interface_name(), hardware_interface::HW_IF_POSITION);
    EXPECT_EQ(
      reference_interfaces[i + joint_names_.size()]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i + joint_names_.size()]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(
      reference_interfaces[i + joint_names_.size()]->get_interface_name(),
      hardware_interface::HW_IF_VELOCITY);
  }
}

TEST_F(AdmittanceControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    controller_->command_interfaces_.size(), command_interface_types_.size() * joint_names_.size());
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AdmittanceControllerTest, missing_pos_state_interface)
{
  auto overrides = {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"velocity"})};
  SetUpController("test_admittance_controller", overrides);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_FAILURE);
}

TEST_F(AdmittanceControllerTest, only_vel_command_interface)
{
  command_interface_types_ = {"velocity"};
  auto overrides = {rclcpp::Parameter("command_interfaces", std::vector<std::string>{"velocity"})};
  SetUpController("test_admittance_controller", overrides);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(AdmittanceControllerTest, only_pos_reference_interface)
{
  auto overrides = {
    rclcpp::Parameter("chainable_command_interfaces", std::vector<std::string>{"position"})};
  SetUpController("test_admittance_controller", overrides);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AdmittanceControllerTest, only_vel_reference_interface)
{
  auto overrides = {
    rclcpp::Parameter("chainable_command_interfaces", std::vector<std::string>{"velocity"})};
  SetUpController("test_admittance_controller", overrides);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AdmittanceControllerTest, invalid_reference_interface)
{
  auto overrides = {rclcpp::Parameter(
    "chainable_command_interfaces", std::vector<std::string>{"invalid_interface"})};
  SetUpController("test_admittance_controller", overrides);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(AdmittanceControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// A force-controlled axis needs stiffness near zero so the spring term stops
// competing with the commanded contact force.  The damping_ratio form derives
// D = zeta * 2 * sqrt(M * S) and therefore collapses to D = 0 exactly there,
// leaving an undamped free mass on real hardware.  damping_absolute must win
// on the axes that configure it, and must not disturb the others.
TEST_F(AdmittanceControllerTest, damping_absolute_overrides_ratio_per_axis)
{
  SetUpController();

  const std::vector<double> absolute = {0.0, 120.0, 0.0, 0.0, 0.0, 0.0};
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("admittance.damping_absolute", absolute));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  const auto & state = controller_->admittance_->get_controller_state();
  ASSERT_EQ(state.damping.data.size(), 6u);
  EXPECT_DOUBLE_EQ(state.damping.data[1], 120.0);
  for (size_t i = 0; i < 6; ++i) {
    if (i == 1) {
      continue;
    }
    const double expected = admittance_damping_ratio_[i] * 2.0 *
      std::sqrt(admittance_mass_[i] * admittance_stiffness_[i]);
    EXPECT_NEAR(state.damping.data[i], expected, 1e-9) << "axis " << i;
  }
}

// Default must stay on the ratio path so existing profiles are unaffected.
TEST_F(AdmittanceControllerTest, damping_defaults_to_the_ratio_derivation)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  const auto & state = controller_->admittance_->get_controller_state();
  ASSERT_EQ(state.damping.data.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    const double expected = admittance_damping_ratio_[i] * 2.0 *
      std::sqrt(admittance_mass_[i] * admittance_stiffness_[i]);
    EXPECT_NEAR(state.damping.data[i], expected, 1e-9) << "axis " << i;
  }
}

TEST_F(AdmittanceControllerTest, nonfinite_ft_state_is_preserved_and_disables_compliance)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();

  fts_state_values_[0] = std::numeric_limits<double>::quiet_NaN();
  const auto sensed = controller_->read_wrench_from_hardware_for_test();
  ASSERT_TRUE(std::isnan(sensed.force.x));

  controller_->seed_fresh_force_inputs_for_test();
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_FALSE(controller_->compliance_active_for_test());
  for (const double command : joint_command_values_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(AdmittanceControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AdmittanceControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assign_interfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(AdmittanceControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  //   // Check that wrench command are all zero since not used
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);
  //   ASSERT_EQ(msg.wrench_base.wrench.force.x, 0.0);
  //   ASSERT_EQ(msg.wrench_base.wrench.force.y, 0.0);
  //   ASSERT_TRUE(msg.wrench_base.wrench.force.z > 0.15);
  //   ASSERT_TRUE(msg.wrench_base.wrench.torque.x != 0.0);
  //   ASSERT_TRUE(msg.wrench_base.wrench.torque.y != 0.0);
  //   ASSERT_EQ(msg.wrench_base.wrench.torque.z, 0.0);

  //   // Check joint command message
  //   for (auto i = 0ul; i < joint_names_.size(); i++)
  //   {
  //     ASSERT_EQ(joint_names_[i], msg.joint_state.name[i]);
  //     ASSERT_FALSE(std::isnan(msg.joint_state.position[i]));
  //     ASSERT_FALSE(std::isnan(msg.joint_state.velocity[i]));
  //     ASSERT_FALSE(std::isnan(msg.joint_state.effort[i]));
  //   }
}

TEST_F(AdmittanceControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // After first update state, commanded position should be near the start state
  for (auto i = 0ul; i < joint_state_values_.size(); i++) {
    ASSERT_NEAR(joint_state_values_[i], joint_command_values_[i], COMMON_THRESHOLD);
  }

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);

  publish_commands();
  controller_->wait_for_commands(executor);

  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NEAR(joint_command_values_[0], joint_state_values_[0], COMMON_THRESHOLD);

  subscribe_and_get_messages(msg);
}

TEST_F(AdmittanceControllerTest, check_frame_ids_in_controller_state)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  broadcast_tfs();  // force torque sensor

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  const auto & msg = controller_->admittance_->get_controller_state();

  // Ensure correct frame IDs
  ASSERT_EQ(
    msg.ref_trans_base_ft.header.frame_id, controller_->admittance_->parameters_.kinematics.base);
  ASSERT_EQ(
    msg.ref_trans_base_ft.child_frame_id, controller_->admittance_->parameters_.ft_sensor.frame.id);
  ASSERT_EQ(
    msg.admittance_position.header.frame_id, controller_->admittance_->parameters_.kinematics.base);
  ASSERT_EQ(msg.admittance_position.child_frame_id, "admittance_offset");
}

TEST_F(AdmittanceControllerTest, failed_rule_does_not_write_nominal_position_to_hardware)
{
  SetUpController();
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  const auto previous_command = joint_command_values_;
  EXPECT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0)),
    controller_interface::return_type::ERROR);
  EXPECT_EQ(joint_command_values_, previous_command);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// Add test, wrong interfaces
