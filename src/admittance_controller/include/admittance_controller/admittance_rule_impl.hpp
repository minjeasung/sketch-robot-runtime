// Copyright (c) 2022, PickNik, Inc.
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
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_

#include "admittance_controller/admittance_rule.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <control_toolbox/filters.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "rclcpp/duration.hpp"

namespace admittance_controller
{

constexpr auto NUM_CARTESIAN_DOF = 6;  // (3 translation + 3 rotation)

/// Configure admittance rule memory for num joints and load kinematics interface
controller_interface::return_type AdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const size_t num_joints,
  const std::string & robot_description)
{
  num_joints_ = num_joints;

  // initialize memory and values to zero  (non-realtime function)
  reset(num_joints);

  // Load the differential IK plugin
  if (!parameters_.kinematics.plugin_name.empty()) {
    try {
      // Make sure we destroy the interface first. Otherwise we might run into a segfault
      if (kinematics_loader_) {
        kinematics_.reset();
      }
      kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          parameters_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));

      if (!kinematics_->initialize(
            robot_description, node->get_node_parameters_interface(), "kinematics"))
      {
        return controller_interface::return_type::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
        parameters_.kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("AdmittanceRule"),
      "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  // configure force torque sensor frame in state message
  state_message_.ft_sensor_frame.data = parameters_.ft_sensor.frame.id;

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset(const size_t num_joints)
{
  // reset state message fields
  state_message_.joint_state.name.assign(num_joints, "");
  state_message_.joint_state.position.assign(num_joints, 0);
  state_message_.joint_state.velocity.assign(num_joints, 0);
  state_message_.joint_state.effort.assign(num_joints, 0);
  for (size_t i = 0; i < parameters_.joints.size(); ++i) {
    state_message_.joint_state.name = parameters_.joints;
  }
  state_message_.mass.data.resize(NUM_CARTESIAN_DOF, 0.0);
  state_message_.selected_axes.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.damping.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.stiffness.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.wrench_base.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_velocity.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_acceleration.header.frame_id = parameters_.kinematics.base;

  // reset admittance state
  admittance_state_ = AdmittanceState(num_joints);

  // reset transforms and rotations
  admittance_transforms_ = AdmittanceTransforms();

  // reset forces
  wrench_world_.setZero();
  end_effector_weight_.setZero();
  normal_limit_reached_ = false;
  normal_rate_saturated_ = false;
  normal_trim_m_ = 0.0;
  normal_force_hold_active_ = false;
  roller_balance_contact_valid_ = false;
  roller_balance_correction_active_ = false;
  roller_balance_limit_reached_ = false;
  roller_balance_rate_saturated_ = false;
  roller_balance_freeze_trim_ = false;
  roller_balance_filter_initialized_ = false;
  roller_balance_filtered_torque_nm_ = 0.0;
  roller_balance_contact_torque_nm_ = 0.0;
  roller_balance_cop_offset_m_ = 0.0;
  roller_balance_rotation_trim_rad_ = 0.0;
  roller_balance_commanded_trim_rad_ = 0.0;
  roller_balance_command_velocity_radps_ = 0.0;
  roller_balance_soft_limit_active_ = false;

  // load/initialize Eigen types from parameters
  apply_parameters_update();

  return controller_interface::return_type::OK;
}

void AdmittanceRule::apply_parameters_update()
{
  if (parameter_handler_->is_old(parameters_)) {
    parameters_ = parameter_handler_->get_params();
  }
  // update param values
  end_effector_weight_[2] = -parameters_.gravity_compensation.CoG.force;
  vec_to_eigen(parameters_.gravity_compensation.CoG.pos, cog_pos_);
  vec_to_eigen(parameters_.admittance.mass, admittance_state_.mass);
  vec_to_eigen(parameters_.admittance.stiffness, admittance_state_.stiffness);
  vec_to_eigen(parameters_.admittance.selected_axes, admittance_state_.selected_axes);
  if (parameters_.roller_balance.control_enabled) {
    const auto balance_axis = static_cast<Eigen::Index>(
      3 + parameters_.roller_balance.rotation_axis_index);
    admittance_state_.selected_axes[balance_axis] = 1.0;
  }

  for (size_t i = 0; i < NUM_CARTESIAN_DOF; ++i) {
    auto idx = static_cast<Eigen::Index>(i);
    admittance_state_.mass_inv[idx] = 1.0 / parameters_.admittance.mass[i];
    // A force-controlled axis needs stiffness ~ 0 so the spring term stops
    // competing with the commanded contact force.  The damping_ratio form
    // derives D from sqrt(M * S) and therefore yields D = 0 exactly there,
    // which turns the axis into an undamped free mass.  An explicit absolute
    // damping value takes precedence whenever it is configured.
    const double absolute_damping = parameters_.admittance.damping_absolute[i];
    admittance_state_.damping[idx] =
      absolute_damping > 0.0 ?
      absolute_damping :
      parameters_.admittance.damping_ratio[i] * 2 *
      sqrt(admittance_state_.mass[idx] * admittance_state_.stiffness[idx]);
  }
}

bool AdmittanceRule::get_all_transforms(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state)
{
  // get reference transforms
  bool success = kinematics_->calculate_link_transform(
    reference_joint_state.positions, parameters_.ft_sensor.frame.id,
    admittance_transforms_.ref_base_ft_);

  // get transforms at current configuration
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.ft_sensor.frame.id, admittance_transforms_.base_ft_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.kinematics.tip, admittance_transforms_.base_tip_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.fixed_world_frame.frame.id,
    admittance_transforms_.world_base_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.gravity_compensation.frame.id,
    admittance_transforms_.base_cog_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.control.frame.id,
    admittance_transforms_.base_control_);

  return success;
}

// Update from reference joint states
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const geometry_msgs::msg::Wrench & commanded_wrench,
  const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state,
  const rclcpp::Duration & period, trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return controller_interface::return_type::ERROR;
  }

  if (parameters_.enable_parameter_update_without_reactivation) {
    apply_parameters_update();
  }

  bool success = get_all_transforms(current_joint_state, reference_joint_state);
  if (!success) {
    return controller_interface::return_type::ERROR;
  }

  // apply filter and update wrench_world_ vector
  Eigen::Matrix<double, 3, 3> rot_world_sensor =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_ft_.rotation();
  Eigen::Matrix<double, 3, 3> rot_world_cog =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_cog_.rotation();
  process_wrench_measurements(measured_wrench, rot_world_sensor, rot_world_cog);

  // transform wrench_world_ into base frame
  admittance_state_.wrench_base.block<3, 1>(0, 0) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.block<3, 1>(0, 0);
  admittance_state_.wrench_base.block<3, 1>(3, 0) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.block<3, 1>(3, 0);

  // A unilateral wall reaction is a magnitude in the commissioned painting
  // profile.  Apply abs() to the MEASURED control-frame normal component only,
  // before adding the signed command.  In particular, a measured -3 N and a
  // commanded -3 N must produce zero drive, not -6 N of inward drive.
  Eigen::Matrix<double, 6, 1> measured_control;
  measured_control.block<3, 1>(0, 0) =
    admittance_transforms_.base_control_.rotation().transpose() *
    admittance_state_.wrench_base.block<3, 1>(0, 0);
  measured_control.block<3, 1>(3, 0) =
    admittance_transforms_.base_control_.rotation().transpose() *
    admittance_state_.wrench_base.block<3, 1>(3, 0);
  const auto normal_axis = static_cast<Eigen::Index>(parameters_.normal_axis_index);

  // Shift the already gravity-compensated wrench from the F/T sensor origin
  // to the roller/wall contact centre.  A moment about the remote sensor
  // contains the lever-arm term r x F and is not a line-load imbalance.
  // M_contact = M_sensor - r(sensor->contact) x F.
  const bool roller_balance_enabled =
    parameters_.roller_balance.monitoring_enabled ||
    parameters_.roller_balance.control_enabled;
  const auto rotation_axis = static_cast<Eigen::Index>(
    parameters_.roller_balance.rotation_axis_index);
  const Eigen::Vector3d contact_offset_control(
    parameters_.roller_balance.contact_center_offset_m[0],
    parameters_.roller_balance.contact_center_offset_m[1],
    parameters_.roller_balance.contact_center_offset_m[2]);
  const Eigen::Vector3d contact_center_base =
    admittance_transforms_.base_control_.translation() +
    admittance_transforms_.base_control_.rotation() * contact_offset_control;
  const Eigen::Vector3d sensor_to_contact_base =
    contact_center_base - admittance_transforms_.base_ft_.translation();
  const Eigen::Vector3d contact_torque_base =
    shift_torque_to_contact_center(
    admittance_state_.wrench_base.block<3, 1>(3, 0),
    admittance_state_.wrench_base.block<3, 1>(0, 0), sensor_to_contact_base);
  const Eigen::Vector3d contact_torque_control =
    admittance_transforms_.base_control_.rotation().transpose() * contact_torque_base;
  const double raw_contact_torque_nm = contact_torque_control[rotation_axis];
  const double torque_filter_tau_s = parameters_.roller_balance.filter_time_constant_s;
  const double torque_filter_alpha = torque_filter_tau_s <= 0.0 ? 1.0 :
    std::clamp(dt / (torque_filter_tau_s + dt), 0.0, 1.0);
  if (!roller_balance_filter_initialized_ || !std::isfinite(roller_balance_filtered_torque_nm_)) {
    roller_balance_filtered_torque_nm_ = raw_contact_torque_nm;
    roller_balance_filter_initialized_ = true;
  } else {
    roller_balance_filtered_torque_nm_ += torque_filter_alpha *
      (raw_contact_torque_nm - roller_balance_filtered_torque_nm_);
  }
  roller_balance_contact_torque_nm_ =
    roller_balance_filtered_torque_nm_ - parameters_.roller_balance.torque_bias_nm;
  const double roller_normal_force_n = std::abs(measured_control[normal_axis]);
  roller_balance_contact_valid_ = bool(
    roller_balance_enabled && std::isfinite(roller_normal_force_n) &&
    std::isfinite(roller_balance_contact_torque_nm_) &&
    roller_normal_force_n >= parameters_.roller_balance.minimum_normal_force_n);
  roller_balance_cop_offset_m_ = roller_balance_contact_valid_ ?
    parameters_.roller_balance.torque_to_cop_sign *
    roller_balance_contact_torque_nm_ / roller_normal_force_n : 0.0;

  measured_control[normal_axis] = measured_normal_force_for_control(
    measured_control[normal_axis], parameters_.admittance.absolute_normal_force);

  Eigen::Matrix<double, 6, 1> commanded_base;
  commanded_base.block<3, 1>(0, 0) = admittance_transforms_.base_ft_.rotation() *
    Eigen::Vector3d(
    commanded_wrench.force.x, commanded_wrench.force.y, commanded_wrench.force.z);
  commanded_base.block<3, 1>(3, 0) = admittance_transforms_.base_ft_.rotation() *
    Eigen::Vector3d(
    commanded_wrench.torque.x, commanded_wrench.torque.y, commanded_wrench.torque.z);

  Eigen::Matrix<double, 6, 1> commanded_control;
  commanded_control.block<3, 1>(0, 0) =
    admittance_transforms_.base_control_.rotation().transpose() *
    commanded_base.block<3, 1>(0, 0);
  commanded_control.block<3, 1>(3, 0) =
    admittance_transforms_.base_control_.rotation().transpose() *
    commanded_base.block<3, 1>(3, 0);

  // A broad force-hold band is deliberately different from a single force
  // target. Outside the band, correct toward the nearest inner entry boundary.
  // Inside it, cancel the drive and let damping settle the normal velocity;
  // calculate_admittance_rule() also removes the normal position spring for
  // the same cycle so the current trim is retained rather than being pulled
  // back toward the nominal path.
  const double commanded_press_n = -commanded_control[normal_axis];

  // The balance band is expressed as a contact location, not a fixed torque:
  // T = Fn*x.  Consequently the 5/8 mm stop/start thresholds automatically
  // become 0.10/0.16 Nm at 20 N and 0.15/0.24 Nm at 30 N.  Outside the band
  // apply only the moment beyond the inner boundary; inside it, cancel the
  // rotational drive and preserve the current trim instead of chasing zero.
  const bool roller_control_contact_valid = bool(
    roller_balance_contact_valid_ && parameters_.roller_balance.control_enabled &&
    commanded_press_n >= parameters_.roller_balance.minimum_normal_force_n);
  roller_balance_correction_active_ = next_roller_balance_correction_state(
    roller_balance_correction_active_, roller_control_contact_valid,
    roller_balance_cop_offset_m_, parameters_.roller_balance.cop_enter_m,
    parameters_.roller_balance.cop_exit_m, parameters_.roller_balance.roller_length_m);
  roller_balance_freeze_trim_ = bool(
    parameters_.roller_balance.control_enabled && commanded_press_n > 0.0);
  if (parameters_.roller_balance.control_enabled) {
    const auto wrench_rotation_axis = static_cast<Eigen::Index>(3 + rotation_axis);
    const double balance_drive_torque_nm = roller_balance_boundary_torque(
      roller_balance_cop_offset_m_, roller_normal_force_n,
      roller_balance_correction_active_, parameters_.roller_balance.cop_exit_m,
      parameters_.roller_balance.rotation_feedback_sign);
    measured_control[wrench_rotation_axis] =
      balance_drive_torque_nm - commanded_control[wrench_rotation_axis];
  }

  normal_force_hold_active_ = next_normal_force_hold_state(
    normal_force_hold_active_, measured_control[normal_axis], commanded_press_n,
    parameters_.admittance.normal_force_hold_lower_n,
    parameters_.admittance.normal_force_hold_upper_n,
    parameters_.admittance.normal_force_hold_hysteresis_n);
  measured_control[normal_axis] = normal_force_component_for_band(
    measured_control[normal_axis], commanded_press_n, normal_force_hold_active_,
    parameters_.admittance.normal_force_hold_lower_n,
    parameters_.admittance.normal_force_hold_upper_n,
    parameters_.admittance.normal_force_hold_hysteresis_n);

  admittance_state_.wrench_base.block<3, 1>(0, 0) =
    admittance_transforms_.base_control_.rotation() * measured_control.block<3, 1>(0, 0) +
    commanded_base.block<3, 1>(0, 0);
  admittance_state_.wrench_base.block<3, 1>(3, 0) =
    admittance_transforms_.base_control_.rotation() * measured_control.block<3, 1>(3, 0) +
    commanded_base.block<3, 1>(3, 0);

  // Compute admittance control law
  vec_to_eigen(current_joint_state.positions, admittance_state_.current_joint_pos);
  vec_to_eigen(reference_joint_state.positions, admittance_state_.reference_joint_pos);
  admittance_state_.rot_base_control = admittance_transforms_.base_control_.rotation();
  admittance_state_.ref_trans_base_ft = admittance_transforms_.ref_base_ft_;
  admittance_state_.ft_sensor_frame = parameters_.ft_sensor.frame.id;
  success &= calculate_admittance_rule(admittance_state_, dt);

  // Do not replace the previous safe output with the nominal path on failure:
  // dropping an existing trim would be a discontinuous position command.
  if (!success) {
    return controller_interface::return_type::ERROR;
  }

  // update joint desired joint state
  for (size_t i = 0; i < num_joints_; ++i) {
    auto idx = static_cast<Eigen::Index>(i);
    desired_joint_state.positions[i] =
      reference_joint_state.positions[i] + admittance_state_.joint_pos[idx];
    desired_joint_state.velocities[i] =
      reference_joint_state.velocities[i] + admittance_state_.joint_vel[idx];
    desired_joint_state.accelerations[i] =
      reference_joint_state.accelerations[i] + admittance_state_.joint_acc[idx];
  }

  return controller_interface::return_type::OK;
}

bool AdmittanceRule::calculate_admittance_rule(AdmittanceState & admittance_state, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  const bool balance_enabled = parameters_.roller_balance.control_enabled;
  const Eigen::VectorXd previous_joint_velocity = admittance_state.joint_vel;
  const auto previous_cartesian_velocity = admittance_state.admittance_velocity;
  // Integrating at measured joints winds commands up when the arm cannot
  // follow. The rotational limiter must use the configuration being commanded.
  const Eigen::VectorXd commanded_joints =
    admittance_state.reference_joint_pos + admittance_state.joint_pos;
  const Eigen::VectorXd & integration_joints =
    balance_enabled ? commanded_joints : admittance_state.current_joint_pos;
  const auto normal_axis = static_cast<Eigen::Index>(parameters_.normal_axis_index);
  const auto balance_rotation_axis = static_cast<Eigen::Index>(
    parameters_.roller_balance.rotation_axis_index);

  // Create stiffness matrix in base frame. The user-provided values of admittance_state.stiffness
  // correspond to the six diagonal elements of the stiffness matrix expressed in the control frame
  auto rot_base_control = admittance_state.rot_base_control;
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> K_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> K_rot = Eigen::Matrix<double, 3, 3>::Zero();
  K_pos.diagonal() = admittance_state.stiffness.block<3, 1>(0, 0);
  K_pos(normal_axis, normal_axis) = normal_stiffness_for_hold(
    K_pos(normal_axis, normal_axis), normal_force_hold_active_);
  K_rot.diagonal() = admittance_state.stiffness.block<3, 1>(3, 0);
  if (parameters_.roller_balance.control_enabled && roller_balance_freeze_trim_) {
    K_rot(balance_rotation_axis, balance_rotation_axis) = 0.0;
  }
  // Transform to the control frame
  // A reference is here:  https://users.wpi.edu/~jfu2/rbe502/files/force_control.pdf
  // Force Control by Luigi Villani and Joris De Schutter
  // Page 200
  K_pos = rot_base_control * K_pos * rot_base_control.transpose();
  K_rot = rot_base_control * K_rot * rot_base_control.transpose();
  K.block<3, 3>(0, 0) = K_pos;
  K.block<3, 3>(3, 3) = K_rot;

  // The same for damping
  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> D_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> D_rot = Eigen::Matrix<double, 3, 3>::Zero();
  D_pos.diagonal() = admittance_state.damping.block<3, 1>(0, 0);
  D_rot.diagonal() = admittance_state.damping.block<3, 1>(3, 0);
  D_pos = rot_base_control * D_pos * rot_base_control.transpose();
  D_rot = rot_base_control * D_rot * rot_base_control.transpose();
  D.block<3, 3>(0, 0) = D_pos;
  D.block<3, 3>(3, 3) = D_rot;

  // calculate admittance relative offset in base frame
  Eigen::Isometry3d desired_trans_base_ft;
  if (!kinematics_->calculate_link_transform(
      admittance_state.current_joint_pos, admittance_state.ft_sensor_frame, desired_trans_base_ft) ||
    !desired_trans_base_ft.matrix().allFinite())
  {
    return false;
  }
  Eigen::Matrix<double, 6, 1> X;
  X.block<3, 1>(0, 0) =
    desired_trans_base_ft.translation() - admittance_state.ref_trans_base_ft.translation();
  auto R_ref = admittance_state.ref_trans_base_ft.rotation();
  auto R_desired = desired_trans_base_ft.rotation();
  auto R = R_desired * R_ref.transpose();
  auto angle_axis = Eigen::AngleAxisd(R);
  X.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

  // get admittance relative velocity
  auto X_dot = Eigen::Matrix<double, 6, 1>(admittance_state.admittance_velocity.data());

  Eigen::Matrix<double, 3, 1> X_control =
    rot_base_control.transpose() * X.block<3, 1>(0, 0);
  normal_trim_m_ = X_control[normal_axis];
  const Eigen::Matrix<double, 3, 1> rotation_control =
    rot_base_control.transpose() * X.block<3, 1>(3, 0);
  roller_balance_rotation_trim_rad_ = rotation_control[balance_rotation_axis];
  // normal_limit_reached_ means one thing only: the compliance displacement
  // budget is exhausted.  Hitting the acceleration or velocity cap is the
  // limiter doing its job during an ordinary transient and is reported
  // separately through normal_rate_saturated_, because a consumer that aborts
  // on it would abort on every normal contact transient.
  normal_limit_reached_ = std::abs(normal_trim_m_) >= parameters_.max_normal_trim_m;
  normal_rate_saturated_ = false;
  roller_balance_limit_reached_ = bool(
    parameters_.roller_balance.control_enabled &&
    std::abs(roller_balance_rotation_trim_rad_) >=
    parameters_.roller_balance.max_rotation_trim_rad);
  roller_balance_rate_saturated_ = false;
  roller_balance_soft_limit_active_ = false;
  Eigen::Isometry3d commanded_transform = Eigen::Isometry3d::Identity();
  if (balance_enabled) {
    if (!kinematics_->calculate_link_transform(
        commanded_joints, admittance_state.ft_sensor_frame, commanded_transform) ||
      !commanded_transform.matrix().allFinite())
    {
      return false;
    }
    const Eigen::AngleAxisd command_rotation(
      commanded_transform.rotation() * R_ref.transpose());
    const Eigen::Vector3d command_rotation_control = rot_base_control.transpose() *
      (command_rotation.angle() * command_rotation.axis());
    roller_balance_commanded_trim_rad_ = command_rotation_control[balance_rotation_axis];
    roller_balance_limit_reached_ |=
      std::abs(roller_balance_commanded_trim_rad_) >=
      parameters_.roller_balance.max_rotation_trim_rad;
    if (roller_balance_limit_reached_) {
      // A real hard-envelope breach is not ordinary soft saturation. No new
      // position is emitted; controller-manager handles the error/deactivation.
      return false;
    }
    // During release, restore the COMMANDED trim, not a lagging measured pose.
    Eigen::Vector3d spring_rotation = rot_base_control.transpose() * X.block<3, 1>(3, 0);
    spring_rotation[balance_rotation_axis] = roller_balance_commanded_trim_rad_;
    X.block<3, 1>(3, 0) = rot_base_control * spring_rotation;
  }

  // external force expressed in the base frame
  auto F_base = admittance_state.wrench_base;

  // zero out any forces in the control frame
  Eigen::Matrix<double, 6, 1> F_control;
  F_control.block<3, 1>(0, 0) = rot_base_control.transpose() * F_base.block<3, 1>(0, 0);
  F_control.block<3, 1>(3, 0) = rot_base_control.transpose() * F_base.block<3, 1>(3, 0);
  F_control = F_control.cwiseProduct(admittance_state.selected_axes);

  // Unilateral-contact floor on the normal axis.  A surface can push the tool
  // away but never pull it in, so a driving force (measured + commanded) more
  // negative than the commanded press is not physically reachable from real
  // contact.  Left unbounded, the admittance reads such a value as "still not
  // pressed hard enough" and drives into the surface at its velocity cap: on
  // 2026-08-14 a normal reading that fell to -46 N took the compliance from
  // -2 mm to -6 mm of trim in 0.3 s and would have run to the trim bound.
  // Clamping keeps the ordinary press rate through a bad reading instead of
  // turning it into a runaway; the force monitor still sees the raw value and
  // remains free to latch on it.
  const double normal_floor = parameters_.admittance.min_normal_drive_force_n;
  if (normal_floor > 0.0 && F_control[normal_axis] < -normal_floor) {
    F_control[normal_axis] = -normal_floor;
  }

  F_base.block<3, 1>(0, 0) = rot_base_control * F_control.block<3, 1>(0, 0);
  F_base.block<3, 1>(3, 0) = rot_base_control * F_control.block<3, 1>(3, 0);

  // Compute admittance control law in the base frame: F = M*x_ddot + D*x_dot + K*x
  Eigen::Matrix<double, 6, 1> X_ddot =
    admittance_state.mass_inv.cwiseProduct(F_base - D * X_dot - K * X);

  // Clamp acceleration in the configured control-frame normal direction before
  // converting it to joint space.  If the trim boundary is reached, only motion
  // farther out of bounds is blocked; stiffness/damping may still return the
  // offset smoothly toward zero.
  Eigen::Matrix<double, 3, 1> acceleration_control =
    rot_base_control.transpose() * X_ddot.block<3, 1>(0, 0);
  const double unclamped_acceleration = acceleration_control[normal_axis];
  acceleration_control[normal_axis] = std::clamp(
    unclamped_acceleration, -parameters_.max_normal_acceleration_mps2,
    parameters_.max_normal_acceleration_mps2);
  normal_rate_saturated_ |=
    acceleration_control[normal_axis] != unclamped_acceleration;
  if (
    std::abs(normal_trim_m_) >= parameters_.max_normal_trim_m &&
    normal_trim_m_ * acceleration_control[normal_axis] > 0.0)
  {
    acceleration_control[normal_axis] = 0.0;
    normal_limit_reached_ = true;
  }
  X_ddot.block<3, 1>(0, 0) = rot_base_control * acceleration_control;

  // Rotational acceleration is limited below, AFTER joint damping and IK,
  // as a bound on the change of the final Cartesian correction velocity.
  bool success = kinematics_->convert_cartesian_deltas_to_joint_deltas(
    integration_joints, X_ddot, admittance_state.ft_sensor_frame,
    admittance_state.joint_acc);
  if (!success || !admittance_state.joint_acc.allFinite()) {
    return false;
  }

  // add damping if cartesian velocity falls below threshold
  for (int64_t i = 0; i < admittance_state.joint_acc.size(); ++i) {
    admittance_state.joint_acc[i] -=
      parameters_.admittance.joint_damping * admittance_state.joint_vel[i];
  }

  // Integrate velocity, then enforce the Cartesian normal velocity cap before
  // integrating position.  Re-conversion keeps the actual joint-space state in
  // sync with the bounded Cartesian velocity.
  admittance_state.joint_vel += (admittance_state.joint_acc) * dt;
  success &= kinematics_->convert_joint_deltas_to_cartesian_deltas(
    integration_joints, admittance_state.joint_vel,
    admittance_state.ft_sensor_frame, admittance_state.admittance_velocity);
  if (!success || !admittance_state.admittance_velocity.allFinite()) {
    return false;
  }
  Eigen::Matrix<double, 3, 1> velocity_control =
    rot_base_control.transpose() *
    admittance_state.admittance_velocity.block<3, 1>(0, 0);
  const double unclamped_velocity = velocity_control[normal_axis];
  velocity_control[normal_axis] = std::clamp(
    unclamped_velocity, -parameters_.max_normal_velocity_mps,
    parameters_.max_normal_velocity_mps);
  normal_rate_saturated_ |= velocity_control[normal_axis] != unclamped_velocity;
  if (
    std::abs(normal_trim_m_) >= parameters_.max_normal_trim_m &&
    normal_trim_m_ * velocity_control[normal_axis] > 0.0)
  {
    velocity_control[normal_axis] = 0.0;
    normal_limit_reached_ = true;
  }
  bool velocity_was_bounded = velocity_control[normal_axis] != unclamped_velocity;
  auto bounded_velocity = admittance_state.admittance_velocity;
  bounded_velocity.block<3, 1>(0, 0) = rot_base_control * velocity_control;
  double previous_angular_velocity = 0.0;
  if (balance_enabled) {
    Eigen::Matrix<double, 3, 1> angular_velocity_control =
      rot_base_control.transpose() *
      admittance_state.admittance_velocity.block<3, 1>(3, 0);
    const double unclamped_angular_velocity =
      angular_velocity_control[balance_rotation_axis];
    previous_angular_velocity = roller_balance_command_velocity_radps_;
    const auto & balance = parameters_.roller_balance;
    if (!limit_roller_command_velocity(
        unclamped_angular_velocity, previous_angular_velocity,
        roller_balance_commanded_trim_rad_, balance.soft_limit_ratio * balance.max_rotation_trim_rad,
        // Leave 2% numerical headroom for finite-step FK versus differential IK.
        0.98 * balance.max_rotation_velocity_radps,
        0.98 * balance.max_rotation_acceleration_radps2, dt,
        angular_velocity_control[balance_rotation_axis], roller_balance_soft_limit_active_))
    {
      return false;
    }
    const bool angular_velocity_was_bounded =
      angular_velocity_control[balance_rotation_axis] !=
      unclamped_angular_velocity;
    velocity_was_bounded |= angular_velocity_was_bounded;
    roller_balance_rate_saturated_ |= angular_velocity_was_bounded;
    bounded_velocity.block<3, 1>(3, 0) =
      rot_base_control * angular_velocity_control;
  }
  if (velocity_was_bounded || balance_enabled) {
    success &= kinematics_->convert_cartesian_deltas_to_joint_deltas(
      integration_joints, bounded_velocity,
      admittance_state.ft_sensor_frame, admittance_state.joint_vel);
    if (!success || !admittance_state.joint_vel.allFinite()) {
      return false;
    }
    // Damped differential IK does not reproduce a requested twist exactly.
    // Keep its damping whenever the mapped output satisfies the limits. Only
    // refine when necessary for the slew/braking contract, not to remove all
    // IK damping or force exact tracking near a singularity.
    bool mapped = false;
    for (int iteration = 0; iteration < 8; ++iteration) {
      if (!kinematics_->convert_joint_deltas_to_cartesian_deltas(
          integration_joints, admittance_state.joint_vel, admittance_state.ft_sensor_frame,
          admittance_state.admittance_velocity) ||
        !admittance_state.admittance_velocity.allFinite())
      {
        return false;
      }
      const Eigen::Matrix<double, 6, 1> residual =
        bounded_velocity - admittance_state.admittance_velocity;
      const Eigen::Vector3d mapped_linear = rot_base_control.transpose() *
        admittance_state.admittance_velocity.head<3>();
      const Eigen::Vector3d mapped_angular = rot_base_control.transpose() *
        admittance_state.admittance_velocity.tail<3>();
      const double mapped_normal = mapped_linear[normal_axis];
      const double mapped_omega = mapped_angular[balance_rotation_axis];
      const auto & balance = parameters_.roller_balance;
      double checked_omega = 0.0;
      bool checked_soft_limit = false;
      const bool angular_ok = !balance_enabled ||
        (limit_roller_command_velocity(
          mapped_omega, previous_angular_velocity, roller_balance_commanded_trim_rad_,
          balance.soft_limit_ratio * balance.max_rotation_trim_rad,
          0.98 * balance.max_rotation_velocity_radps,
          0.98 * balance.max_rotation_acceleration_radps2, dt,
          checked_omega, checked_soft_limit) && std::abs(mapped_omega - checked_omega) <= 1e-10);
      const bool normal_ok =
        std::abs(mapped_normal) <= parameters_.max_normal_velocity_mps + 1e-10 &&
        !(std::abs(normal_trim_m_) >= parameters_.max_normal_trim_m &&
        normal_trim_m_ * mapped_normal > 1e-12);
      if (!balance_enabled || (angular_ok && normal_ok)) {
        mapped = true;
        break;
      }
      Eigen::VectorXd correction = Eigen::VectorXd::Zero(integration_joints.size());
      if (!kinematics_->convert_cartesian_deltas_to_joint_deltas(
          integration_joints, residual, admittance_state.ft_sensor_frame, correction) ||
        !correction.allFinite())
      {
        return false;
      }
      admittance_state.joint_vel += correction;
    }
    if (!mapped) {
      return false;
    }
  }
  const Eigen::VectorXd next_joint_pos =
    admittance_state.joint_pos + admittance_state.joint_vel * dt;
  if (balance_enabled) {
    const auto & balance = parameters_.roller_balance;
    const Eigen::Vector3d final_angular_control = rot_base_control.transpose() *
      admittance_state.admittance_velocity.block<3, 1>(3, 0);
    const double final_omega = final_angular_control[balance_rotation_axis];
    if (std::abs(final_omega) > balance.max_rotation_velocity_radps + 1e-9 ||
      std::abs(final_omega - previous_angular_velocity) >
      balance.max_rotation_acceleration_radps2 * dt + 1e-9)
    {
      return false;
    }
    Eigen::Isometry3d next_transform;
    const Eigen::VectorXd next_command = admittance_state.reference_joint_pos + next_joint_pos;
    if (!kinematics_->calculate_link_transform(
        next_command, admittance_state.ft_sensor_frame, next_transform) ||
      !next_transform.matrix().allFinite())
    {
      return false;
    }
    const Eigen::AngleAxisd next_rotation(next_transform.rotation() * R_ref.transpose());
    const Eigen::Vector3d next_control = rot_base_control.transpose() *
      (next_rotation.angle() * next_rotation.axis());
    const double next_trim = next_control[balance_rotation_axis];
    if (std::abs(next_trim) >= balance.max_rotation_trim_rad) {
      roller_balance_limit_reached_ = true;
      return false;
    }
    // Validate the finite rotation of the position command too, not just J*qdot.
    const Eigen::AngleAxisd step_rotation(
      next_transform.rotation() * commanded_transform.rotation().transpose());
    const Eigen::Vector3d step_control = rot_base_control.transpose() *
      (step_rotation.angle() * step_rotation.axis());
    const double step_omega = step_control[balance_rotation_axis] / dt;
    if (std::abs(step_omega) > balance.max_rotation_velocity_radps + 1e-9 ||
      std::abs(step_omega - previous_angular_velocity) >
      balance.max_rotation_acceleration_radps2 * dt + 1e-9)
    {
      return false;
    }
    roller_balance_command_velocity_radps_ = step_omega;
    roller_balance_commanded_trim_rad_ = next_trim;
  }
  // Commit position only after all bounds and conversions passed. Report the
  // acceleration of the accepted command, including damping and every limiter.
  admittance_state.joint_acc = (admittance_state.joint_vel - previous_joint_velocity) / dt;
  admittance_state.admittance_acceleration =
    (admittance_state.admittance_velocity - previous_cartesian_velocity) / dt;
  if (!admittance_state.joint_acc.allFinite() ||
    !admittance_state.admittance_acceleration.allFinite())
  {
    return false;
  }
  admittance_state.joint_pos = next_joint_pos;
  return true;
}

void AdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench,
  const Eigen::Matrix<double, 3, 3> & sensor_world_rot,
  const Eigen::Matrix<double, 3, 3> & cog_world_rot)
{
  Eigen::Matrix<double, 3, 2, Eigen::ColMajor> new_wrench;
  new_wrench(0, 0) = measured_wrench.force.x;
  new_wrench(1, 0) = measured_wrench.force.y;
  new_wrench(2, 0) = measured_wrench.force.z;
  new_wrench(0, 1) = measured_wrench.torque.x;
  new_wrench(1, 1) = measured_wrench.torque.y;
  new_wrench(2, 1) = measured_wrench.torque.z;

  // transform to world frame
  Eigen::Matrix<double, 3, 2> new_wrench_base = sensor_world_rot * new_wrench;

  // apply gravity compensation
  new_wrench_base(2, 0) -= end_effector_weight_[2];
  new_wrench_base.block<3, 1>(0, 1) -= (cog_world_rot * cog_pos_).cross(end_effector_weight_);

  // apply smoothing filter
  for (Eigen::Index i = 0; i < 6; ++i) {
    wrench_world_(i) = filters::exponentialSmoothing(
      new_wrench_base(i), wrench_world_(i), parameters_.ft_sensor.filter_coefficient);
  }
}

const control_msgs::msg::AdmittanceControllerState & AdmittanceRule::get_controller_state()
{
  for (size_t i = 0; i < NUM_CARTESIAN_DOF; ++i) {
    auto idx = static_cast<Eigen::Index>(i);
    state_message_.stiffness.data[i] = admittance_state_.stiffness[idx];
    state_message_.damping.data[i] = admittance_state_.damping[idx];
    state_message_.selected_axes.data[i] = static_cast<bool>(admittance_state_.selected_axes[idx]);
    state_message_.mass.data[i] = admittance_state_.mass[idx];
  }

  for (size_t i = 0; i < parameters_.joints.size(); ++i) {
    auto idx = static_cast<Eigen::Index>(i);
    state_message_.joint_state.name[i] = parameters_.joints[i];
    state_message_.joint_state.position[i] = admittance_state_.joint_pos[idx];
    state_message_.joint_state.velocity[i] = admittance_state_.joint_vel[idx];
    state_message_.joint_state.effort[i] = admittance_state_.joint_acc[idx];
  }

  state_message_.wrench_base.wrench.force.x = admittance_state_.wrench_base[0];
  state_message_.wrench_base.wrench.force.y = admittance_state_.wrench_base[1];
  state_message_.wrench_base.wrench.force.z = admittance_state_.wrench_base[2];
  state_message_.wrench_base.wrench.torque.x = admittance_state_.wrench_base[3];
  state_message_.wrench_base.wrench.torque.y = admittance_state_.wrench_base[4];
  state_message_.wrench_base.wrench.torque.z = admittance_state_.wrench_base[5];

  state_message_.admittance_velocity.twist.linear.x = admittance_state_.admittance_velocity[0];
  state_message_.admittance_velocity.twist.linear.y = admittance_state_.admittance_velocity[1];
  state_message_.admittance_velocity.twist.linear.z = admittance_state_.admittance_velocity[2];
  state_message_.admittance_velocity.twist.angular.x = admittance_state_.admittance_velocity[3];
  state_message_.admittance_velocity.twist.angular.y = admittance_state_.admittance_velocity[4];
  state_message_.admittance_velocity.twist.angular.z = admittance_state_.admittance_velocity[5];

  state_message_.admittance_acceleration.twist.linear.x =
    admittance_state_.admittance_acceleration[0];
  state_message_.admittance_acceleration.twist.linear.y =
    admittance_state_.admittance_acceleration[1];
  state_message_.admittance_acceleration.twist.linear.z =
    admittance_state_.admittance_acceleration[2];
  state_message_.admittance_acceleration.twist.angular.x =
    admittance_state_.admittance_acceleration[3];
  state_message_.admittance_acceleration.twist.angular.y =
    admittance_state_.admittance_acceleration[4];
  state_message_.admittance_acceleration.twist.angular.z =
    admittance_state_.admittance_acceleration[5];

  state_message_.admittance_position = tf2::eigenToTransform(admittance_state_.admittance_position);
  state_message_.admittance_position.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_position.child_frame_id = "admittance_offset";

  state_message_.ref_trans_base_ft = tf2::eigenToTransform(admittance_state_.ref_trans_base_ft);
  state_message_.ref_trans_base_ft.header.frame_id = parameters_.kinematics.base;
  state_message_.ref_trans_base_ft.child_frame_id = parameters_.ft_sensor.frame.id;

  Eigen::Quaterniond quat(admittance_state_.rot_base_control);
  state_message_.rot_base_control.w = quat.w();
  state_message_.rot_base_control.x = quat.x();
  state_message_.rot_base_control.y = quat.y();
  state_message_.rot_base_control.z = quat.z();

  return state_message_;
}

template<typename T1, typename T2>
void AdmittanceRule::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++) {
    for (auto row = 0; row < matrix.rows(); row++) {
      matrix(row, col) = data[static_cast<size_t>(row + col * matrix.rows())];
    }
  }
}

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
