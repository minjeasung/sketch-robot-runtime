// Copyright (c) 2021, PickNik, Inc.
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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_controller_parameters.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace admittance_controller
{
inline double measured_normal_force_for_control(double measured_force, bool use_absolute_force)
{
  return use_absolute_force ? std::abs(measured_force) : measured_force;
}

inline bool normal_force_hold_band_is_valid(
  double lower_n, double upper_n, double hysteresis_n)
{
  return std::isfinite(lower_n) && std::isfinite(upper_n) &&
         std::isfinite(hysteresis_n) && lower_n > 0.0 && upper_n > lower_n &&
         hysteresis_n >= 0.0 && lower_n + hysteresis_n < upper_n - hysteresis_n;
}

inline bool next_normal_force_hold_state(
  bool was_holding, double measured_reaction_n, double commanded_press_n,
  double lower_n, double upper_n, double hysteresis_n)
{
  if (
    !normal_force_hold_band_is_valid(lower_n, upper_n, hysteresis_n) ||
    !std::isfinite(measured_reaction_n) || !std::isfinite(commanded_press_n) ||
    commanded_press_n < lower_n)
  {
    return false;
  }

  if (was_holding) {
    return measured_reaction_n >= lower_n && measured_reaction_n <= upper_n;
  }
  return measured_reaction_n >= lower_n + hysteresis_n &&
         measured_reaction_n <= upper_n - hysteresis_n;
}

inline double normal_force_component_for_band(
  double measured_reaction_n, double commanded_press_n, bool hold_active,
  double lower_n, double upper_n, double hysteresis_n)
{
  if (
    !normal_force_hold_band_is_valid(lower_n, upper_n, hysteresis_n) ||
    !std::isfinite(measured_reaction_n) || !std::isfinite(commanded_press_n) ||
    commanded_press_n < lower_n)
  {
    return measured_reaction_n;
  }

  // The commanded wrench remains the force-pipeline enable/ramp signal, but
  // it is not a force setpoint once the band is active.  Build the normal
  // component so measured + commanded produces a boundary error:
  //   below band -> measured - (lower + hysteresis)
  //   in band    -> 0
  //   above band -> measured - (upper - hysteresis)
  // This avoids silently chasing the midpoint of a wide operator-selected
  // range.  The inner boundaries are used only to enter; once holding, the
  // outer boundaries decide when correction restarts.
  if (hold_active) {
    return commanded_press_n;
  }
  const double entry_boundary_n =
    measured_reaction_n < lower_n + hysteresis_n ?
    lower_n + hysteresis_n : upper_n - hysteresis_n;
  return commanded_press_n + measured_reaction_n - entry_boundary_n;
}

inline double normal_stiffness_for_hold(double configured_stiffness, bool hold_active)
{
  return hold_active ? 0.0 : configured_stiffness;
}

inline bool roller_balance_band_is_valid(
  double cop_enter_m, double cop_exit_m, double roller_length_m)
{
  return std::isfinite(cop_enter_m) && std::isfinite(cop_exit_m) &&
         std::isfinite(roller_length_m) && cop_exit_m > 0.0 &&
         cop_enter_m > cop_exit_m && roller_length_m > 0.0 &&
         cop_enter_m < 0.5 * roller_length_m;
}

inline bool next_roller_balance_correction_state(
  bool was_correcting, bool contact_valid, double cop_offset_m,
  double cop_enter_m, double cop_exit_m, double roller_length_m)
{
  if (
    !contact_valid || !std::isfinite(cop_offset_m) ||
    !roller_balance_band_is_valid(cop_enter_m, cop_exit_m, roller_length_m))
  {
    return false;
  }
  return was_correcting ? std::abs(cop_offset_m) > cop_exit_m :
         std::abs(cop_offset_m) >= cop_enter_m;
}

inline double roller_balance_boundary_torque(
  double cop_offset_m, double normal_force_n, bool correction_active,
  double cop_exit_m, double rotation_feedback_sign)
{
  if (
    !correction_active || !std::isfinite(cop_offset_m) ||
    !std::isfinite(normal_force_n) || normal_force_n <= 0.0 ||
    !std::isfinite(cop_exit_m) || cop_exit_m <= 0.0 ||
    !std::isfinite(rotation_feedback_sign))
  {
    return 0.0;
  }
  const double outside_inner_band_m = std::max(std::abs(cop_offset_m) - cop_exit_m, 0.0);
  return rotation_feedback_sign * std::copysign(
    normal_force_n * outside_inner_band_m, cop_offset_m);
}

inline Eigen::Vector3d shift_torque_to_contact_center(
  const Eigen::Vector3d & sensor_torque, const Eigen::Vector3d & sensor_force,
  const Eigen::Vector3d & sensor_to_contact)
{
  return sensor_torque - sensor_to_contact.cross(sensor_force);
}

// Backward-Euler position integration needs room for this step AND braking.
// v*dt + v^2/(2*a) <= distance.  This remains conservative when dt varies.
inline double roller_braking_velocity(double distance, double acceleration, double dt)
{
  if (distance <= 0.0) {
    return 0.0;
  }
  const double a_dt = acceleration * dt;
  // Rationalized form avoids cancellation close to the boundary.
  return 2.0 * acceleration * distance /
         (std::sqrt(a_dt * a_dt + 2.0 * acceleration * distance) + a_dt);
}

inline bool limit_roller_command_velocity(
  double requested_velocity, double previous_velocity, double commanded_trim,
  double soft_limit, double max_velocity, double max_acceleration, double dt,
  double & limited_velocity, bool & soft_limit_active)
{
  if (
    !std::isfinite(requested_velocity) || !std::isfinite(previous_velocity) ||
    !std::isfinite(commanded_trim) || !std::isfinite(soft_limit) || soft_limit <= 0.0 ||
    !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
    !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return false;
  }
  const double lower_brake = -roller_braking_velocity(
    soft_limit + commanded_trim, max_acceleration, dt);
  const double upper_brake = roller_braking_velocity(
    soft_limit - commanded_trim, max_acceleration, dt);
  const double lower = std::max(
    {-max_velocity, previous_velocity - max_acceleration * dt, lower_brake});
  const double upper = std::min(
    {max_velocity, previous_velocity + max_acceleration * dt, upper_brake});
  // An infeasible state must not silently bypass either bound.
  if (lower > upper) {
    return false;
  }
  limited_velocity = std::clamp(requested_velocity, lower, upper);
  soft_limit_active = requested_velocity < lower_brake || requested_velocity > upper_brake;
  return true;
}

struct AdmittanceTransforms
{
  // transformation from force torque sensor frame to base link frame at reference joint angles
  Eigen::Isometry3d ref_base_ft_;
  // transformation from force torque sensor frame to base link frame at reference + admittance
  // offset joint angles
  Eigen::Isometry3d base_ft_;
  // transformation from control frame to base link frame at reference + admittance offset joint
  // angles
  Eigen::Isometry3d base_control_;
  // transformation from end effector frame to base link frame at reference + admittance offset
  // joint angles
  Eigen::Isometry3d base_tip_;
  // transformation from center of gravity frame to base link frame at reference + admittance offset
  // joint angles
  Eigen::Isometry3d base_cog_;
  // transformation from world frame to base link frame
  Eigen::Isometry3d world_base_;
};

struct AdmittanceState
{
  explicit AdmittanceState(size_t num_joints)
  {
    admittance_velocity.setZero();
    admittance_acceleration.setZero();
    damping.setZero();
    mass.setOnes();
    mass_inv.setZero();
    stiffness.setZero();
    selected_axes.setZero();
    auto idx = static_cast<Eigen::Index>(num_joints);
    current_joint_pos = Eigen::VectorXd::Zero(idx);
    reference_joint_pos = Eigen::VectorXd::Zero(idx);
    joint_pos = Eigen::VectorXd::Zero(idx);
    joint_vel = Eigen::VectorXd::Zero(idx);
    joint_acc = Eigen::VectorXd::Zero(idx);
  }

  Eigen::VectorXd current_joint_pos;
  Eigen::VectorXd reference_joint_pos;
  Eigen::VectorXd joint_pos;
  Eigen::VectorXd joint_vel;
  Eigen::VectorXd joint_acc;
  Eigen::Matrix<double, 6, 1> damping;
  Eigen::Matrix<double, 6, 1> mass;
  Eigen::Matrix<double, 6, 1> mass_inv;
  Eigen::Matrix<double, 6, 1> selected_axes;
  Eigen::Matrix<double, 6, 1> stiffness;
  Eigen::Matrix<double, 6, 1> wrench_base;
  Eigen::Matrix<double, 6, 1> admittance_acceleration;
  Eigen::Matrix<double, 6, 1> admittance_velocity;
  Eigen::Isometry3d admittance_position;
  Eigen::Matrix<double, 3, 3> rot_base_control;
  Eigen::Isometry3d ref_trans_base_ft;
  std::string ft_sensor_frame;
};

class AdmittanceRule
{
public:
  explicit AdmittanceRule(
    const std::shared_ptr<admittance_controller::ParamListener> & parameter_handler)
  {
    parameter_handler_ = parameter_handler;
    parameters_ = parameter_handler_->get_params();
    num_joints_ = parameters_.joints.size();
    admittance_state_ = AdmittanceState(num_joints_);
    reset(num_joints_);
  }

  /// Configure admittance rule memory using number of joints.
  controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const size_t num_joint,
    const std::string & robot_description);

  /// Reset all values back to default
  controller_interface::return_type reset(const size_t num_joints);

  /**
   * Calculate all transforms needed for admittance control using the loader kinematics plugin. If
   * the transform does not exist in the kinematics model, then TF will be used for lookup. The
   * return value is true if all transformation are calculated without an error \param[in]
   * current_joint_state current joint state of the robot \param[in] reference_joint_state input
   * joint state reference \param[out] success true if no calls to the kinematics interface fail
   */
  bool get_all_transforms(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state);

  /**
   * Updates parameter_ struct if any parameters have changed since last update. Parameter dependent
   * Eigen field members (end_effector_weight_, cog_pos_, mass_, mass_inv_ stiffness, selected_axes,
   * damping_) are also updated
   */
  void apply_parameters_update();

  /**
   * Calculate 'desired joint states' based on the 'measured force', 'reference joint state', and
   * 'current_joint_state'.
   *
   * \param[in] current_joint_state current joint state of the robot
   * \param[in] measured_wrench most recent measured wrench from force torque sensor
   * \param[in] commanded_wrench external wrench command in the force torque sensor frame
   * \param[in] reference_joint_state input joint state reference
   * \param[in] period time in seconds since last controller update
   * \param[out] desired_joint_state joint state reference after the admittance offset is applied to
   * the input reference
   */
  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const geometry_msgs::msg::Wrench & commanded_wrench,
    const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states);

  /**
   * Set fields of `state_message` from current admittance controller state.
   *
   * \param[out] state_message message containing target position/vel/accel, wrench, and actual
   * robot state, among other things
   */
  const control_msgs::msg::AdmittanceControllerState & get_controller_state();

  /// True only when the normal displacement budget (max_normal_trim_m) is exhausted.
  bool normal_limit_reached() const {return normal_limit_reached_;}

  /// True when the normal velocity or acceleration cap clamped this cycle.
  /// This is ordinary limiter action during a transient, not a fault.
  bool normal_rate_saturated() const {return normal_rate_saturated_;}

  /// Latest signed normal displacement in the configured control frame.
  double normal_trim_m() const {return normal_trim_m_;}

  /// True while the measured reaction is inside the configured force-hold band.
  bool normal_force_hold_active() const {return normal_force_hold_active_;}

  bool roller_balance_contact_valid() const {return roller_balance_contact_valid_;}
  bool roller_balance_correction_active() const {return roller_balance_correction_active_;}
  bool roller_balance_limit_reached() const {return roller_balance_limit_reached_;}
  bool roller_balance_rate_saturated() const {return roller_balance_rate_saturated_;}
  double roller_balance_contact_torque_nm() const {return roller_balance_contact_torque_nm_;}
  double roller_balance_cop_offset_m() const {return roller_balance_cop_offset_m_;}
  double roller_balance_rotation_trim_rad() const {return roller_balance_rotation_trim_rad_;}
  double roller_balance_commanded_trim_rad() const {return roller_balance_commanded_trim_rad_;}
  bool roller_balance_soft_limit_active() const {return roller_balance_soft_limit_active_;}

public:
  // admittance config parameters
  std::shared_ptr<admittance_controller::ParamListener> parameter_handler_;
  admittance_controller::Params parameters_;

  // Exposed read-only through accessors for the controller diagnostic topic.
  bool normal_limit_reached_{false};
  bool normal_rate_saturated_{false};
  double normal_trim_m_{0.0};
  bool normal_force_hold_active_{false};
  bool roller_balance_contact_valid_{false};
  bool roller_balance_correction_active_{false};
  bool roller_balance_limit_reached_{false};
  bool roller_balance_rate_saturated_{false};
  bool roller_balance_freeze_trim_{false};
  bool roller_balance_filter_initialized_{false};
  double roller_balance_filtered_torque_nm_{0.0};
  double roller_balance_contact_torque_nm_{0.0};
  double roller_balance_cop_offset_m_{0.0};
  double roller_balance_rotation_trim_rad_{0.0};
  double roller_balance_commanded_trim_rad_{0.0};
  double roller_balance_command_velocity_radps_{0.0};
  bool roller_balance_soft_limit_active_{false};

protected:
  /**
   * Calculates the admittance rule from given the robot's current joint angles. The admittance
   * controller state input is updated with the new calculated values. A boolean value is returned
   * indicating if any of the kinematics plugin calls failed. \param[in] admittance_state contains
   * all the information needed to calculate the admittance offset \param[in] dt controller period
   * \param[out] success true if no calls to the kinematics interface fail
   */
  bool calculate_admittance_rule(AdmittanceState & admittance_state, double dt);

  /**
   * Updates internal estimate of wrench in world frame `wrench_world_` given the new measurement
   * `measured_wrench`, the sensor to base frame rotation `sensor_world_rot`, and the center of
   * gravity frame to base frame rotation `cog_world_rot`. The `wrench_world_` estimate includes
   * gravity compensation \param[in] measured_wrench  most recent measured wrench from force torque
   * sensor \param[in] sensor_world_rot rotation matrix from world frame to sensor frame \param[in]
   * cog_world_rot rotation matrix from world frame to center of gravity frame
   */
  void process_wrench_measurements(
    const geometry_msgs::msg::Wrench & measured_wrench,
    const Eigen::Matrix<double, 3, 3> & sensor_world_rot,
    const Eigen::Matrix<double, 3, 3> & cog_world_rot);

  template<typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

  // number of robot joint
  size_t num_joints_;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // filtered wrench in world frame
  Eigen::Matrix<double, 6, 1> wrench_world_;

  // admittance controllers internal state
  AdmittanceState admittance_state_{0};

  // transforms needed for admittance update
  AdmittanceTransforms admittance_transforms_;

  // position of center of gravity in cog_frame
  Eigen::Vector3d cog_pos_;

  // force applied to sensor due to weight of end effector
  Eigen::Vector3d end_effector_weight_;

  // ROS
  control_msgs::msg::AdmittanceControllerState state_message_;
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
