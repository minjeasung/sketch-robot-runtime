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

#include "admittance_controller/admittance_controller.hpp"

#include <tinyxml2.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{  // utility

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

bool finite_wrench(const geometry_msgs::msg::Wrench & wrench)
{
  return std::isfinite(wrench.force.x) && std::isfinite(wrench.force.y) &&
         std::isfinite(wrench.force.z) && std::isfinite(wrench.torque.x) &&
         std::isfinite(wrench.torque.y) && std::isfinite(wrench.torque.z);
}

// called from RT control loop
void reset_controller_reference_msg(trajectory_msgs::msg::JointTrajectoryPoint & msg)
{
  msg.positions.clear();
  msg.velocities.clear();
}

// called from RT control loop
void reset_wrench_msg(
  geometry_msgs::msg::WrenchStamped & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg.header.stamp = node->now();
  msg.wrench = geometry_msgs::msg::Wrench();
}

}  // namespace

namespace admittance_controller
{

controller_interface::CallbackReturn AdmittanceController::on_init()
{
  // initialize controller config
  try {
    parameter_handler_ = std::make_shared<admittance_controller::ParamListener>(get_node());
    admittance_ = std::make_unique<admittance_controller::AdmittanceRule>(parameter_handler_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // number of joints in controllers is fixed after initialization
  num_joints_ = admittance_->parameters_.joints.size();

  // allocate dynamic memory
  last_reference_.positions.assign(num_joints_, 0.0);
  last_reference_.velocities.assign(num_joints_, 0.0);
  last_reference_.accelerations.assign(num_joints_, 0.0);
  last_commanded_ = last_reference_;
  reference_ = last_reference_;
  reference_admittance_ = last_reference_;
  joint_state_ = last_reference_;

  std::string robot_description = this->get_robot_description();

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_description' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  tinyxml2::XMLDocument doc;
  if (!doc.Parse(robot_description.c_str()) && doc.Error()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description XML from parameter "
      "'robot_description': %s",
      doc.ErrorStr());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (doc.Error()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error parsing robot description XML from parameter "
      "'robot_description': %s",
      doc.ErrorStr());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    for (const auto & joint : command_joint_names_) {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;
  for (size_t i = 0; i < admittance_->parameters_.state_interfaces.size(); ++i) {
    const auto & interface = admittance_->parameters_.state_interfaces[i];
    for (const auto & joint : admittance_->parameters_.joints) {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  state_interfaces_config_names.insert(
    state_interfaces_config_names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

std::vector<hardware_interface::CommandInterface>
AdmittanceController::on_export_reference_interfaces()
{
  // create CommandInterface interfaces that other controllers will be able to chain with
  if (!admittance_) {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
  const auto num_chainable_interfaces =
    admittance_->parameters_.chainable_command_interfaces.size() *
    admittance_->parameters_.joints.size();

  // allocate dynamic memory
  chainable_command_interfaces.reserve(num_chainable_interfaces);
  reference_interfaces_.resize(num_chainable_interfaces, std::numeric_limits<double>::quiet_NaN());
  position_reference_ = {};
  velocity_reference_ = {};

  // assign reference interfaces
  auto index = 0ul;
  for (const auto & interface : admittance_->parameters_.chainable_command_interfaces) {
    for (const auto & joint : admittance_->parameters_.joints) {
      if (hardware_interface::HW_IF_POSITION == interface) {
        position_reference_.emplace_back(reference_interfaces_[index]);
      } else if (hardware_interface::HW_IF_VELOCITY == interface) {
        velocity_reference_.emplace_back(reference_interfaces_[index]);
      }
      const auto exported_prefix = std::string(get_node()->get_name()) + "/" + joint;
      chainable_command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          exported_prefix, interface, reference_interfaces_.data() + index));

      index++;
    }
  }

  return chainable_command_interfaces;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto & admittance_parameters = admittance_->parameters_.admittance;
  const bool force_hold_requested =
    admittance_parameters.normal_force_hold_lower_n > 0.0 ||
    admittance_parameters.normal_force_hold_upper_n > 0.0 ||
    admittance_parameters.normal_force_hold_hysteresis_n > 0.0;
  if (force_hold_requested) {
    if (!admittance_parameters.absolute_normal_force) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The normal force-hold band requires admittance.absolute_normal_force=true.");
      return CallbackReturn::FAILURE;
    }
    if (!normal_force_hold_band_is_valid(
        admittance_parameters.normal_force_hold_lower_n,
        admittance_parameters.normal_force_hold_upper_n,
        admittance_parameters.normal_force_hold_hysteresis_n))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid normal force-hold band: require 0 < lower + hysteresis < "
        "upper - hysteresis.");
      return CallbackReturn::FAILURE;
    }
  }

  const auto & roller_balance = admittance_->parameters_.roller_balance;
  const bool roller_balance_requested =
    roller_balance.monitoring_enabled || roller_balance.control_enabled;
  if (roller_balance_requested) {
    const bool offset_finite = std::all_of(
      roller_balance.contact_center_offset_m.begin(),
      roller_balance.contact_center_offset_m.end(),
      [](double value) {return std::isfinite(value);});
    const bool signs_valid =
      std::abs(roller_balance.torque_to_cop_sign) == 1.0 &&
      std::abs(roller_balance.rotation_feedback_sign) == 1.0;
    if (
      !offset_finite || !std::isfinite(roller_balance.torque_bias_nm) ||
      !signs_valid || !roller_balance_band_is_valid(
        roller_balance.cop_enter_m, roller_balance.cop_exit_m,
        roller_balance.roller_length_m))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid roller balance geometry/band/sign: require finite geometry and bias, "
        "exact +/-1 signs, and 0 < cop_exit < cop_enter < roller_length/2.");
      return CallbackReturn::FAILURE;
    }
  }
  if (roller_balance.control_enabled) {
    const auto damping_index = static_cast<size_t>(
      3 + roller_balance.rotation_axis_index);
    if (admittance_parameters.damping_absolute[damping_index] <= 0.0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Roller balance control requires positive admittance.damping_absolute on its "
        "rotation axis.");
      return CallbackReturn::FAILURE;
    }
    RCLCPP_WARN(
      get_node()->get_logger(),
      "ROLLER BALANCE MOTION ENABLED: verify F/T tare, CoG, contact-centre offset, "
      "and feedback sign before wall contact.");
  }

  command_joint_names_ = admittance_->parameters_.command_joints;
  if (command_joint_names_.empty()) {
    command_joint_names_ = admittance_->parameters_.joints;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  } else if (command_joint_names_.size() != num_joints_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  // print and validate interface types
  for (const auto & tmp : admittance_->parameters_.state_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("state int types are: " + tmp + "\n").c_str());
  }
  for (const auto & tmp : admittance_->parameters_.command_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("command int types are: " + tmp + "\n").c_str());
  }

  // validate exported interfaces
  for (const auto & tmp : admittance_->parameters_.chainable_command_interfaces) {
    if (tmp == hardware_interface::HW_IF_POSITION || tmp == hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_INFO(
        get_node()->get_logger(), "%s", ("chainable int types are: " + tmp + "\n").c_str());
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "chainable interface type %s is not supported. Supported types are %s and %s", tmp.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  auto contains_interface_type =
    [](const std::vector<std::string> & interface_type_list, const std::string & interface_type)
    {
      return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
             interface_type_list.end();
    };

  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Command interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.state_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "State interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  if (!has_position_state_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Position state interface is required.");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types)
    {
      std::stringstream ss_command_interfaces;
      for (size_t index = 0; index < interface_types.size(); ++index) {
        if (index != 0) {
          ss_command_interfaces << " ";
        }
        ss_command_interfaces << interface_types[index];
      }
      return ss_command_interfaces.str();
    };
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(admittance_->parameters_.command_interfaces).c_str(),
    get_interface_list(admittance_->parameters_.state_interfaces).c_str());

  // setup subscribers and publishers
  auto joint_command_callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg)
    {input_joint_command_.set(*msg);};
  input_joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "~/joint_references", rclcpp::SystemDefaultsQoS(), joint_command_callback);

  input_wrench_command_subscriber_ =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "~/wrench_reference", rclcpp::SystemDefaultsQoS(),
    [&](const geometry_msgs::msg::WrenchStamped & msg)
    {
      if (
        msg.header.frame_id != admittance_->parameters_.ft_sensor.frame.id &&
        !msg.header.frame_id.empty())
      {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Ignoring wrench reference as it is on the wrong frame: %s. Expected reference frame: "
            "%s",
            msg.header.frame_id.c_str(), admittance_->parameters_.ft_sensor.frame.id.c_str());
        return;
      }
      TimedWrenchCommand command;
      command.message = msg;
      command.received_steady_ns = steady_now_ns();
      input_wrench_command_.set(command);
      });

  compliance_enable_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/compliance_enable", rclcpp::SystemDefaultsQoS(),
    [&](const std_msgs::msg::Bool & msg)
    {
      TimedComplianceCommand command;
      command.enabled = msg.data;
      command.received_steady_ns = steady_now_ns();
      input_compliance_enable_.set(command);
    });

  s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
    "~/status", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);
  compliance_active_publisher_ =
    get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/compliance_active", rclcpp::SystemDefaultsQoS());
  compliance_active_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    compliance_active_publisher_);
  limit_reached_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/normal_limit_reached", rclcpp::SystemDefaultsQoS());
  limit_reached_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    limit_reached_publisher_);
  // Diagnostics only.  Rate saturation is ordinary limiter action during a
  // transient; safety consumers must key on ~/normal_limit_reached instead.
  rate_saturated_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/normal_rate_saturated", rclcpp::SystemDefaultsQoS());
  rate_saturated_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    rate_saturated_publisher_);
  normal_trim_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/normal_trim_m", rclcpp::SystemDefaultsQoS());
  normal_trim_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
    normal_trim_publisher_);
  roller_contact_valid_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/roller_balance/contact_valid", rclcpp::SystemDefaultsQoS());
  roller_contact_valid_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    roller_contact_valid_publisher_);
  roller_correction_active_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/roller_balance/correction_active", rclcpp::SystemDefaultsQoS());
  roller_correction_active_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    roller_correction_active_publisher_);
  roller_limit_reached_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/roller_balance/limit_reached", rclcpp::SystemDefaultsQoS());
  roller_limit_reached_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    roller_limit_reached_publisher_);
  roller_rate_saturated_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/roller_balance/rate_saturated", rclcpp::SystemDefaultsQoS());
  roller_rate_saturated_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    roller_rate_saturated_publisher_);
  roller_contact_torque_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/roller_balance/contact_torque_nm", rclcpp::SystemDefaultsQoS());
  roller_contact_torque_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
    roller_contact_torque_publisher_);
  roller_cop_offset_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/roller_balance/cop_offset_m", rclcpp::SystemDefaultsQoS());
  roller_cop_offset_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
    roller_cop_offset_publisher_);
  roller_rotation_trim_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/roller_balance/rotation_trim_rad", rclcpp::SystemDefaultsQoS());
  roller_rotation_trim_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
    roller_rotation_trim_publisher_);
  roller_commanded_trim_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/roller_balance/commanded_trim_rad", rclcpp::SystemDefaultsQoS());
  roller_commanded_trim_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
    roller_commanded_trim_publisher_);
  roller_soft_limit_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/roller_balance/soft_limit_active", rclcpp::SystemDefaultsQoS());
  roller_soft_limit_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    roller_soft_limit_publisher_);

  // Initialize state message
  state_msg_ = admittance_->get_controller_state();

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    admittance_->parameters_.ft_sensor.name);

  // configure admittance rule
  if (
    admittance_->configure(get_node(), num_joints_, this->get_robot_description()) ==
    controller_interface::return_type::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // on_activate is called when the lifecycle node activates.
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // order all joints in the storage
  for (const auto & interface : admittance_->parameters_.state_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, admittance_->parameters_.joints, interface,
          joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // update parameters if any have changed
  admittance_->apply_parameters_update();

  // initialize interface of the FTS semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // initialize states
  read_state_from_hardware(joint_state_, ft_values_);
  for (auto val : joint_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  reset_controller_reference_msg(joint_command_msg_);
  reset_wrench_msg(wrench_command_msg_, get_node());
  compliance_enabled_ = admittance_->parameters_.compliance_enabled_by_default;
  compliance_active_ = false;
  last_wrench_received_steady_ns_ = 0;
  last_compliance_received_steady_ns_ = 0;

  // Use current joint_state as a default reference
  last_reference_ = joint_state_;
  last_commanded_ = joint_state_;
  reference_ = joint_state_;
  reference_admittance_ = joint_state_;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // update input reference from ros subscriber message
  if (!admittance_) {
    return controller_interface::return_type::ERROR;
  }
  auto joint_command_msg_op = input_joint_command_.try_get();
  if (joint_command_msg_op.has_value()) {
    joint_command_msg_ = joint_command_msg_op.value();
  }

  // if message exists, load values into references
  if (!joint_command_msg_.positions.empty() || !joint_command_msg_.velocities.empty()) {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces) {
      if (interface == hardware_interface::HW_IF_POSITION) {
        for (size_t i = 0; i < joint_command_msg_.positions.size(); ++i) {
          position_reference_[i].get() = joint_command_msg_.positions[i];
        }
      } else if (interface == hardware_interface::HW_IF_VELOCITY) {
        for (size_t i = 0; i < joint_command_msg_.velocities.size(); ++i) {
          velocity_reference_[i].get() = joint_command_msg_.velocities[i];
        }
      }
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Realtime constraints are required in this function
  if (!admittance_) {
    return controller_interface::return_type::ERROR;
  }

  // update input reference from chainable interfaces
  read_state_reference_interfaces(reference_);

  // get all controller inputs
  read_state_from_hardware(joint_state_, ft_values_);

  auto wrench_command_op = input_wrench_command_.try_get();
  if (wrench_command_op.has_value()) {
    wrench_command_msg_ = wrench_command_op->message;
    last_wrench_received_steady_ns_ = wrench_command_op->received_steady_ns;
  }

  auto compliance_command_op = input_compliance_enable_.try_get();
  if (compliance_command_op.has_value()) {
    compliance_enabled_ = compliance_command_op->enabled;
    last_compliance_received_steady_ns_ = compliance_command_op->received_steady_ns;
  }

  const int64_t now_ns = steady_now_ns();
  const double wrench_age_s =
    last_wrench_received_steady_ns_ > 0 ?
    static_cast<double>(now_ns - last_wrench_received_steady_ns_) * 1e-9 :
    std::numeric_limits<double>::infinity();
  const double compliance_age_s =
    last_compliance_received_steady_ns_ > 0 ?
    static_cast<double>(now_ns - last_compliance_received_steady_ns_) * 1e-9 :
    std::numeric_limits<double>::infinity();
  const bool wrench_fresh =
    wrench_age_s >= 0.0 && wrench_age_s <= admittance_->parameters_.wrench_reference_timeout_s;
  const bool compliance_fresh =
    compliance_age_s >= 0.0 &&
    compliance_age_s <= admittance_->parameters_.compliance_enable_timeout_s;
  const bool sensor_finite = finite_wrench(ft_values_);
  const bool command_finite = finite_wrench(wrench_command_msg_.wrench);
  const bool enable_allowed =
    admittance_->parameters_.compliance_enabled_by_default ?
    compliance_enabled_ :
    (compliance_enabled_ && compliance_fresh);
  compliance_active_ =
    enable_allowed && wrench_fresh && sensor_finite && command_finite;

  if (!compliance_active_) {
    // Keep running the spring/damping dynamics with zero excitation so an
    // existing offset returns smoothly instead of jumping to the nominal path.
    ft_values_ = geometry_msgs::msg::Wrench();
    wrench_command_msg_.wrench = geometry_msgs::msg::Wrench();
  }

  // apply admittance control to reference to determine desired state
  const auto update_result = admittance_->update(
    joint_state_, ft_values_, wrench_command_msg_.wrench, reference_, period,
    reference_admittance_);

  // write calculated values to joint interfaces
  // On failure leave the last hardware command untouched. Returning ERROR
  // deactivates the controller chain; never jump back to the untrimmed path.
  if (update_result == controller_interface::return_type::OK) {
    write_state_to_hardware(reference_admittance_);
  } else {
    compliance_active_ = false;
  }

  // Publish controller state
  if (state_publisher_) {
    state_msg_ = admittance_->get_controller_state();
    state_publisher_->try_publish(state_msg_);
  }
  if (compliance_active_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = compliance_active_;
    compliance_active_rt_publisher_->try_publish(msg);
  }
  if (limit_reached_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->normal_limit_reached();
    limit_reached_rt_publisher_->try_publish(msg);
  }
  if (rate_saturated_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->normal_rate_saturated();
    rate_saturated_rt_publisher_->try_publish(msg);
  }
  if (normal_trim_rt_publisher_) {
    std_msgs::msg::Float64 msg;
    msg.data = admittance_->normal_trim_m();
    normal_trim_rt_publisher_->try_publish(msg);
  }
  if (roller_contact_valid_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->roller_balance_contact_valid();
    roller_contact_valid_rt_publisher_->try_publish(msg);
  }
  if (roller_correction_active_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->roller_balance_correction_active();
    roller_correction_active_rt_publisher_->try_publish(msg);
  }
  if (roller_limit_reached_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->roller_balance_limit_reached();
    roller_limit_reached_rt_publisher_->try_publish(msg);
  }
  if (roller_rate_saturated_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->roller_balance_rate_saturated();
    roller_rate_saturated_rt_publisher_->try_publish(msg);
  }
  if (roller_contact_torque_rt_publisher_) {
    std_msgs::msg::Float64 msg;
    msg.data = admittance_->roller_balance_contact_torque_nm();
    roller_contact_torque_rt_publisher_->try_publish(msg);
  }
  if (roller_cop_offset_rt_publisher_) {
    std_msgs::msg::Float64 msg;
    msg.data = admittance_->roller_balance_cop_offset_m();
    roller_cop_offset_rt_publisher_->try_publish(msg);
  }
  if (roller_rotation_trim_rt_publisher_) {
    std_msgs::msg::Float64 msg;
    msg.data = admittance_->roller_balance_rotation_trim_rad();
    roller_rotation_trim_rt_publisher_->try_publish(msg);
  }
  if (roller_commanded_trim_rt_publisher_) {
    std_msgs::msg::Float64 msg;
    msg.data = admittance_->roller_balance_commanded_trim_rad();
    roller_commanded_trim_rt_publisher_->try_publish(msg);
  }
  if (roller_soft_limit_rt_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = admittance_->roller_balance_soft_limit_active();
    roller_soft_limit_rt_publisher_->try_publish(msg);
  }

  return update_result;
}

controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // release force torque sensor interface
  force_torque_sensor_->release_interfaces();

  // reset to prevent stale references
  for (size_t i = 0; i < num_joints_; i++) {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces) {
      if (interface == hardware_interface::HW_IF_POSITION) {
        position_reference_[i].get() = std::numeric_limits<double>::quiet_NaN();
      } else if (interface == hardware_interface::HW_IF_VELOCITY) {
        velocity_reference_[i].get() = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();
  admittance_->reset(num_joints_);

  reset_controller_reference_msg(joint_command_msg_);
  reset_wrench_msg(wrench_command_msg_, get_node());
  compliance_enabled_ = false;
  compliance_active_ = false;
  last_wrench_received_steady_ns_ = 0;
  last_compliance_received_steady_ns_ = 0;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  admittance_->reset(num_joints_);
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state_current,
  geometry_msgs::msg::Wrench & ft_values)
{
  // if any interface has nan values, assume state_current is the last command state
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + has_velocity_state_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_state_interface_) {
      const auto state_current_position_op =
        state_interfaces_[pos_ind * num_joints_ + joint_ind].get_optional();
      nan_position |=
        !state_current_position_op.has_value() || std::isnan(state_current_position_op.value());
      if (state_current_position_op.has_value()) {
        state_current.positions[joint_ind] = state_current_position_op.value();
      }
    }
    if (has_velocity_state_interface_) {
      auto state_current_velocity_op =
        state_interfaces_[vel_ind * num_joints_ + joint_ind].get_optional();
      nan_velocity |=
        !state_current_velocity_op.has_value() || std::isnan(state_current_velocity_op.value());

      if (state_current_velocity_op.has_value()) {
        state_current.velocities[joint_ind] = state_current_velocity_op.value();
      }
    }
    if (has_acceleration_state_interface_) {
      auto state_current_acceleration_op =
        state_interfaces_[acc_ind * num_joints_ + joint_ind].get_optional();
      nan_acceleration |= !state_current_acceleration_op.has_value() ||
        std::isnan(state_current_acceleration_op.value());
      if (state_current_acceleration_op.has_value()) {
        state_current.accelerations[joint_ind] = state_current_acceleration_op.value();
      }
    }
  }

  if (nan_position) {
    state_current.positions = last_commanded_.positions;
  }
  if (nan_velocity) {
    state_current.velocities = last_commanded_.velocities;
  }
  if (nan_acceleration) {
    state_current.accelerations = last_commanded_.accelerations;
  }

  // Preserve non-finite sensor state as an in-band validity signal. The update
  // loop checks finite_wrench() before enabling compliance and only then
  // replaces the excitation with zero. Converting NaN to zero here would make
  // stale/read-error/tare-invalid hardware indistinguishable from a valid 0 N.
  force_torque_sensor_->get_values_as_message(ft_values);
}

void AdmittanceController::write_state_to_hardware(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_commanded)
{
  // if any interface has nan values, assume state_commanded is the last command state
  size_t pos_ind = 0;
  size_t vel_ind =
    (has_position_command_interface_) ? pos_ind + has_velocity_command_interface_ : pos_ind;
  size_t acc_ind = vel_ind + has_acceleration_command_interface_;

  auto logger = get_node()->get_logger();

  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    bool success = true;
    if (has_position_command_interface_) {
      success &= command_interfaces_[pos_ind * num_joints_ + joint_ind].set_value(
        state_commanded.positions[joint_ind]);
    }
    if (has_velocity_command_interface_) {
      success &= command_interfaces_[vel_ind * num_joints_ + joint_ind].set_value(
        state_commanded.velocities[joint_ind]);
    }
    if (has_acceleration_command_interface_) {
      success &= command_interfaces_[acc_ind * num_joints_ + joint_ind].set_value(
        state_commanded.accelerations[joint_ind]);
    }
    if (!success) {
      RCLCPP_WARN(logger, "Error while setting command for joint %zu.", joint_ind);
    }
  }
  last_commanded_ = state_commanded;
}

void AdmittanceController::read_state_reference_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & state_reference)
{
  // TODO(destogl): check why is this here?

  // if any interface has nan values, assume state_reference is the last set reference
  for (size_t i = 0; i < num_joints_; ++i) {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces) {
      // update position
      if (interface == hardware_interface::HW_IF_POSITION) {
        if (std::isnan(position_reference_[i])) {
          position_reference_[i].get() = last_reference_.positions[i];
        }
        state_reference.positions[i] = position_reference_[i];
      }

      // update velocity
      if (interface == hardware_interface::HW_IF_VELOCITY) {
        if (std::isnan(velocity_reference_[i])) {
          velocity_reference_[i].get() = last_reference_.velocities[i];
        }
        state_reference.velocities[i] = velocity_reference_[i];
      }
    }
  }

  last_reference_.positions = state_reference.positions;
  last_reference_.velocities = state_reference.velocities;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController, controller_interface::ChainableControllerInterface)
