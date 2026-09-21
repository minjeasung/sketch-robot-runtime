/**
* Copyright (c) 2024 Rainbow Robotics
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

#include "rbpodo_hardware/rbpodo_hardware_interface.hpp"

#include "rbpodo_hardware/hardware_safety.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

namespace {
template <typename ArrayLike>
bool isValidCommand(const ArrayLike& arr) {
  return std::all_of(arr.begin(), arr.end(), [](double value) { return std::isfinite(value) && !std::isnan(value); });
}

constexpr int kPostFreedriveHoldCycles{200};  // 2 s at the 100 Hz controller rate
constexpr double kMaxRobotStateAgeS{0.1};
constexpr int64_t kMaxLatestSampleAgeNs{100000000};  // same 0.1 s freshness bound

int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

template <typename Container>
void append_json_array(std::ostringstream& stream, const Container& values) {
  stream << '[';
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      stream << ',';
    }
    first = false;
    stream << value;
  }
  stream << ']';
}

template <typename Value, std::size_t Size>
void append_json_array(std::ostringstream& stream, const Value (&values)[Size]) {
  stream << '[';
  for (std::size_t index = 0; index < Size; ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << values[index];
  }
  stream << ']';
}

std::string make_interlock_snapshot(
    const rb::podo::SystemState& state,
    const std::array<double, 6>& ros_position_command_rad,
    uint32_t severe_reason_mask, bool severe_safety_abort,
    bool robot_safety_interlocks_clear, double read_period_ms,
    bool trajectory_active, bool force_enabled,
    bool compliance_enabled, bool compliance_active) {
  const auto& data = state.sdata;
  std::array<double, 6> sdk_tracking_error_deg{};
  for (std::size_t index = 0; index < sdk_tracking_error_deg.size(); ++index) {
    sdk_tracking_error_deg[index] = data.jnt_ref[index] - data.jnt_ang[index];
  }
  const std::array<double, 6> eft{
      data.eft_fx, data.eft_fy, data.eft_fz,
      data.eft_mx, data.eft_my, data.eft_mz};

  std::ostringstream stream;
  stream.precision(9);
  stream << '{'
         << "\"schema\":1,"
         << "\"trigger_kind\":\""
         << (severe_safety_abort ? "severe_interlock" : "ready_state_loss")
         << "\","
         << "\"reason_mask\":" << severe_reason_mask << ','
         << "\"ready_predicate\":"
         << (robot_safety_interlocks_clear ? "true" : "false") << ','
         << "\"read_period_ms\":" << read_period_ms << ','
         << "\"controller_time_s\":" << data.time << ','
         << "\"robot_state\":" << data.robot_state << ','
         << "\"task_state\":" << data.task_state << ','
         << "\"collision_status_raw\":" << data.op_stat_collision_occur << ','
         << "\"self_collision_status_raw\":" << data.op_stat_self_collision << ','
         << "\"sos_flag_raw\":" << data.op_stat_sos_flag << ','
         << "\"soft_estop_raw\":" << data.op_stat_soft_estop_occur << ','
         << "\"ems_flag_raw\":" << data.op_stat_ems_flag << ','
         << "\"init_state_raw\":" << data.init_state_info << ','
         << "\"init_error_raw\":" << data.init_error << ','
         << "\"freedrive_raw\":" << data.is_freedrive_mode << ','
         << "\"information_chunk_1\":"
         << static_cast<uint32_t>(data.information_chunk_1) << ','
         << "\"information_chunk_2\":"
         << static_cast<uint32_t>(data.information_chunk_2) << ','
         << "\"information_chunk_3\":"
         << static_cast<uint32_t>(data.information_chunk_3) << ','
         << "\"information_chunk_4\":"
         << static_cast<uint32_t>(data.information_chunk_4) << ','
         << "\"safety_board_stat_info\":"
         << static_cast<uint32_t>(data.safety_board_stat_info) << ','
         << "\"trajectory_active\":" << (trajectory_active ? "true" : "false") << ','
         << "\"force_enabled\":" << (force_enabled ? "true" : "false") << ','
         << "\"compliance_enabled\":" << (compliance_enabled ? "true" : "false") << ','
         << "\"compliance_active\":" << (compliance_active ? "true" : "false") << ',';

  stream << "\"jnt_ref_deg\":";
  append_json_array(stream, data.jnt_ref);
  stream << ",\"jnt_ang_deg\":";
  append_json_array(stream, data.jnt_ang);
  stream << ",\"sdk_ref_minus_ang_deg\":";
  append_json_array(stream, sdk_tracking_error_deg);
  stream << ",\"ros_position_command_rad\":";
  append_json_array(stream, ros_position_command_rad);
  stream << ",\"jnt_current_a\":";
  append_json_array(stream, data.jnt_cur);
  stream << ",\"jnt_temperature_c\":";
  append_json_array(stream, data.jnt_temperature);
  stream << ",\"jnt_info_raw\":";
  append_json_array(stream, data.jnt_info);
  stream << ",\"tcp_ref_sdk_mm_deg\":";
  append_json_array(stream, data.tcp_ref);
  stream << ",\"tcp_pos_sdk_mm_deg\":";
  append_json_array(stream, data.tcp_pos);
  stream << ",\"eft_sdk_raw\":";
  append_json_array(stream, eft);
  stream << '}';
  return stream.str();
}

class AtomicBoolReset {
 public:
  explicit AtomicBoolReset(std::atomic<bool>& value) : value_(value) {}
  ~AtomicBoolReset() { value_.store(false, std::memory_order_release); }

  AtomicBoolReset(const AtomicBoolReset&) = delete;
  AtomicBoolReset& operator=(const AtomicBoolReset&) = delete;

 private:
  std::atomic<bool>& value_;
};
}  // namespace

namespace rbpodo_hardware {

RBPodoHardwareInterface::RBPodoHardwareInterface() {
  // Joint Position Controller Interface
  CommandInterfaceInfo jpc_info(
      {hardware_interface::HW_IF_POSITION, RBPodoHardwareInterface::kNumberOfJoints, joint_position_interface_state_});
  command_interface_infos_.push_back(jpc_info);

  // Joint Speed Controller Interface
  CommandInterfaceInfo jvc_info(
      {hardware_interface::HW_IF_VELOCITY, RBPodoHardwareInterface::kNumberOfJoints, joint_velocity_interface_state_});
  command_interface_infos_.push_back(jvc_info);

  // Joint Effort Controller Interface
  CommandInterfaceInfo jec_info(
      {hardware_interface::HW_IF_EFFORT, RBPodoHardwareInterface::kNumberOfJoints, joint_effort_interface_state_});
  command_interface_infos_.push_back(jec_info);

  // Cartesian Pose Controller Interface
  CommandInterfaceInfo cpc_info({HW_IF_CARTESIAN_POSE, k6DoFDim, cartesian_pose_interface_state_});
  command_interface_infos_.push_back(cpc_info);

  // Cartesian Velocity Controller Interface
  CommandInterfaceInfo cvc_info({HW_IF_CARTESIAN_VELOCITY, k6DoFDim, cartesian_velocity_interface_state_});
  command_interface_infos_.push_back(cvc_info);

  // Zero is a valid wrench; before activation/tare there is no valid sample.
  invalidate_ft_states(hw_ft_states_, hw_ft_raw_states_);
}

RBPodoHardwareInterface::~RBPodoHardwareInterface() = default;

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(), kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  // Need to check feasibility

  for (size_t i = 0; i < info_.joints.size(); i++) {
    torque_constants_[i] = atof(info_.joints[i].parameters.at("torque_constant").c_str());
  }

  // Two independent fake switches (forwarded by the URDF from launch args):
  //   fake_mode            - joint motion is faked (no writes to the cobot;
  //                          joint states mirror commands).
  //   fake_sensor_commands - FT signal comes from the ~/inject_ft topic
  //                          instead of the real eft_* stream.
  // The robot is connected whenever EITHER real joint motion or real FT is
  // required, i.e. unless both switches are true.
  try {
    const auto& v = info_.hardware_parameters.at("fake_mode");
    fake_mode_ = (v == "true" || v == "True" || v == "1");
  } catch (const std::out_of_range&) {
    fake_mode_ = false;
  }
  try {
    const auto& v = info_.hardware_parameters.at("fake_sensor_commands");
    inject_ft_enabled_ = (v == "true" || v == "True" || v == "1");
  } catch (const std::out_of_range&) {
    inject_ft_enabled_ = false;
  }
  // human_collab: clamps hw_ft_states_ to [-kFtCollabClamp, +kFtCollabClamp]
  // per axis after bias subtraction, so admittance is bounded to a safe
  // interaction wrench when humans share the workspace. Applies regardless of
  // the data source (real eft_* or injected).
  try {
    const auto& v = info_.hardware_parameters.at("human_collab");
    human_collab_ = (v == "true" || v == "True" || v == "1");
  } catch (const std::out_of_range&) {
    human_collab_ = false;
  }
  if (human_collab_) {
    RCLCPP_INFO(getLogger(),
                "human_collab=true: hw_ft_states_ clamped to +/-%.1f per axis.",
                kFtCollabClamp);
  }

  const bool need_robot = !fake_mode_ || !inject_ft_enabled_;

  if (need_robot && !robot_) {
    std::string robot_ip;
    bool cb_simulation;
    try {
      robot_ip = info_.hardware_parameters.at("robot_ip");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' is not set");
      return CallbackReturn::ERROR;
    }
    try {
      cb_simulation = (info_.hardware_parameters.at("cb_simulation") == "True");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'cb_simulation' is not set");
      return CallbackReturn::ERROR;
    }

    try {
      RCLCPP_INFO(getLogger(),
                  "Connecting to robot at \"%s\" (mode: %s, purpose: %s)...",
                  robot_ip.c_str(), (cb_simulation ? "Simulation" : "Real"),
                  fake_mode_ ? "FT telemetry only" : "motion+telemetry");
      robot_ = std::make_shared<Robot>(robot_ip, cb_simulation, getLogger());
    } catch (const std::exception& e) {
      RCLCPP_FATAL(getLogger(), "Could not connect to robot");
      RCLCPP_FATAL(getLogger(), " - what(): %s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  }

  robot_executor_ = std::make_shared<RobotExecutor>();
  if (!fake_mode_) {
    robot_node_ = std::make_shared<RobotNode>(rclcpp::NodeOptions(), robot_);
    robot_executor_->add_node(robot_node_);
    RCLCPP_INFO(getLogger(), "Robot node start ...");
  } else if (inject_ft_enabled_) {
    RCLCPP_INFO(getLogger(),
                "fake_mode=true, fake_sensor_commands=true: no robot connection. "
                "Joints mirror commands; FT comes from ~/inject_ft.");
  } else {
    RCLCPP_INFO(getLogger(),
                "fake_mode=true, fake_sensor_commands=false: robot connected for "
                "FT telemetry only. Joints mirror commands; FT uses real eft_*.");
  }

  // Re-tare service. Triggers the F/T zero procedure from the current pose;
  // blocks (up to 5 s) until read() reports Ok/Failed so the caller knows the
  // outcome. The service node piggybacks on robot_executor_ so it spins in the
  // existing background thread without us managing a new executor.
  tare_node_ = std::make_shared<rclcpp::Node>("rbpodo_ft_tare");

  const auto as_parameter_vector = [](const HardwareWrench& values) {
    return std::vector<double>(values.begin(), values.end());
  };
  rcl_interfaces::msg::ParameterDescriptor activation_tare_parameter_descriptor;
  activation_tare_parameter_descriptor.read_only = true;
  activation_tare_parameter_descriptor.description =
      "Run the legacy strict F/T tare automatically on hardware activation; "
      "set false when execution performs an explicit pre-contact runtime tare";
  tare_node_->declare_parameter<bool>(
      "auto_tare_on_activate", true, activation_tare_parameter_descriptor);

  rcl_interfaces::msg::ParameterDescriptor supervised_parameter_descriptor;
  supervised_parameter_descriptor.read_only = true;
  supervised_parameter_descriptor.description =
      "Immutable installation-specific supervised F/T tare profile; configure only at node startup";
  tare_node_->declare_parameter<bool>(
      "supervised_tare.enabled", false, supervised_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "supervised_tare.expected_bias", as_parameter_vector(kDefaultSupervisedExpectedBias),
      supervised_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "supervised_tare.max_abs_bias_delta",
      as_parameter_vector(kDefaultSupervisedMaxAbsBiasDelta), supervised_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "supervised_tare.max_stddev", as_parameter_vector(kDefaultSupervisedMaxStddev),
      supervised_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "supervised_tare.expected_joint_positions",
      as_parameter_vector(kDefaultSupervisedExpectedJointPositions),
      supervised_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "supervised_tare.max_abs_joint_position_delta_rad",
      as_parameter_vector(kDefaultSupervisedMaxAbsJointPositionDeltaRad),
      supervised_parameter_descriptor);
  tare_node_->declare_parameter<int64_t>(
      "supervised_tare.min_recent_joint_samples",
      static_cast<int64_t>(kDefaultSupervisedRecentJointSamples),
      supervised_parameter_descriptor);
  tare_node_->declare_parameter<double>(
      "supervised_tare.max_recent_joint_excursion_rad",
      kDefaultSupervisedMaxRecentJointExcursionRad, supervised_parameter_descriptor);
  tare_node_->declare_parameter<double>(
      "supervised_tare.max_tare_joint_excursion_rad",
      kDefaultSupervisedMaxTareJointExcursionRad, supervised_parameter_descriptor);

  rcl_interfaces::msg::ParameterDescriptor runtime_parameter_descriptor;
  runtime_parameter_descriptor.read_only = true;
  runtime_parameter_descriptor.description =
      "Execution-scoped free-space tare limits; no fixed raw bias or joint pose is used";
  tare_node_->declare_parameter<std::vector<double>>(
      "runtime_tare.max_stddev", as_parameter_vector(kDefaultRuntimeMaxStddev),
      runtime_parameter_descriptor);
  tare_node_->declare_parameter<std::vector<double>>(
      "runtime_tare.max_abs_post_tare_residual",
      as_parameter_vector(kDefaultRuntimeMaxAbsPostTareResidual),
      runtime_parameter_descriptor);
  tare_node_->declare_parameter<int64_t>(
      "runtime_tare.min_recent_joint_samples",
      static_cast<int64_t>(kDefaultRuntimeRecentJointSamples),
      runtime_parameter_descriptor);
  tare_node_->declare_parameter<double>(
      "runtime_tare.max_recent_joint_excursion_rad",
      kDefaultRuntimeMaxRecentJointExcursionRad, runtime_parameter_descriptor);
  tare_node_->declare_parameter<double>(
      "runtime_tare.max_tare_joint_excursion_rad",
      kDefaultRuntimeMaxTareJointExcursionRad, runtime_parameter_descriptor);

  const auto record_control_status =
      [this](const std_msgs::msg::Bool::SharedPtr msg,
             std::atomic<bool>& value, std::atomic<int64_t>& timestamp) {
        value.store(msg->data, std::memory_order_release);
        timestamp.store(steady_now_ns(), std::memory_order_release);
        if (msg->data) {
          runtime_control_violation_generation_.fetch_add(1, std::memory_order_acq_rel);
        }
      };
  // The tare services below synchronously wait for the RT read loop.  ROS 2's
  // default callback group is MutuallyExclusive, so putting these heartbeat
  // subscriptions in that same group would stop their callbacks for the full
  // 100+25 sample runtime-tare window and make the 250 ms freshness watchdog
  // reject every otherwise healthy request.  A separate group lets the
  // MultiThreaded RobotExecutor keep processing all safety heartbeats.
  runtime_control_callback_group_ = tare_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions runtime_control_subscription_options;
  runtime_control_subscription_options.callback_group =
      runtime_control_callback_group_;
  trajectory_active_sub_ = tare_node_->create_subscription<std_msgs::msg::Bool>(
      "/painting_admittance/trajectory_active", rclcpp::SystemDefaultsQoS(),
      [this, record_control_status](const std_msgs::msg::Bool::SharedPtr msg) {
        record_control_status(msg, latest_trajectory_active_, latest_trajectory_status_ns_);
      },
      runtime_control_subscription_options);
  force_enable_sub_ = tare_node_->create_subscription<std_msgs::msg::Bool>(
      "/painting_admittance/enable_force", rclcpp::SystemDefaultsQoS(),
      [this, record_control_status](const std_msgs::msg::Bool::SharedPtr msg) {
        record_control_status(msg, latest_force_enabled_, latest_force_enable_status_ns_);
      },
      runtime_control_subscription_options);
  compliance_enable_sub_ = tare_node_->create_subscription<std_msgs::msg::Bool>(
      "/admittance_controller/compliance_enable", rclcpp::SystemDefaultsQoS(),
      [this, record_control_status](const std_msgs::msg::Bool::SharedPtr msg) {
        record_control_status(
            msg, latest_compliance_enabled_, latest_compliance_enable_status_ns_);
      },
      runtime_control_subscription_options);
  compliance_active_sub_ = tare_node_->create_subscription<std_msgs::msg::Bool>(
      "/admittance_controller/compliance_active", rclcpp::SystemDefaultsQoS(),
      [this, record_control_status](const std_msgs::msg::Bool::SharedPtr msg) {
        record_control_status(msg, latest_compliance_active_, latest_compliance_status_ns_);
      },
      runtime_control_subscription_options);

  motion_inhibit_status_pub_ = tare_node_->create_publisher<std_msgs::msg::Bool>(
      "/painting_system/hardware_motion_inhibited",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  motion_abort_sub_ = tare_node_->create_subscription<std_msgs::msg::Bool>(
      "/motion_abort", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
          return;
        }
        bool expected = false;
        if (motion_inhibit_latched_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
          motion_stop_requested_.store(true, std::memory_order_release);
          RCLCPP_ERROR(
              getLogger(),
              "motion abort latched in hardware; all servo writes are inhibited "
              "until the ros2_control process is fully relaunched");
        }
      },
      runtime_control_subscription_options);
  motion_inhibit_status_timer_ = tare_node_->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        std_msgs::msg::Bool status;
        status.data =
            motion_inhibit_acknowledged_.load(std::memory_order_acquire);
        motion_inhibit_status_pub_->publish(status);
      },
      runtime_control_callback_group_);

  // Debug/test injection: only wired up when the launch sets
  // fake_sensor_commands:=true. Publish a 6-element Float64MultiArray to
  // /rbpodo_ft_tare/inject_ft to OVERRIDE the raw eft_* feed for the next
  // read() cycles. Send an empty array to revert to whatever raw_ft would
  // otherwise be (real eft_* or zero in fake_mode without robot).
  if (inject_ft_enabled_) {
    ft_inject_sub_ = tare_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/inject_ft", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(ft_inject_mutex_);
          if (msg->data.empty()) {
            ft_inject_active_.store(false, std::memory_order_release);
            RCLCPP_INFO(getLogger(), "F/T injection cleared - reverting to raw signal");
            return;
          }
          if (msg->data.size() != k6DoFDim) {
            RCLCPP_WARN(getLogger(),
                        "F/T injection ignored: expected %zu values (or 0 to clear), got %zu",
                        k6DoFDim, msg->data.size());
            return;
          }
          for (size_t i = 0; i < k6DoFDim; ++i) {
            ft_inject_values_[i] = msg->data[i];
          }
          if (!ft_inject_active_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(getLogger(),
                        "F/T injection ENABLED. raw_ft is overridden until cleared "
                        "(publish an empty array to ~/inject_ft to revert).");
          }
          ft_inject_active_.store(true, std::memory_order_release);
        });
    RCLCPP_INFO(getLogger(),
                "fake_sensor_commands=true: ~/inject_ft subscription ready.");
  }

  tare_srv_ = tare_node_->create_service<std_srvs::srv::Trigger>(
      "~/tare_ft",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool service_available = false;
        if (!ft_tare_service_in_progress_.compare_exchange_strong(
                service_available, true, std::memory_order_acq_rel)) {
          resp->success = false;
          resp->message = "another F/T tare service request is already in progress";
          return;
        }
        AtomicBoolReset release_service(ft_tare_service_in_progress_);
        if (supervised_tare_cancel_requested_.load(std::memory_order_acquire) ||
            runtime_tare_cancel_requested_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "a timed-out F/T tare is still awaiting RT cancellation acknowledgement";
          return;
        }
        if (ft_tare_state_.load() == TareState::Running ||
            ft_tare_requested_mode_.load() != TareMode::None) {
          resp->success = false;
          resp->message = "F/T tare already running; wait for it to finish";
          return;
        }
        // Reset state first so we don't observe a stale Ok/Failed from the previous run.
        strict_tare_outcome_.store(StrictTareOutcome::NotCompleted);
        ft_tare_state_.store(TareState::Idle);
        ft_tare_requested_mode_.store(TareMode::Strict, std::memory_order_release);

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
          const auto state = ft_tare_state_.load();
          if (state == TareState::Ok) {
            resp->success = true;
            resp->message = "F/T tare succeeded";
            return;
          }
          if (state == TareState::Failed) {
            resp->success = false;
            resp->message = "F/T tare failed: excessive load or noise at rest "
                            "(check the arm is free and undisturbed)";
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        resp->success = false;
        resp->message = "F/T tare timed out (read() not running?)";
      });

  // Explicit operator-confirmed free-space path for this installation's
  // known high raw baseline.  The call is the confirmation; read() still
  // refuses the bias unless transport, finiteness, pre-request joint
  // stationarity, in-window stationarity, low noise, and the commissioned
  // baseline envelope all pass.  This service does not weaken ~/tare_ft or
  // the automatic activation tare.
  supervised_tare_srv_ = tare_node_->create_service<std_srvs::srv::Trigger>(
      "~/confirm_free_space_and_tare",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool service_available = false;
        if (!ft_tare_service_in_progress_.compare_exchange_strong(
                service_available, true, std::memory_order_acq_rel)) {
          resp->success = false;
          resp->message = "another F/T tare service request is already in progress";
          return;
        }
        AtomicBoolReset release_service(ft_tare_service_in_progress_);
        if (supervised_tare_cancel_requested_.load(std::memory_order_acquire) ||
            runtime_tare_cancel_requested_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "a timed-out F/T tare is still awaiting RT cancellation acknowledgement";
          return;
        }
        if (ft_tare_state_.load() == TareState::Running ||
            ft_tare_requested_mode_.load() != TareMode::None) {
          resp->success = false;
          resp->message = "F/T tare already running; wait for it to finish before confirming free space";
          return;
        }
        bool supervised_enabled = false;
        tare_node_->get_parameter("supervised_tare.enabled", supervised_enabled);
        if (!supervised_enabled) {
          resp->success = false;
          resp->message = "supervised F/T tare is disabled: load an immutable commissioned installation "
                          "profile at node startup; existing tare state was not changed";
          return;
        }
        if (ft_tare_state_.load() != TareState::Failed ||
            !supervised_tare_may_follow(strict_tare_outcome_.load())) {
          resp->success = false;
          resp->message = "supervised F/T tare is allowed only immediately after a strict tare failed "
                          "solely because its stable mean exceeded the strict envelope; existing bias preserved";
          return;
        }
        if ((!fake_mode_ && inject_ft_enabled_) ||
            (!fake_mode_ && ft_inject_active_.load(std::memory_order_acquire))) {
          resp->success = false;
          resp->message = "supervised F/T tare rejected: injected/fake F/T provenance is forbidden "
                          "while real joint motion is enabled";
          return;
        }
        const int64_t latest_fresh_ns = latest_fresh_sample_ns_.load(std::memory_order_acquire);
        const int64_t sample_age_ns = latest_fresh_ns > 0 ? steady_now_ns() - latest_fresh_ns
                                                          : std::numeric_limits<int64_t>::max();
        if (!latest_ft_source_valid_.load(std::memory_order_acquire) ||
            !latest_joint_source_valid_.load(std::memory_order_acquire) ||
            sample_age_ns < 0 || sample_age_ns > kMaxLatestSampleAgeNs) {
          resp->success = false;
          resp->message = "supervised F/T tare rejected before entry: latest F/T/joint sample is stale or invalid";
          return;
        }
        if (!latest_robot_safety_clear_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "supervised F/T tare rejected before entry: robot initialization/collision/estop "
                          "safety state is not clear";
          return;
        }
        if (freedrive_on_.load(std::memory_order_acquire) || freedrive_request_.load() != 0 ||
            post_freedrive_hold_cycles_.load(std::memory_order_acquire) > 0) {
          resp->success = false;
          resp->message = "supervised F/T tare rejected: free-drive or post-free-drive hold is active";
          return;
        }
        bool not_attempted = false;
        if (!supervised_tare_attempted_this_activation_.compare_exchange_strong(
                not_attempted, true, std::memory_order_acq_rel)) {
          resp->success = false;
          resp->message = "supervised F/T tare is limited to one confirmed attempt per hardware activation; "
                          "restart Terminal 1 before another attempt";
          return;
        }

        SupervisedTareConfig config;
        config.enabled = supervised_enabled;
        const auto copy_wrench_parameter = [this](const char* name, HardwareWrench& destination) {
          std::vector<double> values;
          if (!tare_node_->get_parameter(name, values) || values.size() != destination.size()) {
            destination.fill(std::numeric_limits<double>::quiet_NaN());
            return;
          }
          std::copy(values.begin(), values.end(), destination.begin());
        };
        copy_wrench_parameter("supervised_tare.expected_bias", config.expected_bias);
        copy_wrench_parameter("supervised_tare.max_abs_bias_delta", config.max_abs_bias_delta);
        copy_wrench_parameter("supervised_tare.max_stddev", config.max_stddev);
        const auto copy_joint_parameter =
            [this](const char* name, HardwareJointPositions& destination) {
              std::vector<double> values;
              if (!tare_node_->get_parameter(name, values) ||
                  values.size() != destination.size()) {
                destination.fill(std::numeric_limits<double>::quiet_NaN());
                return;
              }
              std::copy(values.begin(), values.end(), destination.begin());
            };
        copy_joint_parameter("supervised_tare.expected_joint_positions",
                             config.expected_joint_positions);
        copy_joint_parameter("supervised_tare.max_abs_joint_position_delta_rad",
                             config.max_abs_joint_position_delta_rad);

        int64_t recent_samples = 0;
        tare_node_->get_parameter("supervised_tare.min_recent_joint_samples", recent_samples);
        config.min_recent_joint_samples =
            recent_samples > 0 && recent_samples <= static_cast<int64_t>(kJointHistoryCapacity)
                ? static_cast<size_t>(recent_samples)
                : 0;
        tare_node_->get_parameter("supervised_tare.max_recent_joint_excursion_rad",
                                  config.max_recent_joint_excursion_rad);
        tare_node_->get_parameter("supervised_tare.max_tare_joint_excursion_rad",
                                  config.max_tare_joint_excursion_rad);

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        supervised_tare_pending_config_ = config;
        supervised_tare_pending_deadline_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.time_since_epoch()).count(),
            std::memory_order_relaxed);
        supervised_tare_rejection_.store(SupervisedTareRejection::None);
        supervised_tare_cancel_requested_.store(false);
        // Idle invalidates exported F/T immediately on the next read; the RT
        // thread also clears the previous bias before collecting samples.
        ft_tare_state_.store(TareState::Idle);
        ft_tare_requested_mode_.store(TareMode::Supervised, std::memory_order_release);

        while (std::chrono::steady_clock::now() < deadline) {
          const auto state = ft_tare_state_.load();
          if (state == TareState::Ok) {
            resp->success = true;
            resp->message = "supervised F/T tare succeeded after free-space, freshness, "
                            "stationarity, commissioned-pose, noise, and baseline-envelope checks";
            return;
          }
          if (state == TareState::Failed) {
            resp->success = false;
            resp->message = std::string("supervised F/T tare rejected: ") +
                            supervised_tare_rejection_message(supervised_tare_rejection_.load()) +
                            "; bias remains zero and exported F/T remains NaN";
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        supervised_tare_cancel_requested_.store(true, std::memory_order_release);
        resp->success = false;
        resp->message = "supervised F/T tare timed out and was cancelled (read() not running?); "
                        "bias remains invalid — do not enable compliance";
      });

  // Repeatable execution-scoped tare used after the nominal trajectory has
  // reached the non-contact pre-contact pose.  The Trigger call itself is the
  // explicit free-space assertion.  Unlike the legacy supervised service it
  // neither requires a failed strict tare nor compares the raw offset with a
  // fixed commissioned baseline.  It does require fresh false heartbeats for
  // trajectory, force output, compliance-enable command, and active
  // compliance before and throughout both the bias and post-bias validation
  // windows.
  runtime_tare_srv_ = tare_node_->create_service<std_srvs::srv::Trigger>(
      "~/runtime_free_space_tare",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool service_available = false;
        if (!ft_tare_service_in_progress_.compare_exchange_strong(
                service_available, true, std::memory_order_acq_rel)) {
          resp->success = false;
          resp->message = "another F/T tare service request is already in progress";
          return;
        }
        AtomicBoolReset release_service(ft_tare_service_in_progress_);
        if (supervised_tare_cancel_requested_.load(std::memory_order_acquire) ||
            runtime_tare_cancel_requested_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "a timed-out F/T tare is still awaiting RT cancellation acknowledgement";
          return;
        }
        if (ft_tare_state_.load() == TareState::Running ||
            ft_tare_requested_mode_.load() != TareMode::None) {
          resp->success = false;
          resp->message = "F/T tare already running; wait for it to finish";
          return;
        }
        if ((!fake_mode_ && inject_ft_enabled_) ||
            (!fake_mode_ && ft_inject_active_.load(std::memory_order_acquire))) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected: injected/fake F/T provenance is forbidden "
                          "while real joint motion is enabled";
          return;
        }

        // Bracket the non-atomic multi-topic snapshot with a monotonic
        // violation generation.  Any true pulse racing this validation must
        // reject the request even if a following false heartbeat arrives
        // before the individual values are loaded.
        const uint64_t entry_control_violation_generation =
            runtime_control_violation_generation_.load(std::memory_order_acquire);
        const int64_t latest_fresh_ns =
            latest_fresh_sample_ns_.load(std::memory_order_acquire);
        const int64_t trajectory_status_ns =
            latest_trajectory_status_ns_.load(std::memory_order_acquire);
        const bool trajectory_active =
            latest_trajectory_active_.load(std::memory_order_acquire);
        const int64_t force_enable_status_ns =
            latest_force_enable_status_ns_.load(std::memory_order_acquire);
        const bool force_enabled =
            latest_force_enabled_.load(std::memory_order_acquire);
        const int64_t compliance_status_ns =
            latest_compliance_status_ns_.load(std::memory_order_acquire);
        const bool compliance_active =
            latest_compliance_active_.load(std::memory_order_acquire);
        const int64_t compliance_enable_status_ns =
            latest_compliance_enable_status_ns_.load(std::memory_order_acquire);
        const bool compliance_enabled =
            latest_compliance_enabled_.load(std::memory_order_acquire);
        // Take the comparison time after all timestamp snapshots.  A heartbeat
        // callback may run concurrently in its separate callback group; taking
        // `now` first could make a just-arrived timestamp appear to be in the
        // future and cause a nondeterministic false stale rejection.
        const int64_t now_ns = steady_now_ns();
        const int64_t sample_age_ns =
            latest_fresh_ns > 0 ? now_ns - latest_fresh_ns
                                : std::numeric_limits<int64_t>::max();
        if (!latest_ft_source_valid_.load(std::memory_order_acquire) ||
            !latest_joint_source_valid_.load(std::memory_order_acquire) ||
            sample_age_ns < 0 || sample_age_ns > kMaxLatestSampleAgeNs) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected before entry: latest F/T/joint sample is stale or invalid";
          return;
        }
        if (!latest_robot_safety_clear_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected before entry: robot initialization/collision/estop "
                          "safety state is not clear";
          return;
        }
        if (freedrive_on_.load(std::memory_order_acquire) ||
            freedrive_request_.load() != 0 ||
            post_freedrive_hold_cycles_.load(std::memory_order_acquire) > 0) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected: free-drive or post-free-drive hold is active";
          return;
        }

        const auto status_is_fresh = [now_ns](int64_t status_ns) {
          const int64_t age_ns = status_ns > 0 ? now_ns - status_ns
                                               : std::numeric_limits<int64_t>::max();
          return age_ns >= 0 && age_ns <= kRuntimeControlStatusMaxAgeNs;
        };
        RuntimeTareControlState control_state;
        control_state.trajectory_status_fresh =
            status_is_fresh(trajectory_status_ns);
        control_state.trajectory_active = trajectory_active;
        control_state.force_enable_status_fresh =
            status_is_fresh(force_enable_status_ns);
        control_state.force_enabled = force_enabled;
        control_state.compliance_enable_status_fresh =
            status_is_fresh(compliance_enable_status_ns);
        control_state.compliance_enabled = compliance_enabled;
        control_state.compliance_status_fresh =
            status_is_fresh(compliance_status_ns);
        control_state.compliance_active = compliance_active;
        const auto control_rejection = validate_runtime_tare_control_state(control_state);
        if (control_rejection != RuntimeTareRejection::None) {
          resp->success = false;
          resp->message = std::string("runtime F/T tare rejected before entry: ") +
                          runtime_tare_rejection_message(control_rejection);
          return;
        }
        if (runtime_control_violation_generation_.load(std::memory_order_acquire) !=
            entry_control_violation_generation) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected before entry: a trajectory, force, or "
                          "compliance interlock changed while the request was validated";
          return;
        }

        RuntimeTareConfig config;
        const auto copy_wrench_parameter =
            [this](const char* name, HardwareWrench& destination) {
              std::vector<double> values;
              if (!tare_node_->get_parameter(name, values) ||
                  values.size() != destination.size()) {
                destination.fill(std::numeric_limits<double>::quiet_NaN());
                return;
              }
              std::copy(values.begin(), values.end(), destination.begin());
            };
        copy_wrench_parameter("runtime_tare.max_stddev", config.max_stddev);
        copy_wrench_parameter("runtime_tare.max_abs_post_tare_residual",
                              config.max_abs_post_tare_residual);
        int64_t recent_samples = 0;
        tare_node_->get_parameter("runtime_tare.min_recent_joint_samples", recent_samples);
        config.min_recent_joint_samples =
            recent_samples > 0 && recent_samples <= static_cast<int64_t>(kJointHistoryCapacity)
                ? static_cast<size_t>(recent_samples)
                : 0;
        tare_node_->get_parameter("runtime_tare.max_recent_joint_excursion_rad",
                                  config.max_recent_joint_excursion_rad);
        tare_node_->get_parameter("runtime_tare.max_tare_joint_excursion_rad",
                                  config.max_tare_joint_excursion_rad);
        if (!runtime_tare_config_is_valid(config)) {
          resp->success = false;
          resp->message = "runtime F/T tare rejected: invalid immutable runtime_tare parameters";
          return;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        runtime_tare_pending_config_ = config;
        runtime_tare_pending_control_violation_generation_ =
            entry_control_violation_generation;
        runtime_tare_pending_deadline_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                deadline.time_since_epoch()).count(),
            std::memory_order_relaxed);
        runtime_tare_rejection_.store(RuntimeTareRejection::None);
        runtime_tare_cancel_requested_.store(false);
        // Invalidate the previous run's bias immediately.  A failed attempt
        // must never silently fall back to an older pose's zero.
        ft_tare_state_.store(TareState::Idle);
        ft_tare_requested_mode_.store(TareMode::Runtime, std::memory_order_release);

        while (std::chrono::steady_clock::now() < deadline) {
          const auto state = ft_tare_state_.load();
          if (state == TareState::Ok) {
            std::ostringstream message;
            message << "runtime free-space F/T tare succeeded; bias F=["
                    << ft_bias_[0] << " " << ft_bias_[1] << " " << ft_bias_[2]
                    << "] N T=[" << ft_bias_[3] << " " << ft_bias_[4] << " "
                    << ft_bias_[5]
                    << "] Nm; post-tare residual validated";
            resp->success = true;
            resp->message = message.str();
            return;
          }
          if (state == TareState::Failed) {
            resp->success = false;
            resp->message = std::string("runtime F/T tare rejected: ") +
                            runtime_tare_rejection_message(runtime_tare_rejection_.load()) +
                            "; bias invalid and exported F/T remains NaN";
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        runtime_tare_cancel_requested_.store(true, std::memory_order_release);
        resp->success = false;
        resp->message = "runtime F/T tare timed out and was cancelled (read() not running?); "
                        "bias remains invalid and compliance must stay disabled";
      });

  // Free-drive (direct-teaching) toggle. The callback only records the request;
  // write() (RT thread) issues the actual cobot command and suspends move_servo_j
  // so all cobot socket access stays single-threaded.
  freedrive_srv_ = tare_node_->create_service<std_srvs::srv::SetBool>(
      "~/set_freedrive",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
        if (fake_mode_ || !robot_) {
          resp->success = false;
          resp->message = "no real robot connection";
          return;
        }
        const bool want = req->data;
        if (want) {
          if (motion_inhibit_latched_.load(std::memory_order_acquire)) {
            resp->success = false;
            resp->message =
                "free-drive ON rejected: hardware motion inhibit is latched; "
                "fully relaunch ros2_control";
            return;
          }
          if (!latest_robot_safety_clear_.load(std::memory_order_acquire)) {
            resp->success = false;
            resp->message =
                "free-drive ON rejected: robot safety/startup state is not clear";
            return;
          }

          const uint64_t control_generation_before =
              runtime_control_violation_generation_.load(std::memory_order_acquire);
          const int64_t trajectory_status_ns =
              latest_trajectory_status_ns_.load(std::memory_order_acquire);
          const int64_t force_status_ns =
              latest_force_enable_status_ns_.load(std::memory_order_acquire);
          const int64_t compliance_enable_status_ns =
              latest_compliance_enable_status_ns_.load(std::memory_order_acquire);
          const int64_t compliance_status_ns =
              latest_compliance_status_ns_.load(std::memory_order_acquire);
          const int64_t now_ns = steady_now_ns();
          const auto is_fresh = [now_ns](int64_t stamp_ns) {
            const int64_t age_ns =
                stamp_ns > 0 ? now_ns - stamp_ns
                             : std::numeric_limits<int64_t>::max();
            return age_ns >= 0 && age_ns <= kRuntimeControlStatusMaxAgeNs;
          };
          RuntimeTareControlState controls;
          controls.trajectory_status_fresh = is_fresh(trajectory_status_ns);
          controls.trajectory_active =
              latest_trajectory_active_.load(std::memory_order_acquire);
          controls.force_enable_status_fresh = is_fresh(force_status_ns);
          controls.force_enabled =
              latest_force_enabled_.load(std::memory_order_acquire);
          controls.compliance_enable_status_fresh =
              is_fresh(compliance_enable_status_ns);
          controls.compliance_enabled =
              latest_compliance_enabled_.load(std::memory_order_acquire);
          controls.compliance_status_fresh = is_fresh(compliance_status_ns);
          controls.compliance_active =
              latest_compliance_active_.load(std::memory_order_acquire);
          const auto rejection = validate_runtime_tare_control_state(controls);
          if (rejection != RuntimeTareRejection::None) {
            resp->success = false;
            resp->message =
                std::string("free-drive ON rejected: motion/force/compliance "
                            "interlock is not freshly idle: ") +
                runtime_tare_rejection_message(rejection);
            return;
          }
          const uint64_t control_generation_after =
              runtime_control_violation_generation_.load(std::memory_order_acquire);
          if (control_generation_after != control_generation_before) {
            resp->success = false;
            resp->message =
                "free-drive ON rejected: motion/force/compliance changed "
                "during the entry check";
            return;
          }
          freedrive_request_control_generation_.store(
              control_generation_after, std::memory_order_release);
        }
        if (freedrive_request_.load(std::memory_order_acquire) != 0 ||
            freedrive_transition_in_progress_.load(std::memory_order_acquire)) {
          resp->success = false;
          resp->message = "another free-drive transition is already pending";
          return;
        }
        freedrive_request_.store(want ? 1 : 2);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
          if (freedrive_request_.load(std::memory_order_acquire) == 0 &&
              !freedrive_transition_in_progress_.load(std::memory_order_acquire)) {
            // The RT thread publishes completion by clearing in_progress only
            // after the SDK call and all resulting safety state are committed.
            const bool now_on = freedrive_on_.load();
            resp->success = (now_on == want);
            if (now_on && want) {
              resp->message = "free-drive ON (hand-guide 가능, servo 중단). F/T 계속 발행.";
            } else if (now_on) {
              resp->message = "free-drive OFF 실패 (cobot 거부); servo remains suspended";
            } else if (want) {
              resp->message = "free-drive ON 실패 (teach pendant Remote 모드 / 로봇 상태 확인)";
            } else {
              resp->message =
                  "free-drive OFF; servo motion is inhibited until the complete "
                  "ros2_control stack is relaunched from the measured pose";
            }
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        resp->success = false;
        resp->message = "timeout — ros2_control write() 가 도는지 확인";
      });

  robot_executor_->add_node(tare_node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RBPodoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
  }

  // Built-in estimated F/T (eft_*) exposed as a ros2_control sensor.
  // The xacro declares two sensors:
  //   * <prefix>ft_sensor      -> hw_ft_states_     (tare-corrected, deadbanded, clamped)
  //   * <prefix>ft_sensor_raw  -> hw_ft_raw_states_ (tare-corrected, no deadband/clamp)
  // Dispatch by suffix so any xacro prefix still works.
  for (const auto& sensor : info_.sensors) {
    static const std::array<std::string, k6DoFDim> kFtIfaces{
        "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
    const bool is_raw =
        sensor.name.size() >= 4 &&
        sensor.name.compare(sensor.name.size() - 4, 4, "_raw") == 0;
    double* base = is_raw ? hw_ft_raw_states_.data() : hw_ft_states_.data();
    for (size_t i = 0; i < kFtIfaces.size(); ++i) {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, kFtIfaces[i], &base[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RBPodoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]));
  }

  for (size_t i = 0; i < k6DoFDim; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(kCartesianPosePrefix[i], HW_IF_CARTESIAN_POSE,
                                                                         &hw_cartesian_pose_commands_[i]));
  }

  for (size_t i = 0; i < k6DoFDim; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        kCartesianVelocityPrefix[i], HW_IF_CARTESIAN_VELOCITY, &hw_cartesian_velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type RBPodoHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(getLogger(), "prepare_command_mode_switch");

  // No-op when controller_manager hands us identical start/stop sets. This happens
  // in chained mode: when an upstream controller (e.g. JTC) activates on top of an
  // already-active downstream (e.g. admittance_controller), the hardware-level
  // claim does not change, but controller_manager still calls us with the same
  // interfaces on both sides. Treating it as a real claim would call try_lock on
  // the already-held move_lock and spuriously fail.
  if (start_interfaces.size() == stop_interfaces.size() &&
      std::is_permutation(start_interfaces.begin(), start_interfaces.end(), stop_interfaces.begin())) {
    return hardware_interface::return_type::OK;
  }

  auto revert = [this]() {
    for (auto info : command_interface_infos_) {
      info.state.claimed = info.state.running;
    }
  };

  for (auto info : command_interface_infos_) {
    std::stringstream ss;
    ss << ".*\\/" << info.type;
    std::regex re(ss.str());

    size_t num_start_interface =
        std::count_if(start_interfaces.begin(), start_interfaces.end(),
                      [re](const std::string& interface) { return std::regex_match(interface, re); });
    size_t num_stop_interface =
        std::count_if(stop_interfaces.begin(), stop_interfaces.end(),
                      [re](const std::string& interface) { return std::regex_match(interface, re); });

    if (num_start_interface == info.size) {
      info.state.claimed = true;
    } else if (num_start_interface != 0) {
      RCLCPP_ERROR(getLogger(), "Invalid number of interfaces (%s) to start. Please check the interface.",
                   info.type.c_str());
      revert();
      return hardware_interface::return_type::ERROR;
    }

    if (num_stop_interface == info.size) {
      info.state.claimed = false;
    } else if (num_stop_interface != 0) {
      RCLCPP_ERROR(getLogger(), "Invalid number of interfaces (%s) to stop. Please check the interface.",
                   info.type.c_str());
      revert();
      return hardware_interface::return_type::ERROR;
    }
  }
  size_t num_start_cmd = std::count_if(command_interface_infos_.begin(), command_interface_infos_.end(),
                                       [](const auto& i) { return i.state.claimed; });
  if (num_start_cmd >= 2) {
    RCLCPP_ERROR(getLogger(), "Cannot start more than one command interface.");
    revert();
    return hardware_interface::return_type::ERROR;
  }

  if (!fake_mode_) {
    if (start_interfaces.size() == 0) {
      if (stop_interfaces.size() != 0) {
        robot_->release_move_lock();
      }
    } else {
      if (!robot_->try_aquire_move_lock()) {
        RCLCPP_ERROR(getLogger(), "Cannot claim interfaces. (move_lock is locked.)");
        revert();
        return hardware_interface::return_type::ERROR;
      }
    }
    robot_->stop();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBPodoHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(getLogger(), "perform_command_mode_switch");
  (void)start_interfaces;
  (void)stop_interfaces;

  if (joint_velocity_interface_state_.claimed) {
    for (auto& e : hw_velocity_commands_) {
      e = 0.;
    }
  }
  if (joint_effort_interface_state_.claimed) {
    for (auto& e : hw_effort_commands_) {
      e = 0.;
    }
  }

  if (cartesian_velocity_interface_state_.claimed) {
    for (auto& e : hw_cartesian_velocity_commands_) {
      e = 0.;
    }
  }

  for (auto info : command_interface_infos_) {
    info.state.running = info.state.claimed;
  }
  mode_changed_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  // command and state should be equal when starting
  for (uint i = 0; i < kNumberOfJoints; i++) {
    hw_position_commands_[i] = hw_position_states_[i];
    hw_effort_commands_[i] = 0;
  }

  // Start the tare procedure from a clean slate every activation. read() will
  // pick this up on its next cycle and keep both exported F/T states invalid
  // until validation succeeds.
  ft_bias_.fill(0.0);
  ft_tare_accum_.fill(0.0);
  ft_tare_accum_sq_.fill(0.0);
  ft_tare_remaining_ = 0;
  ft_tare_sample_target_ = kFtTareSamples;
  ft_tare_active_mode_ = TareMode::None;
  ft_tare_state_.store(TareState::Idle);
  strict_tare_outcome_.store(StrictTareOutcome::NotCompleted);
  bool auto_tare_on_activate = true;
  if (tare_node_) {
    tare_node_->get_parameter("auto_tare_on_activate", auto_tare_on_activate);
  }
  const auto activation_action = activation_tare_action(auto_tare_on_activate);
  ft_tare_requested_mode_.store(
      activation_action == ActivationTareAction::RequestStrict
          ? TareMode::Strict
          : TareMode::None);
  supervised_tare_rejection_.store(SupervisedTareRejection::None);
  supervised_tare_pending_deadline_ns_.store(0);
  supervised_tare_active_deadline_ns_ = 0;
  supervised_tare_attempted_this_activation_.store(false);
  supervised_tare_cancel_requested_.store(false);
  supervised_recent_joint_window_ = JointSampleWindow{};
  supervised_tare_joint_window_ = JointSampleWindow{};
  supervised_tare_source_window_valid_ = false;
  supervised_tare_safety_window_valid_ = false;
  runtime_tare_pending_deadline_ns_.store(0);
  runtime_tare_active_deadline_ns_ = 0;
  runtime_tare_rejection_.store(RuntimeTareRejection::None);
  runtime_tare_cancel_requested_.store(false);
  runtime_tare_phase_ = RuntimeTarePhase::None;
  runtime_tare_candidate_bias_.fill(0.0);
  runtime_recent_joint_window_ = JointSampleWindow{};
  runtime_tare_joint_window_ = JointSampleWindow{};
  runtime_tare_source_window_valid_ = false;
  runtime_tare_safety_window_valid_ = false;
  runtime_tare_control_window_state_ = RuntimeTareControlState{};
  joint_history_next_ = 0;
  joint_history_count_ = 0;
  joint_history_last_sample_ns_ = 0;
  latest_ft_source_valid_.store(false);
  latest_joint_source_valid_.store(false);
  latest_robot_safety_clear_.store(false);
  has_observed_robot_safety_clear_.store(false);
  latest_fresh_sample_ns_.store(0);
  latest_trajectory_active_.store(true);
  latest_trajectory_status_ns_.store(0);
  latest_force_enabled_.store(true);
  latest_force_enable_status_ns_.store(0);
  latest_compliance_enabled_.store(true);
  latest_compliance_enable_status_ns_.store(0);
  latest_compliance_active_.store(true);
  latest_compliance_status_ns_.store(0);
  runtime_control_violation_generation_.store(0);
  invalidate_ft_states(hw_ft_states_, hw_ft_raw_states_);
  last_robot_sample_valid_ = fake_mode_;

  if (activation_action == ActivationTareAction::WaitForExplicitRequest) {
    RCLCPP_INFO(getLogger(),
                "auto_tare_on_activate=false: F/T exports remain NaN until an explicit "
                "~/runtime_free_space_tare or ~/tare_ft request succeeds");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  // Bring the cobot back to Idle. write() keepalive keeps move_servo_j alive
  // for the lifetime of the active hardware component; on deactivation
  // (typically MoveIt / ros2_control_node shutdown) we explicitly task_stop
  // so the cobot transitions immediately instead of waiting for the t2
  // timeout. Without this stop, the keepalive's last write would hold the
  // arm in servo mode for ~100 ms post-shutdown — harmless, but explicit
  // stop matches the launch-time precondition that the cobot starts in Idle.
  if (!fake_mode_ && robot_) {
    robot_->stop();
  }

  // Drop any in-flight tare and invalidate the F/T state. The next on_activate will
  // re-run the tare from scratch, so an Ok bias from this session must not
  // leak into the next one (the arm may be in a different pose / tool / load).
  ft_tare_requested_mode_.store(TareMode::None);
  ft_tare_remaining_ = 0;
  ft_tare_sample_target_ = kFtTareSamples;
  ft_tare_active_mode_ = TareMode::None;
  ft_tare_state_.store(TareState::Idle);
  strict_tare_outcome_.store(StrictTareOutcome::NotCompleted);
  supervised_tare_rejection_.store(SupervisedTareRejection::None);
  supervised_tare_pending_deadline_ns_.store(0);
  supervised_tare_active_deadline_ns_ = 0;
  supervised_tare_attempted_this_activation_.store(false);
  supervised_tare_cancel_requested_.store(false);
  supervised_recent_joint_window_ = JointSampleWindow{};
  supervised_tare_joint_window_ = JointSampleWindow{};
  supervised_tare_source_window_valid_ = false;
  supervised_tare_safety_window_valid_ = false;
  runtime_tare_pending_deadline_ns_.store(0);
  runtime_tare_active_deadline_ns_ = 0;
  runtime_tare_rejection_.store(RuntimeTareRejection::None);
  runtime_tare_cancel_requested_.store(false);
  runtime_tare_phase_ = RuntimeTarePhase::None;
  runtime_tare_candidate_bias_.fill(0.0);
  runtime_recent_joint_window_ = JointSampleWindow{};
  runtime_tare_joint_window_ = JointSampleWindow{};
  runtime_tare_source_window_valid_ = false;
  runtime_tare_safety_window_valid_ = false;
  runtime_tare_control_window_state_ = RuntimeTareControlState{};
  joint_history_next_ = 0;
  joint_history_count_ = 0;
  joint_history_last_sample_ns_ = 0;
  latest_ft_source_valid_.store(false);
  latest_joint_source_valid_.store(false);
  latest_robot_safety_clear_.store(false);
  has_observed_robot_safety_clear_.store(false);
  latest_fresh_sample_ns_.store(0);
  latest_trajectory_active_.store(true);
  latest_trajectory_status_ns_.store(0);
  latest_force_enabled_.store(true);
  latest_force_enable_status_ns_.store(0);
  latest_compliance_enabled_.store(true);
  latest_compliance_enable_status_ns_.store(0);
  latest_compliance_active_.store(true);
  latest_compliance_status_ns_.store(0);
  runtime_control_violation_generation_.store(0);
  ft_bias_.fill(0.0);
  ft_tare_accum_.fill(0.0);
  ft_tare_accum_sq_.fill(0.0);
  invalidate_ft_states(hw_ft_states_, hw_ft_raw_states_);
  last_robot_sample_valid_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RBPodoHardwareInterface::read(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) {
  (void)time;

  // Read the robot once if we have a connection (needed for real joint state
  // and/or real FT). Skipped entirely when both fake_mode_ and
  // inject_ft_enabled_ are true, since there is then nothing to fetch. The
  // result is cached in last_read_state_ so write() can reuse it on the same
  // cycle instead of issuing a second network round-trip.
  std::array<double, k6DoFDim> raw_ft{};
  HardwareJointPositions actual_joint_positions{};
  bool ft_source_valid = false;
  bool joint_source_valid = false;
  bool robot_safety_interlocks_clear = fake_mode_;
  last_robot_sample_valid_ = fake_mode_;
  if (robot_) {
    const auto sample = robot_->read_once_with_status(kMaxRobotStateAgeS);
    last_robot_sample_valid_ = sample.fresh;
    if (sample.fresh) {
      last_read_state_ = sample.state;
      if (robot_node_) {
        robot_node_->publish_system_state(last_read_state_);
      }
      const auto& data = last_read_state_;
      for (size_t i = 0; i < kNumberOfJoints; ++i) {
        // jnt_ang is the measured joint angle.  jnt_ref can remain stationary
        // while the physical arm is still settling, so it is not sufficient
        // evidence for a supervised free-space tare.
        actual_joint_positions[i] = data.sdata.jnt_ang[i] * DEG2RAD;
      }
      joint_source_valid = std::all_of(
          actual_joint_positions.begin(), actual_joint_positions.end(),
          [](double value) { return std::isfinite(value); });
      robot_safety_interlocks_clear = real_robot_safety_interlocks_clear(
          data.sdata.is_freedrive_mode != 0, data.sdata.init_state_info,
          data.sdata.init_error, data.sdata.op_stat_collision_occur,
          data.sdata.op_stat_sos_flag, data.sdata.op_stat_self_collision,
          data.sdata.op_stat_soft_estop_occur != 0, data.sdata.op_stat_ems_flag,
          data.sdata.information_chunk_1, data.sdata.information_chunk_3);
      const bool intentional_internal_freedrive_transition =
          freedrive_on_.load(std::memory_order_acquire);
      const bool severe_safety_abort =
          real_robot_motion_abort_interlock_triggered(
              data.sdata.op_stat_collision_occur,
              data.sdata.op_stat_sos_flag,
              data.sdata.op_stat_self_collision,
              data.sdata.op_stat_soft_estop_occur != 0,
              data.sdata.op_stat_ems_flag,
              data.sdata.information_chunk_1,
              data.sdata.information_chunk_3);
      const bool safety_transition_abort =
          robot_safety_loss_requires_motion_inhibit(
              robot_safety_interlocks_clear,
              has_observed_robot_safety_clear_.load(std::memory_order_acquire),
              intentional_internal_freedrive_transition);
      if (robot_safety_interlocks_clear) {
        has_observed_robot_safety_clear_.store(true, std::memory_order_release);
      }
      if (!fake_mode_ && (severe_safety_abort || safety_transition_abort)) {
        bool expected = false;
        if (motion_inhibit_latched_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
          motion_stop_requested_.store(true, std::memory_order_release);
          const uint32_t interlock_reason_mask =
              real_robot_motion_abort_reason_mask(
                  data.sdata.op_stat_collision_occur,
                  data.sdata.op_stat_sos_flag,
                  data.sdata.op_stat_self_collision,
                  data.sdata.op_stat_soft_estop_occur != 0,
                  data.sdata.op_stat_ems_flag,
                  data.sdata.information_chunk_1,
                  data.sdata.information_chunk_3);
          const std::string snapshot = make_interlock_snapshot(
              data, hw_position_commands_, interlock_reason_mask,
              severe_safety_abort, robot_safety_interlocks_clear,
              period.seconds() * 1000.0,
              latest_trajectory_active_.load(std::memory_order_acquire),
              latest_force_enabled_.load(std::memory_order_acquire),
              latest_compliance_enabled_.load(std::memory_order_acquire),
              latest_compliance_active_.load(std::memory_order_acquire));
          RCLCPP_ERROR(
              getLogger(), "[INTERLOCK_SNAPSHOT] %s", snapshot.c_str());
          RCLCPP_ERROR(
              getLogger(),
              severe_safety_abort
                  ? "robot emergency/collision interlock became unsafe; hardware "
                    "motion inhibit latched"
                  : "robot-ready state was lost after activation; hardware motion "
                    "inhibit latched to prevent an open-loop trajectory from resuming");
        }
      }
      if (!fake_mode_) {
        for (size_t i = 0; i < kNumberOfJoints; ++i) {
          // ros2_control position *state* must describe the measured robot,
          // not the controller reference.  Publishing jnt_ref here made
          // /joint_states, robot_state_publisher TF, MoveIt feedback, and the
          // pre-contact clearance check all report the commanded 10 mm pose
          // even if the physical arm lagged or stopped short.  Commands still
          // use hw_position_commands_; only feedback is corrected here.
          hw_position_states_[i] = data.sdata.jnt_ang[i] * DEG2RAD;
          hw_effort_states_[i] = data.sdata.jnt_cur[i] * torque_constants_[i];
        }
      }
      raw_ft = {data.sdata.eft_fx, data.sdata.eft_fy, data.sdata.eft_fz,
                data.sdata.eft_mx, data.sdata.eft_my, data.sdata.eft_mz};
      ft_source_valid = true;
    } else {
      static rclcpp::Clock stale_clock(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(
          getLogger(), stale_clock, 2000,
          "robot telemetry invalid/stale (transport_valid=%s, age=%.6f s); "
          "F/T states are NaN and servo writes are suspended",
          sample.transport_valid ? "true" : "false", sample.age_s);
    }
  }

  if (fake_mode_) {
    // No real motion: mirror commands into states so JTC/admittance see their
    // own setpoints honored and RViz / MoveIt show the commanded pose.
    for (size_t i = 0; i < kNumberOfJoints; ++i) {
      hw_position_states_[i] = hw_position_commands_[i];
      hw_effort_states_[i] = 0.0;
    }
    if (!robot_) {
      actual_joint_positions = hw_position_states_;
      joint_source_valid = std::all_of(
          actual_joint_positions.begin(), actual_joint_positions.end(),
          [](double value) { return std::isfinite(value); });
      robot_safety_interlocks_clear = true;
    }
  }

  // Debug/test injection: when enabled and a value has been received, replace
  // raw_ft BEFORE tare/filter run so the injected signal traverses the same
  // code path the real signal would (tare bias -> median -> hold -> slew ->
  // broadcaster -> admittance -> JTC -> robot).
  //
  // Fast path: check the atomic flag without locking. Injection is the rare
  // case, so most read() cycles skip the mutex entirely. Only when active do
  // we acquire the lock to read ft_inject_values_ coherently (callback always
  // holds the lock when mutating either values or the flag).
  if (inject_ft_enabled_ && ft_inject_active_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(ft_inject_mutex_);
    if (ft_inject_active_.load(std::memory_order_relaxed)) {
      raw_ft = ft_inject_values_;
      ft_source_valid = true;
    }
  }
  ft_source_valid = ft_source_valid && is_finite_ft_sample(raw_ft);
  latest_ft_source_valid_.store(ft_source_valid, std::memory_order_release);
  latest_joint_source_valid_.store(joint_source_valid, std::memory_order_release);
  latest_robot_safety_clear_.store(robot_safety_interlocks_clear, std::memory_order_release);
  const int64_t trajectory_status_ns =
      latest_trajectory_status_ns_.load(std::memory_order_acquire);
  const bool trajectory_active =
      latest_trajectory_active_.load(std::memory_order_acquire);
  const int64_t force_enable_status_ns =
      latest_force_enable_status_ns_.load(std::memory_order_acquire);
  const bool force_enabled =
      latest_force_enabled_.load(std::memory_order_acquire);
  const int64_t compliance_status_ns =
      latest_compliance_status_ns_.load(std::memory_order_acquire);
  const bool compliance_active =
      latest_compliance_active_.load(std::memory_order_acquire);
  const int64_t compliance_enable_status_ns =
      latest_compliance_enable_status_ns_.load(std::memory_order_acquire);
  const bool compliance_enabled =
      latest_compliance_enabled_.load(std::memory_order_acquire);
  // Snapshot heartbeat timestamps before the comparison clock for the same
  // reason as the service entry check above: callbacks run concurrently.
  const int64_t current_sample_ns = steady_now_ns();
  if (ft_source_valid && joint_source_valid) {
    latest_fresh_sample_ns_.store(current_sample_ns, std::memory_order_release);
  } else {
    latest_fresh_sample_ns_.store(0, std::memory_order_release);
  }

  const auto runtime_status_is_fresh = [current_sample_ns](int64_t status_ns) {
    const int64_t age_ns =
        status_ns > 0 ? current_sample_ns - status_ns
                      : std::numeric_limits<int64_t>::max();
    return age_ns >= 0 && age_ns <= kRuntimeControlStatusMaxAgeNs;
  };
  RuntimeTareControlState current_runtime_control_state;
  current_runtime_control_state.trajectory_status_fresh =
      runtime_status_is_fresh(trajectory_status_ns);
  current_runtime_control_state.trajectory_active = trajectory_active;
  current_runtime_control_state.force_enable_status_fresh =
      runtime_status_is_fresh(force_enable_status_ns);
  current_runtime_control_state.force_enabled = force_enabled;
  current_runtime_control_state.compliance_enable_status_fresh =
      runtime_status_is_fresh(compliance_enable_status_ns);
  current_runtime_control_state.compliance_enabled = compliance_enabled;
  current_runtime_control_state.compliance_status_fresh =
      runtime_status_is_fresh(compliance_status_ns);
  current_runtime_control_state.compliance_active = compliance_active;

  // Maintain a contiguous history only while *all* supervised preconditions
  // are eligible.  A telemetry gap or any unsafe/free-drive interval clears
  // the ring so one new sample cannot be combined with old stationary data.
  const bool freedrive_or_post_hold_active =
      freedrive_on_.load(std::memory_order_acquire) || freedrive_request_.load() != 0 ||
      post_freedrive_hold_cycles_.load(std::memory_order_acquire) > 0;
  const bool history_sample_eligible = supervised_joint_history_sample_is_eligible(
      ft_source_valid, joint_source_valid, robot_safety_interlocks_clear,
      freedrive_or_post_hold_active);
  if (history_sample_eligible) {
    if (joint_history_count_ > 0 &&
        !supervised_joint_history_gap_is_contiguous(
            joint_history_last_sample_ns_, current_sample_ns, kMaxLatestSampleAgeNs)) {
      joint_history_next_ = 0;
      joint_history_count_ = 0;
    }
    joint_history_[joint_history_next_] = actual_joint_positions;
    joint_history_next_ = (joint_history_next_ + 1) % kJointHistoryCapacity;
    joint_history_count_ = std::min(joint_history_count_ + 1, kJointHistoryCapacity);
    joint_history_last_sample_ns_ = current_sample_ns;
  } else {
    joint_history_next_ = 0;
    joint_history_count_ = 0;
    joint_history_last_sample_ns_ = 0;
  }

  // A supervised service timeout must not leave an in-flight request able to
  // adopt a bias later.  Cancellation is completed on this RT-owned thread so
  // ft_bias_ is never written concurrently with read().
  if (supervised_tare_cancel_requested_.exchange(false, std::memory_order_acq_rel)) {
    TareMode pending = TareMode::Supervised;
    ft_tare_requested_mode_.compare_exchange_strong(pending, TareMode::None);
    ft_tare_remaining_ = 0;
    ft_tare_active_mode_ = TareMode::None;
    ft_bias_.fill(0.0);
    supervised_tare_rejection_.store(SupervisedTareRejection::CancelledOrTimedOut);
    ft_tare_state_.store(TareState::Failed);
    RCLCPP_ERROR(getLogger(),
                 "supervised F/T tare CANCELLED after service timeout; bias cleared and exports remain NaN");
  }
  if (runtime_tare_cancel_requested_.exchange(false, std::memory_order_acq_rel)) {
    TareMode pending = TareMode::Runtime;
    ft_tare_requested_mode_.compare_exchange_strong(pending, TareMode::None);
    ft_tare_remaining_ = 0;
    ft_tare_active_mode_ = TareMode::None;
    runtime_tare_phase_ = RuntimeTarePhase::None;
    ft_bias_.fill(0.0);
    runtime_tare_candidate_bias_.fill(0.0);
    runtime_tare_rejection_.store(RuntimeTareRejection::CancelledOrTimedOut);
    ft_tare_state_.store(TareState::Failed);
    RCLCPP_ERROR(getLogger(),
                 "runtime F/T tare CANCELLED after service timeout; bias cleared and exports remain NaN");
  }

  // A service callback or on_activate may have asked for a new tare. Pick it
  // up atomically and (re)initialize the accumulators on this control cycle.
  const TareMode requested_mode =
      ft_tare_requested_mode_.exchange(TareMode::None, std::memory_order_acq_rel);
  if (requested_mode != TareMode::None) {
    ft_bias_.fill(0.0);
    ft_tare_accum_.fill(0.0);
    ft_tare_accum_sq_.fill(0.0);
    ft_tare_remaining_ = kFtTareSamples;
    ft_tare_sample_target_ = kFtTareSamples;
    ft_tare_active_mode_ = requested_mode;
    ft_tare_state_.store(TareState::Running);
    if (requested_mode == TareMode::Supervised) {
      supervised_tare_active_config_ = supervised_tare_pending_config_;
      supervised_tare_active_deadline_ns_ =
          supervised_tare_pending_deadline_ns_.load(std::memory_order_relaxed);
      supervised_tare_source_window_valid_ = true;
      supervised_tare_safety_window_valid_ = true;
      supervised_tare_joint_window_ = JointSampleWindow{};
      supervised_tare_joint_window_.minimum.fill(std::numeric_limits<double>::infinity());
      supervised_tare_joint_window_.maximum.fill(-std::numeric_limits<double>::infinity());
      supervised_tare_joint_window_.source_valid = true;

      supervised_recent_joint_window_ = JointSampleWindow{};
      const size_t requested_recent = supervised_tare_active_config_.min_recent_joint_samples;
      const size_t recent_count = std::min(joint_history_count_, requested_recent);
      if (recent_count > 0) {
        supervised_recent_joint_window_.minimum.fill(std::numeric_limits<double>::infinity());
        supervised_recent_joint_window_.maximum.fill(-std::numeric_limits<double>::infinity());
        for (size_t sample_index = 0; sample_index < recent_count; ++sample_index) {
          const size_t history_index =
              (joint_history_next_ + kJointHistoryCapacity - recent_count + sample_index) %
              kJointHistoryCapacity;
          for (size_t joint_index = 0; joint_index < kNumberOfJoints; ++joint_index) {
            supervised_recent_joint_window_.minimum[joint_index] =
                std::min(supervised_recent_joint_window_.minimum[joint_index],
                         joint_history_[history_index][joint_index]);
            supervised_recent_joint_window_.maximum[joint_index] =
                std::max(supervised_recent_joint_window_.maximum[joint_index],
                         joint_history_[history_index][joint_index]);
          }
        }
        supervised_recent_joint_window_.sample_count = recent_count;
        supervised_recent_joint_window_.source_valid = true;
      }
      RCLCPP_WARN(getLogger(),
                  "SUPERVISED F/T tare started after explicit free-space confirmation (%zu samples). "
                  "Bias will be accepted only inside the configured baseline envelope.",
                  kFtTareSamples);
    } else if (requested_mode == TareMode::Runtime) {
      runtime_tare_active_config_ = runtime_tare_pending_config_;
      runtime_tare_active_deadline_ns_ =
          runtime_tare_pending_deadline_ns_.load(std::memory_order_relaxed);
      runtime_tare_active_control_violation_generation_ =
          runtime_tare_pending_control_violation_generation_;
      runtime_tare_phase_ = RuntimeTarePhase::CollectBias;
      runtime_tare_candidate_bias_.fill(0.0);
      runtime_tare_source_window_valid_ = true;
      runtime_tare_safety_window_valid_ = true;
      runtime_tare_control_window_state_ = current_runtime_control_state;
      runtime_tare_joint_window_ = JointSampleWindow{};
      runtime_tare_joint_window_.minimum.fill(std::numeric_limits<double>::infinity());
      runtime_tare_joint_window_.maximum.fill(-std::numeric_limits<double>::infinity());
      runtime_tare_joint_window_.source_valid = true;

      runtime_recent_joint_window_ = JointSampleWindow{};
      const size_t requested_recent = runtime_tare_active_config_.min_recent_joint_samples;
      const size_t recent_count = std::min(joint_history_count_, requested_recent);
      if (recent_count > 0) {
        runtime_recent_joint_window_.minimum.fill(std::numeric_limits<double>::infinity());
        runtime_recent_joint_window_.maximum.fill(-std::numeric_limits<double>::infinity());
        for (size_t sample_index = 0; sample_index < recent_count; ++sample_index) {
          const size_t history_index =
              (joint_history_next_ + kJointHistoryCapacity - recent_count + sample_index) %
              kJointHistoryCapacity;
          for (size_t joint_index = 0; joint_index < kNumberOfJoints; ++joint_index) {
            runtime_recent_joint_window_.minimum[joint_index] =
                std::min(runtime_recent_joint_window_.minimum[joint_index],
                         joint_history_[history_index][joint_index]);
            runtime_recent_joint_window_.maximum[joint_index] =
                std::max(runtime_recent_joint_window_.maximum[joint_index],
                         joint_history_[history_index][joint_index]);
          }
        }
        runtime_recent_joint_window_.sample_count = recent_count;
        runtime_recent_joint_window_.source_valid = true;
      }
      runtime_tare_rejection_.store(RuntimeTareRejection::None);
      RCLCPP_WARN(getLogger(),
                  "RUNTIME free-space F/T tare started (%zu bias + %zu residual samples). "
                  "No fixed raw baseline is used; motion, force, and compliance must remain off.",
                  kFtTareSamples, kRuntimeTarePostSamples);
    } else {
      supervised_tare_rejection_.store(SupervisedTareRejection::None);
      strict_tare_outcome_.store(StrictTareOutcome::NotCompleted);
      RCLCPP_INFO(getLogger(), "F/T tare started (%zu samples)", kFtTareSamples);
    }
  }

  const bool supervised_joint_invalid =
      ft_tare_active_mode_ == TareMode::Supervised && !joint_source_valid;
  const bool supervised_safety_invalid =
      ft_tare_active_mode_ == TareMode::Supervised &&
      (!robot_safety_interlocks_clear || freedrive_on_.load(std::memory_order_acquire) ||
       freedrive_request_.load() != 0 ||
       post_freedrive_hold_cycles_.load(std::memory_order_acquire) > 0);
  RuntimeTareRejection runtime_live_rejection = RuntimeTareRejection::None;
  if (ft_tare_active_mode_ == TareMode::Runtime) {
    if (!ft_source_valid || !joint_source_valid) {
      runtime_live_rejection = RuntimeTareRejection::InvalidOrStaleSource;
    } else if (!robot_safety_interlocks_clear ||
               freedrive_on_.load(std::memory_order_acquire) ||
               freedrive_request_.load() != 0 ||
               post_freedrive_hold_cycles_.load(std::memory_order_acquire) > 0) {
      runtime_live_rejection = RuntimeTareRejection::SafetyInterlockActive;
    } else if (runtime_tare_active_deadline_ns_ <= 0 ||
               steady_now_ns() > runtime_tare_active_deadline_ns_) {
      runtime_live_rejection = RuntimeTareRejection::CancelledOrTimedOut;
    } else if (runtime_control_violation_generation_.load(std::memory_order_acquire) !=
               runtime_tare_active_control_violation_generation_) {
      runtime_live_rejection = RuntimeTareRejection::ControlInterlockViolated;
    } else {
      runtime_live_rejection =
          validate_runtime_tare_control_state(current_runtime_control_state);
    }
  }
  if (ft_tare_remaining_ > 0 &&
      (!ft_source_valid || supervised_joint_invalid || supervised_safety_invalid ||
       runtime_live_rejection != RuntimeTareRejection::None)) {
    ft_tare_remaining_ = 0;
    ft_bias_.fill(0.0);
    if (ft_tare_active_mode_ == TareMode::Supervised) {
      supervised_tare_source_window_valid_ = false;
      if (supervised_joint_invalid || !ft_source_valid) {
        supervised_tare_joint_window_.source_valid = false;
      }
      if (supervised_safety_invalid) {
        supervised_tare_safety_window_valid_ = false;
        supervised_tare_rejection_.store(SupervisedTareRejection::SafetyInterlockActive);
      } else {
        supervised_tare_rejection_.store(SupervisedTareRejection::InvalidOrStaleSource);
      }
    }
    if (ft_tare_active_mode_ == TareMode::Runtime) {
      runtime_tare_source_window_valid_ = false;
      runtime_tare_joint_window_.source_valid = false;
      if (runtime_live_rejection == RuntimeTareRejection::SafetyInterlockActive) {
        runtime_tare_safety_window_valid_ = false;
      }
      runtime_tare_control_window_state_ = current_runtime_control_state;
      runtime_tare_rejection_.store(runtime_live_rejection);
      runtime_tare_candidate_bias_.fill(0.0);
      runtime_tare_phase_ = RuntimeTarePhase::None;
    }
    if (ft_tare_active_mode_ == TareMode::Strict) {
      strict_tare_outcome_.store(StrictTareOutcome::FailedInvalidSource);
    }
    ft_tare_state_.store(TareState::Failed);
    if (ft_tare_active_mode_ == TareMode::Supervised) {
      RCLCPP_ERROR(getLogger(), "supervised F/T tare REJECTED: %s; bias cleared and exports remain NaN",
                   supervised_tare_rejection_message(supervised_tare_rejection_.load()));
    } else if (ft_tare_active_mode_ == TareMode::Runtime) {
      RCLCPP_ERROR(getLogger(),
                   "runtime F/T tare REJECTED: %s; bias cleared and exports remain NaN",
                   runtime_tare_rejection_message(runtime_tare_rejection_.load()));
    } else {
      RCLCPP_ERROR(getLogger(),
                   "F/T tare FAILED because its source became stale, unreadable, or non-finite; "
                   "exported F/T remains NaN. Restore telemetry and call ~/tare_ft again.");
    }
    ft_tare_active_mode_ = TareMode::None;
  } else if (ft_tare_remaining_ > 0) {
    for (size_t i = 0; i < k6DoFDim; ++i) {
      ft_tare_accum_[i] += raw_ft[i];
      ft_tare_accum_sq_[i] += raw_ft[i] * raw_ft[i];
    }
    if (ft_tare_active_mode_ == TareMode::Supervised) {
      for (size_t i = 0; i < kNumberOfJoints; ++i) {
        supervised_tare_joint_window_.minimum[i] =
            std::min(supervised_tare_joint_window_.minimum[i], actual_joint_positions[i]);
        supervised_tare_joint_window_.maximum[i] =
            std::max(supervised_tare_joint_window_.maximum[i], actual_joint_positions[i]);
      }
      ++supervised_tare_joint_window_.sample_count;
    } else if (ft_tare_active_mode_ == TareMode::Runtime) {
      for (size_t i = 0; i < kNumberOfJoints; ++i) {
        runtime_tare_joint_window_.minimum[i] =
            std::min(runtime_tare_joint_window_.minimum[i], actual_joint_positions[i]);
        runtime_tare_joint_window_.maximum[i] =
            std::max(runtime_tare_joint_window_.maximum[i], actual_joint_positions[i]);
      }
      ++runtime_tare_joint_window_.sample_count;
      runtime_tare_control_window_state_ = current_runtime_control_state;
    }
    --ft_tare_remaining_;

    if (ft_tare_remaining_ == 0) {
      const double n = static_cast<double>(ft_tare_sample_target_);
      std::array<double, k6DoFDim> mean{};
      std::array<double, k6DoFDim> stddev{};
      for (size_t i = 0; i < k6DoFDim; ++i) {
        mean[i] = ft_tare_accum_[i] / n;
        // Sample variance: E[x^2] - E[x]^2, clamped against fp noise.
        const double var = std::max(0.0, ft_tare_accum_sq_[i] / n - mean[i] * mean[i]);
        stddev[i] = std::sqrt(var);
      }

      if (ft_tare_active_mode_ == TareMode::Supervised) {
        const bool cancelled_or_expired =
            supervised_tare_cancel_requested_.exchange(false, std::memory_order_acq_rel) ||
            supervised_tare_active_deadline_ns_ <= 0 ||
            steady_now_ns() > supervised_tare_active_deadline_ns_;
        const auto rejection =
            cancelled_or_expired
                ? SupervisedTareRejection::CancelledOrTimedOut
                : validate_supervised_tare(
                      mean, stddev, supervised_tare_source_window_valid_,
                      supervised_tare_safety_window_valid_,
                      supervised_recent_joint_window_, supervised_tare_joint_window_,
                      supervised_tare_active_config_);
        supervised_tare_rejection_.store(rejection);
        if (rejection == SupervisedTareRejection::None) {
          ft_bias_ = mean;
          ft_tare_state_.store(TareState::Ok);
          RCLCPP_WARN(getLogger(),
                      "SUPERVISED F/T tare OK. bias F=[%.2f %.2f %.2f] N  "
                      "T=[%.3f %.3f %.3f] Nm. Explicit confirmation consumed for this activation.",
                      mean[0], mean[1], mean[2], mean[3], mean[4], mean[5]);
        } else {
          ft_bias_.fill(0.0);
          ft_tare_state_.store(TareState::Failed);
          RCLCPP_ERROR(getLogger(),
                       "supervised F/T tare REJECTED (%s). "
                       "mean F=[%.2f %.2f %.2f] N std F=[%.2f %.2f %.2f] N "
                       "mean T=[%.3f %.3f %.3f] Nm std T=[%.3f %.3f %.3f] Nm. "
                       "bias cleared and exports remain NaN.",
                       supervised_tare_rejection_message(rejection),
                       mean[0], mean[1], mean[2], stddev[0], stddev[1], stddev[2],
                       mean[3], mean[4], mean[5], stddev[3], stddev[4], stddev[5]);
        }
      } else if (ft_tare_active_mode_ == TareMode::Runtime) {
        const bool cancelled_or_expired =
            runtime_tare_cancel_requested_.exchange(false, std::memory_order_acq_rel) ||
            runtime_tare_active_deadline_ns_ <= 0 ||
            steady_now_ns() > runtime_tare_active_deadline_ns_;
        auto rejection =
            cancelled_or_expired
                ? RuntimeTareRejection::CancelledOrTimedOut
                : validate_runtime_tare(
                      mean, stddev, runtime_tare_source_window_valid_,
                      runtime_tare_safety_window_valid_,
                      runtime_tare_control_window_state_, runtime_recent_joint_window_,
                      runtime_tare_joint_window_, runtime_tare_active_config_);

        if (rejection == RuntimeTareRejection::None &&
            runtime_tare_phase_ == RuntimeTarePhase::CollectBias) {
          runtime_tare_candidate_bias_ = mean;
          runtime_tare_phase_ = RuntimeTarePhase::ValidateResidual;
          ft_tare_accum_.fill(0.0);
          ft_tare_accum_sq_.fill(0.0);
          ft_tare_sample_target_ = kRuntimeTarePostSamples;
          ft_tare_remaining_ = kRuntimeTarePostSamples;
          RCLCPP_INFO(getLogger(),
                      "runtime F/T bias candidate collected: F=[%.2f %.2f %.2f] N "
                      "T=[%.3f %.3f %.3f] Nm; validating corrected residual",
                      mean[0], mean[1], mean[2], mean[3], mean[4], mean[5]);
        } else {
          HardwareWrench corrected_mean{};
          if (rejection == RuntimeTareRejection::None &&
              runtime_tare_phase_ == RuntimeTarePhase::ValidateResidual) {
            for (size_t i = 0; i < k6DoFDim; ++i) {
              corrected_mean[i] = mean[i] - runtime_tare_candidate_bias_[i];
            }
            rejection = validate_runtime_tare_post_residual(
                corrected_mean, stddev, runtime_tare_active_config_);
            // Close the final callback/RT race immediately before committing
            // the bias.  A true trajectory/force/compliance pulse after the
            // cycle's earlier snapshot must invalidate the candidate rather
            // than becoming active against freshly finite F/T on the next
            // update.
            if (rejection == RuntimeTareRejection::None &&
                runtime_control_violation_generation_.load(
                    std::memory_order_acquire) !=
                    runtime_tare_active_control_violation_generation_) {
              rejection = RuntimeTareRejection::ControlInterlockViolated;
            }
          } else if (rejection == RuntimeTareRejection::None) {
            rejection = RuntimeTareRejection::InvalidConfiguration;
          }

          runtime_tare_rejection_.store(rejection);
          if (rejection == RuntimeTareRejection::None) {
            ft_bias_ = runtime_tare_candidate_bias_;
            ft_tare_state_.store(TareState::Ok);
            RCLCPP_WARN(getLogger(),
                        "RUNTIME free-space F/T tare OK. bias F=[%.2f %.2f %.2f] N "
                        "T=[%.3f %.3f %.3f] Nm; residual F=[%.3f %.3f %.3f] N "
                        "T=[%.4f %.4f %.4f] Nm",
                        ft_bias_[0], ft_bias_[1], ft_bias_[2], ft_bias_[3],
                        ft_bias_[4], ft_bias_[5], corrected_mean[0],
                        corrected_mean[1], corrected_mean[2], corrected_mean[3],
                        corrected_mean[4], corrected_mean[5]);
          } else {
            ft_bias_.fill(0.0);
            runtime_tare_candidate_bias_.fill(0.0);
            ft_tare_state_.store(TareState::Failed);
            RCLCPP_ERROR(getLogger(),
                         "runtime F/T tare REJECTED (%s). "
                         "mean F=[%.2f %.2f %.2f] N std F=[%.2f %.2f %.2f] N "
                         "mean T=[%.3f %.3f %.3f] Nm std T=[%.3f %.3f %.3f] Nm. "
                         "bias invalid and exports remain NaN.",
                         runtime_tare_rejection_message(rejection), mean[0], mean[1],
                         mean[2], stddev[0], stddev[1], stddev[2], mean[3],
                         mean[4], mean[5], stddev[3], stddev[4], stddev[5]);
          }
          runtime_tare_phase_ = RuntimeTarePhase::None;
        }
      } else {
        // Original automatic/manual strict validation remains unchanged.
        bool ok = true;
        bool mean_exceeded = false;
        bool noise_exceeded = false;
        for (size_t i = 0; i < k6DoFDim; ++i) {
          const double max_mean = (i < 3) ? kFtTareMaxAbsMeanForce : kFtTareMaxAbsMeanTorque;
          const double max_std = (i < 3) ? kFtTareMaxStdForce : kFtTareMaxStdTorque;
          mean_exceeded = mean_exceeded || std::abs(mean[i]) > max_mean;
          noise_exceeded = noise_exceeded || stddev[i] > max_std;
          if (std::abs(mean[i]) > max_mean || stddev[i] > max_std) {
            ok = false;
          }
        }
        if (ok) {
          ft_bias_ = mean;
          strict_tare_outcome_.store(StrictTareOutcome::Succeeded);
          ft_tare_state_.store(TareState::Ok);
          RCLCPP_INFO(getLogger(),
                      "F/T tare OK. bias F=[%.2f %.2f %.2f] N  T=[%.3f %.3f %.3f] Nm",
                      mean[0], mean[1], mean[2], mean[3], mean[4], mean[5]);
        } else {
          ft_bias_.fill(0.0);
          strict_tare_outcome_.store(
              noise_exceeded ? StrictTareOutcome::FailedNoise
                             : (mean_exceeded ? StrictTareOutcome::FailedMeanEnvelopeOnly
                                              : StrictTareOutcome::FailedInvalidSource));
          ft_tare_state_.store(TareState::Failed);
          RCLCPP_ERROR(getLogger(),
                       "F/T tare FAILED (arm loaded / in contact / noisy). "
                       "mean F=[%.2f %.2f %.2f] N std F=[%.2f %.2f %.2f] N "
                       "mean T=[%.3f %.3f %.3f] Nm std T=[%.3f %.3f %.3f] Nm. "
                       "exported F/T held at NaN; call ~/tare_ft after freeing the arm.",
                       mean[0], mean[1], mean[2], stddev[0], stddev[1], stddev[2],
                       mean[3], mean[4], mean[5], stddev[3], stddev[4], stddev[5]);
        }
      }
      if (!(ft_tare_active_mode_ == TareMode::Runtime &&
            runtime_tare_phase_ == RuntimeTarePhase::ValidateResidual &&
            ft_tare_state_.load() == TareState::Running)) {
        ft_tare_active_mode_ = TareMode::None;
      }
    }
  }

  // Only a finite source plus a completed, valid tare may publish a wrench.
  // A transient source failure after a successful tare preserves the bias but
  // exports NaN; the next fresh sample therefore recovers automatically.
  update_ft_output_states(raw_ft, ft_bias_, ft_source_valid,
                          ft_tare_state_.load() == TareState::Ok, human_collab_,
                          kFtCollabDeadband, kFtCollabClamp, hw_ft_states_, hw_ft_raw_states_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBPodoHardwareInterface::write(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  const auto suppress_inhibited_motion = [this]() {
    if (!motion_inhibit_latched_.load(std::memory_order_acquire)) {
      return false;
    }
    hw_position_commands_ = hw_position_states_;
    hw_velocity_commands_.fill(0.0);
    hw_effort_commands_.fill(0.0);
    if (fake_mode_) {
      motion_stop_requested_.store(false, std::memory_order_release);
      motion_inhibit_acknowledged_.store(true, std::memory_order_release);
      return true;
    }
    if (robot_ && motion_stop_requested_.exchange(
                      false, std::memory_order_acq_rel)) {
      const bool stopped = robot_->stop();
      if (stopped) {
        RCLCPP_ERROR(
            getLogger(),
            "hardware motion inhibit acknowledged: cobot stop requested and "
            "all subsequent servo writes are suppressed until full relaunch");
      } else {
        RCLCPP_ERROR(
            getLogger(),
            "cobot stop request returned failure; all subsequent servo writes "
            "remain suppressed until full relaunch");
      }
    }
    // The acknowledgement means the hardware write boundary is closed.  Even
    // if the SDK stop call was rejected, no new servo keepalive or trajectory
    // command can be emitted and the cobot-side servo timeout can expire.
    motion_inhibit_acknowledged_.store(true, std::memory_order_release);
    return true;
  };

  if (suppress_inhibited_motion()) {
    return hardware_interface::return_type::OK;
  }
  if (fake_mode_) {
    // No physical arm to drive; read() already mirrors commands into states so
    // the controllers see their setpoints honored.
    return hardware_interface::return_type::OK;
  }
  if (!robot_) {
    RCLCPP_ERROR(getLogger(), "Robot is not initialized yet");
    return hardware_interface::return_type::ERROR;
  }

  // --- Free-drive (direct-teaching) handling (RT thread; single cobot owner) ---
  // A ~/set_freedrive request is processed here so the cobot command shares the
  // read()/write() thread. While free-drive is ON we send NO servo command so
  // the arm is hand-guidable; read() still runs so /aft200/ft keeps publishing.
  {
    int req = freedrive_request_.load(std::memory_order_acquire);
    if (req != 0) {
      bool expected = false;
      if (freedrive_transition_in_progress_.compare_exchange_strong(
              expected, true, std::memory_order_acq_rel)) {
        req = freedrive_request_.exchange(0, std::memory_order_acq_rel);
      } else {
        req = 0;
      }
    }
    if (req == 1) {  // turn ON
      const int64_t trajectory_status_ns =
          latest_trajectory_status_ns_.load(std::memory_order_acquire);
      const int64_t force_status_ns =
          latest_force_enable_status_ns_.load(std::memory_order_acquire);
      const int64_t compliance_enable_status_ns =
          latest_compliance_enable_status_ns_.load(std::memory_order_acquire);
      const int64_t compliance_status_ns =
          latest_compliance_status_ns_.load(std::memory_order_acquire);
      const int64_t now_ns = steady_now_ns();
      const auto is_fresh = [now_ns](int64_t stamp_ns) {
        const int64_t age_ns =
            stamp_ns > 0 ? now_ns - stamp_ns
                         : std::numeric_limits<int64_t>::max();
        return age_ns >= 0 && age_ns <= kRuntimeControlStatusMaxAgeNs;
      };
      RuntimeTareControlState controls;
      controls.trajectory_status_fresh = is_fresh(trajectory_status_ns);
      controls.trajectory_active =
          latest_trajectory_active_.load(std::memory_order_acquire);
      controls.force_enable_status_fresh = is_fresh(force_status_ns);
      controls.force_enabled =
          latest_force_enabled_.load(std::memory_order_acquire);
      controls.compliance_enable_status_fresh =
          is_fresh(compliance_enable_status_ns);
      controls.compliance_enabled =
          latest_compliance_enabled_.load(std::memory_order_acquire);
      controls.compliance_status_fresh = is_fresh(compliance_status_ns);
      controls.compliance_active =
          latest_compliance_active_.load(std::memory_order_acquire);
      const bool generation_unchanged =
          runtime_control_violation_generation_.load(std::memory_order_acquire) ==
          freedrive_request_control_generation_.load(std::memory_order_acquire);
      const bool entry_still_safe = generation_unchanged &&
          !motion_inhibit_latched_.load(std::memory_order_acquire) &&
          latest_robot_safety_clear_.load(std::memory_order_acquire) &&
          validate_runtime_tare_control_state(controls) ==
              RuntimeTareRejection::None;
      // Stop first, then switch mode.  This closes the edge where a new JTC
      // reference could arrive after the service-side snapshot but before
      // free-drive actually suspends servo writes.
      const bool stopped = entry_still_safe && robot_->stop();
      const bool ok = stopped && robot_->set_freedrive_mode(true);
      freedrive_on_.store(ok, std::memory_order_release);
      if (ok) {
        RCLCPP_WARN(getLogger(),
                    "free-drive ON — servo 중단, 손으로 로봇 이동 가능. "
                    "OFF 하려면 ~/set_freedrive {data: false}.");
      } else {
        RCLCPP_WARN(
            getLogger(),
            "free-drive ON 요청 실패 (interlock changed, stop failed, or cobot rejected mode)");
      }
    } else if (req == 2) {  // turn OFF
      const bool was_on = freedrive_on_.load(std::memory_order_acquire);
      const bool ok = robot_->set_freedrive_mode(false);
      freedrive_on_.store(!ok && was_on, std::memory_order_release);
      if (ok && was_on) {
        // Hand-guiding invalidates every open-loop trajectory/reference that
        // was active before or arrived during free-drive.  Do not resume it
        // after a timed hold: close the hardware write boundary permanently
        // and require controllers to relaunch from the new measured pose.
        hw_position_commands_ = hw_position_states_;
        post_freedrive_hold_cycles_.store(0, std::memory_order_release);
        bool expected = false;
        if (motion_inhibit_latched_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
          motion_stop_requested_.store(true, std::memory_order_release);
        }
        RCLCPP_WARN(
            getLogger(),
            "free-drive OFF — hardware motion inhibited; fully relaunch "
            "ros2_control before any servo motion");
      } else if (!ok) {
        RCLCPP_ERROR(getLogger(), "free-drive OFF 요청 실패 (cobot 거부)");
      }
    }
    if (freedrive_transition_in_progress_.load(std::memory_order_acquire)) {
      freedrive_transition_in_progress_.store(false, std::memory_order_release);
    }
  }
  if (suppress_inhibited_motion()) {
    return hardware_interface::return_type::OK;
  }
  if (freedrive_on_.load(std::memory_order_acquire)) {
    return hardware_interface::return_type::OK;  // no move_servo_j while hand-guiding
  }

  // Recoverable states (startup initialization, arm power off, or external
  // direct-teach/free-drive) suppress writes without setting the permanent
  // abort latch.  Once a fresh sample reports the complete safety predicate
  // clear, normal servo output may resume.
  if (!latest_robot_safety_clear_.load(std::memory_order_acquire)) {
    static rclcpp::Clock unsafe_write_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
        getLogger(), unsafe_write_clock, 2000,
        "servo write suppressed while a recoverable robot safety/startup "
        "interlock is not clear");
    return hardware_interface::return_type::OK;
  }

  if (!last_robot_sample_valid_) {
    static rclcpp::Clock stale_write_clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(getLogger(), stale_write_clock, 2000,
                          "servo write suppressed because no fresh robot state sample is available");
    return hardware_interface::return_type::OK;
  }

  bool running = false;
  for (auto info : command_interface_infos_) {
    running |= info.state.running;
  }
  // Reuse the SystemState that read() just fetched. ros2_control invokes
  // read() before write() each control cycle, so last_read_state_ is fresh
  // here (and is only consumed in this branch, which already gated on
  // !fake_mode_ -> robot_ exists -> read() populated the cache).
  const auto& data = last_read_state_;
  // Original strict guard - returned ERROR on any post-mode-switch Idle
  // transition, which deactivated the whole controller stack. This is wrong
  // for admittance + JTC because the cobot legitimately returns to Idle at
  // every end-of-trajectory. Kept here for reference.
  // if (running) {
  //   if (mode_changed_) {
  //     if (data.sdata.robot_state != 1) {
  //       mode_changed_ = false;
  //     }
  //   } else {
  //     if (data.sdata.robot_state == 1) {
  //       RCLCPP_ERROR(getLogger(), "Unexpected robot state changed");
  //       return hardware_interface::return_type::ERROR;
  //     }
  //   }
  // }

  // Diagnostic version: same state tracking, but reports unexpected Idle as
  // a throttled WARN instead of aborting the write cycle. Keeps visibility
  // into cobot-side faults (collision / estop / refused commands) while not
  // breaking normal admittance operation. Re-arms mode_changed_ to suppress
  // spam during steady-state Idle stretches.
  if (running) {
    if (mode_changed_) {
      if (data.sdata.robot_state != 1) {
        mode_changed_ = false;
      }
    } else {
      if (data.sdata.robot_state == 1) {
        static rclcpp::Clock state_warn_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            getLogger(), state_warn_clock, 2000,
            "cobot returned to Idle (robot_state == 1) while commands are "
            "active. Normal at end-of-trajectory; if persistent during a "
            "motion, check the teach pendant for collision / safety stop.");
        mode_changed_ = true;
      }
    }
  }

  if (!joint_position_interface_state_.running) {
    hw_position_commands_ = hw_position_states_;
  }
  if (!cartesian_pose_interface_state_.running) {
    hw_cartesian_pose_commands_[0] = data.sdata.tcp_pos[0] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[1] = data.sdata.tcp_pos[1] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[2] = data.sdata.tcp_pos[2] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[3] = data.sdata.tcp_pos[3] * DEG2RAD;
    hw_cartesian_pose_commands_[4] = data.sdata.tcp_pos[4] * DEG2RAD;
    hw_cartesian_pose_commands_[5] = data.sdata.tcp_pos[5] * DEG2RAD;
  }

  const int hold_cycles = post_freedrive_hold_cycles_.load(std::memory_order_acquire);
  if (hold_cycles > 0) {
    hw_position_commands_ = hw_position_states_;
    hw_velocity_commands_.fill(0.0);
    hw_effort_commands_.fill(0.0);
    if (post_freedrive_hold_cycles_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      RCLCPP_INFO(getLogger(), "post free-drive hold complete");
    }
  }

  // Per-joint command clamp around the current measured state. Bounds how far
  // any upstream controller (incl. admittance with small stiffness) can push
  // the commanded position away from where the cobot actually is in a single
  // cycle. Without this, K=0/1 admittance can integrate residual F/T into
  // multi-radian command drift that the cobot can never catch up to; with
  // this, the divergence is mechanically capped while preserving normal
  // small-step tracking behavior.
  if (joint_position_interface_state_.running) {
    constexpr double kMaxCommandOffset{0.1};  // rad (~5.7 deg)
    for (size_t i = 0; i < kNumberOfJoints; ++i) {
      const double delta = hw_position_commands_[i] - hw_position_states_[i];
      if (delta > kMaxCommandOffset) {
        hw_position_commands_[i] = hw_position_states_[i] + kMaxCommandOffset;
      } else if (delta < -kMaxCommandOffset) {
        hw_position_commands_[i] = hw_position_states_[i] - kMaxCommandOffset;
      }
    }
  }

  // Diagnostic clocks for throttled logging of SDK rejection / divergence.
  static rclcpp::Clock sdk_reject_clock(RCL_STEADY_TIME);
  static rclcpp::Clock divergence_clock(RCL_STEADY_TIME);

  // Recheck immediately before the first SDK motion write.  The abort
  // callback runs concurrently with controller update calculations; without
  // this second boundary one additional 10 ms servo command could escape
  // after the start-of-cycle check.
  if (suppress_inhibited_motion()) {
    return hardware_interface::return_type::OK;
  }

  if (isValidCommand(hw_position_commands_) && joint_position_interface_state_.running) {
    const bool ok = robot_->write_once_joint_positions(hw_position_commands_);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(getLogger(), sdk_reject_clock, 2000,
                           "move_servo_j returned failure - cobot rejected the command. "
                           "Check operation mode (Remote vs Local) on the teach pendant.");
    }
    // Divergence between what we asked for and what the cobot reports it is
    // referencing. If the cobot stops honoring move_servo_j but we keep
    // sending new commands, this gap will grow rapidly.
    double max_err = 0.0;
    for (size_t i = 0; i < kNumberOfJoints; ++i) {
      max_err = std::max(max_err, std::abs(hw_position_commands_[i] - hw_position_states_[i]));
    }
    if (max_err > 0.05) {  // ~3 deg
      RCLCPP_WARN_THROTTLE(
          getLogger(), divergence_clock, 2000,
          "command vs measured joint divergence = %.3f rad (~%.1f deg). "
          "If growing, cobot is ignoring commands despite SDK ACK.",
          max_err, max_err * 180.0 / M_PI);
    }
  }
  if (isValidCommand(hw_velocity_commands_) && joint_velocity_interface_state_.running) {
    robot_->write_once_joint_velocities(hw_velocity_commands_);
  }
  if (isValidCommand(hw_effort_commands_) && joint_effort_interface_state_.running) {
    robot_->write_once_joint_efforts(hw_effort_commands_);
  }
  if (isValidCommand(hw_cartesian_pose_commands_) && cartesian_pose_interface_state_.running) {
    robot_->write_once_cartesian_pose(hw_cartesian_pose_commands_);
  }
  if (isValidCommand(hw_cartesian_velocity_commands_) && cartesian_velocity_interface_state_.running) {
    robot_->write_once_cartesian_velocity(hw_cartesian_velocity_commands_);
  }

  // Keepalive: when no controller is currently claiming a command interface,
  // hold the live joint pose via move_servo_j so the cobot stays in servo
  // mode continuously. Without this, the t2 timeout (~100 ms) inside
  // move_servo_j elapses with no fresh command and the cobot transitions back
  // to Idle. That happens during the launch warm-up window (before JTC /
  // admittance activate), between controller switches, and any time the
  // upstream controller stops emitting valid commands. The Idle -> servo
  // transition that the next trajectory would otherwise have to wait through
  // is also eliminated. on_deactivate calls robot_->stop() explicitly so the
  // cobot still returns to Idle when the hardware lifecycle steps down
  // (i.e. MoveIt shutdown).
  if (!running) {
    robot_->write_once_joint_positions(hw_position_states_);
  }

  return hardware_interface::return_type::OK;
}

rclcpp::Logger RBPodoHardwareInterface::getLogger() {
  return rclcpp::get_logger("RBPodoHardwareInterface");
}

}  // namespace rbpodo_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(rbpodo_hardware::RBPodoHardwareInterface, hardware_interface::SystemInterface)
