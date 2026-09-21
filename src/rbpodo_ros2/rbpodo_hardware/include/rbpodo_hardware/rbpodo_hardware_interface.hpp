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

#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string_view>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rbpodo_hardware/hardware_safety.hpp"
#include "rbpodo_hardware/robot.hpp"
#include "rbpodo_hardware/robot_node.hpp"
#include "rbpodo_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rbpodo_hardware {

class RBPodoHardwareInterface : public hardware_interface::SystemInterface {
 public:
  static constexpr size_t kNumberOfJoints{6};

  static constexpr size_t k6DoFDim{6};
  const std::string HW_IF_CARTESIAN_POSE{"cartesian_pose"};
  const std::string HW_IF_CARTESIAN_VELOCITY{"cartesian_velocity"};

  const std::array<std::string, k6DoFDim> kCartesianPosePrefix{"x", "y", "z", "rx", "ry", "rz"}; // unit: m, rad
  const std::array<std::string, k6DoFDim> kCartesianVelocityPrefix{"x", "y", "z", "rx", "ry", "rz"}; // unit: m, rad

  // RCLCPP_SHARED_PTR_DEFINITIONS(RBPodoHardwareInterface)

  RBPodoHardwareInterface();
  /// `RBPodoHardwareInterface` is not copyable
  RBPodoHardwareInterface(const RBPodoHardwareInterface&) = delete;
  /// `RBPodoHardwareInterface` is not copyable
  RBPodoHardwareInterface& operator=(const RBPodoHardwareInterface& other) = delete;
  /// `RBPodoHardwareInterface` is movable
  RBPodoHardwareInterface& operator=(RBPodoHardwareInterface&& other) = delete;
  /// `RBPodoHardwareInterface` is movable
  RBPodoHardwareInterface(RBPodoHardwareInterface&& other) = delete;

  virtual ~RBPodoHardwareInterface();

  rclcpp::Logger getLogger();

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RBPODO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct CommandInterfaceState {
    bool claimed{false};
    bool running{false};
  };

  struct CommandInterfaceInfo {
    std::string type;
    size_t size;
    CommandInterfaceState& state;
  };

  std::vector<CommandInterfaceInfo> command_interface_infos_;
  bool mode_changed_{false};

  CommandInterfaceState joint_position_interface_state_;
  CommandInterfaceState joint_velocity_interface_state_;
  CommandInterfaceState joint_effort_interface_state_;
  CommandInterfaceState cartesian_pose_interface_state_;
  CommandInterfaceState cartesian_velocity_interface_state_;

  std::shared_ptr<Robot> robot_;
  std::array<double, kNumberOfJoints> torque_constants_;

  // When fake_mode_ is true, no Robot is constructed and read()/write() do not
  // talk to the cobot. Joint states mirror the commands, eft_* defaults to 0,
  // and the same tare/filter pipeline runs — so injecting via ~/inject_ft
  // exercises the filter end-to-end without needing a real arm.
  bool fake_mode_{false};

  // Driven by the URDF 'fake_sensor_commands' hardware parameter. When true,
  // the ~/inject_ft subscription is created AND the injected values override
  // the raw eft_* feed in read(). When false, the subscription is never
  // created and the real eft_* (or zero, in fake_mode_) is used unchanged.
  bool inject_ft_enabled_{false};

  // command interface
  std::array<double, kNumberOfJoints> hw_position_commands_{};
  std::array<double, kNumberOfJoints> hw_velocity_commands_{};
  std::array<double, kNumberOfJoints> hw_effort_commands_{};
  std::array<double, k6DoFDim> hw_cartesian_pose_commands_{};
  std::array<double, k6DoFDim> hw_cartesian_velocity_commands_{};

  // states
  std::array<double, kNumberOfJoints> hw_position_states_{};
  std::array<double, kNumberOfJoints> hw_effort_states_{};
  std::array<double, k6DoFDim> hw_ft_states_{};  // fx, fy, fz, tx, ty, tz
  // Pre-deadband, pre-clamp wrench exposed via a second ros2_control sensor
  // ("ft_sensor_raw"). Tare bias is still subtracted so the signal is the
  // same calibrated force the admittance controller would see, just without
  // the human_collab_ deadband / clamp masking small or large forces.
  // Populated alongside hw_ft_states_ in read(); NaN while the source or tare
  // is invalid so downstream controllers cannot mistake missing data for 0 N.
  std::array<double, k6DoFDim> hw_ft_raw_states_{};

  // F/T tare:
  //   On activate (or via the /tare_ft service), average the first kFtTareSamples
  //   raw eft_* readings and subtract that bias from subsequent reads so admittance
  //   sees ~0 in free space. While the tare is running OR if it failed validation,
  //   both exported F/T sensors are NaN so compliance fails closed.
  enum class TareState : uint8_t { Idle, Running, Ok, Failed };
  enum class TareMode : uint8_t { None, Strict, Supervised, Runtime };
  enum class RuntimeTarePhase : uint8_t { None, CollectBias, ValidateResidual };

  static constexpr size_t kFtTareSamples{100};        // ~1 s at 100 Hz
  static constexpr size_t kRuntimeTarePostSamples{25};  // ~0.25 s at 100 Hz
  // Validation thresholds: above these we assume the arm is loaded / in contact /
  // vibrating, and we refuse to tare instead of baking the contact force into bias.
  static constexpr double kFtTareMaxAbsMeanForce{80.0};   // [N]  per axis (gravity of ~8 kg payload)
  static constexpr double kFtTareMaxAbsMeanTorque{10.0};  // [Nm]
  static constexpr double kFtTareMaxStdForce{5.0};        // [N]  sample std
  static constexpr double kFtTareMaxStdTorque{1.0};       // [Nm]

  // The supervised path is deliberately separate from the strict automatic
  // and ~/tare_ft paths above.  Its defaults describe the commissioned raw
  // free-space baseline of this RB10/AFT installation, rather than merely
  // raising the generic 80 N strict limit.  They are exposed as parameters on
  // /rbpodo_ft_tare so a new commissioned sensor/tool can replace them.
  static constexpr HardwareWrench kDefaultSupervisedExpectedBias{
      65.77, 2.47, 276.00, -1.955, 0.997, -0.1155};
  static constexpr HardwareWrench kDefaultSupervisedMaxAbsBiasDelta{
      1.0, 1.0, 1.0, 0.1, 0.1, 0.05};
  static constexpr HardwareWrench kDefaultSupervisedMaxStddev{
      0.5, 0.5, 0.5, 0.05, 0.05, 0.05};
  static constexpr HardwareJointPositions kDefaultSupervisedExpectedJointPositions{
      0.0003189465, -0.9519414306, 2.4623770714,
      -1.6286282539, 1.5665779114, -0.00009492455};
  static constexpr HardwareJointPositions kDefaultSupervisedMaxAbsJointPositionDeltaRad{
      0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  static constexpr size_t kDefaultSupervisedRecentJointSamples{50};  // ~0.5 s at 100 Hz
  static constexpr double kDefaultSupervisedMaxRecentJointExcursionRad{0.0005};
  static constexpr double kDefaultSupervisedMaxTareJointExcursionRad{0.001};
  static constexpr size_t kJointHistoryCapacity{100};

  // Repeatable execution-scoped tare.  It has no expected raw bias or fixed
  // joint-pose envelope; the request is valid only while the robot is
  // stationary in explicitly confirmed free space and every motion/force
  // control heartbeat is fresh and false.
  static constexpr HardwareWrench kDefaultRuntimeMaxStddev{
      0.5, 0.5, 0.5, 0.05, 0.05, 0.05};
  static constexpr HardwareWrench kDefaultRuntimeMaxAbsPostTareResidual{
      0.5, 0.5, 0.5, 0.05, 0.05, 0.05};
  static constexpr size_t kDefaultRuntimeRecentJointSamples{50};  // ~0.5 s
  static constexpr double kDefaultRuntimeMaxRecentJointExcursionRad{0.0005};
  static constexpr double kDefaultRuntimeMaxTareJointExcursionRad{0.001};
  static constexpr int64_t kRuntimeControlStatusMaxAgeNs{250000000};  // 0.25 s

  std::array<double, k6DoFDim> ft_bias_{};
  std::array<double, k6DoFDim> ft_tare_accum_{};
  std::array<double, k6DoFDim> ft_tare_accum_sq_{};
  size_t ft_tare_remaining_{0};
  size_t ft_tare_sample_target_{kFtTareSamples};
  std::atomic<TareMode> ft_tare_requested_mode_{TareMode::None};
  TareMode ft_tare_active_mode_{TareMode::None};
  std::atomic<TareState> ft_tare_state_{TareState::Idle};
  std::atomic<StrictTareOutcome> strict_tare_outcome_{StrictTareOutcome::NotCompleted};
  std::atomic<bool> ft_tare_service_in_progress_{false};

  // Pending config is populated by the supervised service callback before its
  // release-store request, then copied by read() after the matching exchange.
  // The request atomic therefore provides the cross-thread synchronization.
  SupervisedTareConfig supervised_tare_pending_config_{};
  SupervisedTareConfig supervised_tare_active_config_{};
  std::atomic<int64_t> supervised_tare_pending_deadline_ns_{0};
  int64_t supervised_tare_active_deadline_ns_{0};
  std::atomic<SupervisedTareRejection> supervised_tare_rejection_{
      SupervisedTareRejection::None};
  std::atomic<bool> supervised_tare_attempted_this_activation_{false};
  std::atomic<bool> supervised_tare_cancel_requested_{false};
  JointSampleWindow supervised_recent_joint_window_{};
  JointSampleWindow supervised_tare_joint_window_{};
  bool supervised_tare_source_window_valid_{false};
  bool supervised_tare_safety_window_valid_{false};

  RuntimeTareConfig runtime_tare_pending_config_{};
  RuntimeTareConfig runtime_tare_active_config_{};
  std::atomic<int64_t> runtime_tare_pending_deadline_ns_{0};
  int64_t runtime_tare_active_deadline_ns_{0};
  std::atomic<RuntimeTareRejection> runtime_tare_rejection_{
      RuntimeTareRejection::None};
  std::atomic<bool> runtime_tare_cancel_requested_{false};
  RuntimeTarePhase runtime_tare_phase_{RuntimeTarePhase::None};
  HardwareWrench runtime_tare_candidate_bias_{};
  JointSampleWindow runtime_recent_joint_window_{};
  JointSampleWindow runtime_tare_joint_window_{};
  bool runtime_tare_source_window_valid_{false};
  bool runtime_tare_safety_window_valid_{false};
  RuntimeTareControlState runtime_tare_control_window_state_{};
  uint64_t runtime_tare_pending_control_violation_generation_{0};
  uint64_t runtime_tare_active_control_violation_generation_{0};

  // Fresh actual-joint history used to prove the arm was already stationary
  // before the operator confirmed free space.  Any stale joint sample clears
  // the history instead of allowing old stationary samples to qualify.
  std::array<HardwareJointPositions, kJointHistoryCapacity> joint_history_{};
  size_t joint_history_next_{0};
  size_t joint_history_count_{0};
  int64_t joint_history_last_sample_ns_{0};
  std::atomic<bool> latest_ft_source_valid_{false};
  std::atomic<bool> latest_joint_source_valid_{false};
  std::atomic<bool> latest_robot_safety_clear_{false};
  // Startup may legitimately report init/power not-ready.  Once a complete
  // healthy real-robot sample has been observed, losing that predicate must
  // latch motion inhibit so an open-loop JTC goal cannot resume at a future
  // setpoint when the condition clears.
  std::atomic<bool> has_observed_robot_safety_clear_{false};
  std::atomic<int64_t> latest_fresh_sample_ns_{0};

  // Fresh fail-closed heartbeats proving the pre-contact tare is not racing a
  // trajectory, non-zero force request, armed compliance command, or active
  // admittance compliance.
  std::atomic<bool> latest_trajectory_active_{true};
  std::atomic<int64_t> latest_trajectory_status_ns_{0};
  std::atomic<bool> latest_force_enabled_{true};
  std::atomic<int64_t> latest_force_enable_status_ns_{0};
  std::atomic<bool> latest_compliance_enabled_{true};
  std::atomic<int64_t> latest_compliance_enable_status_ns_{0};
  std::atomic<bool> latest_compliance_active_{true};
  std::atomic<int64_t> latest_compliance_status_ns_{0};
  std::atomic<uint64_t> runtime_control_violation_generation_{0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compliance_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compliance_active_sub_;

  // A true /motion_abort is a hardware-activation-scoped motion inhibit.  It
  // is deliberately not reset by a ROS service: after an abort the complete
  // ros2_control stack must be relaunched before servo writes can resume.
  std::atomic<bool> motion_inhibit_latched_{false};
  std::atomic<bool> motion_stop_requested_{false};
  std::atomic<bool> motion_inhibit_acknowledged_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_abort_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_inhibit_status_pub_;
  rclcpp::TimerBase::SharedPtr motion_inhibit_status_timer_;

  // Human-collaboration safety clamp + deadband. When human_collab_ is true:
  //   * |v| < kFtCollabDeadband -> v = 0   (rejects sensor noise so admittance
  //     with stiffness=0 does not integrate residual drift into unintended
  //     joint motion; this is the root cause of cobot dropping out of Moving
  //     when no human is actually pushing).
  //   * |v| > kFtCollabClamp    -> v = +/-kFtCollabClamp (caps how much wrench
  //     admittance can ever react to even if the raw signal goes higher).
  // Applies regardless of the data source (real eft_* or injected).
  static constexpr double kFtCollabDeadband{1.2};  // [N] for force axes, [Nm] for torque axes
  static constexpr double kFtCollabClamp{30.0};    // [N] for force axes, [Nm] for torque axes
  bool human_collab_{false};

  // F/T injection (debug / live test of the spike filter):
  //   Subscribes to ~/inject_ft (std_msgs/Float64MultiArray, 6 elements).
  //   When active, the injected values REPLACE the raw eft_* readings before
  //   tare and the spike filter run, so the same code path the real signal
  //   would take is exercised (tare bias, median, hold, slew, broadcaster,
  //   admittance, JTC, MoveIt). Send an empty array to clear the injection
  //   and revert to the real eft_* feed.
  // ft_inject_active_ is checked lock-free on every read() cycle as a fast path
  // (injection is the rare case). Only when active do we acquire ft_inject_mutex_
  // to safely copy ft_inject_values_ — the callback always takes the mutex when
  // mutating either, so the values seen under the lock are coherent.
  std::mutex ft_inject_mutex_;
  std::array<double, k6DoFDim> ft_inject_values_{};
  std::atomic<bool> ft_inject_active_{false};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ft_inject_sub_;

  // Cached SystemState from the most recent read(). write() reuses this instead
  // of re-issuing robot_->read_once(), which saves one network RTT per control
  // cycle in real-hardware mode. Only consumed in write() when !fake_mode_,
  // which implies robot_ exists and read() populated this on the same cycle.
  rb::podo::SystemState last_read_state_{};
  // False until read() observes a fresh SDK sample. write() suppresses all
  // servo commands while false and resumes automatically on a fresh sample.
  bool last_robot_sample_valid_{false};

  // ROS Node
  std::shared_ptr<RobotNode> robot_node_;
  std::shared_ptr<RobotExecutor> robot_executor_;

  // Dedicated node hosting the re-tare service (spun on robot_executor_).
  rclcpp::Node::SharedPtr tare_node_;
  // Runtime-tare interlock heartbeats must remain executable while the
  // Trigger service waits for the RT read loop to collect its sample windows.
  // Keeping them in a callback group separate from the blocking services
  // prevents the node's default MutuallyExclusive group from starving these
  // safety updates.
  rclcpp::CallbackGroup::SharedPtr runtime_control_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr supervised_tare_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr runtime_tare_srv_;

  // Free-drive (direct-teaching) toggle via ~/set_freedrive (SetBool).
  //   The service callback only records a request; the actual cobot command
  //   (set_freedrive_mode) and the servo suspension happen on the RT read()/
  //   write() thread so all cobot socket access stays single-threaded.
  //   While freedrive_on_ is true, write() sends NO move_servo_j so the arm is
  //   hand-guidable; read() still runs so F/T (/aft200/ft) keeps publishing.
  //   Turning free-drive OFF permanently inhibits servo writes for this
  //   process.  Controllers must be reinitialized from the newly measured
  //   pose by a full ros2_control relaunch; an old open-loop trajectory is
  //   never allowed to resume after hand-guiding.
  //   freedrive_request_: 0 = none, 1 = turn on, 2 = turn off.
  std::atomic<bool> freedrive_on_{false};
  std::atomic<int> freedrive_request_{0};
  std::atomic<bool> freedrive_transition_in_progress_{false};
  // Control-interlock generation captured by the service and rechecked on
  // the RT write edge before free-drive can actually be enabled.
  std::atomic<uint64_t> freedrive_request_control_generation_{0};
  // After free-drive OFF, upstream controllers can still hold references from
  // before hand-guiding. Hold the measured joint pose for a short window while
  // the admittance/JTC chain is reset.
  std::atomic<int> post_freedrive_hold_cycles_{0};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr freedrive_srv_;
};

}  // namespace rbpodo_hardware
