#pragma once
#include "isaac_painting_hardware/state_contract.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <isaac_painting_msgs/msg/state.hpp>
#include <isaac_painting_msgs/msg/command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace isaac_painting_hardware {
class IsaacPaintingSystem : public hardware_interface::SystemInterface {
 public:
  ~IsaacPaintingSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  friend class IsaacPaintingSystemTestPeer;
  static double steady();
  void on_sample(const isaac_painting_msgs::msg::State&);
  void watchdog();
  void tare(std::shared_ptr<std_srvs::srv::Trigger::Response>);
  bool controls_off(double) const;
  void publish_command(bool enabled, const Joints& positions);
  std::unique_ptr<StateContract> contract_;
  std::string session_, revision_;
  std::array<std::string,6> joint_names_{JOINTS};
  std::string topic_namespace_, control_namespace_, wrench_frame_, base_frame_, sensor_name_;
  std::string control_topic(const std::string& suffix) const {return control_namespace_+suffix;}
  builtin_interfaces::msg::Time source_stamp_;
  std::mutex mutex_;
  Joints position_{}, velocity_{}, effort_{}, command_{};
  Wrench ft_{}, raw_{}, bias_{};
  std::array<bool,4> control_active_{};
  std::array<double,4> control_seen_{};
  bool free_space_confirmed_{false};
  double free_space_seen_{};
  uint64_t violations_{}, sequence_{};
  std::deque<Joints> recent_joints_;
  bool active_{false}, inhibited_{false}, tared_{false}, ever_active_{false}, tare_running_{false};
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<isaac_painting_msgs::msg::State>::SharedPtr state_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> controls_;
  rclcpp::Publisher<isaac_painting_msgs::msg::Command>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inhibited_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
