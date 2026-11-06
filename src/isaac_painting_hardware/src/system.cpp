#include "isaac_painting_hardware/system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>

namespace isaac_painting_hardware {
using Return = hardware_interface::CallbackReturn;
using IO = hardware_interface::return_type;
double IsaacPaintingSystem::steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
IsaacPaintingSystem::~IsaacPaintingSystem() {
  if(executor_) executor_->cancel();
  if(thread_.joinable()) thread_.join();
}
Return IsaacPaintingSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if(SystemInterface::on_init(info)!=Return::SUCCESS) return Return::ERROR;
  try {session_=info.hardware_parameters.at("session_id"); revision_=info.hardware_parameters.at("scene_revision");}
  catch(const std::exception&){return Return::ERROR;}
  if(session_.empty() || revision_.empty() || info.joints.size()!=6 || info.sensors.size()!=2) return Return::ERROR;
  const auto parameter=[&info](const std::string& name,const std::string& fallback) {
    const auto it=info.hardware_parameters.find(name);
    return it==info.hardware_parameters.end()?fallback:it->second;
  };
  topic_namespace_=parameter("topic_namespace","/isaac/rb20_painting");
  control_namespace_=parameter("control_namespace","");
  wrench_frame_=parameter("wrench_frame","ft_link");
  base_frame_=parameter("base_frame","link0");
  sensor_name_=parameter("ft_sensor_name","ft_sensor");
  while(topic_namespace_.size()>1 && topic_namespace_.back()=='/') topic_namespace_.pop_back();
  while(!control_namespace_.empty() && control_namespace_.back()=='/') control_namespace_.pop_back();
  if(topic_namespace_.empty() || topic_namespace_[0]!='/' || topic_namespace_=="/" ||
     (!control_namespace_.empty() && control_namespace_[0]!='/') ||
     wrench_frame_.empty() || base_frame_.empty() || sensor_name_.empty()) return Return::ERROR;
  for(size_t i=0;i<6;i++) joint_names_[i]=info.joints[i].name;
  if(info.sensors[0].name!=sensor_name_ || info.sensors[1].name!=sensor_name_+"_raw") return Return::ERROR;
  try {contract_=std::make_unique<StateContract>(session_,revision_,joint_names_);}
  catch(const std::invalid_argument&){return Return::ERROR;}
  position_.fill(NaN);velocity_.fill(NaN);effort_.fill(NaN);command_.fill(NaN);ft_.fill(NaN);raw_.fill(NaN);
  rclcpp::NodeOptions options;
  // Tare is driven by steady time and must continue even while /clock is paused.
  options.use_global_arguments(false);
  node_=std::make_shared<rclcpp::Node>("rbpodo_ft_tare", control_namespace_.empty()?"/":control_namespace_, options);
  auto qos=rclcpp::QoS(1).reliable().durability_volatile();
  command_pub_=node_->create_publisher<isaac_painting_msgs::msg::Command>(topic_namespace_+"/command",qos);
  // Measured samples are replaceable: a dropped sample must not block physics
  // while a reliable reader acknowledges an older value. The independent
  // steady-time watchdog still rejects a missing/stalled stream after 0.20 s.
  state_sub_=node_->create_subscription<isaac_painting_msgs::msg::State>(topic_namespace_+"/state",rclcpp::SensorDataQoS().keep_last(1),
    [this](isaac_painting_msgs::msg::State::ConstSharedPtr msg){on_sample(*msg);});
  const std::array<std::string,4> topics={"/painting_admittance/trajectory_active","/painting_admittance/enable_force",
    "/admittance_controller/compliance_enable","/admittance_controller/compliance_active"};
  for(size_t i=0;i<4;i++) controls_.push_back(node_->create_subscription<std_msgs::msg::Bool>(control_topic(topics[i]),10,
    [this,i](std_msgs::msg::Bool::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);
      control_active_[i]=msg->data;control_seen_[i]=steady();if(msg->data) violations_++;}));
  controls_.push_back(node_->create_subscription<std_msgs::msg::Bool>(control_topic("/painting_admittance/free_space_confirmed"),10,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);
      free_space_confirmed_=msg->data;free_space_seen_=steady();if(!msg->data) violations_++;}));
  controls_.push_back(node_->create_subscription<std_msgs::msg::Bool>("/motion_abort",10,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg){if(msg->data){std::lock_guard<std::mutex> lock(mutex_);
      if(ever_active_){if(!inhibited_) RCLCPP_ERROR(node_->get_logger(),"Isaac motion inhibited: /motion_abort received");
        inhibited_=true;tared_=false;violations_++;publish_command(false,contract_->positions());}}}));
  inhibited_pub_=node_->create_publisher<std_msgs::msg::Bool>(control_topic("/painting_system/hardware_motion_inhibited"),rclcpp::QoS(1).transient_local());
  diagnostic_pub_=node_->create_publisher<std_msgs::msg::String>(topic_namespace_+"/hardware_status",rclcpp::QoS(1).transient_local());
  service_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tare_service_=node_->create_service<std_srvs::srv::Trigger>(control_topic("/rbpodo_ft_tare/runtime_free_space_tare"),
    [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){tare(response);},
    rmw_qos_profile_services_default,service_group_);
  timer_=node_->create_wall_timer(std::chrono::milliseconds(20),[this]{watchdog();});
  executor_=std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
  executor_->add_node(node_); thread_=std::thread([this]{executor_->spin();});
  return Return::SUCCESS;
}
std::vector<hardware_interface::StateInterface> IsaacPaintingSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for(size_t i=0;i<6;i++) {
    result.emplace_back(joint_names_[i],"position",&position_[i]);result.emplace_back(joint_names_[i],"velocity",&velocity_[i]);result.emplace_back(joint_names_[i],"effort",&effort_[i]);
  }
  const std::array<std::string,6> axes={"force.x","force.y","force.z","torque.x","torque.y","torque.z"};
  for(size_t i=0;i<6;i++){result.emplace_back(sensor_name_,axes[i],&ft_[i]);result.emplace_back(sensor_name_+"_raw",axes[i],&raw_[i]);}
  return result;
}
std::vector<hardware_interface::CommandInterface> IsaacPaintingSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for(size_t i=0;i<6;i++) result.emplace_back(joint_names_[i],"position",&command_[i]);
  return result;
}
Return IsaacPaintingSystem::on_activate(const rclcpp_lifecycle::State&) {
  const auto end=steady()+5.;
  while(steady()<end && rclcpp::ok()) {
    { std::lock_guard<std::mutex> lock(mutex_);
      if(contract_->fresh(steady()) && !inhibited_) {
        position_=contract_->positions();velocity_=contract_->velocities();effort_=contract_->efforts();command_=position_;
        active_=ever_active_=true;return Return::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(node_->get_logger(),"No matching live Isaac state; activation refused"); return Return::ERROR;
}
Return IsaacPaintingSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  std::lock_guard<std::mutex> lock(mutex_);active_=false;tared_=false;
  publish_command(false,contract_->positions());return Return::SUCCESS;
}
static Wrench array(const geometry_msgs::msg::Wrench& w){return {w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z};}
void IsaacPaintingSystem::on_sample(const isaac_painting_msgs::msg::State& msg) {
  Sample s; s.session=msg.session_id;s.revision=msg.scene_revision;s.step=msg.step_index;
  s.stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;s.names=msg.joint_names;
  s.position=msg.positions;s.velocity=msg.velocities;s.effort=msg.efforts;
  s.wrench=array(msg.wrench);s.gravity=array(msg.gravity_wrench);s.playing=msg.playing;
  s.joints_valid=msg.joints_valid;s.wrench_valid=msg.wrench_valid && msg.header.frame_id==wrench_frame_;
  std::lock_guard<std::mutex> lock(mutex_);
  if(contract_->accept(s,steady())) {
    source_stamp_=msg.header.stamp;
    recent_joints_.push_back(contract_->positions()); if(recent_joints_.size()>50) recent_joints_.pop_front();
  } else if(ever_active_ && contract_->reason()!="OLD_STATE") {
    if(!inhibited_) RCLCPP_ERROR(node_->get_logger(),"Isaac motion inhibited: invalid sample %s at step %lu",
      contract_->reason().c_str(),msg.step_index);
    inhibited_=true;tared_=false;recent_joints_.clear();violations_++;
  }
}
bool IsaacPaintingSystem::controls_off(double now) const {
  if(!free_space_confirmed_ || free_space_seen_<=0 || now-free_space_seen_>.25) return false;
  for(size_t i=0;i<4;i++) if(control_active_[i] || control_seen_[i]<=0 || now-control_seen_[i]>.25) return false;
  return true;
}
void IsaacPaintingSystem::publish_command(bool enabled,const Joints& positions) {
  isaac_painting_msgs::msg::Command msg;
  msg.session_id=session_;msg.scene_revision=revision_;msg.sequence=++sequence_;msg.enabled=enabled;
  // Echo the original source exactly: the simulator authenticates this sample's
  // steady-time lease. Converting through double loses nanoseconds.
  msg.header.stamp=source_stamp_;
  msg.header.frame_id=base_frame_;msg.positions.assign(positions.begin(),positions.end());
  command_pub_->publish(msg);
}
void IsaacPaintingSystem::watchdog() {
  std::lock_guard<std::mutex> lock(mutex_);
  if(ever_active_ && !contract_->fresh(steady())) {
    if(!inhibited_) RCLCPP_ERROR(node_->get_logger(),"Isaac motion inhibited: measured state exceeded 0.20 s lease at step %lu",contract_->step());
    inhibited_=true;tared_=false;
  }
  if(inhibited_) publish_command(false,contract_->positions());
  std_msgs::msg::Bool inhibit;inhibit.data=inhibited_;inhibited_pub_->publish(inhibit);
  std_msgs::msg::String diagnostic;
  diagnostic.data=std::string("{\"simulation\":true,\"session_id\":\"")+session_+"\",\"scene_revision\":\""+revision_+"\",\"inhibited\":"+(inhibited_?"true":"false")+
    ",\"tare_ready\":"+(tared_?"true":"false")+",\"state_fresh\":"+(contract_->fresh(steady())?"true":"false")+"}";
  diagnostic_pub_->publish(diagnostic);
}
IO IsaacPaintingSystem::read(const rclcpp::Time&,const rclcpp::Duration&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(contract_->fresh(steady())) {
    position_=contract_->positions();velocity_=contract_->velocities();effort_=contract_->efforts();
    // Untared is not invalid: expose the genuine gravity-compensated reading
    // so the original monitor can assess the stopped free-space condition.
    // The original executor still requires its runtime tare before contact.
    if(!inhibited_) for(size_t i=0;i<6;i++) raw_[i]=ft_[i]=contract_->compensated()[i]-(tared_?bias_[i]:0.);
    else {raw_.fill(NaN);ft_.fill(NaN);}
  } else {velocity_.fill(NaN);effort_.fill(NaN);raw_.fill(NaN);ft_.fill(NaN);}
  return IO::OK;
}
IO IsaacPaintingSystem::write(const rclcpp::Time&,const rclcpp::Duration&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(active_ && !inhibited_ && contract_->fresh(steady()) && finite(command_)) publish_command(true,command_);
  else if(active_) publish_command(false,contract_->positions());
  return IO::OK;
}
void IsaacPaintingSystem::tare(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  uint64_t generation;
  {std::lock_guard<std::mutex> lock(mutex_);
    if(tare_running_ || !active_ || inhibited_ || !contract_->fresh(steady()) || !controls_off(steady())) {
      response->message="Fresh stationary simulator, confirmed free space, force/compliance/trajectory off required";return;
    }
    tare_running_=true;tared_=false;generation=violations_;
  }
  TareWindow window(100);uint64_t last_step=0; bool failed=false, stationary_ready=false;std::string failure_reason;
  const double deadline=steady()+5.;
  while(steady()<deadline && rclcpp::ok()) {
    {std::lock_guard<std::mutex> lock(mutex_);
      const bool eligible=active_ && !inhibited_ && contract_->fresh(steady()) && controls_off(steady()) && generation==violations_;
      if(!eligible){failure_reason="interlock or input freshness changed during sampling";failed=true;break;}
      // Executor settling uses wall time; the authoritative 50 distinct
      // simulator samples can still contain physical settling. Wait for the
      // same motion bound inside the existing deadline, with every interlock
      // continuously checked. No baseline samples are collected before ready.
      if(!stationary_ready) {
        stationary_ready=recent_joints_.size()==50;
        if(stationary_ready) for(const auto& q:recent_joints_)
          for(size_t i=0;i<6;i++) if(std::abs(q[i]-recent_joints_.front()[i])>.0005)
            stationary_ready=false;
      }
      if(stationary_ready && contract_->step()!=last_step) {
        last_step=contract_->step();
        if(!window.add(contract_->compensated(),contract_->positions(),eligible)){failure_reason=window.reason();failed=true;break;}
        if(window.ready()) break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // Hold independent samples for post-tare residual verification; the mean
  // sample window must not verify itself.
  Wrench candidate=window.bias();int verified=0;
  const double verify_deadline=steady()+1.;
  while(!failed && window.ready() && verified<25 && steady()<verify_deadline && rclcpp::ok()) {
    {std::lock_guard<std::mutex> lock(mutex_);
      if(!active_ || inhibited_ || !contract_->fresh(steady()) || !controls_off(steady()) || generation!=violations_){failure_reason="interlock or input freshness changed during verification";failed=true;break;}
      if(contract_->step()!=last_step) {
        last_step=contract_->step();
        failure_reason=window.verification_error(contract_->compensated(),contract_->positions(),true);
        if(!failure_reason.empty()) failed=true;
        verified++;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::lock_guard<std::mutex> lock(mutex_);tare_running_=false;
  response->success=!failed && window.ready() && verified>=25 && generation==violations_ && active_ &&
    !inhibited_ && contract_->fresh(steady()) && controls_off(steady());
  if(response->success){bias_=candidate;tared_=true;response->message="Simulator free-space tare and independent residual check complete";}
  else response->message="Simulator tare failed: "+(failure_reason.empty()?"sampling/verification deadline or final interlock":failure_reason);
}
}
PLUGINLIB_EXPORT_CLASS(isaac_painting_hardware::IsaacPaintingSystem, hardware_interface::SystemInterface)
