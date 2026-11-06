#include <gtest/gtest.h>
#include <future>
#include "isaac_painting_hardware/system.hpp"

namespace isaac_painting_hardware {
// Exercise the actual blocking service implementation without a DDS graph or
// simulator. The feeder uses the production sample acceptance/history path.
class IsaacPaintingSystemTestPeer {
 public:
  static void initialize(IsaacPaintingSystem& system) {
    system.contract_=std::make_unique<StateContract>("run", "scene");
    system.wrench_frame_="ft";
    system.active_=system.ever_active_=true;
    refresh_controls(system);
  }
  static void refresh_controls(IsaacPaintingSystem& system) {
    std::lock_guard<std::mutex> lock(system.mutex_);
    system.control_seen_.fill(IsaacPaintingSystem::steady());
    system.free_space_seen_=IsaacPaintingSystem::steady();
    system.free_space_confirmed_=true;
  }
  static void feed(IsaacPaintingSystem& system, double joint_position) {
    isaac_painting_msgs::msg::State sample;
    sample.session_id="run";sample.scene_revision="scene";
    {
      std::lock_guard<std::mutex> lock(system.mutex_);
      sample.step_index=system.contract_->step()+1;
    }
    sample.header.stamp.sec=sample.step_index/100;
    sample.header.stamp.nanosec=(sample.step_index%100)*10000000;
    sample.header.frame_id="ft";
    sample.joint_names.assign(JOINTS.begin(),JOINTS.end());
    sample.positions.assign(6,joint_position);
    sample.velocities.assign(6,0.);sample.efforts.assign(6,0.);
    sample.wrench.force.x=.1;
    sample.playing=sample.joints_valid=sample.wrench_valid=true;
    system.on_sample(sample);
  }
  static std::shared_ptr<std_srvs::srv::Trigger::Response> tare(IsaacPaintingSystem& system) {
    auto response=std::make_shared<std_srvs::srv::Trigger::Response>();
    system.tare(response);return response;
  }
  static bool busy(IsaacPaintingSystem& system) {
    std::lock_guard<std::mutex> lock(system.mutex_);return system.tare_running_;
  }
  static bool tared(IsaacPaintingSystem& system) {
    std::lock_guard<std::mutex> lock(system.mutex_);return system.tared_;
  }
  static void invalidate(IsaacPaintingSystem& system, int condition) {
    std::lock_guard<std::mutex> lock(system.mutex_);
    switch(condition) {
      case 0: system.active_=false;break;
      case 1: system.inhibited_=true;break;
      case 2: system.violations_++;break;
      case 3: system.control_active_[0]=true;break;
      case 4: system.free_space_confirmed_=false;break;
      case 5: system.contract_->accept(Sample{},IsaacPaintingSystem::steady());break;
    }
  }
};
using Peer=IsaacPaintingSystemTestPeer;
using namespace std::chrono_literals;
class TareReadiness : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {if(!rclcpp::ok()) rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    Peer::initialize(system);
    for(int i=0;i<50;i++) Peer::feed(system,i%2?.002:0.);
  }
  IsaacPaintingSystem system;
};
TEST_F(TareReadiness, WaitsForFiftyStationarySamplesThenMeanAndIndependentResidual) {
  auto pending=std::async(std::launch::async,[this]{return Peer::tare(system);});
  EXPECT_EQ(pending.wait_for(30ms),std::future_status::timeout);
  EXPECT_TRUE(Peer::busy(system));
  EXPECT_FALSE(Peer::tared(system));
  auto concurrent=Peer::tare(system);
  EXPECT_FALSE(concurrent->success);
  int supplied=0;
  while(pending.wait_for(0ms)!=std::future_status::ready && supplied<350) {
    Peer::refresh_controls(system);Peer::feed(system,0.);supplied++;
    if(supplied<174) EXPECT_FALSE(Peer::tared(system));
    std::this_thread::sleep_for(12ms);
  }
  auto response=pending.get();
  EXPECT_TRUE(response->success)<<response->message;
  EXPECT_GE(supplied,174); // 50-sample readiness, 100 mean, 25 independent.
  EXPECT_TRUE(Peer::tared(system));EXPECT_FALSE(Peer::busy(system));
}
TEST_F(TareReadiness, MotionNeverSettlingExpiresWithinOriginalSamplingDeadline) {
  auto start=std::chrono::steady_clock::now();
  auto pending=std::async(std::launch::async,[this]{return Peer::tare(system);});
  int supplied=0;
  while(pending.wait_for(0ms)!=std::future_status::ready && supplied<500) {
    Peer::refresh_controls(system);Peer::feed(system,supplied++%2?.002:0.);
    std::this_thread::sleep_for(12ms);
  }
  auto response=pending.get();
  auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  EXPECT_FALSE(response->success);EXPECT_GE(elapsed,4.9);EXPECT_LT(elapsed,5.5);
  EXPECT_FALSE(Peer::tared(system));EXPECT_FALSE(Peer::busy(system));
}
TEST_F(TareReadiness, LostInterlockOrInvalidDataDuringReadinessCannotRecover) {
  for(int condition=0;condition<6;condition++) {
    IsaacPaintingSystem candidate;Peer::initialize(candidate);
    for(int i=0;i<50;i++) Peer::feed(candidate,i%2?.002:0.);
    auto pending=std::async(std::launch::async,[&candidate]{return Peer::tare(candidate);});
    EXPECT_EQ(pending.wait_for(30ms),std::future_status::timeout)<<condition;
    Peer::invalidate(candidate,condition);
    EXPECT_EQ(pending.wait_for(100ms),std::future_status::ready)<<condition;
    auto response=pending.get();
    EXPECT_FALSE(response->success);EXPECT_FALSE(Peer::tared(candidate));EXPECT_FALSE(Peer::busy(candidate));
  }
}
TEST_F(TareReadiness, MissingFreshSamplesDuringReadinessRejectsBeforeDeadline) {
  auto pending=std::async(std::launch::async,[this]{return Peer::tare(system);});
  EXPECT_EQ(pending.wait_for(30ms),std::future_status::timeout);
  // Keep controls fresh while deliberately stalling the measured stream.
  for(int i=0;i<30 && pending.wait_for(0ms)!=std::future_status::ready;i++) {
    Peer::refresh_controls(system);std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(pending.wait_for(20ms),std::future_status::ready);
  EXPECT_FALSE(pending.get()->success);EXPECT_FALSE(Peer::tared(system));
}
}  // namespace isaac_painting_hardware
