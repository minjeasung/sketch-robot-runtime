#include <gtest/gtest.h>
#include "isaac_painting_hardware/state_contract.hpp"
using namespace isaac_painting_hardware;

Sample sample() {
  Sample s;
  s.session = "run"; s.revision = "scene"; s.step = 1; s.stamp = 1.;
  s.names = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
  s.position = {0., .1, .2, .3, .4, .5}; s.velocity = s.effort = std::vector<double>(6, 0.);
  s.wrench = {1, 2, 3, 4, 5, 6}; s.gravity = {1, 1, 1, 1, 1, 1};
  s.joints_valid = s.wrench_valid = s.playing = true;
  return s;
}
TEST(StateContract, AcceptsMeasuredZeroAndSubtractsGravityOnce) {
  StateContract c("run", "scene"); auto s = sample(); s.wrench = s.gravity;
  ASSERT_TRUE(c.accept(s, 10.)); EXPECT_TRUE(c.fresh(10.19));
  EXPECT_EQ(c.compensated(), Wrench({0,0,0,0,0,0}));
  EXPECT_DOUBLE_EQ(c.positions()[1], .1);
  EXPECT_FALSE(c.fresh(10.21));
}
TEST(StateContract, RejectsSessionRevisionAndDuplicateOrRewoundStep) {
  StateContract c("run", "scene"); auto s = sample();
  ASSERT_TRUE(c.accept(s, 10.));
  EXPECT_FALSE(c.accept(s, 10.19)); EXPECT_FALSE(c.fresh(10.21));
  s.step = 2; s.stamp = 1.01; s.session = "foreign"; EXPECT_FALSE(c.accept(s, 10.22));
  s.session = "run"; s.revision = "old"; EXPECT_FALSE(c.accept(s, 10.22));
  s.revision = "scene"; s.stamp = .9; EXPECT_FALSE(c.accept(s, 10.22));
}
TEST(StateContract, ReordersByNamesAndRejectsBadNumbersOrNames) {
  StateContract c("run", "scene"); auto s = sample();
  std::swap(s.names[0], s.names[1]); std::swap(s.position[0], s.position[1]);
  ASSERT_TRUE(c.accept(s, 1.)); EXPECT_DOUBLE_EQ(c.positions()[0], 0.);
  s.step++; s.stamp++; s.names[0] = s.names[1]; EXPECT_FALSE(c.accept(s, 2.));
  s = sample(); s.step = 3; s.stamp = 3; s.wrench[0] = NAN; EXPECT_FALSE(c.accept(s, 3.));
  s = sample(); s.position.pop_back(); EXPECT_FALSE(c.accept(s, 3.));
}
TEST(StateContract, PauseInvalidatesImmediately) {
  StateContract c("run", "scene"); auto s=sample(); ASSERT_TRUE(c.accept(s, 1.));
  s.step++; s.stamp++; s.playing=false;
  EXPECT_FALSE(c.accept(s, 1.01)); EXPECT_FALSE(c.fresh(1.01));
}
TEST(TareWindow, StableSamplesProduceBiasButMotionOrForceGateReject) {
  TareWindow t(50); Wrench w={.1,.2,.3,.01,.02,.03}; Joints q{};
  for(int i=0;i<49;i++) { EXPECT_TRUE(t.add(w,q,true)); EXPECT_FALSE(t.ready()); }
  EXPECT_TRUE(t.add(w,q,true)); ASSERT_TRUE(t.ready()); EXPECT_NEAR(t.bias()[2], .3, 1e-9);
  TareWindow motion(50); EXPECT_TRUE(motion.add(w,q,true)); q[0]=.002;
  EXPECT_FALSE(motion.add(w,q,true)); EXPECT_FALSE(motion.ready());
  TareWindow gate(50); EXPECT_FALSE(gate.add(w,Joints{},false)); EXPECT_FALSE(gate.ready());
}
TEST(TareWindow, RejectsNoisyOrNonFiniteAndExcessiveFreeSpaceLoad) {
  TareWindow t(50); Joints q{}; Wrench w{};
  for(int i=0;i<50;i++){ w[0]=(i%2) ? 1. : -1.; t.add(w,q,true); }
  EXPECT_FALSE(t.ready());
  TareWindow bad(50); w[0]=NAN; EXPECT_FALSE(bad.add(w,q,true));
  TareWindow contact(50); w={20,0,0,0,0,0}; EXPECT_FALSE(contact.add(w,q,true));
}
TEST(TareWindow, IndependentVerificationRejectsMotionResidualAndLostInterlock) {
  TareWindow t(3); Joints q{}; Wrench w={.1,.2,.3,.01,.02,.03};
  EXPECT_FALSE(t.verify(w,q,true));
  for(int i=0;i<3;i++) ASSERT_TRUE(t.add(w,q,true));
  EXPECT_TRUE(t.verify(w,q,true));
  EXPECT_FALSE(t.verify(w,q,false));
  q[0]=.002; EXPECT_FALSE(t.verify(w,q,true)); q[0]=0.;
  w[0]+=.6; EXPECT_FALSE(t.verify(w,q,true));
}

TEST(StateContract, SelectedArmRejectsOtherArmAndReordersMeasuredJoints) {
  const std::array<std::string,6> right={"right1","right2","right3","right4","right5","right6"};
  StateContract c("run", "scene", right);
  auto s=sample();
  s.names={"left1","left2","left3","left4","left5","left6"};
  EXPECT_FALSE(c.accept(s,1.)); EXPECT_EQ(c.reason(),"INVALID_JOINT_NAMES");
  s.names={"right6","right5","right4","right3","right2","right1"};
  ASSERT_TRUE(c.accept(s,1.));
  EXPECT_DOUBLE_EQ(c.positions()[0],.5); EXPECT_DOUBLE_EQ(c.positions()[5],0.);
  s.names[0]="left6";s.step++;s.stamp++;
  EXPECT_FALSE(c.accept(s,1.1)); EXPECT_FALSE(c.fresh(1.1));
}
TEST(StateContract, RefusesAmbiguousExpectedJointConfiguration) {
  const std::array<std::string,6> repeated={"base","base","elbow","wrist1","wrist2","wrist3"};
  EXPECT_THROW(StateContract("run","scene",repeated),std::invalid_argument);
}
