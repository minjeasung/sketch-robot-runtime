#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>

namespace isaac_painting_hardware {
using Joints = std::array<double, 6>;
using Wrench = std::array<double, 6>;
inline const std::array<std::string, 6> JOINTS = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
inline constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
template<class Range> inline bool finite(const Range& r) {
  return std::all_of(r.begin(), r.end(), [](double v){return std::isfinite(v);});
}
struct Sample {
  std::string session, revision;
  uint64_t step{};
  double stamp{};
  std::vector<std::string> names;
  std::vector<double> position, velocity, effort;
  Wrench wrench{}, gravity{};
  bool joints_valid{}, wrench_valid{}, playing{};
};
class StateContract {
 public:
  StateContract(std::string session, std::string revision,
                std::array<std::string,6> expected_joints=JOINTS):
      session_(session), revision_(revision), expected_joints_(expected_joints) {
    for(const auto& name:expected_joints_) {
      if(name.empty() || std::count(expected_joints_.begin(),expected_joints_.end(),name)!=1)
        throw std::invalid_argument("Expected exactly six unique nonempty joint names");
    }
  }
  bool accept(const Sample& s, double now) {
    if(s.session!=session_ || s.revision!=revision_ || session_.empty() || revision_.empty()) {
      valid_=false; reason_="SESSION_MISMATCH"; return false;
    }
    if(!s.playing || !s.joints_valid || !s.wrench_valid) {
      valid_=false; reason_="SENSOR_INVALID_OR_PAUSED"; return false;
    }
    if(s.names.size()!=6 || s.position.size()!=6 || s.velocity.size()!=6 || s.effort.size()!=6 ||
       !finite(s.position) || !finite(s.velocity) || !finite(s.effort) || !finite(s.wrench) ||
       !finite(s.gravity) || !std::isfinite(s.stamp) || s.stamp<0 || !std::isfinite(now)) {
      valid_=false; reason_="MALFORMED_STATE"; return false;
    }
    if(seen_ && (s.step<=step_ || s.stamp<=stamp_ || now<received_)) {
      reason_="OLD_STATE"; return false;
    }
    Joints pos{},vel{},eff{};
    for(size_t i=0;i<6;i++) {
      if(std::count(s.names.begin(),s.names.end(),expected_joints_[i])!=1) {
        valid_=false; reason_="INVALID_JOINT_NAMES"; return false;
      }
      auto j=std::find(s.names.begin(),s.names.end(),expected_joints_[i])-s.names.begin();
      pos[i]=s.position[j]; vel[i]=s.velocity[j]; eff[i]=s.effort[j];
    }
    position_=pos; velocity_=vel; effort_=eff;
    for(size_t i=0;i<6;i++) compensated_[i]=s.wrench[i]-s.gravity[i];
    if(!finite(compensated_)){ valid_=false; reason_="WRENCH_OVERFLOW"; return false; }
    stamp_=s.stamp; step_=s.step; received_=now; seen_=valid_=true; reason_=""; return true;
  }
  bool fresh(double now) const { return valid_ && seen_ && now>=received_ && now-received_<=.20; }
  bool seen() const {return seen_;}
  uint64_t step() const {return step_;}
  double stamp() const {return stamp_;}
  const Joints& positions() const {return position_;}
  const Joints& velocities() const {return velocity_;}
  const Joints& efforts() const {return effort_;}
  const Wrench& compensated() const {return compensated_;}
  const std::string& reason() const {return reason_;}
 private:
  std::string session_,revision_,reason_;
  std::array<std::string,6> expected_joints_;
  bool seen_{false},valid_{false};
  uint64_t step_{};
  double stamp_{},received_{};
  Joints position_{},velocity_{},effort_{};
  Wrench compensated_{};
};
class TareWindow {
 public:
  explicit TareWindow(size_t target=100):target_(target) {}
  bool add(const Wrench& w,const Joints& q,bool eligible) {
    if(failed_) return false;
    if(!eligible || !finite(w) || !finite(q)) {reason_="ineligible or nonfinite sample";failed_=true; return false;}
    // Gravity is already removed. Tare cannot erase a real contact load.
    if(std::hypot(w[0],w[1],w[2])>2.5 || std::hypot(w[3],w[4],w[5])>.3) {
      reason_="free-space wrench exceeds contact limit";failed_=true; return false;
    }
    if(count_==0) initial_=q;
    for(size_t i=0;i<6;i++) {
      if(std::abs(q[i]-initial_[i])>.001) {reason_="joint moved during tare";failed_=true; return false;}
      sum_[i]+=w[i]; square_[i]+=w[i]*w[i];
    }
    count_++;
    if(count_>=target_) for(size_t i=0;i<6;i++) {
      const double mean=sum_[i]/count_, variance=std::max(0.,square_[i]/count_-mean*mean);
      if(std::sqrt(variance)>(i<3 ? .5:.05)) {reason_="sample noise on axis "+std::to_string(i)+": "+std::to_string(std::sqrt(variance));failed_=true;}
    }
    return !failed_;
  }
  bool ready() const {return !failed_ && count_>=target_;}
  Wrench bias() const {Wrench b{}; for(size_t i=0;i<6;i++) b[i]=count_?sum_[i]/count_:NaN; return b;}
  const std::string& reason() const {return reason_;}
  std::string verification_error(const Wrench& w,const Joints& q,bool eligible) const {
    if(!ready() || !eligible || !finite(w) || !finite(q)) return "verification not ready, ineligible or nonfinite";
    const auto candidate=bias();
    for(size_t i=0;i<6;i++) {
      if(std::abs(q[i]-initial_[i])>.001) return "joint moved during residual verification";
      if(std::abs(w[i]-candidate[i])>(i<3?.5:.05))
        return "residual on axis "+std::to_string(i)+": "+std::to_string(w[i]-candidate[i])+" exceeds "+std::to_string(i<3?.5:.05);
    }
    return "";
  }
  bool verify(const Wrench& w,const Joints& q,bool eligible) const {
    return verification_error(w,q,eligible).empty();
  }
 private:
  size_t target_,count_{}; bool failed_{}; Joints initial_{}; Wrench sum_{},square_{};
  std::string reason_;
};
}  // namespace isaac_painting_hardware
