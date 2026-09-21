// Offline regression tests. No robot, hardware interface or motion publisher.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include "admittance_controller/admittance_rule.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/test_asset_6d_robot_description.hpp"

namespace
{
constexpr double DEG = M_PI / 180.0;
using Vector6 = Eigen::Matrix<double, 6, 1>;

// Analytic Cartesian mechanism with optional coupling, curvature and damped IK.
// In particular, the tests do not assume that inverse and forward maps cancel.
class TestKinematics : public kinematics_interface::KinematicsInterface
{
public:
  double coupling = 0.0;
  double curvature = 0.0;
  double inverse_scale = 1.0;
  bool fail_fk = false;
  bool fail_ik = false;

  bool initialize(const std::string &,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>,
    const std::string &) override {return true;}

  Eigen::Matrix<double, 6, 6> jacobian(const Eigen::VectorXd & q) const
  {
    auto j = Eigen::Matrix<double, 6, 6>::Identity().eval();
    j(0, 5) = coupling;
    j(5, 5) = 1.0 + 2.0 * curvature * q[5];
    return j;
  }

  bool convert_cartesian_deltas_to_joint_deltas(const Eigen::VectorXd & q,
    const Vector6 & x, const std::string &, Eigen::VectorXd & result) override
  {
    result = inverse_scale * jacobian(q).inverse() * x;
    return !fail_ik;
  }

  bool convert_joint_deltas_to_cartesian_deltas(const Eigen::VectorXd & q,
    const Eigen::VectorXd & v, const std::string &, Vector6 & result) override
  {
    result = jacobian(q) * v;
    return true;
  }

  bool calculate_link_transform(const Eigen::VectorXd & q, const std::string & link,
    Eigen::Isometry3d & result) override
  {
    result.setIdentity();
    if (link != "base_link") {
      result.translation() = q.head<3>();
      result.translation().x() += coupling * q[5];
      result.linear() = Eigen::AngleAxisd(
        q[5] + curvature * q[5] * q[5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    return !fail_fk;
  }

  bool calculate_jacobian(const Eigen::VectorXd & q, const std::string &,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & result) override
  {result = jacobian(q); return true;}

  bool calculate_jacobian_inverse(const Eigen::VectorXd & q, const std::string &,
    Eigen::Matrix<double, Eigen::Dynamic, 6> & result) override
  {result = inverse_scale * jacobian(q).inverse(); return !fail_ik;}
};

class TestRule : public admittance_controller::AdmittanceRule
{
public:
  using AdmittanceRule::AdmittanceRule;
  using AdmittanceRule::admittance_state_;
  using AdmittanceRule::calculate_admittance_rule;
  TestKinematics * model = nullptr;

  void prepare()
  {
    auto mock = std::make_unique<TestKinematics>();
    model = mock.get();
    kinematics_ = std::move(mock);
    parameters_.enable_parameter_update_without_reactivation = false;
    parameters_.normal_axis_index = 1;
    parameters_.roller_balance.control_enabled = true;
    parameters_.roller_balance.monitoring_enabled = true;
    parameters_.roller_balance.rotation_axis_index = 2;
    parameters_.roller_balance.max_rotation_trim_rad = 0.5 * DEG;
    parameters_.roller_balance.soft_limit_ratio = 0.9;
    parameters_.roller_balance.max_rotation_velocity_radps = 0.2 * DEG;
    parameters_.roller_balance.max_rotation_acceleration_radps2 = DEG;
    parameters_.roller_balance.contact_center_offset_m = {0.0, 0.0, 0.0};
    parameters_.admittance.joint_damping = 12.0;
    parameters_.admittance.mass = {10.0, 6.0, 10.0, 1.0, 1.0, 1.0};
    parameters_.admittance.stiffness = {1000.0, 100.0, 1000.0, 100.0, 100.0, 10.0};
    parameters_.admittance.damping_absolute = {0.0, 300.0, 0.0, 0.0, 0.0, 25.0};
    parameters_.admittance.selected_axes = {false, true, false, false, false, false};
    parameters_.admittance.absolute_normal_force = true;
    parameters_.admittance.normal_force_hold_lower_n = 20.0;
    parameters_.admittance.normal_force_hold_upper_n = 30.0;
    parameters_.admittance.normal_force_hold_hysteresis_n = 1.0;
    parameters_.gravity_compensation.CoG.force = 0.0;
    parameters_.ft_sensor.filter_coefficient = 0.2;
    parameters_.max_normal_trim_m = 0.020;
    parameters_.max_normal_velocity_mps = 0.003;
    parameters_.max_normal_acceleration_mps2 = 0.05;
    end_effector_weight_.setZero();
    cog_pos_.setZero();
    roller_balance_freeze_trim_ = true;
    admittance_state_ = admittance_controller::AdmittanceState(6);
    admittance_state_.mass_inv.setOnes();
    admittance_state_.mass_inv[1] = 1.0 / 6.0;
    admittance_state_.damping[1] = 300.0;
    admittance_state_.damping[5] = 25.0;
    admittance_state_.stiffness[5] = 10.0;
    admittance_state_.selected_axes[1] = 1.0;
    admittance_state_.selected_axes[5] = 1.0;
    admittance_state_.rot_base_control.setIdentity();
    admittance_state_.ref_trans_base_ft.setIdentity();
    admittance_state_.ft_sensor_frame = "tool0";
    admittance_state_.wrench_base.setZero();
  }

  bool step(double torque, double measured_angle = 0.0, double dt = 0.01)
  {
    admittance_state_.current_joint_pos[5] = measured_angle;
    admittance_state_.wrench_base[5] = torque;
    return calculate_admittance_rule(admittance_state_, dt);
  }

  bool real_kinematics_step(double torque)
  {
    auto & state = admittance_state_;
    state.current_joint_pos = state.reference_joint_pos + state.joint_pos;
    Eigen::Isometry3d control;
    if (!kinematics_->calculate_link_transform(
        state.current_joint_pos, parameters_.control.frame.id, control))
    {
      return false;
    }
    state.rot_base_control = control.rotation();
    state.wrench_base.setZero();
    state.wrench_base.tail<3>() = state.rot_base_control * Eigen::Vector3d(0.0, 0.0, torque);
    return calculate_admittance_rule(state, 0.01);
  }

  bool prepare_real_pose()
  {
    auto & state = admittance_state_;
    state.reference_joint_pos << 0.2, -0.6, 0.9, 0.3, 0.5, -0.2;
    state.current_joint_pos = state.reference_joint_pos;
    state.ft_sensor_frame = parameters_.ft_sensor.frame.id;
    roller_balance_freeze_trim_ = true;
    normal_force_hold_active_ = true;
    return kinematics_->calculate_link_transform(
      state.reference_joint_pos, state.ft_sensor_frame, state.ref_trans_base_ft);
  }
};

class RollerDynamics : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_admittance_controller");
    listener = std::make_shared<admittance_controller::ParamListener>(node);
    rule = std::make_unique<TestRule>(listener);
    rule->prepare();
  }

  void check_step(double torque, double measured = 0.0, double dt = 0.01)
  {
    const double previous_omega = rule->roller_balance_command_velocity_radps_;
    const auto previous_joint_v = rule->admittance_state_.joint_vel.eval();
    ASSERT_TRUE(rule->step(torque, measured, dt));
    EXPECT_LE(std::abs(rule->roller_balance_commanded_trim_rad()), 0.45 * DEG + 1e-7);
    EXPECT_LE(std::abs(rule->roller_balance_command_velocity_radps_), 0.2 * DEG + 1e-9);
    EXPECT_LE(std::abs(rule->roller_balance_command_velocity_radps_ - previous_omega) / dt,
      DEG + 1e-7);
    EXPECT_TRUE(rule->admittance_state_.joint_acc.isApprox(
      (rule->admittance_state_.joint_vel - previous_joint_v) / dt, 1e-9));
    EXPECT_FALSE(rule->roller_balance_limit_reached());
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<admittance_controller::ParamListener> listener;
  std::unique_ptr<TestRule> rule;
};

TEST_F(RollerDynamics, blocked_arm_cannot_wind_up_command)
{
  for (int i = 0; i < 3000; ++i) {
    check_step(0.25);  // 25 N, CoP=15 mm, inner band=5 mm.
    ASSERT_FALSE(HasFatalFailure());
  }
  EXPECT_NEAR(rule->roller_balance_rotation_trim_rad(), 0.0, 1e-12);
  EXPECT_NEAR(rule->roller_balance_commanded_trim_rad(), 0.45 * DEG, 1e-7);
  EXPECT_TRUE(rule->roller_balance_soft_limit_active());
  EXPECT_NEAR(rule->roller_balance_command_velocity_radps_, 0.0, 1e-8);
}

TEST_F(RollerDynamics, soft_saturation_does_not_fault_and_can_reverse)
{
  for (double torque : {0.25, -0.25, 0.25}) {
    for (int i = 0; i < 1200; ++i) {
      check_step(torque, rule->admittance_state_.joint_pos[5]);
      ASSERT_FALSE(HasFatalFailure());
    }
    EXPECT_NEAR(rule->roller_balance_commanded_trim_rad(), std::copysign(0.45 * DEG, torque),
      1e-7);
  }
}

TEST_F(RollerDynamics, reversal_and_deadband_stop_obey_final_acceleration)
{
  for (double torque : {0.25, -0.25, 0.0, 0.25, 0.0}) {
    for (int i = 0; i < 100; ++i) {
      check_step(torque);
      ASSERT_FALSE(HasFatalFailure());
    }
  }
  EXPECT_NEAR(rule->roller_balance_command_velocity_radps_, 0.0, 1e-8);
}

TEST_F(RollerDynamics, release_restores_command_even_when_measured_arm_is_blocked)
{
  for (int i = 0; i < 400; ++i) {
    check_step(0.25);
    ASSERT_FALSE(HasFatalFailure());
  }
  rule->roller_balance_freeze_trim_ = false;
  for (int i = 0; i < 3000; ++i) {
    check_step(0.0);
    ASSERT_FALSE(HasFatalFailure());
  }
  EXPECT_NEAR(rule->roller_balance_commanded_trim_rad(), 0.0, 0.001 * DEG);
}

TEST_F(RollerDynamics, braking_survives_variable_periods)
{
  const double periods[] = {0.005, 0.01, 0.013, 0.017, 0.025};
  for (int i = 0; i < 2000; ++i) {
    check_step(i < 1000 ? 0.25 : -0.25, 0.0, periods[i % 5]);
    ASSERT_FALSE(HasFatalFailure());
  }
}

TEST_F(RollerDynamics, coupled_nonlinear_damped_mapping_is_checked_at_final_output)
{
  rule->model->coupling = 0.3;
  rule->model->curvature = 0.1;
  rule->model->inverse_scale = 0.98;
  for (int i = 0; i < 2400; ++i) {
    check_step(i < 1200 ? 0.25 : -0.25);
    ASSERT_FALSE(HasFatalFailure());
  }
}

TEST_F(RollerDynamics, hard_measured_boundary_still_faults_without_integrating)
{
  const auto position = rule->admittance_state_.joint_pos.eval();
  EXPECT_FALSE(rule->step(0.25, 0.51 * DEG));
  EXPECT_TRUE(rule->roller_balance_limit_reached());
  EXPECT_TRUE(rule->admittance_state_.joint_pos.isApprox(position));
}

TEST_F(RollerDynamics, invalid_period_and_failed_kinematics_do_not_integrate)
{
  EXPECT_FALSE(rule->step(0.25, 0.0, 0.0));
  EXPECT_FALSE(rule->step(0.25, 0.0, std::numeric_limits<double>::quiet_NaN()));
  rule->model->fail_fk = true;
  EXPECT_FALSE(rule->step(0.25));
  rule->model->fail_fk = false;
  rule->model->fail_ik = true;
  EXPECT_FALSE(rule->step(0.25));
  EXPECT_TRUE(rule->admittance_state_.joint_pos.isZero());
}

TEST_F(RollerDynamics, full_update_preserves_20_mmps_nominal_traverse_at_soft_limit)
{
  trajectory_msgs::msg::JointTrajectoryPoint reference, measured, output;
  reference.positions.assign(6, 0.0);
  reference.velocities.assign(6, 0.0);
  reference.accelerations.assign(6, 0.0);
  reference.velocities[0] = 0.020;
  measured = output = reference;
  geometry_msgs::msg::Wrench force, command;
  force.force.y = 25.0;
  force.torque.z = 25.0 * 0.015;
  command.force.y = -25.0;
  for (int i = 0; i < 1500; ++i) {
    reference.positions[0] = i * 0.01 * 0.020;
    measured.positions[0] = reference.positions[0];
    // Angular plant is deliberately blocked, but tangential reference continues.
    ASSERT_EQ(rule->update(measured, force, command, reference,
      rclcpp::Duration::from_seconds(0.01), output), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(output.positions[0], reference.positions[0]);
    EXPECT_DOUBLE_EQ(output.velocities[0], 0.020);
    EXPECT_FALSE(rule->roller_balance_limit_reached());
  }
  EXPECT_TRUE(rule->roller_balance_soft_limit_active());
  EXPECT_NEAR(output.positions[5], 0.45 * DEG, 1e-7);
}

TEST_F(RollerDynamics, failed_update_keeps_previous_output_not_nominal_path)
{
  trajectory_msgs::msg::JointTrajectoryPoint reference, measured, output;
  reference.positions.assign(6, 0.0);
  reference.velocities.assign(6, 0.0);
  reference.accelerations.assign(6, 0.0);
  measured = output = reference;
  output.positions[5] = 0.2 * DEG;
  rule->model->fail_fk = true;
  const geometry_msgs::msg::Wrench zero;
  EXPECT_EQ(rule->update(measured, zero, zero, reference, rclcpp::Duration::from_seconds(0.01), output),
    controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(output.positions[5], 0.2 * DEG);
}

TEST_F(RollerDynamics, real_kdl_plugin_brakes_and_reverses_without_soft_limit_faults)
{
  auto lifecycle = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_admittance_controller");
  auto params = std::make_shared<admittance_controller::ParamListener>(lifecycle);
  TestRule real(params);
  real.prepare();
  ASSERT_EQ(real.configure(lifecycle, 6, ros2_control_test_assets::valid_6d_robot_urdf),
    controller_interface::return_type::OK);
  ASSERT_TRUE(real.prepare_real_pose());
  for (int i = 0; i < 2400; ++i) {
    const double previous_velocity = real.roller_balance_command_velocity_radps_;
    ASSERT_TRUE(real.real_kinematics_step(i < 1200 ? 0.25 : -0.25)) << "step " << i;
    EXPECT_LE(std::abs(real.roller_balance_commanded_trim_rad()), 0.5 * DEG);
    EXPECT_LE(std::abs(real.roller_balance_command_velocity_radps_), 0.2 * DEG + 1e-9);
    EXPECT_LE(std::abs(real.roller_balance_command_velocity_radps_ - previous_velocity) / 0.01,
      DEG + 1e-7);
    EXPECT_FALSE(real.roller_balance_limit_reached());
  }
  EXPECT_NEAR(real.roller_balance_commanded_trim_rad(), -0.45 * DEG, 0.001 * DEG);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
