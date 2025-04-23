#include "wheeled_humanoid/base_kinematics.hpp"

namespace wheeled_humanoid {

BaseKinematics::BaseKinematics(double L, double W, double wheel_radius)
    : L_(L), W_(W), wheel_radius_(wheel_radius) {}

void BaseKinematics::set_wheel_state(const WheelState &s) {
  s_.v_l = s.v_l * wheel_radius_;
  s_.v_r = s.v_r * wheel_radius_;
  s_.steering = s.steering;

  v = calculate_base_velocity();
};

void BaseKinematics::set_properties(double L, double W) {
  L_ = L;
  W_ = W;
};

BaseVelocity BaseKinematics::calculate_base_velocity() const {
  auto linear = (s_.v_l + s_.v_r) / 2;

  auto ang1 = (s_.v_r - s_.v_l) / W_;
  auto ang2 = linear * tan(s_.steering) / L_;

  return {linear, ang1 + ang2};
};

Pose2D BaseKinematics::get_base_displacement(double dt) const {
  if (v.angular == 0) {
    auto x_delta = v.linear * cos(p.theta) * dt;
    auto y_delta = v.linear * sin(p.theta) * dt;

    return {x_delta, y_delta, 0};
  }

  auto x_delta =
      (v.linear / v.angular) * (sin(v.angular * dt + p.theta) - sin(p.theta));
  auto y_delta =
      (v.linear / v.angular) * (-cos(v.angular * dt + p.theta) + cos(p.theta));
  auto theta_delta = v.angular * dt;

  return {x_delta, y_delta, theta_delta};
};

} // namespace wheeled_humanoid