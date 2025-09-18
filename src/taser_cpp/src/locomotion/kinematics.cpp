#include "taser_cpp/locomotion/kinematics.hpp"

namespace taser_cpp::locomotion {

Kinematics::Kinematics(double L, double wheel_radius, double dt)
    : L_(L), wheel_radius_(wheel_radius), dt_(dt) {}

void Kinematics::set_base_velocity(double v, double omega) {
  base_velocity.v = v;
  base_velocity.omega = omega;

  v_l = (v - (L_ / 2) * omega) / wheel_radius_;
  v_r = (v + (L_ / 2) * omega) / wheel_radius_;
}

void Kinematics::set_wheel_velocities(double v_l, double v_r) {
  this->v_l = v_l;
  this->v_r = v_r;

  base_velocity.v = (v_l + v_r) * wheel_radius_ / 2;
  base_velocity.omega = (v_r - v_l) * wheel_radius_ / L_;
}

void Kinematics::Kinematics::step(std::optional<double> dt) {
  pose.x = pose.x + base_velocity.v * cos(pose.theta) * dt.value_or(dt_);
  pose.y = pose.y + base_velocity.v * sin(pose.theta) * dt.value_or(dt_);
  pose.theta = pose.theta + base_velocity.omega * dt.value_or(dt_);
}

} // namespace taser_cpp::locomotion