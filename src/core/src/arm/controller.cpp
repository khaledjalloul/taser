#include "wheeled_humanoid/arm/controller.hpp"

namespace wheeled_humanoid::arm {

Controller::Controller(double kp) : kp_(kp) {}

Twist3D Controller::step(Pose3D x0, Pose3D x_ss) const {
  Twist3D err;

  err.linear.x = kp_ * (x_ss.position.x - x0.position.x);
  err.linear.y = kp_ * (x_ss.position.y - x0.position.y);
  err.linear.z = kp_ * (x_ss.position.z - x0.position.z);
  err.angular.roll = kp_ * (x_ss.orientation.roll - x0.orientation.roll);
  err.angular.pitch = kp_ * (x_ss.orientation.pitch - x0.orientation.pitch);
  err.angular.yaw = kp_ * (x_ss.orientation.yaw - x0.orientation.yaw);

  return err;
}

} // namespace wheeled_humanoid::arm