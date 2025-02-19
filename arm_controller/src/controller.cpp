#include "arm_controller/controller.hpp"

namespace arm_controller {

Controller::Controller(double kp) : kp_(kp) {}

Twist Controller::step(Pose x0, Pose x_ss) {
  Twist err;

  err.linear.x = kp_ * (x_ss.position.x - x0.position.x);
  err.linear.y = kp_ * (x_ss.position.y - x0.position.y);
  err.linear.z = kp_ * (x_ss.position.z - x0.position.z);
  err.angular.roll = kp_ * (x_ss.orientation.roll - x0.orientation.roll);
  err.angular.pitch = kp_ * (x_ss.orientation.pitch - x0.orientation.pitch);
  err.angular.yaw = kp_ * (x_ss.orientation.yaw - x0.orientation.yaw);

  return err;
}

} // namespace arm_controller