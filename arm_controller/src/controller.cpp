#include "arm_controller/controller.hpp"

namespace arm_controller {

Controller::Controller(double kp) : kp_(kp) {}

Twist Controller::step(Pose x0, Pose x_ss) {
  Twist err;

  err.linear = kp_ * (x_ss.position - x0.position);
  err.angular = kp_ * (x_ss.orientation - x0.orientation);

  return err;
}

} // namespace arm_controller