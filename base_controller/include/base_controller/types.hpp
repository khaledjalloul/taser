#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform.hpp>

namespace base_controller {

using RobotJointState = Eigen::Vector<double, 10>;
using TransformMsg = geometry_msgs::msg::Transform;

struct BasePosition {
  double x, y, theta; // base orientation
};

struct BaseVelocity {
  double linear, angular; // base orientation rate
};

struct WheelState {
  double v_l, v_r, steering; // steering angle
};

} // namespace base_controller