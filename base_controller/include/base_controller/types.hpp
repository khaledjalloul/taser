#pragma once

#include <eigen3/Eigen/Dense>

namespace base_controller {

struct BasePosition {
  double x, y, theta; // base orientation
};

struct BaseVelocity {
  double v, w; // base orientation rate
};

struct WheelState {
  double v_l, v_r, steering; // steering angle
};

} // namespace base_controller