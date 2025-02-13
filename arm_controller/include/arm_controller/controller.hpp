#pragma once

#include "arm_controller/types.hpp"

namespace arm_controller {

class Controller {
public:
  Controller(double kp);

  Twist step(Pose x0, Pose x_ss);

private:
  double kp_;
};

} // namespace arm_controller