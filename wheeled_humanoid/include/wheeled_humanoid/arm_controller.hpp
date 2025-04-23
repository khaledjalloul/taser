#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

class ArmController {
public:
  ArmController(double kp);

  Twist3D step(Pose3D x0, Pose3D x_ss) const;

private:
  double kp_;
};

} // namespace wheeled_humanoid