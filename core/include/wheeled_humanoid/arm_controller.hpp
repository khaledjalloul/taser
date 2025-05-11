#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

/**
 * Position controller for the arm
 */
class ArmController {
public:
  /**
   * @param kp Proportional gain
   */
  ArmController(double kp);

  /**
   * Compute the desired end effector velocity command
   * @param x0 Current pose of the end effector
   * @param x_ss Desired pose of the end effector
   * @return End effector velocity command
   */
  Twist3D step(Pose3D x0, Pose3D x_ss) const;

private:
  double kp_;
};

} // namespace wheeled_humanoid