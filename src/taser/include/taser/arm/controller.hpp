#pragma once

#include "taser/types.hpp"

namespace taser::arm {

/**
 * Position controller for the arm
 */
class Controller {
public:
  /**
   * @param kp Proportional gain
   */
  Controller(double kp);

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

} // namespace taser::arm