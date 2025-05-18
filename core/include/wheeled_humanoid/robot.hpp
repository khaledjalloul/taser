#pragma once

#include <map>
#include <string>

#include "wheeled_humanoid/arm/controller.hpp"
#include "wheeled_humanoid/arm/kinematics.hpp"
#include "wheeled_humanoid/base/controller.hpp"
#include "wheeled_humanoid/base/kinematics.hpp"
#include "wheeled_humanoid/base/path_planner.hpp"

namespace wheeled_humanoid {

/**
 * Main class combining all components of the robot:
 * - Arm kinematics
 * - Arm controller
 * - Base kinematics
 * - Base controller
 * - RRT* path planner
 */
class Robot {
public:
  /**
   * Get the desired arm joint velocities using the arm controller and IK solver
   * @param arm_name Name of the arm (left_arm or right_arm)
   * @param desired_position Desired end effector position
   * @param tfs Transforms from the base frame to each of the end effector link
   * frames
   * @return Tuple of joint velocities and error
   */
  std::tuple<ArmJointVelocities, double>
  move_arm_step(const std::string &arm_name, const Position3D &desired_position,
                const Transforms &tfs) const;

  /**
   * Plan a path from the current base pose using the RRT* path planner
   * @param goal Target base pose
   */
  void plan_path(const Pose2D &goal);

  /**
   * Get the desired base wheel velocities using using the base MPC controller
   * @details Updates the base pose internally
   * @return Tuple of left and right wheel velocities and error
   */
  std::tuple<double, double, double> move_base_step();

  double T = 5.0;  // Time to reach targets
  double dt = 0.1; // Time step

  std::map<std::string, arm::Kinematics> arms = {
      {"left_arm", arm::Kinematics("left_arm")},
      {"right_arm", arm::Kinematics("right_arm")}};
  arm::Controller arm_controller{1.0};

  // L = 2.25, obtained using transform from left wheel to right wheel
  base::Kinematics base{2.25, 0.5, dt};
  base::Controller base_controller{dt, 30};
  base::PathPlanner rrt{200, dt, 2.25};

private:
  Path path_;
  VelocityProfile path_vel_;
  int path_step_ = 0;
};

} // namespace wheeled_humanoid