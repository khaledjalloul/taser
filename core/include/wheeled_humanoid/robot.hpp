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
   * Constructor
   * @param dt Time step
   * @param arm_controller_kp Proportional gain for the arm controller
   * @param base_L Distance between the base wheels
   * @param base_wheel_radius Radius of the base wheels
   * @param base_mpc_horizon Number of prediction steps for the MPC controller
   * @param base_velocity Desired base velocity
   * @param base_rrt_num_samples Number of samples for the RRT* path planner
   * @param base_rrt_dim Dimensions of the environment for the RRT* path planner
   * to sample from
   */
  Robot(double dt, double arm_controller_kp, double base_L,
        double base_wheel_radius, int base_mpc_horizon, double base_velocity,
        int base_rrt_num_samples, const base::Dimensions &base_rrt_dim);

  /**
   * Get the desired arm joint velocities using the arm controller and IK solver
   * @param arm_name Name of the arm (left or right)
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
   * @return Number of steps in the path
   */
  int plan_path(const Pose2D &goal);

  /**
   * Get the desired base wheel velocities using using the base MPC controller
   * @details Updates the base pose internally
   * @return Tuple of left and right wheel velocities and error
   */
  std::tuple<double, double, double> move_base_step();

  double dt;

  std::map<std::string, arm::Kinematics> arms;
  std::unique_ptr<arm::Controller> arm_controller;

  std::unique_ptr<base::Kinematics> base;
  std::unique_ptr<base::Controller> base_controller;
  std::unique_ptr<base::PathPlanner> rrt;

private:
  Path path_;
  VelocityProfile path_vel_;
  int path_step_ = 0;
};

} // namespace wheeled_humanoid