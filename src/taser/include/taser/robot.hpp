#pragma once

#include <map>
#include <string>

#include "taser/arm/controller.hpp"
#include "taser/arm/kinematics.hpp"
#include "taser/base/controller.hpp"
#include "taser/base/kinematics.hpp"
#include "taser/base/path_planner.hpp"

namespace taser {

/**
 * Robot configuration with all configurable parameters
 */
struct RobotConfig {
  double dt;                // Time step
  double arm_controller_kp; // Proportional gain for the arm controller
  double base_L;            // Distance between the base wheels
  double base_wheel_radius; // Radius of the base wheels
  int64_t base_mpc_horizon; // Number of prediction steps for the MPC controller
  double base_velocity;     // Desired base velocity
  int64_t base_rrt_num_samples;  // Number of samples for the RRT* path planner
  base::Dimensions base_rrt_dim; // Dimensions of the environment for the RRT*
                                 // path planner to sample from
};

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
   * @param config Configuration of the robot
   */
  Robot(const RobotConfig &config);

  /**
   * Get the desired arm joint velocities at a given time step using the arm
   * controller and IK solver
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
   * Plan a path from the current base pose to a given goal pose using the RRT*
   * path planner
   * @param goal Target base pose
   * @return Number of steps in the path
   */
  int plan_path(const Pose2D &goal);

  /**
   * Get the desired base wheel velocities at a given time step using the base
   * MPC controller
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

} // namespace taser