#include "wheeled_humanoid/robot.hpp"

namespace wheeled_humanoid {

Robot::Robot(double dt, double arm_controller_kp, double base_L,
             double base_wheel_radius, int base_mpc_horizon,
             double base_velocity, int base_rrt_num_samples,
             const base::Dimensions &base_rrt_dim)
    : dt(dt) {
  arms = {{"left_arm", arm::Kinematics("left_arm")},
          {"right_arm", arm::Kinematics("right_arm")}};
  arm_controller = std::make_unique<arm::Controller>(arm_controller_kp);

  base = std::make_unique<base::Kinematics>(base_L, base_wheel_radius, dt);
  base_controller = std::make_unique<base::Controller>(dt, base_mpc_horizon);
  rrt = std::make_unique<base::PathPlanner>(base_rrt_num_samples, dt, base_L,
                                            base_velocity, base_rrt_dim);
}

std::tuple<ArmJointVelocities, double>
Robot::move_arm_step(const std::string &arm_name,
                     const Position3D &desired_position,
                     const Transforms &tfs) const {

  const auto &arm = arms.at(arm_name);

  // Get current and desired end effector poses
  auto p_cur = arm.get_end_effector_pose(tfs.TBE);
  Pose3D p_desired = {desired_position, {0, 0, 0}};

  // Get control output and solve for joint velocities
  auto w_desired = arm_controller->step(p_cur, p_desired);
  auto dq_desired = arm.solve_ik_for_joint_velocities(w_desired, tfs);

  // Limit joint velocities
  dq_desired = dq_desired.cwiseMin(1.0).cwiseMax(-1.0);

  // Calculate error between current and desired end effector positions
  auto err = pow(p_cur.position.x - desired_position.x, 2) +
             pow(p_cur.position.y - desired_position.y, 2) +
             pow(p_cur.position.z - desired_position.z, 2);
  err = sqrt(err);

  return {dq_desired, err};
}

int Robot::plan_path(const Pose2D &goal) {
  auto dubins_path = rrt->generate_path(base->pose, goal);
  if (dubins_path.empty()) {
    std::cerr << "RRT* path not found." << std::endl;
    return 0;
  }
  path_ = rrt->sample_path(dubins_path);
  for (int i = 0; i < 2 * base_controller->N; i++) {
    path_.push_back(path_.back());
  }
  path_vel_ = rrt->get_velocity_profile(path_);
  path_step_ = 0;

  return path_.size();
}

std::tuple<double, double, double> Robot::move_base_step() {
  if (path_.empty()) {
    std::cerr << "No path planned. Call `Robot::plan_path` first." << std::endl;
    return {0, 0, -1};
  }

  auto local_path = Path(path_.begin() + path_step_,
                         path_.begin() + path_step_ + base_controller->N);
  auto local_path_vel =
      VelocityProfile(path_vel_.begin() + path_step_,
                      path_vel_.begin() + path_step_ + base_controller->N);

  auto u = base_controller->step(base->pose, local_path, local_path_vel);

  // TODO: Don't update the base pose here, only return wheel velocities and
  // move the robot in the ROS node
  base->set_base_velocity(u.v, u.omega);
  base->step();
  path_step_ += 1;

  auto err = base::get_euclidean_distance(base->pose, path_.back());
  return {base->v_l, base->v_r, err};
}

} // namespace wheeled_humanoid