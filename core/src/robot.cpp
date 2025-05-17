#include "wheeled_humanoid/robot.hpp"

namespace wheeled_humanoid {

std::tuple<ArmJointVelocities, double>
Robot::move_arm_step(const std::string &arm_name,
                     const Position3D &desired_position,
                     const Transforms &tfs) const {

  const auto &arm = arms.at(arm_name);

  // Get current and desired end effector poses
  auto p_cur = arm.get_end_effector_pose(tfs.TBE);
  Pose3D p_desired = {desired_position, {0, 0, 0}};

  // Get control output and solve for joint velocities
  auto w_desired = arm_controller.step(p_cur, p_desired);
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

std::tuple<double, double, double>
Robot::move_base_step(const Pose2D &desored_pose) {
  auto dubins_path = rrt.generate_path(base.pose, desored_pose);
  if (dubins_path.empty()) {
    std::cerr << "Cannot move base, RRT* path not found." << std::endl;
    return {0, 0, -1};
  }
  auto path = rrt.sample_path(dubins_path, base_controller.N);
  auto path_vel = rrt.get_velocity_profile(path);

  auto u = base_controller.step(base.pose, path, path_vel);

  base.set_base_velocity(u.v, u.omega);
  base.step();

  auto err = base::get_euclidean_distance(base.pose, path.back());
  return {base.v_l, base.v_r, err};
}

} // namespace wheeled_humanoid