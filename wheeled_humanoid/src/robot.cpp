#include "wheeled_humanoid/robot.hpp"

namespace wheeled_humanoid {

std::tuple<ArmJointVelocities, double>
Robot::get_arm_dq_step(const std::string &arm_name,
                       const Position3D &desired_position,
                       const Transforms &tfs) const {

  const auto &arm = arms.at(arm_name);

  // Get current and desired end effector poses
  auto p_cur = arm.get_end_effector_pose(tfs.TBE);
  Pose3D p_desired = {desired_position, {0, 0, 0}};

  // Get control output and solve for joint velocities
  auto w_desired = arm_controller_.step(p_cur, p_desired);
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

void Robot::follow_path(const Path &path) {
  Path extended_path = rrt_.interpolate_path(path);
  for (int i = 0; i < base_controller_.N; i++) {
    extended_path.push_back(path.back());
  }

  for (int step = 0; step < extended_path.size() - base_controller_.N; ++step) {
    Path local_path(extended_path.begin() + step,
                    extended_path.begin() + step + base_controller_.N + 1);
    auto u = base_controller_.step(base.pose, local_path);

    base.set_base_velocity(u.v, u.omega);
    base.step();
  }
}

void Robot::move_to(const Pose2D &pose) {
  // TODO: Get RRT path
  Path desired_path{{0, 0, 0}, {1, 1, 0}, {2, 2, 0}, {3, 1, 0}, {4, 1, 0}};

  follow_path(desired_path);
}

} // namespace wheeled_humanoid