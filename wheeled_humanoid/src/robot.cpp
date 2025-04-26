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

void Robot::plan_path() {
  planned_path.clear();

  // TODO: Get RRT path
  Path rrt_path = {{0, 0, 0}, {1, 1, 0}, {2, 2, 0}, {3, 1, 0}, {4, 1, 0}};
  planned_path = rrt_.interpolate_path(rrt_path);

  for (int i = 0; i < base_controller.N; i++) {
    planned_path.push_back(planned_path.back());
  }
}

std::tuple<double, double, double> Robot::follow_path_step() {
  if (planned_path.empty()) {
    std::cerr << "No path planned. Call plan_path function first." << std::endl;
    return {0.0, 0.0, 0.0};
  }

  Path local_path(planned_path.begin() + planned_path_index,
                  planned_path.begin() + planned_path_index +
                      base_controller.N + 1);
  auto u = base_controller.step(base.pose, local_path);

  base.set_base_velocity(u.v, u.omega);
  base.step();

  planned_path_index += 1;

  auto err = pow(base.pose.x - planned_path.back().x, 2) +
             pow(base.pose.y - planned_path.back().y, 2);
  err = sqrt(err);

  return {base.v_l, base.v_r, err};
}

} // namespace wheeled_humanoid