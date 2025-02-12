#pragma once

#include <string>

#include "arm_controller/ros_interface.hpp"
#include "arm_controller/types.hpp"

namespace arm_controller {

class Arm {

public:
  Arm(std::string name, std::string base_frame,
      std::shared_ptr<RosInterface> ros_interface);

  ArmJointState get_joint_positions();

  Pose get_end_effector_pose(bool absolute_in_map = false);

  Pose get_end_effector_twist();

  Jacobian get_geometric_jacobian();

  Matrix get_pseudoinverse(Matrix mat);

  ArmJointState solve_ik_for_joint_positions(
      Vector p_desired, std::optional<std::vector<int>> jacobian_indices);

  ArmJointState solve_ik_for_joint_velocities(
      Vector w_desired, std::optional<std::vector<int>> jacobian_indices);

private:
  std::string name_, base_frame_;
  std::shared_ptr<RosInterface> ros_interface_;

  // Rotation axes
  Axis n0_{0, 1, 0};
  Axis n1_{0, 0, 1};
  Axis n2_{1, 0, 0};
};

} // namespace arm_controller