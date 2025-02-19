#pragma once

#include <string>

#include "arm_controller/types.hpp"

namespace arm_controller {

class ArmKinematics {
public:
  ArmKinematics(std::string name);

  Pose get_end_effector_pose(Transform TBE) const;

  Twist get_end_effector_twist(Transforms tfs, ArmJointState dq) const;

  Jacobian get_geometric_jacobian(Transforms tfs) const;

  Matrix get_pseudoinverse(Matrix mat) const;

  // TODO
  ArmJointState solve_ik_for_joint_positions(Pose p_desired,
                                             Transforms tfs) const {
    return ArmJointState();
  }

  ArmJointState solve_ik_for_joint_velocities(Twist w_desired,
                                              Transforms tfs) const;

private:
  std::string name_;

  // Rotation axes
  int num_joints_{3};
  Axis n0_{0, 1, 0};
  Axis n1_{0, 0, 1};
  Axis n2_{0, 1, 0};
};

} // namespace arm_controller