#pragma once

#include <string>

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

class ArmKinematics {
public:
  ArmKinematics(std::string name);

  Pose3D get_end_effector_pose(Transform TBE) const;

  Twist3D get_end_effector_twist(Transforms tfs, ArmJointVelocities dq) const;

  Jacobian get_geometric_jacobian(Transforms tfs) const;

  Matrix get_pseudoinverse(Matrix mat) const;

  // TODO
  ArmJointPositions solve_ik_for_joint_positions(Pose3D p_desired,
                                                     Transforms tfs) const {
    return ArmJointPositions();
  }

  ArmJointVelocities solve_ik_for_joint_velocities(Twist3D w_desired,
                                                   Transforms tfs) const;

private:
  std::string name_;

  // Rotation axes
  int num_joints_{3};
  Axis n0_{0, 1, 0};
  Axis n1_{0, 0, 1};
  Axis n2_{0, 1, 0};
};

} // namespace wheeled_humanoid