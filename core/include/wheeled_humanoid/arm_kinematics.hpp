#pragma once

#include <string>

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

/**
 * Arm kinematics class to solve inverse kinematics for the 3-DOF arm
 */
class ArmKinematics {
public:
  /**
   * @param name Name of the arm (left_arm or right_arm)
   */
  ArmKinematics(std::string name);

  /**
   * Get the end effector pose
   * @param TBE Transform from base to end effector
   * @return Pose of the end effector
   */
  Pose3D get_end_effector_pose(Transform TBE) const;

  /**
   * Get the end effector twist
   * @param tfs Transforms from the base frame to each of the end effector link
   * frames
   * @param dq Arm joint velocities
   * @return Twist of the end effector
   */
  Twist3D get_end_effector_twist(Transforms tfs, ArmJointVelocities dq) const;

  /**
   * TODO: Solve inverse kinematics for joint positions
   * @param p_desired Desired end effector pose
   * @param tfs Transforms from the base frame to each of the end effector link
   * frames
   * @return Joint positions
   */
  ArmJointPositions solve_ik_for_joint_positions(Pose3D p_desired,
                                                 Transforms tfs) const {
    return ArmJointPositions();
  }

  /**
   * Solve inverse kinematics for joint velocities
   * @param w_desired Desired end effector twist
   * @param tfs Transforms from the base frame to each of the end effector link
   * frames
   * @return Joint velocities
   */
  ArmJointVelocities solve_ik_for_joint_velocities(Twist3D w_desired,
                                                   Transforms tfs) const;

private:
  /**
   * Get the geometric Jacobian
   * @param tfs Transforms from the base frame to each of the end effector link
   * frames
   * @return Geometric Jacobian
   */
  Jacobian get_geometric_jacobian_(Transforms tfs) const;

  /**
   * Get the pseudoinverse of a matrix
   * @param mat Matrix to get the pseudoinverse of
   * @return Pseudoinverse of the matrix
   */
  Matrix get_pseudoinverse_(Matrix mat) const;

  std::string name_;

  // Rotation axes
  int num_joints_{3};
  Axis n0_{0, 1, 0};
  Axis n1_{0, 0, 1};
  Axis n2_{0, 1, 0};
};

} // namespace wheeled_humanoid