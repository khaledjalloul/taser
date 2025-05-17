#include "wheeled_humanoid/arm/kinematics.hpp"

namespace wheeled_humanoid::arm {

Kinematics::Kinematics(std::string name) : name_(name) {}

Pose3D Kinematics::get_end_effector_pose(Transform TBE) const {
  auto orientation = TBE.rotation.toRotationMatrix().eulerAngles(0, 1, 2);

  return {TBE.translation[0], TBE.translation[1], TBE.translation[2],
          orientation[0],     orientation[1],     orientation[2]};
}

Twist3D Kinematics::get_end_effector_twist(Transforms tfs,
                                           ArmJointVelocities dq) const {
  auto J = get_geometric_jacobian_(tfs);

  auto w = J * dq;
  return {w[0], w[1], w[2], w[3], w[4], w[5]};
}

Jacobian Kinematics::get_geometric_jacobian_(Transforms tfs) const {
  auto B_r0E = tfs.TBE.translation - tfs.TB0.translation;
  auto B_r1E = tfs.TBE.translation - tfs.TB1.translation;
  auto B_r2E = tfs.TBE.translation - tfs.TB2.translation;

  auto B_n0 = n0_;
  auto B_n1 = tfs.TB1.rotation * n1_;
  auto B_n2 = tfs.TB2.rotation * n2_;

  Jacobian J;
  J.block<3, 1>(0, 0) = B_n0.cross(B_r0E);
  J.block<3, 1>(0, 1) = B_n1.cross(B_r1E);
  J.block<3, 1>(0, 2) = B_n2.cross(B_r2E);
  J.block<3, 1>(3, 0) = B_n0;
  J.block<3, 1>(3, 1) = B_n1;
  J.block<3, 1>(3, 2) = B_n2;

  return J;
}

Matrix Kinematics::get_pseudoinverse_(Matrix mat) const {
  // if (mat.rows() == mat.cols()) {
  //   return mat.inverse();
  // } else if (mat.rows() > mat.cols()) {
  //   // Full column rank
  //   return (mat.transpose() * mat.transpose()).inverse() * mat.transpose();
  // } else {
  //   // Full row rank
  //   return mat.transpose() * (mat * mat.transpose()).inverse();
  // }
  return mat.completeOrthogonalDecomposition().pseudoInverse();
}

ArmJointVelocities
Kinematics::solve_ik_for_joint_velocities(Twist3D w_desired,
                                          Transforms tfs) const {
  auto J = get_geometric_jacobian_(tfs);
  auto J_positional = J.topRows(3);
  auto w_des =
      Vector3(w_desired.linear.x, w_desired.linear.y, w_desired.linear.z);

  // Matrix J_selected;
  // if (!jacobian_indices.has_value()) {
  //   J_selected = J;
  // } else {
  //   for (size_t i = 0; i < jacobian_indices.value().size(); i++) {
  //     J_selected.row(i) = J.row(jacobian_indices.value()[i]);
  //   }
  // }

  auto J_inv = get_pseudoinverse_(J_positional);

  auto dq = J_inv * w_des;
  return dq;
}

} // namespace wheeled_humanoid::arm