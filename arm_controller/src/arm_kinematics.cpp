#include "arm_controller/arm_kinematics.hpp"

namespace arm_controller {

ArmKinematics::ArmKinematics(std::string name) : name_(name) {}

Pose ArmKinematics::get_end_effector_pose(Transform TBE) {
  auto orientation = TBE.rotation.toRotationMatrix().eulerAngles(0, 1, 2);

  return {TBE.translation, orientation};
}

Twist ArmKinematics::get_end_effector_twist(Transforms tfs, ArmJointState dq) {
  auto J = get_geometric_jacobian(tfs);

  auto w = J * dq;
  return {w.head(3), w.tail(3)};
}

Jacobian ArmKinematics::get_geometric_jacobian(Transforms tfs) {
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

Matrix ArmKinematics::get_pseudoinverse(Matrix mat) {
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

ArmJointState ArmKinematics::solve_ik_for_joint_velocities(Twist w_desired,
                                                           Transforms tfs) {
  auto J = get_geometric_jacobian(tfs);
  auto J_positional = J.topRows(3);

  // Matrix J_selected;
  // if (!jacobian_indices.has_value()) {
  //   J_selected = J;
  // } else {
  //   for (size_t i = 0; i < jacobian_indices.value().size(); i++) {
  //     J_selected.row(i) = J.row(jacobian_indices.value()[i]);
  //   }
  // }

  auto J_inv = get_pseudoinverse(J_positional);

  auto dq = J_inv * w_desired.linear;
  return dq;
}

} // namespace arm_controller