#include "taser/arm_motion/kinematics.hpp"

namespace taser::arm_motion {

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

ArmJointVelocities
Kinematics::solve_ik_for_joint_velocities(Twist3D w_desired,
                                          Transforms tfs) const {
  auto J = get_geometric_jacobian_(tfs);
  auto J_positional = J.topRows(3);
  auto w_des =
      Vector3(w_desired.linear.x, w_desired.linear.y, w_desired.linear.z);

  auto J_inv = get_pseudoinverse_(J_positional);

  auto dq = J_inv * w_des;
  return dq;
}

Pose3D Kinematics::transform(Pose3D pose, Transform tf) const {
  // Prepare Eigen transform
  Eigen::Isometry3d T_AB = Eigen::Isometry3d::Identity();
  T_AB.rotate(tf.rotation);
  T_AB.pretranslate(tf.translation);

  // Prepare original orientation as an Eigen quaternion
  Eigen::AngleAxisd rot_x(pose.orientation.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y(pose.orientation.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z(pose.orientation.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_B = rot_x * rot_y * rot_z;

  // Prepare original position as an Eigen vector
  Eigen::Vector3d p_B(pose.position.x, pose.position.y, pose.position.z);

  // Apply transformation
  Eigen::Vector3d p_A = T_AB * p_B;
  Eigen::Quaterniond q_A = tf.rotation * q_B;

  Eigen::Vector3d euler_A = q_A.toRotationMatrix().eulerAngles(0, 1, 2);

  return {{p_A(0), p_A(1), p_A(2)}, {euler_A(0), euler_A(1), euler_A(2)}};
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
  return mat.completeOrthogonalDecomposition().pseudoInverse();
}

} // namespace taser::arm_motion