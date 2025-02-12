#include "arm_controller/arm.hpp"

namespace arm_controller {

Arm::Arm(std::string name, std::string base_frame,
         std::shared_ptr<RosInterface> ros_interface)
    : name_(name), base_frame_(base_frame), ros_interface_(ros_interface) {}

ArmJointState Arm::get_joint_positions() {
  if (name_ == "arm_1") {
    return ros_interface_->get_joint_positions().head(num_joints_);
  } else {
    return ros_interface_->get_joint_positions().segment(num_joints_,
                                                         num_joints_);
  }
}

Pose Arm::get_end_effector_pose(bool absolute_in_map) {
  auto target_frame = absolute_in_map ? "map" : base_frame_;
  auto tf = ros_interface_->get_transform(target_frame, name_ + "_eef");

  auto orientation = tf.rotation.toRotationMatrix().eulerAngles(0, 1, 2);

  return {tf.translation, orientation};
}

// TODO
Pose Arm::get_end_effector_twist() { return Pose(); }

Jacobian Arm::get_geometric_jacobian() {
  auto TBE = ros_interface_->get_transform(base_frame_, name_ + "_eef");
  auto TB0 = ros_interface_->get_transform(base_frame_, name_ + "_1");
  auto TB1 = ros_interface_->get_transform(base_frame_, name_ + "_2");
  auto TB2 = ros_interface_->get_transform(base_frame_, name_ + "_3");

  auto B_r0E = TBE.translation - TB0.translation;
  auto B_r1E = TBE.translation - TB1.translation;
  auto B_r2E = TBE.translation - TB2.translation;

  auto B_n0 = n0_;
  auto B_n1 = TB1.rotation * n1_;
  auto B_n2 = TB2.rotation * n2_;

  Jacobian J;
  J.block<3, 1>(0, 0) = B_n0.cross(B_r0E);
  J.block<3, 1>(0, 1) = B_n1.cross(B_r1E);
  J.block<3, 1>(0, 2) = B_n2.cross(B_r2E);
  J.block<3, 1>(3, 0) = B_n0;
  J.block<3, 1>(3, 1) = B_n1;
  J.block<3, 1>(3, 2) = B_n2;

  return J;
}

Matrix Arm::get_pseudoinverse(Matrix mat) {
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

// TODO
ArmJointState Arm::solve_ik_for_joint_positions(
    Vector p_desired, std::optional<std::vector<int>> jacobian_indices) {
  return ArmJointState();
}

ArmJointState Arm::solve_ik_for_joint_velocities(
    Vector w_desired, std::optional<std::vector<int>> jacobian_indices) {
  auto J = get_geometric_jacobian();
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

  auto dq = J_inv * w_desired;
  return dq;
}

} // namespace arm_controller