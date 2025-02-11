#include "arm_controller/arm.hpp"

namespace arm_controller {

Arm::Arm(std::string name, std::string inertial_frame,
         std::shared_ptr<TransformListener> transformListener)
    : name_(name), inertial_frame_(inertial_frame),
      transformListener_(transformListener) {}

Jacobian Arm::get_geometric_jacobian() {
  auto TIE = transformListener_->get_tf(inertial_frame_, name_ + "_eef");
  auto TI0 = transformListener_->get_tf(inertial_frame_, name_ + "_1");
  auto TI1 = transformListener_->get_tf(inertial_frame_, name_ + "_2");
  auto TI2 = transformListener_->get_tf(inertial_frame_, name_ + "_3");

  auto I_r0E = TIE.translation - TI0.translation;
  auto I_r1E = TIE.translation - TI1.translation;
  auto I_r2E = TIE.translation - TI2.translation;

  auto I_n0 = n0_;
  auto I_n1 = TI1.rotation * n1_;
  auto I_n2 = TI2.rotation * n2_;

  Jacobian J;
  J.block<3, 1>(0, 0) = I_n0.cross(I_r0E);
  J.block<3, 1>(0, 1) = I_n1.cross(I_r1E);
  J.block<3, 1>(0, 2) = I_n2.cross(I_r2E);
  J.block<3, 1>(3, 0) = I_n0;
  J.block<3, 1>(3, 1) = I_n1;
  J.block<3, 1>(3, 2) = I_n2;

  return J;
}

} // namespace arm_controller