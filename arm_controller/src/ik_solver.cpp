#include "arm_controller/ik_solver.hpp"

namespace arm_controller {

IKSolver::IKSolver(const rclcpp::Node::SharedPtr &node) {
  transformListener_ = std::make_shared<TransformListener>(node);
  left_arm_ = std::make_shared<Arm>("arm_1", "base", transformListener_);
  right_arm_ = std::make_shared<Arm>("arm_2", "base", transformListener_);
}

} // namespace arm_controller