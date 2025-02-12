#pragma once

#include "arm_controller/arm.hpp"
// #include "arm_controller/transform_listener.hpp"

namespace arm_controller {

class IKSolver {
public:
  IKSolver(const rclcpp::Node::SharedPtr &node);

private:
  // std::shared_ptr<TransformListener> transformListener_;
  std::shared_ptr<Arm> left_arm_, right_arm_;
};

} // namespace arm_controller
