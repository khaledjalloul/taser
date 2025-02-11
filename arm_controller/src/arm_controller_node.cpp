#include <rclcpp/rclcpp.hpp>

#include "arm_controller/arm.hpp"
#include "arm_controller/transform_listener.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_controller_node");
  auto transformListener =
      std::make_shared<arm_controller::TransformListener>(node);
  auto left_arm_ =
      std::make_shared<arm_controller::Arm>("arm_1", "base", transformListener);
  while (rclcpp::ok()) {
    auto J = left_arm_->get_geometric_jacobian();
    RCLCPP_INFO(node->get_logger(), "Jacobian: %f %f %f %f %f %f %f %f %f",
                J(0, 0), J(0, 1), J(0, 2), J(1, 0), J(1, 1), J(1, 2), J(2, 0),
                J(2, 1), J(2, 2));
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
