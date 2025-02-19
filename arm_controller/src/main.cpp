#include <rclcpp/rclcpp.hpp>

#include "arm_controller/ros_node.hpp"

using namespace arm_controller;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("arm_controller");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}