#include <rclcpp/rclcpp.hpp>

#include "base_controller/ros_node.hpp"

using namespace base_controller;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("base_controller");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

// TODO: use consts after function names