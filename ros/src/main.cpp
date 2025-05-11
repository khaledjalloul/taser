#include <rclcpp/rclcpp.hpp>

#include "wheeled_humanoid_ros/ros_node.hpp"

using namespace wheeled_humanoid_ros;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("wheeled_humanoid");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}