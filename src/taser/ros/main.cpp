#include <rclcpp/rclcpp.hpp>

#include "taser_ros/ros_node.hpp"

using namespace taser_ros;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("taser");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}