#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "base_controller/ackermann_model.hpp"
#include "base_controller/ros_node.hpp"

using namespace base_controller;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("base_controller");

  auto s = WheelState{1, 1, 0};
  auto L = 1.0;
  auto W = 0.5;

  auto v = get_base_velocity(s, L, W);
  auto p = get_base_displacement(v, 0.1, 0.1);

  std::cout << "v: " << v.v << ", w: " << v.w << std::endl;
  std::cout << "x: " << p.x << ", y: " << p.y << ", theta: " << p.theta
            << std::endl;

  TransformMsg tf;
  tf.translation.x = 1;
  tf.translation.y = 0;
  tf.translation.z = 0;
  tf.rotation.w = 1;
  tf.rotation.x = 0;
  tf.rotation.y = 0;
  tf.rotation.z = 0;
  node->publish_transform(tf);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

// TODO: use consts after function names