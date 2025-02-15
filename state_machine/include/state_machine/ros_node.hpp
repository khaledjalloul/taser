#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arm_controller/types.hpp>

namespace state_machine {

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name);
  ~RosNode();

  // Current arm end effector positions
  geometry_msgs::msg::Point left_arm_current_pos;
  geometry_msgs::msg::Point right_arm_current_pos;

  // Publish to topics holding desired arm positions
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      left_arm_desired_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      right_arm_desired_pos_pub;

private:
  // Subscribe to topics holding current arm end effector positions
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      left_arm_current_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      right_arm_current_pos_sub_;

  // Executor
  std::thread executor_thread_;
};

} // namespace state_machine
