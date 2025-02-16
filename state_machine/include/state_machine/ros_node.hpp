#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <arm_controller/types.hpp>

#include "state_machine/types.hpp"

namespace state_machine {

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name, StateType &state);
  ~RosNode();

  void set_state_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr &request,
      const std_srvs::srv::Trigger::Response::SharedPtr &response);

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

  // Service to set the state of the state machine
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_state_srv_;

  // Executor
  std::thread executor_thread_;

  // State
  StateType &state_;
};

} // namespace state_machine
