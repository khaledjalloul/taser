#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "arm_controller/types.hpp"

namespace arm_controller {

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name);
  ~RosNode();

  Transform get_transform(std::string target_frame, std::string source_frame);

  Transforms get_arm_transforms(std::string arm_name);

  void joint_state_callback(sensor_msgs::msg::JointState msg);

  // Desired arm end effector positions
  geometry_msgs::msg::Point left_arm_desired_pos;
  geometry_msgs::msg::Point right_arm_desired_pos;

  // Publish to topics that control the joints
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_arm_joint_velocity_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_arm_joint_velocity_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      left_arm_current_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      right_arm_current_pos_pub;

private:
  RobotJointState joint_positions_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  RobotJointState joint_velocities_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Listen to robot transforms
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Subscribe to joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  // Subscribe to topics holding desired positions
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      left_arm_desired_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      right_arm_desired_pos_sub_;

  // Executor
  std::thread executor_thread_;
};

} // namespace arm_controller