#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_controller/types.hpp"

namespace arm_controller {

class RosInterface : public rclcpp::Node {
public:
  RosInterface(std::string name);

  RobotJointState get_joint_positions();

  RobotJointState get_joint_velocities();

  Transform get_transform(std::string target_frame, std::string source_frame);

  void joint_state_callback(sensor_msgs::msg::JointState msg);

private:
  RobotJointState joint_positions_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  RobotJointState joint_velocities_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
};

} // namespace arm_controller