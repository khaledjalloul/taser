#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "base_controller/base_kinematics.hpp"
#include "base_controller/types.hpp"

namespace base_controller {

class RosNode : public rclcpp::Node {
  using TransformStamped = geometry_msgs::msg::TransformStamped;

public:
  RosNode(std::string name);

  TransformMsg get_transform(std::string target_frame,
                             std::string source_frame) const;

  void publish_transform(const TransformMsg &tf) const;

  void joint_state_callback(sensor_msgs::msg::JointState msg);

private:
  std::unique_ptr<BaseKinematics> base_;
  double callback_time_ = -1;

  // Joint states
  RobotJointState joint_positions_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  RobotJointState joint_velocities_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Subscribe to joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  // Publish wheel velocities
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      wheels_joint_velocity_pub_;

  // Transform listener and broadcaster
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace base_controller