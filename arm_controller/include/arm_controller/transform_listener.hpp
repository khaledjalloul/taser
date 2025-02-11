#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arm_controller {

using Vector = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

struct EigenTransform {
  Vector translation;
  Quaternion rotation;
};

class TransformListener {
public:
  using Transform = geometry_msgs::msg::TransformStamped;

  TransformListener(const rclcpp::Node::SharedPtr &node);

  EigenTransform get_tf(std::string target_frame, std::string source_frame);

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tfListener_;
};

} // namespace arm_controller