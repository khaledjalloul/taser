#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "base_controller/types.hpp"

namespace base_controller {

class RosNode : public rclcpp::Node {
  using TransformStamped = geometry_msgs::msg::TransformStamped;

public:
  RosNode(std::string name);

  void publish_transform(const TransformMsg &tf);

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace base_controller