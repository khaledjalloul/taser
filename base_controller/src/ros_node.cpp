#include "base_controller/ros_node.hpp"

namespace base_controller {

RosNode::RosNode(std::string name) : rclcpp::Node(name) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "Node started.");
}

void RosNode::publish_transform(const TransformMsg &tf) {
  TransformStamped tf_stamped;
  tf_stamped.header.stamp = now();
  tf_stamped.header.frame_id = "odom";
  tf_stamped.child_frame_id = "base_wrapper";
  tf_stamped.transform = tf;

  tf_broadcaster_->sendTransform(tf_stamped);
}

} // namespace base_controller