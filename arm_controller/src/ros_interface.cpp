#include "arm_controller/ros_interface.hpp"

namespace arm_controller {

RosInterface::RosInterface(std::string name)
    : rclcpp::Node(name), buffer_(get_clock()), tf_listener_(buffer_) {

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState msg) { joint_state_callback(msg); });
}

RobotJointState RosInterface::get_joint_positions() { return joint_positions_; }

RobotJointState RosInterface::get_joint_velocities() {
  return joint_velocities_;
}

Transform RosInterface::get_transform(std::string target_frame,
                                      std::string source_frame) {
  try {
    auto tf =
        buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return {Position(tf.transform.translation.x, tf.transform.translation.y,
                     tf.transform.translation.z),
            Quaternion(tf.transform.rotation.w, tf.transform.rotation.x,
                       tf.transform.rotation.y, tf.transform.rotation.z)};
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
    return {Position(0, 0, 0), Quaternion(1, 0, 0, 0)};
  }
}

void RosInterface::joint_state_callback(sensor_msgs::msg::JointState msg) {
  for (size_t i; i < msg.position.size(); i++) {
    joint_positions_[i] = msg.position[i];
  }

  for (size_t i; i < msg.velocity.size(); i++) {
    joint_velocities_[i] = msg.velocity[i];
  }
}

} // namespace arm_controller