#include "arm_controller/ros_node.hpp"

namespace arm_controller {

RosNode::RosNode(std::string name)
    : rclcpp::Node(name), buffer_(get_clock()), tf_listener_(buffer_) {

  left_arm_desired_pos.x = 0;
  left_arm_desired_pos.y = 1.25;
  left_arm_desired_pos.z = -1;
  right_arm_desired_pos.x = 0;
  right_arm_desired_pos.y = -1.25;
  right_arm_desired_pos.z = -1;

  left_arm_joint_velocity_pub =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/left_arm_velocity_controller/commands", 10);

  right_arm_joint_velocity_pub =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/right_arm_velocity_controller/commands", 10);

  left_arm_desired_pos_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/left_arm_velocity_controller/desired_position", 10,
      [this](geometry_msgs::msg::Point msg) { left_arm_desired_pos = msg; });

  right_arm_desired_pos_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/right_arm_velocity_controller/desired_position", 10,
      [this](geometry_msgs::msg::Point msg) { right_arm_desired_pos = msg; });

  left_arm_current_pos_pub = create_publisher<geometry_msgs::msg::Point>(
      "/left_arm_velocity_controller/current_position", 10);

  right_arm_current_pos_pub = create_publisher<geometry_msgs::msg::Point>(
      "/right_arm_velocity_controller/current_position", 10);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState msg) { joint_state_callback(msg); });

  executor_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });

  RCLCPP_INFO(get_logger(), "Node started.");
}

RosNode::~RosNode() { executor_thread_.join(); }

Transform RosNode::get_transform(std::string target_frame,
                                 std::string source_frame) {
  try {
    auto tf =
        buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return {Position(tf.transform.translation.x, tf.transform.translation.y,
                     tf.transform.translation.z),
            Quaternion(tf.transform.rotation.w, tf.transform.rotation.x,
                       tf.transform.rotation.y, tf.transform.rotation.z)};
  } catch (tf2::TransformException &ex) {
    return {Position(0, 0, 0), Quaternion(1, 0, 0, 0)};
  }
}

Transforms RosNode::get_arm_transforms(std::string arm_name) {
  Transforms tfs;
  tfs.TBE = get_transform("base", arm_name + "_eef");
  tfs.TB0 = get_transform("base", arm_name + "_1");
  tfs.TB1 = get_transform("base", arm_name + "_2");
  tfs.TB2 = get_transform("base", arm_name + "_3");
  return tfs;
}

// TODO: optimize to store msg as is, then get later as eigen, maybe a template
// function
void RosNode::joint_state_callback(sensor_msgs::msg::JointState msg) {
  for (size_t i; i < msg.position.size(); i++) {
    joint_positions_[i] = msg.position[i];
  }

  for (size_t i; i < msg.velocity.size(); i++) {
    joint_velocities_[i] = msg.velocity[i];
  }
}

} // namespace arm_controller
