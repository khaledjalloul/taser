#include "state_machine/ros_node.hpp"

namespace state_machine {

RosNode::RosNode(std::string name, StateType &state)
    : rclcpp::Node(name), state_(state) {
  left_arm_desired_pos_pub = create_publisher<geometry_msgs::msg::Point>(
      "/left_arm_velocity_controller/desired_position", 10);

  right_arm_desired_pos_pub = create_publisher<geometry_msgs::msg::Point>(
      "/right_arm_velocity_controller/desired_position", 10);

  left_arm_current_pos_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/left_arm_velocity_controller/current_position", 10,
      [this](geometry_msgs::msg::Point msg) { left_arm_current_pos = msg; });

  right_arm_current_pos_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/right_arm_velocity_controller/current_position", 10,
      [this](geometry_msgs::msg::Point msg) { right_arm_current_pos = msg; });

  set_state_srv_ = create_service<std_srvs::srv::Trigger>(
      "/state_machine/set_state",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->set_state_callback(request, response);
      });

  executor_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });

  RCLCPP_INFO(get_logger(), "Node started.");
}

RosNode::~RosNode() { executor_thread_.join(); }

void RosNode::set_state_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr &request,
    const std_srvs::srv::Trigger::Response::SharedPtr &response) {
  state_ = StateType::GRAB;
  response->success = true;
  response->message = "State set to Grab.";
}

} // namespace state_machine
