#include "state_machine/ros_node.hpp"

namespace state_machine {

RosNode::RosNode(std::string name) : rclcpp::Node(name) {
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

  executor_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });

  RCLCPP_INFO(get_logger(), "Node started.");
}

RosNode::~RosNode() { executor_thread_.join(); }

void RosNode::create_set_state_service(
    std::function<void(int state, std::string &state_name_res)> callback) {
  set_state_srv_ = create_service<SetStateMsg>(
      "/state_machine/set_state",
      [callback](const SetStateMsg::Request::SharedPtr req,
                 SetStateMsg::Response::SharedPtr res) {
        callback(req->state, res->data);
      });
}

} // namespace state_machine
