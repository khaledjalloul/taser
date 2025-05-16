#include "state_machine/ros_node.hpp"

namespace state_machine {

RosNode::RosNode(std::string name) : rclcpp::Node(name) {
  arms_action_client_ = rclcpp_action::create_client<MoveArmsAction>(
      this, "/arm_controller/move_to");

  base_action_client_ = rclcpp_action::create_client<MoveBaseAction>(
      this, "/base_controller/move_to");

  target_sub_ = create_subscription<Marker>(
      "/targets", 10, [this](const Marker &msg) { targets.push_back(msg); });

  executor_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });

  RCLCPP_INFO(get_logger(), "Node started.");
}

RosNode::~RosNode() { executor_thread_.join(); }

void RosNode::create_set_states_service(
    std::function<void(SetStatesMsg::Request::SharedPtr,
                       SetStatesMsg::Response::SharedPtr)>
        callback) {
  set_states_srv_ =
      create_service<SetStatesMsg>("/state_machine/set_states", callback);
}

void RosNode::send_move_arms_action(MoveArmsAction::Goal goal,
                                    Status &status) const {
  if (!arms_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    status = Status::FAILURE;
    return;
  }

  auto goal_options = rclcpp_action::Client<MoveArmsAction>::SendGoalOptions();

  goal_options.result_callback =
      [this, &status](const MoveArmsGoalHandle::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          status = Status::SUCCESS;
          break;
        default:
          status = Status::FAILURE;
          break;
        }
      };

  arms_action_client_->async_send_goal(goal, goal_options);
}

void RosNode::send_move_base_action(MoveBaseAction::Goal goal,
                                    Status &status) const {
  if (!base_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    status = Status::FAILURE;
    return;
  }

  auto goal_options = rclcpp_action::Client<MoveBaseAction>::SendGoalOptions();

  goal_options.result_callback =
      [this, &status](const MoveBaseGoalHandle::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          status = Status::SUCCESS;
          break;
        default:
          status = Status::FAILURE;
          break;
        }
      };

  base_action_client_->async_send_goal(goal, goal_options);
}

} // namespace state_machine
