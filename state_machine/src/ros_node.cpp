#include "state_machine/ros_node.hpp"

namespace state_machine {

RosNode::RosNode(std::string name) : rclcpp::Node(name) {
  action_client_ = rclcpp_action::create_client<MoveArmsAction>(
      this, "/arm_controller/move_to");

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

void RosNode::send_move_arms_action(
    MoveArmsAction::Goal goal,
    std::function<void(std::optional<StateType>)> done_cb) {
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    done_cb(StateType::IDLE);
    return;
  }

  auto goal_options = rclcpp_action::Client<MoveArmsAction>::SendGoalOptions();

  goal_options.result_callback =
      [this, done_cb](const MoveArmsGoalHandle::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Move arms succeeded.");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Move arms failed.");
          return;
        }
        done_cb(std::nullopt);
      };

  action_client_->async_send_goal(goal, goal_options);
}

} // namespace state_machine
