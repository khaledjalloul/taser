#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <wheeled_humanoid_msgs/action/move_arms.hpp>
#include <wheeled_humanoid_msgs/action/move_base.hpp>
#include <wheeled_humanoid_msgs/srv/set_state.hpp>

#include "state_machine/types.hpp"

namespace state_machine {

using MoveArmsAction = wheeled_humanoid_msgs::action::MoveArms;
using MoveArmsGoalHandle = rclcpp_action::ClientGoalHandle<MoveArmsAction>;
using MoveBaseAction = wheeled_humanoid_msgs::action::MoveBase;
using MoveBaseGoalHandle = rclcpp_action::ClientGoalHandle<MoveBaseAction>;
using SetStateMsg = wheeled_humanoid_msgs::srv::SetState;

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name);
  ~RosNode();

  void create_set_state_service(
      std::function<void(int state, std::string &state_name_res)> callback);

  void send_move_arms_action(
      MoveArmsAction::Goal goal,
      std::function<void(std::optional<StateType>)> done_cb) const;

  void send_move_base_action(
      MoveBaseAction::Goal goal,
      std::function<void(std::optional<StateType>)> done_cb) const;

private:
  // Service to set the state of the state machine
  rclcpp::Service<SetStateMsg>::SharedPtr set_state_srv_;

  // Action client to move the arms
  rclcpp_action::Client<MoveArmsAction>::SharedPtr arms_action_client_;

  // Action client to move the base
  rclcpp_action::Client<MoveBaseAction>::SharedPtr base_action_client_;

  // Executor
  std::thread executor_thread_;
};

} // namespace state_machine
