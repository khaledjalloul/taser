#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <wheeled_humanoid_msgs/action/move_arms.hpp>
#include <wheeled_humanoid_msgs/srv/set_state.hpp>

#include "state_machine/types.hpp"

namespace state_machine {

using MoveArmsAction = wheeled_humanoid_msgs::action::MoveArms;
using MoveArmsGoalHandle = rclcpp_action::ClientGoalHandle<MoveArmsAction>;
using SetStateMsg = wheeled_humanoid_msgs::srv::SetState;

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name);
  ~RosNode();

  void create_set_state_service(
      std::function<void(int state, std::string &state_name_res)> callback);

  void send_move_arms_action(MoveArmsAction::Goal goal,
                             std::function<void(StateType)> done_cb);

private:
  // Service to set the state of the state machine
  rclcpp::Service<SetStateMsg>::SharedPtr set_state_srv_;

  // Action client to move the arms
  rclcpp_action::Client<MoveArmsAction>::SharedPtr action_client_;

  // Executor
  std::thread executor_thread_;
};

} // namespace state_machine
