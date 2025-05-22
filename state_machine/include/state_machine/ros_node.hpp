#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <wheeled_humanoid_msgs/action/move_arms.hpp>
#include <wheeled_humanoid_msgs/action/move_base.hpp>
#include <wheeled_humanoid_msgs/srv/set_states.hpp>

#include "state_machine/types.hpp"

namespace state_machine {

using Marker = visualization_msgs::msg::Marker;
using MoveArmsAction = wheeled_humanoid_msgs::action::MoveArms;
using MoveArmsGoalHandle = rclcpp_action::ClientGoalHandle<MoveArmsAction>;
using MoveBaseAction = wheeled_humanoid_msgs::action::MoveBase;
using MoveBaseGoalHandle = rclcpp_action::ClientGoalHandle<MoveBaseAction>;
using SetStatesMsg = wheeled_humanoid_msgs::srv::SetStates;

class RosNode : public rclcpp::Node {
public:
  RosNode(std::string name);
  ~RosNode();

  /**
   * Create a service to set the states of the state machine
   * @param callback The callback function to call when the service is called
   */
  void create_set_states_service(
      std::function<void(SetStatesMsg::Request::SharedPtr,
                         SetStatesMsg::Response::SharedPtr)>
          callback);

  /**
   * Send an action goal to the move arms action server
   * @param[in] goal The goal to send
   * @param[out] status The status to set to the state that called this function
   */
  void send_move_arms_action(MoveArmsAction::Goal goal, Status &status) const;

  /**
   * Send an action goal to the move base action server
   * @param[in] goal The goal to send
   * @param[out] status The status to set to the state that called this function
   */
  void send_move_base_action(MoveBaseAction::Goal goal, Status &status) const;

  // List of targets to follow in order
  std::vector<Marker> targets;

private:
  // Service to set the states of the state machine
  rclcpp::Service<SetStatesMsg>::SharedPtr set_states_srv_;

  // Subscription to target markers
  rclcpp::Subscription<Marker>::SharedPtr target_sub_;

  // Action client to move the arms
  rclcpp_action::Client<MoveArmsAction>::SharedPtr arms_action_client_;

  // Action client to move the base
  rclcpp_action::Client<MoveBaseAction>::SharedPtr base_action_client_;

  // Executor
  std::thread executor_thread_;
};

} // namespace state_machine
