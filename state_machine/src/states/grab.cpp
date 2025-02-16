#include "state_machine/states/grab.hpp"

#include <tuple>

namespace state_machine {

Grab::Grab(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Grabbing...");

  next_state_ = StateType::GRAB;

  MoveArmsAction::Goal goal;
  goal.left_arm.x = 1.725;
  goal.left_arm.y = 0.565;
  goal.left_arm.z = -0.065;
  goal.right_arm.x = 1.725;
  goal.right_arm.y = -0.565;
  goal.right_arm.z = -0.065;

  ros_node_->send_move_arms_action(
      goal, [this](StateType next_state) { next_state_ = next_state; });
}

Grab::~Grab() { RCLCPP_INFO(ros_node_->get_logger(), "Finished grabbing."); }

StateType Grab::update() { return next_state_; }

} // namespace state_machine