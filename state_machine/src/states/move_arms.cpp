#include "state_machine/states/move_arms.hpp"

namespace state_machine {

MoveArms::MoveArms(std::shared_ptr<RosNode> ros_node, StateType state,
                   const std::string &name)
    : State(ros_node, name), next_state_(state) {
  next_state_ = state;

  MoveArmsAction::Goal goal;
  set_goal(goal);

  ros_node_->send_move_arms_action(
      goal, [this](StateType next_state) { next_state_ = next_state; });
}

StateType MoveArms::update() { return next_state_; }

void Grab::set_goal(MoveArmsAction::Goal &goal) {
  goal.left_arm.x = 1.725;
  goal.left_arm.y = 0.565;
  goal.left_arm.z = -0.065;
  goal.right_arm.x = 1.725;
  goal.right_arm.y = -0.565;
  goal.right_arm.z = -0.065;
}

} // namespace state_machine