#include "state_machine/states/move_arms.hpp"

namespace state_machine {

void MoveArms::publish_goal(StateType desired_next_state) {
  ros_node_->send_move_arms_action(
      goal_, [this, desired_next_state](std::optional<StateType> next_state) {
        if (next_state.has_value()) {
          next_state_ = next_state.value();
        } else {
          next_state_ = desired_next_state;
        }
      });
}

StateType MoveArms::update() { return next_state_; }

RestArms::RestArms(std::shared_ptr<RosNode> ros_node)
    : MoveArms(ros_node, StateType::REST_ARMS, "REST_ARMS") {
  goal_.left_arm.x = 0;
  goal_.left_arm.y = 1.25;
  goal_.left_arm.z = -1;
  goal_.right_arm.x = 0;
  goal_.right_arm.y = -1.25;
  goal_.right_arm.z = -1;
  publish_goal(StateType::IDLE);
}

Grab::Grab(std::shared_ptr<RosNode> ros_node)
    : MoveArms(ros_node, StateType::GRAB, "GRAB") {
  goal_.left_arm.x = 1.725;
  goal_.left_arm.y = 0.565;
  goal_.left_arm.z = -0.065;
  goal_.right_arm.x = 1.725;
  goal_.right_arm.y = -0.565;
  goal_.right_arm.z = -0.065;
  publish_goal(StateType::LIFT);
}

Lift::Lift(std::shared_ptr<RosNode> ros_node)
    : MoveArms(ros_node, StateType::LIFT, "LIFT") {
  goal_.left_arm.x = 1.725;
  goal_.left_arm.y = 0.565;
  goal_.left_arm.z = 1;
  goal_.right_arm.x = 1.725;
  goal_.right_arm.y = -0.565;
  goal_.right_arm.z = 1;
  publish_goal(StateType::REST_ARMS);
}

} // namespace state_machine