#include "state_machine/states/move_base.hpp"

namespace state_machine {

MoveBase::MoveBase(std::shared_ptr<RosNode> ros_node)
    : State(ros_node, "MOVE_BASE") {
  //   goal_.position = ?;
  publish_goal(StateType::IDLE);
}

void MoveBase::publish_goal(StateType desired_next_state) {
  ros_node_->send_move_base_action(
      goal_, [this, desired_next_state](std::optional<StateType> next_state) {
        if (next_state.has_value()) {
          next_state_ = next_state.value();
        } else {
          next_state_ = desired_next_state;
        }
      });
}

StateType MoveBase::update() const { return next_state_; }

} // namespace state_machine