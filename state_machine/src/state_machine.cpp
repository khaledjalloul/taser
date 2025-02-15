#include "state_machine/state_machine.hpp"

namespace state_machine {

StateMachine::StateMachine()
    : current_state_(StateType::MOVE),
      current_state_obj_(new Move(current_state_)) {}

void StateMachine::check_state() {
  if (current_state_ == current_state_obj_->get_type()) {
    return;
  }

  switch (current_state_) {
  case StateType::MOVE:
    current_state_obj_.reset(new Move(current_state_));
    break;
  case StateType::WAVE:
    current_state_obj_.reset(new Wave(current_state_));
    break;
  default:
    break;
  }
}

void StateMachine::start() {
  rclcpp::Rate r(1);

  while (rclcpp::ok()) {
    check_state();

    current_state_obj_->update();

    r.sleep();
  }
}

} // namespace state_machine