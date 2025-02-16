#include "state_machine/state_machine.hpp"

#include "state_machine/states/grab.hpp"
#include "state_machine/states/idle.hpp"
#include "state_machine/states/lift.hpp"

namespace state_machine {

StateMachine::StateMachine(std::shared_ptr<RosNode> ros_node, StateType &state)
    : ros_node_(ros_node), next_state_(state) {}

void StateMachine::check_state() {
  if (next_state_ == state_)
    return;

  switch (next_state_) {
  case StateType::IDLE:
    state_obj_.reset(new Idle(ros_node_));
    break;
  case StateType::GRAB:
    state_obj_.reset(new Grab(ros_node_));
    break;
  case StateType::LIFT:
    state_obj_.reset(new Lift(ros_node_));
    break;
  default:
    break;
  }

  state_ = next_state_;
}

void StateMachine::update() {
  check_state();

  state_obj_->update(next_state_);
}

} // namespace state_machine