#include "state_machine/state_machine.hpp"

#include "state_machine/states/idle.hpp"
#include "state_machine/states/move_arms.hpp"
#include "state_machine/states/move_base.hpp"

namespace state_machine {

StateMachine::StateMachine(std::shared_ptr<RosNode> ros_node)
    : ros_node_(ros_node), state_{StateType::IDLE},
      state_obj_(new Idle(ros_node)) {

  ros_node->create_set_state_service([this](int state, std::string &res) {
    set_state_mutex_.lock();
    res = set_state(static_cast<StateType>(state));
    set_state_mutex_.unlock();
  });
}

std::string StateMachine::set_state(StateType next_state) {
  if (next_state == state_)
    return "State is already active";

  auto old_state = state_;
  state_ = next_state;

  switch (next_state) {
  case StateType::IDLE:
    state_obj_.reset(new Idle(ros_node_));
    return "State set to IDLE";
  case StateType::MOVE_BASE:
    state_obj_.reset(new MoveBase(ros_node_));
    return "State set to MOVE_BASE";
  case StateType::REST_ARMS:
    state_obj_.reset(new RestArms(ros_node_));
    return "State set to REST_ARMS";
  case StateType::GRAB:
    state_obj_.reset(new Grab(ros_node_));
    return "State set to GRAB";
  case StateType::LIFT:
    state_obj_.reset(new Lift(ros_node_));
    return "State set to LIFT";
  default:
    state_ = old_state;
    return "Invalid state";
  }
}

void StateMachine::update() {
  set_state_mutex_.lock();
  auto next_state = state_obj_->update();
  (void)set_state(next_state);
  set_state_mutex_.unlock();
}

} // namespace state_machine