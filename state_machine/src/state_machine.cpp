#include "state_machine/state_machine.hpp"

#include "state_machine/states/grab.hpp"
#include "state_machine/states/idle.hpp"
#include "state_machine/states/lift.hpp"

namespace state_machine {

StateMachine::StateMachine(std::shared_ptr<RosNode> ros_node)
    : ros_node_(ros_node), state_{StateType::IDLE},
      state_obj_(new Idle(ros_node)) {

  ros_node->create_set_state_service(
      [this](int state, std::string &state_name_res) {
        state_name_res = set_state(static_cast<StateType>(state));
      });
}

std::string StateMachine::set_state(StateType next_state) {
  if (next_state == state_)
    return "State is already active";

  state_ = next_state;

  switch (next_state) {
  case StateType::IDLE:
    state_obj_.reset(new Idle(ros_node_));
    return "State set to IDLE";
  case StateType::GRAB:
    state_obj_.reset(new Grab(ros_node_));
    return "State set to GRAB";
  case StateType::LIFT:
    state_obj_.reset(new Lift(ros_node_));
    return "State set to LIFT";
  default:
    return "Invalid state";
  }
}

void StateMachine::update() {
  auto next_state = state_obj_->update();
  (void)set_state(next_state);
}

} // namespace state_machine