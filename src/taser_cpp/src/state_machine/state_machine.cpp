#include "state_machine/state_machine.hpp"

#include "state_machine/states/idle.hpp"
#include "state_machine/states/move_arms.hpp"
#include "state_machine/states/move_base.hpp"

namespace state_machine {

StateMachine::StateMachine(std::shared_ptr<RosNode> ros_node)
    : ros_node_(ros_node) {

  ros_node->create_set_states_service(
      [this](SetStatesMsg::Request::SharedPtr req,
             SetStatesMsg::Response::SharedPtr res) {
        set_state_mutex_.lock();
        res->data = set_states(req);
        set_state_mutex_.unlock();
      });

  state_ = std::make_unique<RestArms>(ros_node_);
  state_->enter();
}

std::string StateMachine::set_states(SetStatesMsg::Request::SharedPtr req) {
  if (req->states.empty())
    return "No states provided";

  StatePtr first_state = nullptr;
  for (int i = req->states.size() - 1; i >= 0; i--) {
    auto state_type = req->states[i];
    StatePtr new_state;

    switch (state_type) {
    case StateType::IDLE:
      new_state = std::make_unique<Idle>(ros_node_);
      break;
    case StateType::MOVE_BASE:
      new_state = std::make_unique<MoveBase>(ros_node_);
      break;
    case StateType::REST_ARMS:
      new_state = std::make_unique<RestArms>(ros_node_);
      break;
    case StateType::GRAB:
      new_state = std::make_unique<Grab>(ros_node_);
      break;
    case StateType::LIFT:
      new_state = std::make_unique<Lift>(ros_node_);
      break;
    default:
      return "Invalid state: " + state_type;
    }

    new_state->on_success = std::move(first_state);
    first_state = std::move(new_state);
  }

  state_ = std::move(first_state);
  RCLCPP_INFO(ros_node_->get_logger(), "Reached state %s.",
              state_->name.c_str());
  state_->enter();

  return "Created state(s)";
}

void StateMachine::update() {
  // Lock mutex to avoid executing and setting states at the same time
  set_state_mutex_.lock();

  auto status = state_->update();

  // Do nothing if current state is running
  if (status == Status::RUNNING) {
    set_state_mutex_.unlock();
    return;
  }

  if (status == Status::SUCCESS) {
    RCLCPP_INFO(ros_node_->get_logger(), "Finished state %s.",
                state_->name.c_str());
    state_ = std::move(state_->on_success);
  }

  if (status == Status::FAILURE) {
    RCLCPP_INFO(ros_node_->get_logger(), "Failed state %s.",
                state_->name.c_str());
    state_ = std::move(state_->on_failure);
  }

  if (!state_)
    state_ = std::make_unique<Idle>(ros_node_);

  RCLCPP_INFO(ros_node_->get_logger(), "Reached state %s.",
              state_->name.c_str());
  state_->enter();

  set_state_mutex_.unlock();
}

} // namespace state_machine