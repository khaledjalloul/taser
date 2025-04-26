#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveBase : public State {
public:
  MoveBase(std::shared_ptr<RosNode> ros_node);

  void publish_goal(StateType desired_next_state);

  StateType update() const override;

private:
  MoveBaseAction::Goal goal_;
  StateType next_state_ = StateType::IDLE;
};

} // namespace state_machine