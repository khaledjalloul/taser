#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveBase : public State {
public:
  MoveBase(std::shared_ptr<RosNode> ros_node) : State(ros_node, "MOVE_BASE") {}

  void enter() override;

  Status update() const override;

private:
  MoveBaseAction::Goal goal_;
  Status status_ = Status::RUNNING;
};

} // namespace state_machine