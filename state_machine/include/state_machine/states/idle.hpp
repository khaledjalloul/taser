#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Idle : public State {
public:
  Idle(std::shared_ptr<RosNode> ros_node) : State(ros_node, "IDLE") {}

  void enter() override {}

  Status update() const override { return Status::SUCCESS; }
};

} // namespace state_machine