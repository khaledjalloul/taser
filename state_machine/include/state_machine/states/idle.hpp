#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Idle : public State {
public:
  Idle(std::shared_ptr<RosNode> ros_node);

  void enter() override {};

  Status update() override;
};

} // namespace state_machine