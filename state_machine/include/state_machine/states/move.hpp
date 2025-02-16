#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Move : public State {
public:
  Move(std::shared_ptr<RosNode> ros_node);
  ~Move() override;

  StateType update() override;
};

} // namespace state_machine