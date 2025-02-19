#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveBase : public State {
public:
  MoveBase(std::shared_ptr<RosNode> ros_node);

  StateType update() const override;
};

} // namespace state_machine