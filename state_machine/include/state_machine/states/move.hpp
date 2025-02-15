#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Move : public State {
public:
  Move(std::shared_ptr<RosNode> ros_node);
  ~Move() override;

  void update(StateType &next_state) override;
};

} // namespace state_machine