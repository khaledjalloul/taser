#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Lift : public State {
public:
  Lift(std::shared_ptr<RosNode> ros_node);
  ~Lift() override;

  void update(StateType &next_state) override;
};

} // namespace state_machine