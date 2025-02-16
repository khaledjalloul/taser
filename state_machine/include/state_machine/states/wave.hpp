#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Wave : public State {
public:
  Wave(std::shared_ptr<RosNode> ros_node);
  ~Wave() override;

  StateType update() override;
};

} // namespace state_machine