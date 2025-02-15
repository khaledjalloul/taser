#pragma once

#include <iostream>
#include <memory>

#include "state_machine/ros_node.hpp"

namespace state_machine {

enum class StateType { IDLE, MOVE, GRAB, LIFT, WAVE };

class State {
public:
  State(std::shared_ptr<RosNode> ros_node) : ros_node_(ros_node) {}
  virtual ~State() {}

  virtual void update(StateType &next_state) = 0;

protected:
  std::shared_ptr<RosNode> ros_node_;
};

} // namespace state_machine