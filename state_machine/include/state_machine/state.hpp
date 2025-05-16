#pragma once

#include <iostream>
#include <memory>

#include "state_machine/ros_node.hpp"
#include "state_machine/types.hpp"

namespace state_machine {

class State;
using StatePtr = std::unique_ptr<State>;

class State {
public:
  State(std::shared_ptr<RosNode> ros_node, const std::string &name)
      : ros_node_(ros_node), name(name) {}

  virtual void enter() = 0;

  virtual Status update() const = 0;

  const std::string name;
  StatePtr on_success = nullptr;
  StatePtr on_failure = nullptr;

protected:
  std::shared_ptr<RosNode> ros_node_;
};

} // namespace state_machine