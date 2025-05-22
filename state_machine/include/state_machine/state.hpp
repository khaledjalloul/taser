#pragma once

#include <iostream>
#include <memory>

#include "state_machine/ros_node.hpp"
#include "state_machine/types.hpp"

namespace state_machine {

class State;
using StatePtr = std::unique_ptr<State>;

/**
 * Base class for all states
 */
class State {
public:
  /**
   * Constructor for the state
   * @param ros_node The ROS node
   * @param name The name of the state
   */
  State(std::shared_ptr<RosNode> ros_node, const std::string &name)
      : ros_node_(ros_node), name(name) {}

  /**
   * Enter the state
   */
  virtual void enter() = 0;

  /**
   * Update the state one time step
   * @return The status of the state
   */
  virtual Status update() = 0;

  const std::string name;
  StatePtr on_success = nullptr;
  StatePtr on_failure = nullptr;

protected:
  std::shared_ptr<RosNode> ros_node_;
};

} // namespace state_machine