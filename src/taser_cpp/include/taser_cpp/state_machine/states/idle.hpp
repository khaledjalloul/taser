#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

/**
 * Idle state that waits for a target to be spawned
 */
class Idle : public State {
public:
  /**
   * Constructor for the idle state
   * @param ros_node The ROS node
   */
  Idle(std::shared_ptr<RosNode> ros_node);

  /**
   * Enter the idle state
   */
  void enter() override {};

  /**
   * Update the idle state one time step
   * @return The status of the state
   */
  Status update() override;
};

} // namespace state_machine