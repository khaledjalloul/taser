#pragma once

#include <mutex>

#include "state_machine/ros_node.hpp"
#include "state_machine/state.hpp"

namespace state_machine {

/**
 * State machine that manages the states of the robot
 */
class StateMachine {
public:
  /**
   * Constructor for the state machine
   * @param ros_node The ROS node
   */
  StateMachine(std::shared_ptr<RosNode> ros_node);

  /**
   * Set the states of the state machine
   * @param req The ROS service request containing a list of desired states
   * @return The result message to return to the service caller
   */
  std::string set_states(SetStatesMsg::Request::SharedPtr req);

  /**
   * Update the state machine one time step
   */
  void update();

private:
  StatePtr state_;
  std::shared_ptr<RosNode> ros_node_;
  std::mutex set_state_mutex_;
};

} // namespace state_machine