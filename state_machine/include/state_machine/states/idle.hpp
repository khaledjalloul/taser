#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class Idle : public State {
public:
  // TODO: Find out why constructor is being called before other deconstructor
  Idle(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
    RCLCPP_INFO(ros_node->get_logger(), "Reached idle state.");
  }

  StateType update() override { return StateType::IDLE; }
};

} // namespace state_machine