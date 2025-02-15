#pragma once

#include <iostream>
#include <memory>

#include "state_machine/ros_node.hpp"
#include "state_machine/state.hpp"

namespace state_machine {

class StateMachine {
public:
  StateMachine(std::shared_ptr<RosNode> ros_node);

  void check_state();

  void update();

private:
  StateType state_{-1};
  StateType next_state_{StateType::GRAB};
  std::unique_ptr<State> state_obj_;
  std::shared_ptr<RosNode> ros_node_;
};

} // namespace state_machine