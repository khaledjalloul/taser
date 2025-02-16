#pragma once

#include <iostream>
#include <memory>
#include <mutex>

#include "state_machine/ros_node.hpp"
#include "state_machine/state.hpp"

namespace state_machine {

class StateMachine {
public:
  StateMachine(std::shared_ptr<RosNode> ros_node);

  std::string set_state(StateType desired_state);

  void update();

private:
  StateType state_;
  std::unique_ptr<State> state_obj_;
  std::shared_ptr<RosNode> ros_node_;
  std::mutex set_state_mutex_;
};

} // namespace state_machine