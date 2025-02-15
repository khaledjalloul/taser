#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "state_machine/states/move.hpp"
#include "state_machine/states/wave.hpp"

namespace state_machine {

class StateMachine {
public:
  StateMachine();

  void check_state();

  void start();

private:
  StateType current_state_;
  std::unique_ptr<State> current_state_obj_;
};

} // namespace state_machine