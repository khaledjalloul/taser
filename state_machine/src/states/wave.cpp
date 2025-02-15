#include "state_machine/states/wave.hpp"

#include <iostream>

namespace state_machine {

Wave::Wave(StateType &current_state) : State(current_state) {}
Wave::~Wave() {}

void Wave::update() {
  std::cout << "Wave" << std::endl;
  current_state_ = StateType::MOVE;
}

} // namespace state_machine