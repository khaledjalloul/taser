#include "state_machine/states/move.hpp"

#include <iostream>

namespace state_machine {

Move::Move(StateType &current_state) : State(current_state) {}
Move::~Move() {}

void Move::update() {
  std::cout << "Move" << std::endl;
  current_state_ = StateType::WAVE;
}

} // namespace state_machine