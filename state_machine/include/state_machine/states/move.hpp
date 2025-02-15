#pragma once

#include "state_machine/states/state.hpp"

namespace state_machine {

class Move : public State {
public:
  Move(StateType &current_state);
  ~Move() override;

  void update() override;

  StateType get_type() override { return StateType::MOVE; }
};

} // namespace state_machine