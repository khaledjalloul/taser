#pragma once

#include "state_machine/states/state.hpp"

namespace state_machine {

class Wave : public State {
public:
  Wave(StateType &current_state);
  ~Wave() override;

  void update() override;

  StateType get_type() override { return StateType::WAVE; }
};

} // namespace state_machine