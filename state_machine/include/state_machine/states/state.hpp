#pragma once

namespace state_machine {

enum class StateType { MOVE, WAVE };

class State {
public:
  State(StateType &current_state) : current_state_(current_state) {}
  virtual ~State(){};

  virtual void update() = 0;

  virtual StateType get_type() = 0;

protected:
  StateType &current_state_;
};

} // namespace state_machine