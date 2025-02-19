#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveArms : public State {
public:
  MoveArms(std::shared_ptr<RosNode> ros_node, StateType state,
           const std::string &name)
      : State(ros_node, name), next_state_(state) {}

  StateType update() const override;

  void publish_goal(StateType desired_next_state);

protected:
  MoveArmsAction::Goal goal_;

private:
  StateType next_state_;
};

class RestArms : public MoveArms {
public:
  RestArms(std::shared_ptr<RosNode> ros_node);
};

class Grab : public MoveArms {
public:
  Grab(std::shared_ptr<RosNode> ros_node);
};

class Lift : public MoveArms {
public:
  Lift(std::shared_ptr<RosNode> ros_node);
};

class Wave : public MoveArms {
public:
  Wave(std::shared_ptr<RosNode> ros_node)
      : MoveArms(ros_node, StateType::WAVE, "WAVE") {}
};

} // namespace state_machine