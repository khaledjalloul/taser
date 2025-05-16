#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveArms : public State {
public:
  MoveArms(std::shared_ptr<RosNode> ros_node, const std::string &name)
      : State(ros_node, name) {}

  void enter() override;

  Status update() const override;

protected:
  MoveArmsAction::Goal goal_;
  Status status_ = Status::RUNNING;
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
  Wave(std::shared_ptr<RosNode> ros_node) : MoveArms(ros_node, "WAVE") {}
};

} // namespace state_machine