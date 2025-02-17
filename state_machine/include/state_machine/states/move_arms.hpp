#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

class MoveArms : public State {
public:
  MoveArms(std::shared_ptr<RosNode> ros_node,
           StateType state = StateType::MOVE_ARMS,
           const std::string &name = "MOVE_ARMS");

  StateType update() override;

  virtual void set_goal(MoveArmsAction::Goal &goal) {}

private:
  StateType next_state_;
};

class Grab : public MoveArms {
public:
  Grab(std::shared_ptr<RosNode> ros_node)
      : MoveArms(ros_node, StateType::GRAB, "GRAB") {}

  void set_goal(MoveArmsAction::Goal &goal) override;
};

class Lift : public MoveArms {
public:
  Lift(std::shared_ptr<RosNode> ros_node)
      : MoveArms(ros_node, StateType::LIFT, "LIFT") {}

  void set_goal(MoveArmsAction::Goal &goal) override {}
};

class Wave : public MoveArms {
public:
  Wave(std::shared_ptr<RosNode> ros_node)
      : MoveArms(ros_node, StateType::WAVE, "WAVE") {}

  void set_goal(MoveArmsAction::Goal &goal) override {}
};

} // namespace state_machine