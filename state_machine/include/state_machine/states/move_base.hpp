#pragma once

#include <wheeled_humanoid/types.hpp>

#include "state_machine/state.hpp"

namespace state_machine {

class MoveBase : public State {
public:
  MoveBase(std::shared_ptr<RosNode> ros_node,
           wheeled_humanoid::Pose2D pose = wheeled_humanoid::Pose2D());

  void enter() override;

  Status update() override;

private:
  MoveBaseAction::Goal goal_;
  Status status_ = Status::RUNNING;
};

} // namespace state_machine