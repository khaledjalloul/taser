#pragma once

#include <wheeled_humanoid/types.hpp>

#include "state_machine/state.hpp"

namespace state_machine {

/**
 * State that moves the base of the robot to a given pose or target
 */
class MoveBase : public State {
public:
  /**
   * Constructor for the move base state
   * @param ros_node The ROS node
   * @param pose The pose to move the base to
   * @param target_id The id of the target to move the base to (overrides pose)
   */
  MoveBase(std::shared_ptr<RosNode> ros_node,
           wheeled_humanoid::Pose2D pose = wheeled_humanoid::Pose2D(),
           int target_id = -1);

  /**
   * Enter the move base state
   */
  void enter() override;

  /**
   * Update the move base state one time step
   * @return The status of the state
   */
  Status update() override;

private:
  MoveBaseAction::Goal goal_;
  int target_id_;
  Status status_ = Status::RUNNING;
};

} // namespace state_machine