#pragma once

#include "state_machine/state.hpp"

namespace state_machine {

/**
 * Generic state to be inherited from to move the arms of the robot through an
 * action client
 */
class MoveArms : public State {
public:
  /**
   * Constructor for the move arms state
   * @param ros_node The ROS node
   * @param name The name of the state that will inherit from this class
   */
  MoveArms(std::shared_ptr<RosNode> ros_node, const std::string &name)
      : State(ros_node, name) {
    goal_.target_id = -1;
  }

  /**
   * Enter the move arms state
   */
  void enter() override;

  /**
   * Update the move arms state one time step
   * @return The status of the state
   */
  Status update() override;

protected:
  MoveArmsAction::Goal goal_;
  Status status_ = Status::RUNNING;
};

/**
 * State that rests the arms to the sides of the robot
 */
class RestArms : public MoveArms {
public:
  /**
   * Constructor for the rest arms state
   * @param ros_node The ROS node
   */
  RestArms(std::shared_ptr<RosNode> ros_node);
};

/**
 * State that grabs a target
 */
class Grab : public MoveArms {
public:
  /**
   * Constructor for the grab state
   * @param ros_node The ROS node
   * @param target_id The id of the target to grab
   */
  Grab(std::shared_ptr<RosNode> ros_node, int target_id = -1);

private:
  int target_id_;
};

/**
 * State that lifts the arms up
 */
class Lift : public MoveArms {
public:
  /**
   * Constructor for the lift state
   * @param ros_node The ROS node
   */
  Lift(std::shared_ptr<RosNode> ros_node);
};

/**
 * State that waves the arms
 */
class Wave : public MoveArms {
public:
  /**
   * Constructor for the wave state
   * @param ros_node The ROS node
   */
  Wave(std::shared_ptr<RosNode> ros_node) : MoveArms(ros_node, "WAVE") {}
};

} // namespace state_machine