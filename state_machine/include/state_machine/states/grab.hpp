#pragma once

#include <geometry_msgs/msg/point.hpp>

#include "state_machine/state.hpp"

namespace state_machine {

// TODO: combine move arm state machine classes into one
class Grab : public State {
public:
  Grab(std::shared_ptr<RosNode> ros_node);
  ~Grab() override;

  StateType update() override;

private:
  StateType next_state_;
};

} // namespace state_machine