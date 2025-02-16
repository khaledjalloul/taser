#pragma once

#include <geometry_msgs/msg/point.hpp>

#include "state_machine/state.hpp"

namespace state_machine {

class Grab : public State {
public:
  Grab(std::shared_ptr<RosNode> ros_node);
  ~Grab() override;

  StateType update() override;

  std::tuple<double, double> get_pos_error();

private:
  geometry_msgs::msg::Point left_arm_desired_pos_, right_arm_desired_pos_;
};

} // namespace state_machine