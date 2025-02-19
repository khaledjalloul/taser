#pragma once

#include <iostream>
#include <memory>

#include "state_machine/ros_node.hpp"
#include "state_machine/types.hpp"

namespace state_machine {

class State {
public:
  State(std::shared_ptr<RosNode> ros_node, const std::string &name)
      : ros_node_(ros_node), name_(name) {
    RCLCPP_INFO(ros_node->get_logger(), "Reached state %s.", name.c_str());
  }

  virtual StateType update() const = 0;

protected:
  std::shared_ptr<RosNode> ros_node_;
  const std::string name_;
};

} // namespace state_machine