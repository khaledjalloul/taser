#pragma once

#include <mutex>

#include "state_machine/ros_node.hpp"
#include "state_machine/state.hpp"

namespace state_machine {

class StateMachine {
public:
  StateMachine(std::shared_ptr<RosNode> ros_node);

  std::string set_states(SetStatesMsg::Request::SharedPtr req);

  void update();

private:
  StatePtr state_;
  std::shared_ptr<RosNode> ros_node_;
  std::mutex set_state_mutex_;
};

} // namespace state_machine