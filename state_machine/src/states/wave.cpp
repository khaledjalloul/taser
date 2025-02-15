#include "state_machine/states/wave.hpp"

namespace state_machine {

Wave::Wave(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Waving...");
}
Wave::~Wave() { RCLCPP_INFO(ros_node_->get_logger(), "Finished waving."); }

void Wave::update(StateType &next_state) { next_state = StateType::MOVE; }

} // namespace state_machine