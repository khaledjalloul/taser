#include "state_machine/states/wave.hpp"

namespace state_machine {

Wave::Wave(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Waving...");
}
Wave::~Wave() { RCLCPP_INFO(ros_node_->get_logger(), "Finished waving."); }

StateType Wave::update() { return StateType::IDLE; }

} // namespace state_machine