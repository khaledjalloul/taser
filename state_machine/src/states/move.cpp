#include "state_machine/states/move.hpp"

namespace state_machine {

Move::Move(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Moving...");
}
Move::~Move() { RCLCPP_INFO(ros_node_->get_logger(), "Finished moving."); }

void Move::update(StateType &next_state) { next_state = StateType::WAVE; }

} // namespace state_machine