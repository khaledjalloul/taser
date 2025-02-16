#include "state_machine/states/move.hpp"

namespace state_machine {

Move::Move(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Moving...");
}
Move::~Move() { RCLCPP_INFO(ros_node_->get_logger(), "Finished moving."); }

StateType Move::update() { return StateType::IDLE; }

} // namespace state_machine