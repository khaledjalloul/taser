#include "state_machine/states/lift.hpp"

namespace state_machine {

Lift::Lift(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Lifting...");
}
Lift::~Lift() { RCLCPP_INFO(ros_node_->get_logger(), "Finished lifting."); }

StateType Lift::update() { return StateType::IDLE; }

} // namespace state_machine