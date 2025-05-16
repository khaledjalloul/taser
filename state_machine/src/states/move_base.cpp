#include "state_machine/states/move_base.hpp"

namespace state_machine {

MoveBase::MoveBase(std::shared_ptr<RosNode> ros_node,
                   wheeled_humanoid::Pose2D pose)
    : State(ros_node, "MOVE_BASE") {
  goal_.x = pose.x;
  goal_.y = pose.y;
  goal_.theta = pose.theta;
}

void MoveBase::enter() {
  RCLCPP_INFO(ros_node_->get_logger(), "Moving to %f, %f", goal_.x, goal_.y);
  ros_node_->send_move_base_action(goal_, status_);
}

Status MoveBase::update() { return status_; }

} // namespace state_machine