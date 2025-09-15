#include "state_machine/states/move_base.hpp"

namespace state_machine {

MoveBase::MoveBase(std::shared_ptr<RosNode> ros_node, taser::Pose2D pose,
                   int target_id)
    : State(ros_node, "MOVE_BASE"), target_id_(target_id) {
  if (target_id_ != -1) {
    goal_.target_id = target_id_;
  } else {
    goal_.x = pose.x;
    goal_.y = pose.y;
    goal_.theta = pose.theta;
  }
}

void MoveBase::enter() {
  if (target_id_ != -1)
    RCLCPP_INFO(ros_node_->get_logger(), "Moving to target %d", target_id_);
  else
    RCLCPP_INFO(ros_node_->get_logger(), "Moving to %f, %f", goal_.x, goal_.y);

  ros_node_->send_move_base_action(goal_, status_);
}

Status MoveBase::update() { return status_; }

} // namespace state_machine