#include "state_machine/states/idle.hpp"

#include "state_machine/states/move_arms.hpp"
#include "state_machine/states/move_base.hpp"

namespace state_machine {

using Marker = visualization_msgs::msg::Marker;

Idle::Idle(std::shared_ptr<RosNode> ros_node) : State(ros_node, "IDLE") {}

Status Idle::update() {
  if (ros_node_->targets.empty())
    return Status::RUNNING;

  const auto target = ros_node_->targets.front();
  ros_node_->targets.erase(ros_node_->targets.begin());
  const auto &pos = target.pose.position;

  RCLCPP_INFO(ros_node_->get_logger(), "Detected target at (x: %f, y: %f).",
              pos.x, pos.y);

  auto move_to_target =
      std::make_unique<MoveBase>(ros_node_, taser_cpp::Pose2D(), target.id);
  auto grab_target = std::make_unique<Grab>(ros_node_, target.id);
  auto rest_arms = std::make_unique<RestArms>(ros_node_);

  grab_target->on_success = std::move(rest_arms);
  move_to_target->on_success = std::move(grab_target);
  on_success = std::move(move_to_target);

  return Status::SUCCESS;
}

} // namespace state_machine