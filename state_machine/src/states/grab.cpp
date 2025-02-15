#include "state_machine/states/grab.hpp"

#include <tuple>

namespace state_machine {

Grab::Grab(std::shared_ptr<RosNode> ros_node) : State(ros_node) {
  RCLCPP_INFO(ros_node->get_logger(), "Grabbing...");

  left_arm_desired_pos_.x = 1;
  left_arm_desired_pos_.y = 1.25;
  left_arm_desired_pos_.z = 0;
  right_arm_desired_pos_.x = 1;
  right_arm_desired_pos_.y = -1.25;
  right_arm_desired_pos_.z = 0;

  ros_node->left_arm_desired_pos_pub->publish(left_arm_desired_pos_);
  ros_node->right_arm_desired_pos_pub->publish(right_arm_desired_pos_);
}

Grab::~Grab() { RCLCPP_INFO(ros_node_->get_logger(), "Finished grabbing."); }

void Grab::update(StateType &next_state) {
  auto err = get_pos_error();

  if (std::get<0>(err) < 0.1 && std::get<1>(err) < 0.1) {
    next_state = StateType::IDLE;
  }
}

std::tuple<double, double> Grab::get_pos_error() {
  auto left_arm_current_pos = ros_node_->left_arm_current_pos;
  auto left_err = pow(left_arm_current_pos.x - left_arm_desired_pos_.x, 2) +
                  pow(left_arm_current_pos.y - left_arm_desired_pos_.y, 2) +
                  pow(left_arm_current_pos.z - left_arm_desired_pos_.z, 2);
  left_err = sqrt(left_err);

  auto right_arm_current_pos = ros_node_->right_arm_current_pos;
  auto right_err = pow(right_arm_current_pos.x - right_arm_desired_pos_.x, 2) +
                   pow(right_arm_current_pos.y - right_arm_desired_pos_.y, 2) +
                   pow(right_arm_current_pos.z - right_arm_desired_pos_.z, 2);
  right_err = sqrt(right_err);

  return {left_err, right_err};
}

} // namespace state_machine