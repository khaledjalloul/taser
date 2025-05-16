#include "state_machine/states/move_arms.hpp"

namespace state_machine {

void MoveArms::enter() { ros_node_->send_move_arms_action(goal_, status_); }

Status MoveArms::update() const { return status_; }

RestArms::RestArms(std::shared_ptr<RosNode> ros_node)
    : MoveArms(ros_node, "REST_ARMS") {
  goal_.left_arm.x = 0;
  goal_.left_arm.y = 1.25;
  goal_.left_arm.z = -1;
  goal_.right_arm.x = 0;
  goal_.right_arm.y = -1.25;
  goal_.right_arm.z = -1;
}

Grab::Grab(std::shared_ptr<RosNode> ros_node) : MoveArms(ros_node, "GRAB") {
  goal_.left_arm.x = 1.725;
  goal_.left_arm.y = 0.565;
  goal_.left_arm.z = -0.065;
  goal_.right_arm.x = 1.725;
  goal_.right_arm.y = -0.565;
  goal_.right_arm.z = -0.065;
}

Lift::Lift(std::shared_ptr<RosNode> ros_node) : MoveArms(ros_node, "LIFT") {
  goal_.left_arm.x = 1.725;
  goal_.left_arm.y = 0.565;
  goal_.left_arm.z = 1;
  goal_.right_arm.x = 1.725;
  goal_.right_arm.y = -0.565;
  goal_.right_arm.z = 1;
}

} // namespace state_machine