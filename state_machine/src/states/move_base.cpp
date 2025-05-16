#include "state_machine/states/move_base.hpp"

namespace state_machine {

void MoveBase::enter() { ros_node_->send_move_base_action(goal_, status_); }

Status MoveBase::update() const { return status_; }

} // namespace state_machine