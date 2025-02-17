#include "state_machine/states/move_base.hpp"

namespace state_machine {

MoveBase::MoveBase(std::shared_ptr<RosNode> ros_node)
    : State(ros_node, "MOVE_BASE") {}

StateType MoveBase::update() { return StateType::IDLE; }

} // namespace state_machine