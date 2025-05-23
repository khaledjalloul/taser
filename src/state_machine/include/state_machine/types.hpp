#pragma once

namespace state_machine {

enum StateType { IDLE, MOVE_BASE, REST_ARMS, GRAB, LIFT, WAVE };

enum Status { SUCCESS, FAILURE, RUNNING };

} // namespace state_machine
