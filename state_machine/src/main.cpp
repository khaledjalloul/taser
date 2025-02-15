#include "state_machine/state_machine.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  state_machine::StateMachine sm;
  sm.start();

  rclcpp::shutdown();
  return 0;
}