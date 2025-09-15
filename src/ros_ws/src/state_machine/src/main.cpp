#include <rclcpp/rclcpp.hpp>

#include "state_machine/state_machine.hpp"

using namespace state_machine;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosNode>("state_machine");
  StateMachine sm(node);

  rclcpp::Rate r(10);
  while (rclcpp::ok()) {
    sm.update();
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}