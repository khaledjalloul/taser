#include <rclcpp/rclcpp.hpp>

#include "state_machine/state_machine.hpp"

using namespace state_machine;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<RosNode>("state_machine");
  StateMachine sm(ros_node);

  rclcpp::Rate r(10);
  while (rclcpp::ok()) {
    sm.update();
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}