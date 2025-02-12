#include <rclcpp/rclcpp.hpp>

#include "arm_controller/arm.hpp"
#include "arm_controller/ros_interface.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto ros_interface =
      std::make_shared<arm_controller::RosInterface>("arm_controller_node");
  auto left_arm_ =
      std::make_shared<arm_controller::Arm>("arm_1", "base", ros_interface);

  while (rclcpp::ok()) {
    auto p = left_arm_->get_end_effector_pose();
    std::stringstream ss;
    ss << "Pose:\n" << p.position << " " << p.orientation;
    RCLCPP_INFO(ros_interface->get_logger(), ss.str().c_str());
  }

  rclcpp::spin(ros_interface);

  rclcpp::shutdown();
  return 0;
}
