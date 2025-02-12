#include <rclcpp/rclcpp.hpp>

#include "arm_controller/arm.hpp"
#include "arm_controller/ros_interface.hpp"
#include "arm_controller/types.hpp"

arm_controller::ArmJointState
get_arm_dq(std::shared_ptr<arm_controller::Arm> arm,
           std::shared_ptr<arm_controller::RosInterface> ros_interface,
           arm_controller::Pose pose) {
  auto B_rE = arm->get_end_effector_pose().position;
  auto w_desired = (pose.position - B_rE) * 0.5;

  auto dq = arm->solve_ik_for_joint_velocities(w_desired);

  return dq.cwiseMin(1.0).cwiseMax(-1.0);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto ros_interface =
      std::make_shared<arm_controller::RosInterface>("arm_controller_node");
  auto left_arm =
      std::make_shared<arm_controller::Arm>("arm_1", "base", ros_interface);
  auto right_arm =
      std::make_shared<arm_controller::Arm>("arm_2", "base", ros_interface);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    arm_controller::Pose desired_pose{};

    std_msgs::msg::Float64MultiArray msg;
    auto dq_current = ros_interface->get_joint_velocities();
    msg.data.resize(dq_current.size());
    Eigen::VectorXd::Map(&msg.data[0], dq_current.size()) = dq_current;

    auto arm1_dq =
        get_arm_dq(left_arm, ros_interface, {{1.5, 1, 0.5}, {0, 0, 0}});
    auto arm2_dq =
        get_arm_dq(right_arm, ros_interface, {{1.5, -1, 0}, {0, 0, 0}});

    msg.data[0] = arm1_dq[0];
    msg.data[1] = arm1_dq[1];
    msg.data[2] = arm1_dq[2];
    msg.data[3] = arm2_dq[0];
    msg.data[4] = arm2_dq[1];
    msg.data[5] = arm2_dq[2];

    ros_interface->publish_joint_velocities(msg);

    rate.sleep();
  }

  rclcpp::spin(ros_interface);

  rclcpp::shutdown();
  return 0;
}
