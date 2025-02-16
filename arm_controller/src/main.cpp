#include <rclcpp/rclcpp.hpp>

#include "arm_controller/arm_kinematics.hpp"
#include "arm_controller/controller.hpp"
#include "arm_controller/ros_node.hpp"
#include "arm_controller/types.hpp"

using namespace arm_controller;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  ArmKinematics left_arm("left_arm");
  ArmKinematics right_arm("right_arm");
  Controller controller(1);
  RosNode ros_node("arm_controller");

  rclcpp::Rate r(10);

  // TODO: Refactor
  while (rclcpp::ok()) {
    auto left_arm_tfs = ros_node.get_arm_transforms("left_arm");
    auto right_arm_tfs = ros_node.get_arm_transforms("right_arm");

    auto lpp = ros_node.left_arm_desired_pos;
    auto rpp = ros_node.right_arm_desired_pos;

    {
      // Get current and desired end effector poses
      auto p_cur = left_arm.get_end_effector_pose(left_arm_tfs.TBE);
      Pose p_desired = {Position(lpp.x, lpp.y, lpp.z), {0, 0, 0}};

      // Get control output and solve for joint velocities
      auto w_desired = controller.step(p_cur, p_desired);
      auto dq_desired =
          left_arm.solve_ik_for_joint_velocities(w_desired, left_arm_tfs);

      // Limit joint velocities
      dq_desired = dq_desired.cwiseMin(1.0).cwiseMax(-1.0);

      // Publish current end effector position
      geometry_msgs::msg::Point p_cur_msg;
      p_cur_msg.x = p_cur.position.x();
      p_cur_msg.y = p_cur.position.y();
      p_cur_msg.z = p_cur.position.z();
      ros_node.left_arm_current_pos_pub->publish(p_cur_msg);

      // Publish joint velocities
      std_msgs::msg::Float64MultiArray vel_msg;
      vel_msg.data.resize(dq_desired.size());
      Eigen::VectorXd::Map(&vel_msg.data[0], dq_desired.size()) = dq_desired;
      ros_node.left_arm_joint_velocity_pub->publish(vel_msg);
    }

    {
      // Get current and desired end effector poses
      auto p_cur = right_arm.get_end_effector_pose(right_arm_tfs.TBE);
      Pose p_desired = {Position(rpp.x, rpp.y, rpp.z), {0, 0, 0}};

      // Get control output and solve for joint velocities
      auto w_desired = controller.step(p_cur, p_desired);
      auto dq_desired =
          right_arm.solve_ik_for_joint_velocities(w_desired, right_arm_tfs);

      // Limit joint velocities
      dq_desired = dq_desired.cwiseMin(1.0).cwiseMax(-1.0);

      // Publish current end effector position
      geometry_msgs::msg::Point p_cur_msg;
      p_cur_msg.x = p_cur.position.x();
      p_cur_msg.y = p_cur.position.y();
      p_cur_msg.z = p_cur.position.z();
      ros_node.right_arm_current_pos_pub->publish(p_cur_msg);

      // Publish joint velocities
      std_msgs::msg::Float64MultiArray vel_msg;
      vel_msg.data.resize(dq_desired.size());
      Eigen::VectorXd::Map(&vel_msg.data[0], dq_desired.size()) = dq_desired;
      ros_node.right_arm_joint_velocity_pub->publish(vel_msg);
    }

    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}