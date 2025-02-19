#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wheeled_humanoid_msgs/action/move_arms.hpp>

#include "arm_controller/arm_kinematics.hpp"
#include "arm_controller/controller.hpp"
#include "arm_controller/types.hpp"

namespace arm_controller {

class RosNode : public rclcpp::Node {
  using MoveArmsAction = wheeled_humanoid_msgs::action::MoveArms;
  using MoveArmsGoalHandle = rclcpp_action::ServerGoalHandle<MoveArmsAction>;

public:
  RosNode(std::string name);

  Transform get_transform(std::string target_frame,
                          std::string source_frame) const;

  Transforms get_arm_transforms(std::string arm_name) const;

  void move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle);
  double move_arm_step(std::string name, Position p) const;

private:
  ArmKinematics left_arm_;
  ArmKinematics right_arm_;
  Controller controller_;

  // Listen to robot transforms
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publish to topics that control the joints
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_arm_joint_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_arm_joint_velocity_pub_;

  // Action Server to receive desired positions
  rclcpp_action::Server<MoveArmsAction>::SharedPtr action_server_;
  std::shared_ptr<MoveArmsGoalHandle> active_goal_;
};

} // namespace arm_controller