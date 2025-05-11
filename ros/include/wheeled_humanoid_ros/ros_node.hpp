#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wheeled_humanoid_msgs/action/move_arms.hpp>
#include <wheeled_humanoid_msgs/action/move_base.hpp>

#include <wheeled_humanoid/robot.hpp>

namespace wheeled_humanoid_ros {

class RosNode : public rclcpp::Node {
  using MoveArmsAction = wheeled_humanoid_msgs::action::MoveArms;
  using MoveArmsGoalHandle = rclcpp_action::ServerGoalHandle<MoveArmsAction>;
  using MoveBaseAction = wheeled_humanoid_msgs::action::MoveBase;
  using MoveBaseGoalHandle = rclcpp_action::ServerGoalHandle<MoveBaseAction>;
  using TransformMsg = geometry_msgs::msg::Transform;

public:
  RosNode(std::string name);

  wheeled_humanoid::Transform get_transform(std::string target_frame,
                                            std::string source_frame) const;

  wheeled_humanoid::Transforms get_arm_transforms(std::string arm_name) const;

  void publish_transform(const TransformMsg &tf) const;

  void move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle);
  double move_arm_step(std::string name, wheeled_humanoid::Position3D p) const;

  void move_base(const std::shared_ptr<MoveBaseGoalHandle> goal_handle);
  double
  move_base_step(const std::shared_ptr<const MoveBaseAction::Goal> goal) const;

  void joint_state_callback(sensor_msgs::msg::JointState msg);

private:
  wheeled_humanoid::Robot robot_;
  double callback_time_ = -1;

  // Transform listener and broadcaster
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Publish to topics that control the joints
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_arm_joint_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_arm_joint_velocity_pub_;

  // Publish to topic that controls wheel velocities
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      wheels_joint_velocity_pub_;

  // Subscribe to joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  // Action Server to receive desired eef positions
  rclcpp_action::Server<MoveArmsAction>::SharedPtr arm_action_server_;
  std::shared_ptr<MoveArmsGoalHandle> arm_active_goal_;

  // Action Server to receive desired base position
  rclcpp_action::Server<MoveBaseAction>::SharedPtr base_action_server_;
  std::shared_ptr<MoveBaseGoalHandle> base_active_goal_;

  // Joint states
  wheeled_humanoid::RobotJointPositions joint_positions_{0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0};
  wheeled_humanoid::RobotJointVelocities joint_velocities_{0, 0, 0, 0, 0,
                                                           0, 0, 0, 0, 0};
};

} // namespace wheeled_humanoid_ros