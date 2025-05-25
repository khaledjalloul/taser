#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <taser_msgs/action/move_arms.hpp>
#include <taser_msgs/action/move_base.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <taser/robot.hpp>

namespace taser_ros {

class RosNode : public rclcpp::Node {
  using Marker = visualization_msgs::msg::Marker;
  using MoveArmsAction = taser_msgs::action::MoveArms;
  using MoveArmsGoalHandle = rclcpp_action::ServerGoalHandle<MoveArmsAction>;
  using MoveBaseAction = taser_msgs::action::MoveBase;
  using MoveBaseGoalHandle = rclcpp_action::ServerGoalHandle<MoveBaseAction>;
  using TransformMsg = geometry_msgs::msg::Transform;

public:
  RosNode(std::string name);

  /**
   * Create an instance of the robot configured from ROS parameters
   */
  void create_robot_instance();

  /**
   * Get the ROS published transform between two frames
   * @param target_frame The frame to transform to (ex. T_AB -> A)
   * @param source_frame The frame to transform from (ex. T_AB -> B)
   */
  taser::Transform get_transform(std::string target_frame,
                                 std::string source_frame) const;

  /**
   * Get the transforms of a given arm
   * @param arm_name The name of the arm to get the transforms for
   */
  taser::Transforms get_arm_transforms(std::string arm_name) const;

  /**
   * Set the pose of the robot in the simulation
   * @param pose The pose to set the robot to
   */
  void set_robot_pose_in_sim(const taser::Pose2D &pose) const;

  /**
   * Load obstacles from ROS parameters and spawn them in the simulation
   */
  void spawn_obstacles();

  /**
   * Spawn a target in the simulation, or despawn a target if a despawn id is
   * provided
   * @param despawn_id (Optional) The id of the target to despawn
   */
  void spawn_despawn_target(std::optional<int> despawn_id = std::nullopt);

  /**
   * Move the arms of the robot to the position received from the action client
   * @param goal_handle The goal handle for the move arms action
   */
  void move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle);

  /**
   * Move the the arms one time step, used internally in `move_arms`
   * @param name The name of the arm to move
   * @param p The position to move the arm to
   */
  double move_arm_step(std::string name, taser::Position3D p) const;

  /**
   * Move the base of the robot to the pose received from the action client
   * @param goal_handle The goal handle for the move base action
   */
  void move_base(const std::shared_ptr<MoveBaseGoalHandle> goal_handle);

  /**
   * Callback for the joint states topic
   * @param msg The joint states message
   */
  void joint_state_callback(sensor_msgs::msg::JointState msg);

private:
  std::unique_ptr<taser::Robot> robot_;
  double callback_time_ = -1;
  std::vector<Marker> targets_;
  int num_spawned_targets_ = 0;

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

  // Publish targets to visualize and trigger missions
  rclcpp::Publisher<Marker>::SharedPtr targets_pub_;

  // Service to spawn a random target
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_target_srv_;

  // Joint states
  taser::RobotJointPositions joint_positions_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  taser::RobotJointVelocities joint_velocities_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

} // namespace taser_ros