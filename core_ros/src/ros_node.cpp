#include "wheeled_humanoid_ros/ros_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace wheeled_humanoid_ros {

RosNode::RosNode(std::string name)
    : rclcpp::Node(name), buffer_(get_clock()), tf_listener_(buffer_) {

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  left_arm_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/left_arm_velocity_controller/commands", 10);

  right_arm_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/right_arm_velocity_controller/commands", 10);

  wheels_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/wheels_velocity_controller/commands", 10);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState msg) { joint_state_callback(msg); });

  arm_action_server_ = rclcpp_action::create_server<MoveArmsAction>(
      this, "/arm_controller/move_to",
      [this](const rclcpp_action::GoalUUID & /*uuid*/,
             std::shared_ptr<const MoveArmsAction::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<MoveArmsGoalHandle> /*goal_handle*/) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<MoveArmsGoalHandle> goal_handle) {
        auto execute_in_thread = [this, goal_handle]() {
          return move_arms(goal_handle);
        };
        std::thread{execute_in_thread}.detach();
      });

  base_action_server_ = rclcpp_action::create_server<MoveBaseAction>(
      this, "/base_controller/move_to",
      [this](const rclcpp_action::GoalUUID & /*uuid*/,
             std::shared_ptr<const MoveBaseAction::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<MoveBaseGoalHandle> /*goal_handle*/) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<MoveBaseGoalHandle> goal_handle) {
        auto execute_in_thread = [this, goal_handle]() {
          return move_base(goal_handle);
        };
        std::thread{execute_in_thread}.detach();
      });

  RCLCPP_INFO(get_logger(), "Node started.");

  spawn_obstacles();
}

wheeled_humanoid::Transform
RosNode::get_transform(std::string target_frame,
                       std::string source_frame) const {
  try {
    auto tf =
        buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return {wheeled_humanoid::Vector3(tf.transform.translation.x,
                                      tf.transform.translation.y,
                                      tf.transform.translation.z),
            wheeled_humanoid::Quaternion(
                tf.transform.rotation.w, tf.transform.rotation.x,
                tf.transform.rotation.y, tf.transform.rotation.z)};
  } catch (tf2::TransformException &ex) {
    return {wheeled_humanoid::Vector3(0, 0, 0),
            wheeled_humanoid::Quaternion(1, 0, 0, 0)};
  }
}

wheeled_humanoid::Transforms
RosNode::get_arm_transforms(std::string arm_name) const {
  wheeled_humanoid::Transforms tfs;
  tfs.TBE = get_transform("base", arm_name + "_eef");
  tfs.TB0 = get_transform("base", arm_name + "_1");
  tfs.TB1 = get_transform("base", arm_name + "_2");
  tfs.TB2 = get_transform("base", arm_name + "_3");
  return tfs;
}

void RosNode::publish_transform(const TransformMsg &tf) const {
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = now();
  tf_stamped.header.frame_id = "odom";
  tf_stamped.child_frame_id = "base_wrapper";
  tf_stamped.transform = tf;

  tf_broadcaster_->sendTransform(tf_stamped);
}

void RosNode::spawn_obstacles() {
  auto qos = rclcpp::QoS(10).transient_local();
  auto obstacles_pub =
      create_publisher<visualization_msgs::msg::Marker>("/obstacles", qos);

  while (rclcpp::ok() && obstacles_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::vector<wheeled_humanoid::Obstacle> obstacles = {
      {{6, -3}, {6, 3}, {8, 3}, {8, -3}}};
  robot_.rrt.set_obstacles(obstacles);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.ns = "wheeled_humanoid";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 7.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 2.5;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2.0;
  marker.scale.y = 6.0;
  marker.scale.z = 5.0;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 0.5f;
  marker.color.a = 0.8f;
  marker.lifetime = rclcpp::Duration::from_seconds(0.0); // 0 = forever

  obstacles_pub->publish(marker);
}

void RosNode::move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle) {
  // Cancel active goal if any is running
  if (arm_active_goal_ && arm_active_goal_->is_active()) {
    arm_active_goal_->abort(std::make_shared<MoveArmsAction::Result>());
  }
  arm_active_goal_ = goal_handle;

  rclcpp::Rate r(10);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveArmsAction::Result>();

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    } else if (!goal_handle->is_active()) {
      return;
    }

    auto l_p = wheeled_humanoid::Position3D{goal->left_arm.x, goal->left_arm.y,
                                            goal->left_arm.z};
    auto l_err = move_arm_step("left_arm", l_p);

    auto r_p = wheeled_humanoid::Position3D{
        goal->right_arm.x, goal->right_arm.y, goal->right_arm.z};
    auto r_err = move_arm_step("right_arm", r_p);

    if (l_err < 0.05 && r_err < 0.05) {
      // Stop arm movement by publishing zero velocities
      std_msgs::msg::Float64MultiArray zero_vel_msg;
      zero_vel_msg.data.resize(3);
      left_arm_joint_velocity_pub_->publish(zero_vel_msg);
      right_arm_joint_velocity_pub_->publish(zero_vel_msg);

      break;
    }

    r.sleep();
  }

  goal_handle->succeed(result);
}

double RosNode::move_arm_step(std::string name,
                              wheeled_humanoid::Position3D p) const {
  const auto &pub = name == "left_arm" ? left_arm_joint_velocity_pub_
                                       : right_arm_joint_velocity_pub_;

  // Get arm transforms
  auto tfs = get_arm_transforms(name);

  auto res = robot_.move_arm_step(name, p, tfs);
  auto dq_desired = std::get<0>(res);
  auto err = std::get<1>(res);

  // Publish joint velocities
  std_msgs::msg::Float64MultiArray vel_msg;
  vel_msg.data.resize(dq_desired.size());
  Eigen::VectorXd::Map(&vel_msg.data[0], dq_desired.size()) = dq_desired;
  pub->publish(vel_msg);

  return err;
}

void RosNode::move_base(const std::shared_ptr<MoveBaseGoalHandle> goal_handle) {
  // Cancel active goal if any is running
  if (base_active_goal_ && base_active_goal_->is_active()) {
    base_active_goal_->abort(std::make_shared<MoveBaseAction::Result>());
  }
  base_active_goal_ = goal_handle;

  rclcpp::Rate r(10);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveBaseAction::Result>();

  double x = ((double)std::rand() / RAND_MAX) * 3 + 10;
  double y = ((double)std::rand() / RAND_MAX) * 6 - 3;
  wheeled_humanoid::Pose2D goal_pose{x, y};
  RCLCPP_INFO(get_logger(), "Moving to %f, %f", goal_pose.x, goal_pose.y);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    } else if (!goal_handle->is_active()) {
      return;
    }

    auto res = robot_.move_base_step(goal_pose);
    auto v_l = std::get<0>(res);
    auto v_r = std::get<1>(res);
    auto err = std::get<2>(res);

    if (err < 0.1) {
      RCLCPP_INFO(get_logger(), "Reached goal at %f, %f", robot_.base.pose.x,
                  robot_.base.pose.y);
      // Stop base movement by publishing zero velocities
      std_msgs::msg::Float64MultiArray zero_vel_msg;
      zero_vel_msg.data.resize(2);
      wheels_joint_velocity_pub_->publish(zero_vel_msg);
      break;
    }

    // Publish joint velocities
    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data = {v_l, v_r};
    wheels_joint_velocity_pub_->publish(vel_msg);

    r.sleep();
  }

  goal_handle->succeed(result);
}

double RosNode::move_base_step(
    const std::shared_ptr<const MoveBaseAction::Goal> goal) const {
  auto err = 0;
  return err;
}

// TODO: optimize to store msg as is, then get later as eigen, maybe a template
// function
void RosNode::joint_state_callback(sensor_msgs::msg::JointState msg) {
  for (size_t i = 0; i < msg.position.size(); i++) {
    joint_positions_[i] = msg.position[i];
  }

  for (size_t i = 0; i < msg.velocity.size(); i++) {
    joint_velocities_[i] = msg.velocity[i];
  }

  // Move robot in simulation based on wheel velocities
  if (callback_time_ == -1) {
    callback_time_ = now().seconds();
    return;
  }

  auto dt = now().seconds() - callback_time_;
  // robot_.base.set_wheel_velocities(joint_velocities_[6],
  // joint_velocities_[8]); robot_.base.step(dt);

  TransformMsg tf;
  tf.translation.x = robot_.base.pose.x;
  tf.translation.y = robot_.base.pose.y;
  tf.rotation.w = cos(robot_.base.pose.theta / 2);
  tf.rotation.z = sin(robot_.base.pose.theta / 2);

  publish_transform(tf);
  callback_time_ = now().seconds();
}

} // namespace wheeled_humanoid_ros
