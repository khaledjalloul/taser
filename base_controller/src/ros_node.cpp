#include "base_controller/ros_node.hpp"

namespace base_controller {

RosNode::RosNode(std::string name)
    : rclcpp::Node(name), buffer_(get_clock()), tf_listener_(buffer_) {

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState msg) { joint_state_callback(msg); });

  wheels_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/wheels_velocity_controller/commands", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  action_server_ = rclcpp_action::create_server<MoveBaseAction>(
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

  rclcpp::Rate r(10);
  double L = 0, W = 0, wheel_radius = 0.5;
  while (rclcpp::ok() && !L && !W) {
    L = get_transform("wheel_2", "wheel_1").translation.y;
    W = get_transform("wheel_1", "wheel_3").translation.x;
    r.sleep();
  }
  base_ = std::make_unique<BaseKinematics>(L, W, wheel_radius);

  RCLCPP_INFO(get_logger(), "Node started.");
}

TransformMsg RosNode::get_transform(std::string target_frame,
                                    std::string source_frame) const {
  try {
    auto tf =
        buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return tf.transform;
  } catch (tf2::TransformException &ex) {
    return TransformMsg();
  }
}

void RosNode::publish_transform(const TransformMsg &tf) const {
  TransformStamped tf_stamped;
  tf_stamped.header.stamp = now();
  tf_stamped.header.frame_id = "odom";
  tf_stamped.child_frame_id = "base_wrapper";
  tf_stamped.transform = tf;

  tf_broadcaster_->sendTransform(tf_stamped);
}

void RosNode::move_base(const std::shared_ptr<MoveBaseGoalHandle> goal_handle) {
  // Cancel active goal if any is running
  if (active_goal_ && active_goal_->is_active()) {
    active_goal_->abort(std::make_shared<MoveBaseAction::Result>());
  }
  active_goal_ = goal_handle;

  rclcpp::Rate r(10);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveBaseAction::Result>();

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    } else if (!goal_handle->is_active()) {
      return;
    }

    auto err = move_base_step(goal);

    if (err < 0.05) {
      // Stop base movement by publishing zero velocities
      std_msgs::msg::Float64MultiArray zero_vel_msg;
      zero_vel_msg.data.resize(4);
      wheels_joint_velocity_pub_->publish(zero_vel_msg);
      break;
    }

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
  base_->set_wheel_state(
      {joint_velocities_[6], joint_velocities_[8], joint_positions_[7]});

  auto p_delta = base_->get_base_displacement(dt);

  TransformMsg tf;
  tf.translation.x = (base_->p.x += p_delta.x);
  tf.translation.y = (base_->p.y += p_delta.y);
  base_->p.theta += p_delta.theta;
  tf.rotation.w = cos(base_->p.theta / 2);
  tf.rotation.z = sin(base_->p.theta / 2);

  publish_transform(tf);
  callback_time_ = now().seconds();
}

} // namespace base_controller