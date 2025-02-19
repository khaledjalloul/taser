#include "arm_controller/ros_node.hpp"

namespace arm_controller {

RosNode::RosNode(std::string name)
    : rclcpp::Node(name), left_arm_("left_arm"), right_arm_("right_arm"),
      controller_(1), buffer_(get_clock()), tf_listener_(buffer_) {

  left_arm_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/left_arm_velocity_controller/commands", 10);

  right_arm_joint_velocity_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
          "/right_arm_velocity_controller/commands", 10);

  action_server_ = rclcpp_action::create_server<MoveArmsAction>(
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

  RCLCPP_INFO(get_logger(), "Node started.");
}

Transform RosNode::get_transform(std::string target_frame,
                                 std::string source_frame) {
  try {
    auto tf =
        buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return {Vector3(tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.translation.z),
            Quaternion(tf.transform.rotation.w, tf.transform.rotation.x,
                       tf.transform.rotation.y, tf.transform.rotation.z)};
  } catch (tf2::TransformException &ex) {
    return {Vector3(0, 0, 0), Quaternion(1, 0, 0, 0)};
  }
}

Transforms RosNode::get_arm_transforms(std::string arm_name) {
  Transforms tfs;
  tfs.TBE = get_transform("base", arm_name + "_eef");
  tfs.TB0 = get_transform("base", arm_name + "_1");
  tfs.TB1 = get_transform("base", arm_name + "_2");
  tfs.TB2 = get_transform("base", arm_name + "_3");
  return tfs;
}

void RosNode::move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle) {
  // Cancel active goal if any is running
  if (active_goal_ && active_goal_->is_active()) {
    active_goal_->abort(std::make_shared<MoveArmsAction::Result>());
  }
  active_goal_ = goal_handle;

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

    auto l_p = Position{goal->left_arm.x, goal->left_arm.y, goal->left_arm.z};
    auto l_err = move_arm_step("left_arm", l_p);

    auto r_p =
        Position{goal->right_arm.x, goal->right_arm.y, goal->right_arm.z};
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

double RosNode::move_arm_step(std::string name, Position p) {
  auto &arm = name == "left_arm" ? left_arm_ : right_arm_;
  auto &pub = name == "left_arm" ? left_arm_joint_velocity_pub_
                                 : right_arm_joint_velocity_pub_;

  // Get arm transforms
  auto tfs = get_arm_transforms(name);

  // Get current and desired end effector poses
  auto p_cur = arm.get_end_effector_pose(tfs.TBE);
  Pose p_desired = {p, {0, 0, 0}};

  // Get control output and solve for joint velocities
  auto w_desired = controller_.step(p_cur, p_desired);
  auto dq_desired = arm.solve_ik_for_joint_velocities(w_desired, tfs);

  // Limit joint velocities
  dq_desired = dq_desired.cwiseMin(1.0).cwiseMax(-1.0);

  // Publish joint velocities
  std_msgs::msg::Float64MultiArray vel_msg;
  vel_msg.data.resize(dq_desired.size());
  Eigen::VectorXd::Map(&vel_msg.data[0], dq_desired.size()) = dq_desired;
  pub->publish(vel_msg);

  // Calculate error between current and desired end effector positions
  auto err = pow(p_cur.position.x - p.x, 2) + pow(p_cur.position.y - p.y, 2) +
             pow(p_cur.position.z - p.z, 2);
  err = sqrt(err);

  return err;
}

} // namespace arm_controller
