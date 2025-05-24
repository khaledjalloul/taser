#include "wheeled_humanoid_ros/ros_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

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

  auto qos = rclcpp::QoS(10).transient_local();
  targets_pub_ = create_publisher<Marker>("/targets", qos);

  spawn_target_srv_ = create_service<std_srvs::srv::Trigger>(
      "/spawn_random_target",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
             const std_srvs::srv::Trigger::Response::SharedPtr /*res*/) {
        spawn_despawn_target();
      });

  RCLCPP_INFO(get_logger(), "Node started.");

  create_robot_instance();
  spawn_obstacles();
}

void RosNode::create_robot_instance() {
  declare_parameter<double>("dt");
  declare_parameter<double>("initial_pose.x");
  declare_parameter<double>("initial_pose.y");
  declare_parameter<double>("initial_pose.theta");
  declare_parameter<double>("arm.controller.kp");
  declare_parameter<double>("base.kinematics.L");
  declare_parameter<double>("base.kinematics.wheel_radius");
  declare_parameter<int>("base.controller.mpc_horizon");
  declare_parameter<double>("base.path_planner.velocity");
  declare_parameter<int>("base.path_planner.rrt_num_samples");
  declare_parameter<double>("base.path_planner.dimensions.x_min");
  declare_parameter<double>("base.path_planner.dimensions.x_max");
  declare_parameter<double>("base.path_planner.dimensions.y_min");
  declare_parameter<double>("base.path_planner.dimensions.y_max");

  wheeled_humanoid::RobotConfig config{
      get_parameter("dt").as_double(),
      get_parameter("arm.controller.kp").as_double(),
      get_parameter("base.kinematics.L").as_double(),
      get_parameter("base.kinematics.wheel_radius").as_double(),
      get_parameter("base.controller.mpc_horizon").as_int(),
      get_parameter("base.path_planner.velocity").as_double(),
      get_parameter("base.path_planner.rrt_num_samples").as_int(),
      wheeled_humanoid::base::Dimensions{
          get_parameter("base.path_planner.dimensions.x_min").as_double(),
          get_parameter("base.path_planner.dimensions.x_max").as_double(),
          get_parameter("base.path_planner.dimensions.y_min").as_double(),
          get_parameter("base.path_planner.dimensions.y_max").as_double()}};

  robot_ = std::make_unique<wheeled_humanoid::Robot>(config);

  robot_->base->pose.x = get_parameter("initial_pose.x").as_double();
  robot_->base->pose.y = get_parameter("initial_pose.y").as_double();
  robot_->base->pose.theta = get_parameter("initial_pose.theta").as_double();

  set_robot_pose_in_sim(robot_->base->pose);
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
  tfs.TBE = get_transform("base", arm_name + "_arm_eef");
  tfs.TB0 = get_transform("base", arm_name + "_arm_1");
  tfs.TB1 = get_transform("base", arm_name + "_arm_2");
  tfs.TB2 = get_transform("base", arm_name + "_arm_3");
  return tfs;
}

void RosNode::set_robot_pose_in_sim(
    const wheeled_humanoid::Pose2D &pose) const {
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = now();
  tf_stamped.header.frame_id = "odom";
  tf_stamped.child_frame_id = "base_wrapper";
  tf_stamped.transform.translation.x = pose.x;
  tf_stamped.transform.translation.y = pose.y;
  tf_stamped.transform.rotation.w = cos(pose.theta / 2);
  tf_stamped.transform.rotation.z = sin(pose.theta / 2);

  tf_broadcaster_->sendTransform(tf_stamped);
}

void RosNode::spawn_obstacles() {
  auto qos = rclcpp::QoS(10).transient_local();
  auto obstacles_pub = create_publisher<Marker>("/obstacles", qos);

  while (rclcpp::ok() && obstacles_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  declare_parameter<int>("environment.obstacle_count");
  auto obstacle_count = get_parameter("environment.obstacle_count").as_int();

  std::vector<wheeled_humanoid::Obstacle> obstacles;

  for (int i = 0; i < obstacle_count; i++) {
    auto i_str = std::to_string(i);
    declare_parameter<std::vector<double>>("environment.obstacles.obstacle_" +
                                           i_str + ".position");
    declare_parameter<std::vector<double>>("environment.obstacles.obstacle_" +
                                           i_str + ".size");

    auto position =
        get_parameter("environment.obstacles.obstacle_" + i_str + ".position")
            .as_double_array();
    auto size =
        get_parameter("environment.obstacles.obstacle_" + i_str + ".size")
            .as_double_array();

    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.ns = "wheeled_humanoid";
    marker.id = i;
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size[0];
    marker.scale.y = size[1];
    marker.scale.z = size[2];
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0); // 0 = forever

    obstacles_pub->publish(marker);

    auto obstacle = wheeled_humanoid::base::get_box_corners(
        position[0], position[1], size[0], size[1]);

    obstacles.push_back(obstacle);
  }

  robot_->rrt->set_obstacles(obstacles);
}

void RosNode::spawn_despawn_target(std::optional<int> despawn_id) {
  double x = ((double)std::rand() / RAND_MAX) * 3 + 3;
  x *= num_spawned_targets_ % 2 == 0 ? 1 : -1;
  double y = ((double)std::rand() / RAND_MAX) * 6 - 3;
  double z = ((double)std::rand() / RAND_MAX) * 1 + 2.2;

  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.ns = "wheeled_humanoid";
  marker.id =
      despawn_id.has_value() ? despawn_id.value() : num_spawned_targets_++;
  marker.type = Marker::SPHERE;
  marker.action = despawn_id.has_value() ? Marker::DELETE : Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.lifetime = rclcpp::Duration::from_seconds(0); // 0 = forever

  if (despawn_id.has_value()) {
    targets_.erase(std::find_if(targets_.begin(), targets_.end(),
                                [despawn_id](const Marker &item) {
                                  return item.id == despawn_id.value();
                                }));
  } else {
    targets_.push_back(marker);
  }

  targets_pub_->publish(marker);
}

void RosNode::move_arms(const std::shared_ptr<MoveArmsGoalHandle> goal_handle) {
  // Cancel active goal if any is running
  if (arm_active_goal_ && arm_active_goal_->is_active()) {
    arm_active_goal_->abort(std::make_shared<MoveArmsAction::Result>());
  }
  arm_active_goal_ = goal_handle;

  rclcpp::Rate r(1 / robot_->dt);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveArmsAction::Result>();

  wheeled_humanoid::Position3D l_p;
  wheeled_humanoid::Position3D r_p;

  if (goal->target_id != -1) {
    int id = goal->target_id;
    auto target =
        std::find_if(targets_.begin(), targets_.end(),
                     [id](const Marker &item) { return item.id == id; });

    if (target != targets_.end()) {
      auto target_pose_map = wheeled_humanoid::Pose3D{
          {target->pose.position.x, target->pose.position.y,
           target->pose.position.z},
          {target->pose.orientation.x, target->pose.orientation.y,
           target->pose.orientation.z}};
      auto target_pose_base = robot_->arms.at("left").transform(
          target_pose_map, get_transform("base", "odom"));

      l_p = wheeled_humanoid::Position3D{target_pose_base.position.x,
                                         target_pose_base.position.y + 0.5,
                                         target_pose_base.position.z};
      r_p = wheeled_humanoid::Position3D{target_pose_base.position.x,
                                         target_pose_base.position.y - 0.5,
                                         target_pose_base.position.z};
    } else {
      RCLCPP_ERROR(get_logger(), "Target %d not found.", id);
      goal_handle->abort(result);
      return;
    }
  } else {
    l_p = wheeled_humanoid::Position3D{goal->left_arm.x, goal->left_arm.y,
                                       goal->left_arm.z};
    r_p = wheeled_humanoid::Position3D{goal->right_arm.x, goal->right_arm.y,
                                       goal->right_arm.z};
  }

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    } else if (!goal_handle->is_active()) {
      return;
    }

    auto l_err = move_arm_step("left", l_p);
    auto r_err = move_arm_step("right", r_p);

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

  // Despawn target if successfully grabbed
  if (goal->target_id != -1)
    spawn_despawn_target(goal->target_id);

  goal_handle->succeed(result);
}

double RosNode::move_arm_step(std::string name,
                              wheeled_humanoid::Position3D p) const {
  const auto &pub = name == "left" ? left_arm_joint_velocity_pub_
                                   : right_arm_joint_velocity_pub_;

  // Get arm transforms
  auto tfs = get_arm_transforms(name);

  auto res = robot_->move_arm_step(name, p, tfs);
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

  rclcpp::Rate r(1 / robot_->dt);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveBaseAction::Result>();

  int path_length = 0;

  if (goal->target_id != -1) {
    int id = goal->target_id;
    auto target =
        std::find_if(targets_.begin(), targets_.end(),
                     [id](const Marker &item) { return item.id == id; });
    if (target != targets_.end()) {
      path_length =
          robot_->plan_path({target->pose.position.x, target->pose.position.y});
    } else {
      RCLCPP_ERROR(get_logger(), "Target %d not found", id);
      goal_handle->abort(result);
      return;
    }
  } else {
    path_length = robot_->plan_path({goal->x, goal->y});
  }

  if (path_length == 0) {
    goal_handle->abort(result);
    return;
  }

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    } else if (!goal_handle->is_active()) {
      return;
    }

    auto res = robot_->move_base_step();
    auto v_l = std::get<0>(res);
    auto v_r = std::get<1>(res);
    auto err = std::get<2>(res);

    // Stay a bit away from the target to pick it up
    if (err < 1.7) {
      if (err == -1) {
        RCLCPP_INFO(get_logger(), "Base movement aborted early at %f, %f",
                    robot_->base->pose.x, robot_->base->pose.y);
        goal_handle->abort(result);
      } else {
        RCLCPP_INFO(get_logger(), "Reached goal at %f, %f",
                    robot_->base->pose.x, robot_->base->pose.y);
        goal_handle->succeed(result);
      }

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

  // Move the robot in simulation by publishing transform
  set_robot_pose_in_sim(robot_->base->pose);

  callback_time_ = now().seconds();
}

} // namespace wheeled_humanoid_ros
