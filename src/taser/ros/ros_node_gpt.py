import math
import random
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker

try:
    from taser_msgs.action import MoveArms, MoveBase
except ImportError:  # pragma: no cover - generated interfaces may be absent at build time
    MoveArms = None
    MoveBase = None

_repo_root = Path(__file__).resolve().parents[2]
_taser_cpp_build = _repo_root / "taser_cpp" / "build"
if _taser_cpp_build.exists() and str(_taser_cpp_build) not in sys.path:
    sys.path.append(str(_taser_cpp_build))

try:  # pragma: no cover - optional dependency
    import taser_cpp
    from taser_cpp import Pose2D, Robot, RobotConfig
    from taser_cpp.navigation import Dimensions
except ImportError:  # pragma: no cover - allow running without native bindings
    taser_cpp = None
    Pose2D = None
    Robot = None
    RobotConfig = None
    Dimensions = None


class RosNode(Node):
    """Python replica of the taser ROS2 node."""

    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.left_arm_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/left_arm_velocity_controller/commands", 10
        )
        self.right_arm_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/right_arm_velocity_controller/commands", 10
        )
        self.wheels_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/wheels_velocity_controller/commands", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.targets_pub = self.create_publisher(Marker, "/targets", qos)

        self.spawn_target_srv = self.create_service(
            Trigger, "/spawn_random_target", self._spawn_target_callback
        )

        self.arm_action_server = None
        if MoveArms is not None:
            self.arm_action_server = ActionServer(
                self,
                MoveArms,
                "/arm_controller/move_to",
                execute_callback=self.move_arms,
                goal_callback=self._always_accept_goal,
                cancel_callback=self._always_accept_cancel,
            )

        self.base_action_server = None
        if MoveBase is not None:
            self.base_action_server = ActionServer(
                self,
                MoveBase,
                "/base_controller/move_to",
                execute_callback=self.move_base,
                goal_callback=self._always_accept_goal,
                cancel_callback=self._always_accept_cancel,
            )

        self.robot: Optional[Robot] = None
        self.robot_dt = 0.0
        self.base_L = 0.0
        self.base_wheel_radius = 0.0

        self.joint_positions: List[float] = [0.0] * 10
        self.joint_velocities: List[float] = [0.0] * 10
        self.callback_time = -1.0

        self.targets: List[Marker] = []
        self.num_spawned_targets = 0

        self.arm_active_goal = None
        self.base_active_goal = None

        self._arm_bindings_available = False
        if taser_cpp is not None:
            self._arm_bindings_available = hasattr(taser_cpp, "Position3D") and hasattr(
                taser_cpp, "Transforms"
            )

        self._sim_pose = None

        self.get_logger().info("Node started.")

        self.create_robot_instance()
        self.spawn_obstacles()

    # ------------------------------------------------------------------
    # Robot helpers
    # ------------------------------------------------------------------
    def create_robot_instance(self) -> None:
        if Robot is None or RobotConfig is None or Dimensions is None:
            self.get_logger().warn("taser_cpp bindings unavailable; robot disabled.")
            return

        self._declare_parameter("dt")
        self._declare_parameter("initial_pose.x")
        self._declare_parameter("initial_pose.y")
        self._declare_parameter("initial_pose.theta")
        self._declare_parameter("arm.controller.kp")
        self._declare_parameter("base.kinematics.L")
        self._declare_parameter("base.kinematics.wheel_radius")
        self._declare_parameter("base.controller.mpc_horizon")
        self._declare_parameter("base.path_planner.velocity")
        self._declare_parameter("base.path_planner.rrt_num_samples")
        self._declare_parameter("base.path_planner.dimensions.x_min")
        self._declare_parameter("base.path_planner.dimensions.x_max")
        self._declare_parameter("base.path_planner.dimensions.y_min")
        self._declare_parameter("base.path_planner.dimensions.y_max")

        dt = self._get_param_double("dt")
        kp = self._get_param_double("arm.controller.kp")
        base_L = self._get_param_double("base.kinematics.L")
        wheel_radius = self._get_param_double("base.kinematics.wheel_radius")
        mpc_horizon = int(self._get_param_double("base.controller.mpc_horizon"))
        base_velocity = self._get_param_double("base.path_planner.velocity")
        num_samples = int(self._get_param_double("base.path_planner.rrt_num_samples"))
        dim = Dimensions(
            self._get_param_double("base.path_planner.dimensions.x_min"),
            self._get_param_double("base.path_planner.dimensions.x_max"),
            self._get_param_double("base.path_planner.dimensions.y_min"),
            self._get_param_double("base.path_planner.dimensions.y_max"),
        )

        config = RobotConfig(
            dt,
            kp,
            base_L,
            wheel_radius,
            mpc_horizon,
            base_velocity,
            num_samples,
            dim,
        )

        try:
            self.robot = Robot(config)
        except Exception as exc:  # pragma: no cover - safeguard around native call
            self.get_logger().error(f"Failed to create robot instance: {exc}")
            self.robot = None
            return

        self.robot_dt = dt
        self.base_L = base_L
        self.base_wheel_radius = wheel_radius

        initial_pose = (
            self._get_param_double("initial_pose.x"),
            self._get_param_double("initial_pose.y"),
            self._get_param_double("initial_pose.theta"),
        )

        if Pose2D is not None:
            self._sim_pose = Pose2D(*initial_pose)
        else:
            self._sim_pose = list(initial_pose)

        self.set_robot_pose_in_sim(initial_pose)

    # ------------------------------------------------------------------
    # Transform utilities
    # ------------------------------------------------------------------
    def get_transform(self, target_frame: str, source_frame: str):
        try:
            tf = self.buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.1)
            )
            return tf
        except TransformException:
            return None

    def get_arm_transforms(self, arm_name: str):
        return {
            "TBE": self.get_transform("base", f"{arm_name}_arm_eef"),
            "TB0": self.get_transform("base", f"{arm_name}_arm_1"),
            "TB1": self.get_transform("base", f"{arm_name}_arm_2"),
            "TB2": self.get_transform("base", f"{arm_name}_arm_3"),
        }

    # ------------------------------------------------------------------
    # Simulation helpers
    # ------------------------------------------------------------------
    def set_robot_pose_in_sim(self, pose) -> None:
        if pose is None:
            return

        if hasattr(pose, "x"):
            x = pose.x
            y = pose.y
            theta = pose.theta
        else:
            x, y, theta = pose

        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = "odom"
        tf_stamped.child_frame_id = "base_wrapper"
        tf_stamped.transform.translation.x = x
        tf_stamped.transform.translation.y = y
        tf_stamped.transform.rotation.w = math.cos(theta / 2.0)
        tf_stamped.transform.rotation.z = math.sin(theta / 2.0)

        self.tf_broadcaster.sendTransform(tf_stamped)

    def spawn_obstacles(self) -> None:
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        obstacles_pub = self.create_publisher(Marker, "/obstacles", qos)

        while rclpy.ok() and obstacles_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self._declare_parameter("environment.obstacle_count")
        obstacle_count = int(self._get_param_double("environment.obstacle_count", default=0.0))

        for i in range(obstacle_count):
            position_param = f"environment.obstacles.obstacle_{i}.position"
            size_param = f"environment.obstacles.obstacle_{i}.size"
            self._declare_parameter(position_param)
            self._declare_parameter(size_param)
            position = self._get_param_array(position_param)
            size = self._get_param_array(size_param)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "taser"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 0.8
            marker.lifetime = Duration(seconds=0.0).to_msg()

            obstacles_pub.publish(marker)

    def spawn_despawn_target(self, despawn_id: Optional[int] = None) -> None:
        x = random.random() * 3.0 + 3.0
        if self.num_spawned_targets % 2 == 1:
            x *= -1.0
        y = random.random() * 6.0 - 3.0
        z = random.random() * 1.0 + 2.2

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "taser"
        marker.id = despawn_id if despawn_id is not None else self.num_spawned_targets
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE if despawn_id is not None else Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.0).to_msg()

        if despawn_id is not None:
            self.targets = [item for item in self.targets if item.id != despawn_id]
        else:
            self.targets.append(marker)
            self.num_spawned_targets += 1

        self.targets_pub.publish(marker)

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------
    def move_arms(self, goal_handle):
        result = MoveArms.Result()

        if not self._arm_bindings_available or self.robot is None:
            self.get_logger().error("Arm control bindings unavailable; aborting arm goal.")
            goal_handle.abort()
            return result

        self.get_logger().error("Arm motion not implemented in Python replica; aborting goal.")
        goal_handle.abort()
        return result

    def move_base(self, goal_handle):
        result = MoveBase.Result()

        if self.robot is None:
            self.get_logger().error("Robot instance unavailable; aborting base goal.")
            goal_handle.abort()
            return result

        goal = goal_handle.request

        if self.base_active_goal is not None and self.base_active_goal.is_active:
            self.base_active_goal.abort()
        self.base_active_goal = goal_handle

        if goal.target_id != -1:
            target = next((item for item in self.targets if item.id == goal.target_id), None)
            if target is None:
                self.get_logger().error(f"Target {goal.target_id} not found.")
                goal_handle.abort()
                return result
            path_length = self.robot.plan_path(Pose2D(target.pose.position.x, target.pose.position.y))
        else:
            path_length = self.robot.plan_path(Pose2D(goal.x, goal.y))

        if path_length == 0:
            goal_handle.abort()
            return result

        period = self.robot_dt if self.robot_dt > 0.0 else 0.1
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            if not goal_handle.is_active:
                return result

            v_l, v_r, err = self.robot.move_base_step()

            if err < 1.7:
                if err == -1:
                    self.get_logger().info(
                        "Base movement aborted early at %.3f, %.3f",
                        getattr(self._sim_pose, "x", 0.0),
                        getattr(self._sim_pose, "y", 0.0),
                    )
                    goal_handle.abort()
                else:
                    self.get_logger().info(
                        "Reached goal at %.3f, %.3f",
                        getattr(self._sim_pose, "x", 0.0),
                        getattr(self._sim_pose, "y", 0.0),
                    )
                    goal_handle.succeed()

                zero_msg = Float64MultiArray()
                zero_msg.data = [0.0, 0.0]
                self.wheels_joint_velocity_pub.publish(zero_msg)
                break

            vel_msg = Float64MultiArray()
            vel_msg.data = [v_l, v_r]
            self.wheels_joint_velocity_pub.publish(vel_msg)

            self._integrate_base_motion(v_l, v_r)

            time.sleep(period)

        return result

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState) -> None:
        for index, value in enumerate(msg.position):
            if index < len(self.joint_positions):
                self.joint_positions[index] = value

        for index, value in enumerate(msg.velocity):
            if index < len(self.joint_velocities):
                self.joint_velocities[index] = value

        if self.callback_time < 0:
            self.callback_time = self.get_clock().now().nanoseconds / 1e9
            return

        if self._sim_pose is not None:
            self.set_robot_pose_in_sim(self._sim_pose)

        self.callback_time = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------
    # Internal utilities
    # ------------------------------------------------------------------
    @staticmethod
    def _always_accept_goal(_goal_request):
        return GoalResponse.ACCEPT

    @staticmethod
    def _always_accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _integrate_base_motion(self, v_l: float, v_r: float) -> None:
        if self._sim_pose is None or self.robot_dt <= 0.0:
            return

        radius = self.base_wheel_radius or 1.0
        base_width = self.base_L or 1.0

        v = radius * (v_r + v_l) * 0.5
        omega = radius * (v_r - v_l) / base_width

        if hasattr(self._sim_pose, "x"):
            self._sim_pose.x += v * math.cos(self._sim_pose.theta) * self.robot_dt
            self._sim_pose.y += v * math.sin(self._sim_pose.theta) * self.robot_dt
            self._sim_pose.theta += omega * self.robot_dt
        else:
            x, y, theta = self._sim_pose
            x += v * math.cos(theta) * self.robot_dt
            y += v * math.sin(theta) * self.robot_dt
            theta += omega * self.robot_dt
            self._sim_pose = [x, y, theta]

        self.set_robot_pose_in_sim(self._sim_pose)

    def _declare_parameter(self, name: str) -> None:
        if not self.has_parameter(name):
            self.declare_parameter(name)

    def _get_param_double(self, name: str, default: Optional[float] = None) -> float:
        param = self.get_parameter(name)
        if param.type_ == param.Type.NOT_SET:
            if default is None:
                raise RuntimeError(f"Required parameter '{name}' is not set")
            return default
        value = param.value
        if value is None:
            raise RuntimeError(f"Required parameter '{name}' is not set")
        return float(value)

    def _get_param_array(self, name: str) -> List[float]:
        param = self.get_parameter(name)
        value = param.value
        if value is None:
            raise RuntimeError(f"Required parameter '{name}' is not set")
        return list(value)

    def _spawn_target_callback(self, request, response):
        self.spawn_despawn_target()
        response.success = True
        response.message = "Target spawned"
        return response


def main(args=None):  # pragma: no cover - ROS entry point
    rclpy.init(args=args)
    node = RosNode("taser_ros_node_py")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - ROS entry point
    main()
