import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker

from taser.common.datatypes import Pose
from taser_ros.parameters import TaserSimParameters


class RosNode(Node):
    def __init__(self):
        super().__init__("taser")
        self._joint_state = JointState()

        self._buffer = Buffer()
        self._tf_listener = TransformListener(self._buffer, self, spin_thread=True)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._left_arm_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/left_arm_velocity_controller/commands", 10
        )
        self._right_arm_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/right_arm_velocity_controller/commands", 10
        )
        self._wheels_joint_velocity_pub = self.create_publisher(
            Float64MultiArray, "/wheels_velocity_controller/commands", 10
        )

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._targets_pub = self.create_publisher(Marker, "/targets", qos)

        self._spawn_target_srv = self.create_service(
            Trigger, "/spawn_random_target", self._spawn_target_callback
        )

    def load_params(self) -> TaserSimParameters:
        params = TaserSimParameters(
            general=TaserSimParameters.General(
                dt=self._load_param("dt", 0.0).double_value,
                initial_pose=Pose(
                    self._load_param("initial_pose.x", 0.0).double_value,
                    self._load_param("initial_pose.y", 0.0).double_value,
                    self._load_param("initial_pose.theta", 0.0).double_value,
                ),
            ),
            locomotion=TaserSimParameters.Locomotion(
                wheel_base=self._load_param(
                    "locomotion.kinematics.wheel_base", 0.0
                ).double_value,
                wheel_radius=self._load_param(
                    "locomotion.kinematics.wheel_radius", 0.0
                ).double_value,
            ),
            manipulation=TaserSimParameters.Manipulation(
                kp=self._load_param("manipulation.controller.kp", 0.0).double_value
            ),
            navigation=TaserSimParameters.Navigation(
                workspace=(
                    self._load_param(
                        "navigation.path_planner.workspace.x_min", 0.0
                    ).double_value,
                    self._load_param(
                        "navigation.path_planner.workspace.x_max", 0.0
                    ).double_value,
                    self._load_param(
                        "navigation.path_planner.workspace.y_min", 0.0
                    ).double_value,
                    self._load_param(
                        "navigation.path_planner.workspace.y_max", 0.0
                    ).double_value,
                ),
                v_max=self._load_param(
                    "navigation.path_planner.v_max", 0.0
                ).double_value,
                w_max=self._load_param(
                    "navigation.path_planner.w_max", 0.0
                ).double_value,
                num_rrt_samples=self._load_param(
                    "navigation.path_planner.num_rrt_samples", 0
                ).integer_value,
                mpc_horizon=self._load_param(
                    "navigation.path_planner.mpc_horizon", 0
                ).integer_value,
                polygons_sim=[],
                polygons=[],
            ),
        )

        polygon_count = self._load_param(
            "navigation.path_planner.polygons._polygon_count", 0
        ).integer_value

        for i in range(polygon_count):
            position = self._load_param(
                f"navigation.path_planner.polygons.polygon_{i}.position", [0.0, 0.0]
            ).double_array_value
            size = self._load_param(
                f"navigation.path_planner.polygons.polygon_{i}.size", [0.0, 0.0]
            ).double_array_value

            front_left = position[0] + size[0] / 2, position[1] - size[1] / 2
            front_right = position[0] + size[0] / 2, position[1] + size[1] / 2
            back_left = position[0] - size[0] / 2, position[1] - size[1] / 2
            back_right = position[0] - size[0] / 2, position[1] + size[1] / 2
            polygon_corners = (back_left, back_right, front_right, front_left)
            polygon = [Pose(*p) for p in polygon_corners]

            params.navigation.polygons_sim.append({"position": position, "size": size})
            params.navigation.polygons.append(polygon)

        return params

    def set_robot_pose_in_sim(self, pose: Pose) -> None:
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = "odom"
        tf_stamped.child_frame_id = "base_wrapper"
        tf_stamped.transform.translation.x = pose.x
        tf_stamped.transform.translation.y = pose.y
        tf_stamped.transform.rotation.w = math.cos(pose.theta / 2.0)
        tf_stamped.transform.rotation.z = math.sin(pose.theta / 2.0)
        self._tf_broadcaster.sendTransform(tf_stamped)

    def spawn_polygons_in_sim(self, polygons: list[dict]) -> None:
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        obstacles_pub = self.create_publisher(Marker, "/obstacles", qos)

        while rclpy.ok() and obstacles_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        for p_id, polygon in enumerate(polygons):
            print("Spawning polygon:", polygon)
            position = polygon["position"]
            size = polygon["size"]

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "taser_polygons"
            marker.id = p_id
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
            marker.lifetime = Duration(seconds=0.0).to_msg()  # 0 = forever

            obstacles_pub.publish(marker)

    def _load_param(self, param_name: str, default_value) -> ParameterValue:
        self.declare_parameter(param_name, default_value)
        return self.get_parameter(param_name).get_parameter_value()

    def _joint_state_callback(self, msg: JointState):
        self._joint_state = msg

    def _spawn_target_callback(self, req: Trigger.Request, res: Trigger.Response):
        x = random.random() * 3.0 + 3.0
        if self.num_spawned_targets % 2 == 1:
            x *= -1.0
        y = random.random() * 6.0 - 3.0
        z = random.random() * 1.0 + 2.2

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "taser_targets"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.0).to_msg()  # 0 = forever

        self._targets_pub.publish(marker)
