import math

import rclpy
from geometry_msgs.msg import Pose2D, TransformStamped, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker

from taser.common.datatypes import Pose, TaserJointState


class RosNode(Node):
    def __init__(
        self,
        navigation_target_pose_cb: callable,
        left_arm_velocity_cb: callable,
        right_arm_velocity_cb: callable,
    ):
        super().__init__("sim", namespace="taser")
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

        self._navigation_target_pose_sub = self.create_subscription(
            Pose2D, "/taser/navigation_target_pose", navigation_target_pose_cb, 10
        )

        self._left_arm_velocity_sub = self.create_subscription(
            Vector3, "/taser/arm_1_target_velocity", left_arm_velocity_cb, 10
        )
        self._right_arm_velocity_sub = self.create_subscription(
            Vector3, "/taser/arm_2_target_velocity", right_arm_velocity_cb, 10
        )

        # RViz markers for targets and obstacles
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._targets_pub = self.create_publisher(Marker, "/targets", qos)
        self._obstacles_pub = self.create_publisher(Marker, "/obstacles", qos)

    def set_robot_pose_in_sim(self, pose: Pose) -> None:
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = "map"
        tf_stamped.child_frame_id = "base_wrapper"
        tf_stamped.transform.translation.x = pose.x
        tf_stamped.transform.translation.y = pose.y
        tf_stamped.transform.rotation.w = math.cos(pose.theta / 2.0)
        tf_stamped.transform.rotation.z = math.sin(pose.theta / 2.0)
        self._tf_broadcaster.sendTransform(tf_stamped)

    def get_robot_pose_from_sim(self) -> Pose:
        tf = self._buffer.lookup_transform(
            target_frame="map",
            source_frame="base_wrapper",
            time=Time(),
        )

        return Pose(
            tf.transform.translation.x,
            tf.transform.translation.y,
            2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w),
        )

    def spawn_polygons_in_sim(self, polygons: list[dict]) -> None:
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        while rclpy.ok() and self._obstacles_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        for p_id, polygon in enumerate(polygons):
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

            self._obstacles_pub.publish(marker)

    def spawn_target_in_sim(self, pose: Pose) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "taser_targets"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.0).to_msg()  # 0 = forever

        self._targets_pub.publish(marker)

    def _joint_state_callback(self, msg: JointState):
        self._joint_state = msg

    @property
    def joint_positions(self) -> TaserJointState:
        pos = self._joint_state.position if self._joint_state.position else [0.0] * 8
        return TaserJointState.from_ros(pos)

    @property
    def joint_velocities(self) -> TaserJointState:
        vel = self._joint_state.velocity if self._joint_state.velocity else [0.0] * 8
        return TaserJointState.from_ros(vel)
