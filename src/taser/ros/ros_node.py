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
from taser.navigation.custom_types import Pose
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker

from taser.locomotion.differential_drive import get_wheel_velocities
from taser.navigation import GridNavigator


class RosNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

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

        self.navigator = GridNavigator(occupancy_grid=None)

    def load_params():
        pass

    def set_robot_pose_in_sim(self, pose: Pose) -> None:
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = "odom"
        tf_stamped.child_frame_id = "base_wrapper"
        tf_stamped.transform.translation.x = pose.x
        tf_stamped.transform.translation.y = pose.y
        tf_stamped.transform.rotation.w = math.cos(pose.theta / 2.0)
        tf_stamped.transform.rotation.z = math.sin(pose.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf_stamped)

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

    def navigate_to(self, target: Pose):
        rate = self.create_rate(1 / self.robot.dt)

        path = self.navigator.plan_path(
            start=(self.robot.base.pose.x, self.robot.base.pose.y),
            goal=target,
        )

        while rclpy.ok():
            current_pose = ...
            v_current = ...
            vel_cmd, reached = self.navigator.step(
                current_pose=current_pose, v_current=v_current
            )

            if reached:
                break

            v_l, v_r = get_wheel_velocities(
                vel_cmd, self.robot.L, self.robot.wheel_radius
            )

            vel_msg = Float64MultiArray()
            vel_msg.data = [v_l, v_r]
            self.wheels_joint_velocity_pub.publish(vel_msg)

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = RosNode("taser_ros_node_py")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
