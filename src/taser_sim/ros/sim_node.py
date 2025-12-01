from threading import Thread

import numpy as np
from geometry_msgs.msg import Pose2D as Pose2DRos
from nav_msgs.msg import OccupancyGrid as OccupancyGridRos
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32

from taser.navigation import OccupancyGrid


class TaserIsaacSimRosNode(Node):
    def __init__(
        self,
        navigation_target_pose_cb: callable,
        manipulation_task_cb: callable,
    ):
        super().__init__("sim", namespace="taser")
        self._navigation_target_pose = None
        self._left_arm_target_velocity = None
        self._right_arm_target_velocity = None

        self._navigation_target_pose_sub = self.create_subscription(
            Pose2DRos, "/taser/navigation/target_pose", navigation_target_pose_cb, 10
        )

        self._manipulation_task_sub = self.create_subscription(
            Int32,
            "/taser/manipulation/task",
            manipulation_task_cb,
            10,
        )

        # RViz markers for targets and obstacles
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._occupancy_grid_publisher = self.create_publisher(
            OccupancyGridRos, "/taser/navigation/occupancy_grid", qos
        )

        self._executor = SingleThreadedExecutor()

        def run_executor():
            self._executor.add_node(self)
            self._executor.spin()
            self._executor.remove_node(self)

        self._executor_thread = Thread(target=run_executor, daemon=True)
        self._executor_thread.start()

    def __del__(self):
        self._executor.shutdown()
        self._executor_thread.join()

    def publish_occupancy_grid(self, occupancy_grid: OccupancyGrid) -> None:
        workspace = occupancy_grid.workspace

        msg = OccupancyGridRos()
        msg.header.stamp = Time().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = occupancy_grid.cellsize
        msg.info.width = int(
            (workspace.x_max - workspace.x_min) / occupancy_grid.cellsize
        )
        msg.info.height = int(
            (workspace.y_max - workspace.y_min) / occupancy_grid.cellsize
        )
        msg.info.origin.position.x = (workspace.x_min + workspace.x_max) / 2.0
        msg.info.origin.position.y = (workspace.y_min + workspace.y_max) / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy_grid.grid.astype(np.int8).flatten().tolist()
        self._occupancy_grid_publisher.publish(msg)
