import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridRos
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

from taser.common.datatypes import Pose, TaserJointState, Workspace
from taser.common.logger import logger
from taser.locomotion import LocomotionPolicy
from taser.manipulation.pick_controller import PickController
from taser.navigation import OccupancyGrid
from taser.navigation.grid_navigator import GridNavigator


class TaserControllerRosInterface(Node):
    def __init__(self):
        super().__init__("controller", namespace="taser")

        self.joint_positions = TaserJointState()
        self.joint_velocities = TaserJointState()
        self.pose = Pose()
        self.quaternion_w = np.array([1.0, 0.0, 0.0, 0.0])
        self.base_linear_velocity_w = np.zeros(3)
        self.base_angular_velocity_w = np.zeros(3)
        self.vel_cmd = np.zeros(3)
        self.vel_cmd_multipliers = np.array([1.0, 1.0, 1.0])
        self.navigation_target_pose = None

        self._joint_state_sub = self.create_subscription(
            JointState, "/taser/sensors/joint_states", self._joint_state_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/taser/sensors/odometry", self._odometry_cb, 10
        )
        self._occupancy_grid_sub = self.create_subscription(
            OccupancyGridRos,
            "/taser/sensors/occupancy_grid",
            self._occupancy_grid_cb,
            10,
        )
        self._navigation_goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/taser/navigation/goal_pose",
            self._navigation_goal_pose_cb,
            10,
        )

        self.arm_joint_velocity_action_pub = self.create_publisher(
            JointState, "/taser/commands/arm_joint_velocity", 10
        )
        self.base_velocity_action_pub = self.create_publisher(
            Twist,
            "/taser/commands/base_velocity",
            10,
        )

        self.timer = self.create_timer(1 / 60, self.step)
        logger.info("Controller node running...")

    def step(self):
        return NotImplementedError("This should be implemented in the subclass.")

    def _joint_state_cb(self, msg: JointState):
        self.joint_positions = TaserJointState.construct_from("ros", msg.position)
        self.joint_velocities = TaserJointState.construct_from("ros", msg.velocity)

    def _odometry_cb(self, msg: Odometry):
        self.quaternion_w = np.array(
            [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        self.pose = Pose(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            rz=R.from_quat(self.quaternion_w, scalar_first=True).as_euler("zyx")[0],
        )
        self.base_linear_velocity_w = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        self.base_angular_velocity_w = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )

    def _occupancy_grid_cb(self, occupancy_grid: OccupancyGridRos):
        return NotImplementedError("This should be implemented in the subclass.")

    def _navigation_goal_pose_cb(self, pose: PoseStamped):
        target_quat = np.array(
            [
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            ]
        )
        self.navigation_target_pose = Pose(
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            rz=R.from_quat(target_quat, scalar_first=True).as_euler("zyx")[0],
        )


class TaserControllerRosNode(TaserControllerRosInterface):
    def __init__(self):
        super().__init__()

        self._pick_controller = PickController()
        self._locomotion_policy = LocomotionPolicy()

        workspace = Workspace(x_min=-5, x_max=5, y_min=-5, y_max=5)
        self._occupancy_grid = OccupancyGrid(workspace=workspace, cellsize=0.1)
        self._navigator = GridNavigator(
            workspace=workspace,
            occupancy_grid=self._occupancy_grid,
            v_max=self._locomotion_policy.v_max,
            w_max=self._locomotion_policy.w_max,
            wheel_base=0.6,
            goal_pos_tol=0.5,
        )

    def step(self):
        R_IB = R.from_quat(self.quaternion_w, scalar_first=True)
        R_BI: np.ndarray = R_IB.as_matrix().transpose()
        base_linear_velocity_b = np.matmul(R_BI, self.base_linear_velocity_w)

        vel_cmd = np.zeros(3)
        if self.navigation_target_pose is not None:
            base_vel_cmd, reached = self._navigator.step(
                self.pose, base_linear_velocity_b[0]
            )
            vel_cmd = np.array([base_vel_cmd.v, 0.0, base_vel_cmd.w])
            if reached:
                self.navigation_target_pose = None

        # if self._is_picking:
        #     target_pos_w = get_world_transform_matrix(
        #         self._target_prim
        #     ).ExtractTranslation()
        #     target_pos_b = np.matmul(R_BI, target_pos_w - position_w)
        #     self._pick_controller.set_target(
        #         Pose(
        #             x=target_pos_b[0],
        #             y=target_pos_b[1],
        #             z=target_pos_b[2],
        #         )
        #     )

        manipulation_action, _ = self._pick_controller.step(self.joint_positions)

        arm_joint_vel_action_msg = JointState()
        arm_joint_vel_action_msg.velocity = TaserJointState(
            left_arm=manipulation_action.left_arm,
            right_arm=manipulation_action.right_arm,
        ).to("ros")
        self.arm_joint_velocity_action_pub.publish(arm_joint_vel_action_msg)

        base_vel_action_msg = Twist()
        base_vel_action_msg.linear.x = vel_cmd[0]
        base_vel_action_msg.linear.y = vel_cmd[1]
        base_vel_action_msg.angular.z = vel_cmd[2]
        self.base_velocity_action_pub.publish(base_vel_action_msg)

    def _navigation_goal_pose_cb(self, pose: PoseStamped):
        super()._navigation_goal_pose_cb(pose)

        self._navigator.plan_path(
            start=self.pose,
            goal=self.navigation_target_pose,
            occupancy_grid=self._occupancy_grid,
        )

    def _navigation_occupancy_grid_cb(self, occupancy_grid: OccupancyGridRos):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        cellsize = occupancy_grid.info.resolution
        x_min = occupancy_grid.info.origin.position.x - (width * cellsize) / 2.0
        x_max = occupancy_grid.info.origin.position.x + (width * cellsize) / 2.0
        y_min = occupancy_grid.info.origin.position.y - (height * cellsize) / 2.0
        y_max = occupancy_grid.info.origin.position.y + (height * cellsize) / 2.0

        self.occupancy_grid = OccupancyGrid(
            workspace=Workspace(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max),
            cellsize=cellsize,
            grid=np.array(occupancy_grid.data).reshape((height, width)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaserControllerRosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
