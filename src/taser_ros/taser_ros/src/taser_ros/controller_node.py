import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridRos
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

from taser.common.datatypes import Pose, TaserJointState, Workspace
from taser.common.logger import logger
from taser.locomotion import LocomotionPolicy
from taser.manipulation.pick_controller import PickController
from taser.navigation import OccupancyGrid
from taser.navigation.grid_navigator import GridNavigator
from taser_ros.parameters import load_parameters


class TaserControllerRosInterface(Node):
    def __init__(self):
        super().__init__("controller", namespace="taser")

        self.params = load_parameters(self)

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

        self.navigation_path_pub = self.create_publisher(
            Path, "/taser/navigation/path", 10
        )
        self.workspace_pub = self.create_publisher(
            PolygonStamped, "/taser/navigation/workspace", 10
        )

        self.timer = self.create_timer(self.params.dt, self.step)
        logger.info("Controller node running...")

    def step(self):
        raise NotImplementedError("This should be implemented in the subclass.")

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
        raise NotImplementedError("This should be implemented in the subclass.")

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

        # Manipulation
        self._pick_controller = PickController()

        # Locomotion (only used for velocity limits)
        self._locomotion_policy = LocomotionPolicy()
        if self.params.navigation.v_max > self._locomotion_policy.v_max:
            logger.warning(
                f"Navigation v_max ({self.params.navigation.v_max}) is greater than locomotion v_max ({self._locomotion_policy.v_max}). Using locomotion v_max."
            )
        if self.params.navigation.w_max > self._locomotion_policy.w_max:
            logger.warning(
                f"Navigation w_max ({self.params.navigation.w_max}) is greater than locomotion w_max ({self._locomotion_policy.w_max}). Using locomotion w_max."
            )
        v_max = min(self.params.navigation.v_max, self._locomotion_policy.v_max)
        w_max = min(self.params.navigation.w_max, self._locomotion_policy.w_max)

        # Navigation
        workspace = self.params.navigation.workspace
        self._occupancy_grid = OccupancyGrid(workspace=workspace, cellsize=0.1)
        self._navigator = GridNavigator(
            workspace=workspace,
            occupancy_grid=self._occupancy_grid,
            v_max=v_max,
            w_max=w_max,
            wheel_base=self.params.navigation.wheel_base,
            goal_pos_tol=self.params.navigation.goal_pose_tolerance,
        )

        self.workspace_polygon = PolygonStamped()
        self.workspace_polygon.header.frame_id = self.params.world_frame
        self.workspace_polygon.header.stamp = self.get_clock().now().to_msg()
        self.workspace_polygon.polygon.points = [
            Point32(x=workspace.x_min, y=workspace.y_min, z=0.0),
            Point32(x=workspace.x_max, y=workspace.y_min, z=0.0),
            Point32(x=workspace.x_max, y=workspace.y_max, z=0.0),
            Point32(x=workspace.x_min, y=workspace.y_max, z=0.0),
        ]
        self.workspace_pub.publish(self.workspace_polygon)

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

        ros_path = Path()
        ros_path.header = pose.header
        for pt in self._navigator.path:
            ros_pose = PoseStamped()
            ros_pose.header = pose.header
            ros_pose.pose.position.x = pt.x
            ros_pose.pose.position.y = pt.y
            ros_pose.pose.orientation.w = np.cos(pt.rz / 2.0)
            ros_pose.pose.orientation.z = np.sin(pt.rz / 2.0)
            ros_path.poses.append(ros_pose)

        self.navigation_path_pub.publish(ros_path)

    def _occupancy_grid_cb(self, occupancy_grid: OccupancyGridRos):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        cellsize = occupancy_grid.info.resolution
        # Convert from ROS notation (bottom-left corner) to workspace notation (centered)
        x_min = occupancy_grid.info.origin.position.x
        x_max = occupancy_grid.info.origin.position.x + (width * cellsize)
        y_min = occupancy_grid.info.origin.position.y
        y_max = occupancy_grid.info.origin.position.y + (height * cellsize)

        self._occupancy_grid = OccupancyGrid(
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
