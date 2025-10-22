import numpy as np
from geometry_msgs.msg import Pose2D as Pose2DRos
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles, quat_to_rot_matrix
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from omni.usd import get_context, get_world_transform_matrix
from std_msgs.msg import Int32

from taser.common.datatypes import Pose, TaserJointState, Workspace
from taser.common.model import USD_PATH
from taser.isaacsim.utils.occupancy_grid import OccupancyGrid
from taser.isaacsim.utils.ros2_tf_publisher import add_tf_publisher
from taser.isaacsim.utils.teleop import Teleop
from taser.locomotion import LocomotionPolicy
from taser.manipulation import PickController
from taser.navigation import GridNavigator
from taser.ros.isaac.sim_node import TaserIsaacSimRosNode

NAME = "taser"
PRIM_PATH = "/World/Taser"


class TaserIsaacSimRobot(SingleArticulation):
    def __init__(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float],
    ) -> None:
        """
        Initialize the robot.

        Args:
            position (tuple[float, float, float]): Initial position of the robot in meters.
            orientation (tuple[float, float, float]): Initial orientation of the robot as a quaternion [w, x, y, z].
        """
        add_reference_to_stage(usd_path=str(USD_PATH), prim_path=PRIM_PATH)

        super().__init__(
            name=NAME,
            prim_path=PRIM_PATH,
            position=position,
            orientation=orientation,
        )

        add_tf_publisher(
            robot_name=NAME,
            target_prim=f"{PRIM_PATH}/base_link",
            tf_publisher_topic="/tf",
        )

        self._pick_controller = PickController()
        self._locomotion_policy = LocomotionPolicy()
        self._teleop = Teleop(
            v_max=self._locomotion_policy.v_max,
            w_max=self._locomotion_policy.w_max,
        )

        self._path_plan: list[Pose] = []

        self._ros_node = TaserIsaacSimRosNode(
            navigation_target_pose_cb=self._navigation_target_pose_cb,
            manipulation_task_cb=self._manipulation_task_cb,
        )

        stage = get_context().get_stage()
        self._target_prim = stage.GetPrimAtPath("/World/target")
        self._is_picking = True

    def initialize(self, workspace: Workspace) -> None:
        super().initialize()
        self._occupancy_grid = OccupancyGrid(workspace=workspace, cellsize=0.1)
        self._navigator = GridNavigator(
            workspace=workspace,
            occupancy_grid=self._occupancy_grid,
            v_max=self._locomotion_policy.v_max,
            w_max=self._locomotion_policy.w_max,
            wheel_base=0.6,
            goal_pos_tol=0.5,
        )

    def step(self, dt: float, occupancy_grid: OccupancyGrid) -> None:
        self._occupancy_grid = occupancy_grid
        self._hide_robot_from_occupancy_grid()

        q = TaserJointState.from_isaac(self.get_joint_positions())
        dq = TaserJointState.from_isaac(self.get_joint_velocities())

        position_w, quaternion_w = self.get_world_pose()
        R_IB = quat_to_rot_matrix(quaternion_w)
        R_BI = R_IB.transpose()
        root_linear_velocity_b = np.matmul(R_BI, self.get_linear_velocity())
        root_angular_velocity_b = np.matmul(R_BI, self.get_angular_velocity())

        base_pose = Pose(
            x=position_w[0],
            y=position_w[1],
            rz=quat_to_euler_angles(quaternion_w)[2],
        )

        if np.any(self._teleop.get_command() != 0):
            vel_cmd = self._teleop.get_command()
        elif self._path_plan:
            base_vel_cmd, reached = self._navigator.step(
                base_pose, root_linear_velocity_b[0]
            )
            vel_cmd = np.array([base_vel_cmd.v, 0.0, base_vel_cmd.w])
            if reached:
                self._path_plan = []
        else:
            vel_cmd = np.array([0.0, 0.0, 0.0])

        if self._is_picking:
            target_pos_w = get_world_transform_matrix(
                self._target_prim
            ).ExtractTranslation()
            target_pos_b = np.matmul(R_BI, target_pos_w - position_w)
            self._pick_controller.set_target(
                Pose(
                    x=target_pos_b[0],
                    y=target_pos_b[1],
                    z=target_pos_b[2],
                )
            )

        joint_velocities, _ = self._pick_controller.step(q)

        joint_velocities.wheels = self._locomotion_policy.step(
            joint_positions=q,
            joint_velocities=dq,
            base_quaternion_w=quaternion_w,
            base_linear_velocity_b=root_linear_velocity_b,
            base_angular_velocity_b=root_angular_velocity_b,
            base_target_planar_velocity_b=vel_cmd,
        )

        action = ArticulationAction(joint_velocities=joint_velocities.ordered_isaac)
        self.apply_action(action)

    def _hide_robot_from_occupancy_grid(self) -> None:
        """Hide the robot from the occupancy grid to avoid self-collisions."""
        position_w, _ = self.get_world_pose()
        x_min = np.floor(position_w[0] - 0.3)
        x_max = np.ceil(position_w[0] + 0.3)
        y_min = np.floor(position_w[1] - 0.3)
        y_max = np.ceil(position_w[1] + 0.3)

        self._occupancy_grid.set((x_min, x_max, y_min, y_max), 0)

        self._ros_node.publish_occupancy_grid(self._occupancy_grid)

    def _navigation_target_pose_cb(self, target: Pose2DRos):
        position_w, quaternion_w = self.get_world_pose()
        theta = quat_to_euler_angles(quaternion_w)[2]

        current_pose = Pose(x=position_w[0], y=position_w[1], rz=theta)
        target_pose = Pose(x=target.x, y=target.y, rz=theta)

        self._path_plan = self._navigator.plan_path(
            start=current_pose,
            goal=target_pose,
            occupancy_grid=self._occupancy_grid,
        )

    def _manipulation_task_cb(self, task: Int32):
        self._is_picking = bool(task.data)

        if not self._is_picking:
            self._pick_controller.reset()
