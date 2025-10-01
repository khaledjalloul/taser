from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationActions

from taser.isaacsim.utils.occupancy_grid import OccupancyGrid
from taser.isaacsim.utils.ros2_tf_publisher import add_tf_publisher
from taser.isaacsim.utils.teleop import Teleop
from taser.locomotion import LocomotionPolicy
from taser.manipulation import IKManipulator

NAME = "taser"
PRIM_PATH = "/World/Taser"
USD_PATH = str(
    Path(get_package_share_directory("taser_ros"))
    / "robot_description"
    / "usd"
    / "taser.usd"
)


class TaserIsaacSimRobot(Articulation):
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
        add_reference_to_stage(usd_path=USD_PATH, prim_path=PRIM_PATH)

        super().__init__(
            name=NAME,
            prim_paths_expr=PRIM_PATH,
            translations=np.array(position).reshape(1, -1),
            orientations=np.array(orientation).reshape(1, -1),
        )

        add_tf_publisher(
            robot_name=NAME,
            target_prim=f"{PRIM_PATH}/base_link",
            tf_publisher_topic="/tf",
        )

        self._manipulator = IKManipulator()
        self._locomotion_policy = LocomotionPolicy()
        self._teleop = Teleop()

    def step(self, dt: float, occupancy_grid: OccupancyGrid) -> None:
        # base_link = f"{PRIM_PATH}/base_link"
        # base_link_idx = self._physics_view.link_paths[0].index(base_link)

        vel_cmd = self._teleop.get_command()

        wheel_velocities = self._locomotion_policy.step(
            joint_positions=self.get_joint_positions(),
            joint_velocities=self.get_joint_velocities(),
            base_position=self.get_world_poses()[0],
            base_quaternion=self.get_world_poses()[1],
            base_linear_velocity=self.get_linear_velocities(),
            base_angular_velocity=self.get_angular_velocities(),
            base_target_planar_velocity=vel_cmd,
        )

        action = ArticulationActions(
            joint_velocities=wheel_velocities,
            joint_names=["base_link_left_wheel_joint", "base_link_right_wheel_joint"],
        )
        self.apply_action(action)
