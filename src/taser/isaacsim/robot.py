from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationActions
from omni.isaac.core.prims import XFormPrim

from taser.isaacsim.utils.occupancy_grid import OccupancyGrid
from taser.isaacsim.utils.ros2_tf_publisher import add_tf_publisher
from taser.manipulation import IKManipulator

NAME = "Taser"
PRIM_PATH = f"/World/{NAME}"


class TaserIsaacSimRobot(XFormPrim):
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
        self._position = np.array(position)
        self._orientation = np.array(orientation)

        asset_path = (
            Path(get_package_share_directory("taser_ros"))
            / "robot_description"
            / "usd"
            / "taser.usd"
        )
        self._prim = add_reference_to_stage(
            usd_path=str(asset_path.resolve()), prim_path=PRIM_PATH
        )

        super().__init__(
            name=NAME,
            prim_path=PRIM_PATH,
            position=position,
            orientation=orientation,
        )

        add_tf_publisher(
            robot_name=NAME,
            target_prim=f"{PRIM_PATH}/base_wrapper",
            tf_publisher_topic="/tf",
        )

        self._articulation = Articulation(prim_paths_expr=PRIM_PATH, name="robot")

        self._manipulator = IKManipulator()

    def step(self, dt: float, occupancy_grid: OccupancyGrid) -> None:
        action = ArticulationActions(
            joint_velocities=np.zeros(len(self._articulation.dof_names)),
            joint_names=self._articulation.dof_names,
        )
        self._articulation.apply_action(action)
