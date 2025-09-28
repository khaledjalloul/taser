from pathlib import Path

import numpy as np
import omni.kit.commands
from ament_index_python.packages import get_package_share_directory
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions
from omni.isaac.core.prims import XFormPrim

from taser.isaacsim.utils.occupancy_grid import OccupancyGrid
from taser.isaacsim.utils.ros2_tf_publisher import add_tf_publisher
from taser.manipulation import IKManipulator


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

        self._import_urdf()

        super().__init__(
            name="taser",
            prim_path=self._prim_path,
            position=position,
            orientation=orientation,
        )

        add_tf_publisher(
            robot_name="taser",
            target_prim=f"{self._prim_path}/base_wrapper",
            tf_publisher_topic="/tf",
        )

        self._articulation = Articulation(prim_paths_expr=self._prim_path, name="taser")

        self._manipulator = IKManipulator()

    def step(self, dt: float, occupancy_grid: OccupancyGrid) -> None:
        action = ArticulationActions(
            joint_velocities=np.ones(len(self._articulation.dof_names)) * 10,
            joint_names=self._articulation.dof_names,
        )
        self._articulation.apply_action(action)

    def _import_urdf(self) -> None:
        urdf_path = str(
            Path(get_package_share_directory("taser_ros"))
            / "robot_description"
            / "urdf"
            / "taser.urdf"
        )

        import_config = _urdf.ImportConfig()
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = False
        import_config.self_collision = True
        import_config.distance_scale = 1
        import_config.density = 0.0

        # Parse the robot's URDF file to generate a robot model
        result, robot_model = omni.kit.commands.execute(
            "URDFParseFile", urdf_path=urdf_path, import_config=import_config
        )

        # Update the joint drive parameters for better stiffness and damping
        for joint in robot_model.joints:
            robot_model.joints[joint].drive.strength = 0.0
            robot_model.joints[joint].drive.damping = 0.5

        # Import the robot onto the current stage and retrieve its prim path
        result, self._prim_path = omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_robot=robot_model,
            import_config=import_config,
        )
