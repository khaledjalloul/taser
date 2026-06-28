import numpy as np

from taser.common.datatypes import Pose, VelocityCommand, Workspace
from taser.common.logger import logger
from taser.navigation import (
    DistanceTransformPathPlanner,
    OccupancyGrid,
    PurePursuitController,
)


class GridNavigator:
    def __init__(
        self,
        workspace: Workspace,
        occupancy_grid: OccupancyGrid | np.ndarray,
        v_max: float,
        w_max: float,
        wheel_base: float,
        goal_pos_tol: float = 0.2,
    ):
        self._planner = DistanceTransformPathPlanner(
            occupancy_grid=occupancy_grid,
            wheel_base=wheel_base,
            workspace=workspace,
        )

        self._controller = PurePursuitController(
            v_max=v_max,
            w_max=w_max,
            goal_pos_tol=goal_pos_tol,
        )

    def plan_path(
        self,
        start: Pose,
        goal: Pose,
        occupancy_grid: OccupancyGrid | np.ndarray = None,
    ) -> list[Pose]:
        if occupancy_grid is not None:
            self._planner.set_occupancy_grid(occupancy_grid)

        try:
            path = self._planner.plan(start, goal)
            self._controller.set_path(path, goal_yaw=goal.rot.as_euler("zyx")[0])
        except Exception as e:
            logger.error(f"Path planning failed: {e}")
            return []

        return path

    def step(
        self, current_pose: Pose, v_current: float
    ) -> tuple[VelocityCommand, bool]:
        cmd, reached, info = self._controller.step(current_pose, v_current)
        return cmd, reached

    def reset(self):
        self._controller.reset()

    @property
    def inflated_occupancy_grid(self) -> OccupancyGrid:
        return self._planner.inflated_occupancy_grid

    @property
    def path(self) -> list[Pose]:
        return self._controller._path
