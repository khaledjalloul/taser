import numpy as np

from taser.navigation.custom_types import Pose, VelocityCommand
from taser.navigation.grid import OccupancyGrid, PathPlanner, PurePursuitController


class GridNavigator:
    def __init__(
        self,
        workspace: tuple[float, float, float, float],
        occupancy_grid: OccupancyGrid | np.ndarray,
        v_max: float,
        w_max: float,
        wheel_base: float,
    ):
        if isinstance(occupancy_grid, np.ndarray):
            self._occupancy_grid = OccupancyGrid(
                workspace=workspace,
                cellsize=0.1,
                grid=occupancy_grid,
            )
        else:
            self._occupancy_grid = occupancy_grid

        self._planner = PathPlanner(
            occupancy_grid=self._occupancy_grid,
            wheel_base=wheel_base,
        )

        self._controller = PurePursuitController(
            lookahead_base=0.25,
            lookahead_gain=0.6,
            v_max=v_max,
            w_max=w_max,
            curve_slowdown=1.0,
        )

    def plan_path(self, start: Pose, goal: Pose) -> list[Pose]:
        path = self._planner.plan(start, goal)
        self._controller.set_path(path, goal_yaw=goal.theta)

        return [Pose(p[0], p[1], 0.0) for p in path]

    def step(self, current_pose: Pose, v_current: float) -> VelocityCommand:
        cmd, reached, info = self._controller.step(current_pose, v_current)
        return cmd

    @property
    def inflated_occupancy_grid(self) -> OccupancyGrid:
        return self._planner.inflated_occupancy_grid
