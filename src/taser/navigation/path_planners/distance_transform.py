import numpy as np
from roboticstoolbox import DistanceTransformPlanner

from taser.common.datatypes import Pose2D, Workspace
from taser.navigation import OccupancyGrid

FILTER_THRESHOLD = 0.4


class DistanceTransformPathPlanner:
    """
    Path planner using distance transform method from roboticstoolbox.
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid | np.ndarray,
        wheel_base: float,
        workspace: Workspace,
    ):
        self._wheel_base = wheel_base
        self._workspace = workspace

        self.set_occupancy_grid(occupancy_grid)

    def set_occupancy_grid(self, occupancy_grid: OccupancyGrid):
        if isinstance(occupancy_grid, np.ndarray):
            self._occupancy_grid = OccupancyGrid(
                workspace=self._workspace,
                cellsize=0.1,
                grid=occupancy_grid,
            )
        else:
            self._occupancy_grid = occupancy_grid

        self._planner = DistanceTransformPlanner(
            occgrid=occupancy_grid.grid,
            # Inflate by half the robot's length + a safety margin
            # Converted to grid cell coordinates
            inflate=((self._wheel_base / occupancy_grid.cellsize) / 2) * 1.5,
        )

    def plan(self, start: Pose2D, goal: Pose2D) -> list[Pose2D]:
        """
        Plan a path from start to goal.
        The resulting poses are downsampled and filtered to remove points that are too close to each other.
        """
        # Convert start and goal from world to grid coordinates
        start_g = self._occupancy_grid.w2g((start.x, start.y))
        goal_g = self._occupancy_grid.w2g((goal.x, goal.y))

        self._planner.plan(goal=goal_g)
        path_g = self._planner.query(start=(start_g[0], start_g[1]))

        # Convert path from grid to world coordinates
        path_w = [self._occupancy_grid.g2w((p[0], p[1])) for p in path_g]

        # Filter points that are very close to each other
        filtered_path = []
        for pt in path_w[::-1]:
            if (
                not filtered_path
                or np.linalg.norm(np.array(pt) - np.array(filtered_path[-1]))
                > FILTER_THRESHOLD
            ):
                filtered_path.append(pt)
        path_w = filtered_path[::-1]
        path_w = [Pose2D(x=p[0], y=p[1], theta=0.0) for p in path_w]

        return path_w

    @property
    def inflated_occupancy_grid(self) -> np.ndarray:
        return self._planner.occgrid.grid.astype(np.float16)
