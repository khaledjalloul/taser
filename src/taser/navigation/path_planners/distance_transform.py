import numpy as np
from roboticstoolbox import DistanceTransformPlanner

from taser.common.datatypes import Pose
from taser.navigation import OccupancyGrid

FILTER_THRESHOLD = 0.4


class DistanceTransformPathPlanner:
    """
    Path planner using distance transform method from roboticstoolbox.
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        wheel_base: float = 0.0,
    ):
        self.occupancy_grid = occupancy_grid

        self._planner = DistanceTransformPlanner(
            occgrid=occupancy_grid.grid,
            # Inflate by half the robot's length + a safety margin
            # Converted to grid cell coordinates
            inflate=((wheel_base / occupancy_grid.cellsize) / 2) * 1.5,
        )

    def plan(self, start: Pose, goal: Pose) -> list[Pose]:
        """
        Plan a path from start to goal.
        The resulting poses are downsampled and filtered to remove points that are too close to each other.
        """
        # Convert start and goal from world to grid coordinates
        start_g = self.occupancy_grid.w2g((start.x, start.y))
        goal_g = self.occupancy_grid.w2g((goal.x, goal.y))

        self._planner.plan(goal=goal_g)
        path_g = self._planner.query(start=(start_g[0], start_g[1]))

        # Convert path from grid to world coordinates
        path_w = [self.occupancy_grid.g2w((p[0], p[1])) for p in path_g]

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
        path_w = [Pose(x=p[0], y=p[1], theta=0.0) for p in path_w]

        return path_w

    @property
    def inflated_occupancy_grid(self) -> np.ndarray:
        return self._planner.occgrid.grid.astype(np.float16)
