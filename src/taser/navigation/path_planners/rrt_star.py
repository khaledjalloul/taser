from logging import getLogger

from taser_cpp.navigation import (
    Dimensions,
    PathPlanner,
)

from taser.common.datatypes import Pose2D, VelocityCommand, Workspace
from taser_cpp import Pose2D as Pose2DCpp

FILTER_THRESHOLD = 0.4


class RRTStarPathPlanner:
    """
    Path planner using RRT* algorithm.
    """

    def __init__(
        self,
        polygons: list[list[Pose2D]],
        num_rrt_samples: int,
        mpc_horizon: int,
        dt: float,
        wheel_base: float,
        v_max: float,
        workspace: Workspace,
    ):
        self._mpc_horizon = mpc_horizon

        self._planner = PathPlanner(
            num_rrt_samples, dt, wheel_base, v_max, Dimensions(*workspace.tuple())
        )

        self.set_polygons(polygons)

        self._logger = getLogger(__name__)

    def set_polygons(self, polygons: list[list[Pose2D]]):
        self._polygons = polygons
        polygons_cpp = [[Pose2DCpp(p.x, p.y) for p in poly] for poly in polygons]
        self._inflated_polygons = self._planner.set_obstacles(polygons_cpp)

    def plan(self, start: Pose2D, goal: Pose2D) -> list[Pose2D]:
        """
        Plan a path from start to goal.
        """
        dubins_path = self._planner.generate_path(
            Pose2DCpp(start.x, start.y, start.theta),
            Pose2DCpp(goal.x, goal.y, goal.theta),
        )

        if not dubins_path:
            self._logger.warning(f"No dubins path found from {start} to {goal}")
            return []

        path_cpp = self._planner.sample_path(dubins_path)
        path_cpp.extend(path_cpp[-1] for _ in range(2 * self._mpc_horizon))
        path = [Pose2D(p.x, p.y, p.theta) for p in path_cpp]

        path_vel_cpp = self._planner.get_velocity_profile(path_cpp)
        path_vel = [VelocityCommand(v=v.v, w=v.omega) for v in path_vel_cpp]

        return path, path_vel

    @property
    def inflated_polygons(self) -> list[list[Pose2D]]:
        return self._inflated_polygons
