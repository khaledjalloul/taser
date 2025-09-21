from taser_cpp.navigation import (
    Controller,
    Dimensions,
    PathPlanner,
)

from taser.common.datatypes import Pose, VelocityCommand
from taser_cpp import Pose2D


class PolygonNavigator:
    def __init__(
        self,
        workspace: tuple[float, float, float, float],
        polygons: list[list[Pose]],
        v_max: float,
        w_max: float,
        wheel_base: float,
        dt: float,
        num_rrt_samples: int,
        mpc_horizon: int,
    ):
        self._num_rrt_samples = num_rrt_samples
        self._mpc_horizon = mpc_horizon

        self._planner = PathPlanner(
            num_rrt_samples, dt, wheel_base, v_max, Dimensions(*workspace)
        )

        self._set_polygons(polygons)

        self._controller = Controller(dt, mpc_horizon, v_max, w_max)

        self._path = None
        self._step = 0

    def plan_path(
        self,
        start: Pose,
        goal: Pose,
        polygons: list[list[Pose]] = None,
    ) -> list[Pose]:
        if polygons is not None:
            self._set_polygons(polygons)

        dubins_path = self._planner.generate_path(
            Pose2D(start.x, start.y, start.theta),
            Pose2D(goal.x, goal.y, goal.theta),
        )

        self._path = self._planner.sample_path(dubins_path)
        self._path.extend(self._path[-1] for _ in range(2 * self._mpc_horizon))
        self._path_vel = self._planner.get_velocity_profile(self._path)

        self._step = 0

        return [Pose(p.x, p.y, p.theta) for p in self._path]

    def step(self, current_pose: Pose, v_current: float = None) -> VelocityCommand:
        if self._path is None:
            raise ValueError("Path not set. Call plan_path() first.")

        if self._step + self._mpc_horizon >= len(self._path):
            return VelocityCommand(0.0, 0.0)

        local_path = self._path[self._step : self._step + self._mpc_horizon]
        local_path_vel = self._path_vel[self._step : self._step + self._mpc_horizon]

        cmd = self._controller.step(
            Pose2D(current_pose.x, current_pose.y, current_pose.theta),
            local_path,
            local_path_vel,
        )

        self._step += 1
        return VelocityCommand(cmd.v, cmd.omega)

    @property
    def inflated_polygons(self) -> list[list[Pose]]:
        return self._inflated_polygons

    def _set_polygons(self, polygons: list[list[Pose]]):
        polygons_cpp = [[Pose2D(p.x, p.y) for p in poly] for poly in polygons]
        self._inflated_polygons = self._planner.set_obstacles(polygons_cpp)
