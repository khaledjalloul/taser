from taser_cpp import Pose2D
from taser_cpp.navigation import (
    Controller,
    Dimensions,
    PathPlanner,
)

from taser.navigation.custom_types import Pose, VelocityCommand

RRT_NUM_SAMPLES = 120
MPC_HORIZON = 10


class PolygonNavigator:
    def __init__(
        self,
        workspace: tuple[float, float, float, float],
        polygons: list[list[Pose]],
        v_max: float,
        w_max: float,
        wheel_base: float,
        dt: float,
    ):
        self._planner = PathPlanner(
            RRT_NUM_SAMPLES, dt, wheel_base, v_max, Dimensions(*workspace)
        )

        polygons_cpp = [[Pose2D(p.x, p.y) for p in poly] for poly in polygons]
        self._inflated_polygons = self._planner.set_obstacles(polygons_cpp)

        self._controller = Controller(dt, MPC_HORIZON, v_max, w_max)

        self._path = None
        self._step = 0

    def plan_path(self, start: Pose, goal: Pose) -> list[Pose]:
        dubins_path = self._planner.generate_path(
            Pose2D(start.x, start.y, start.theta),
            Pose2D(goal.x, goal.y, goal.theta),
        )

        self._path = self._planner.sample_path(dubins_path)
        self._path.extend(self._path[-1] for _ in range(2 * MPC_HORIZON))
        self._path_vel = self._planner.get_velocity_profile(self._path)

        self._step = 0

        return [Pose(p.x, p.y, p.theta) for p in self._path]

    def step(self, current_pose: Pose, v_current: float) -> VelocityCommand:
        if self._path is None:
            raise ValueError("Path not set. Call plan_path() first.")

        if self._step + MPC_HORIZON >= len(self._path):
            return VelocityCommand(0.0, 0.0)

        local_path = self._path[self._step : self._step + MPC_HORIZON]
        local_path_vel = self._path_vel[self._step : self._step + MPC_HORIZON]

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
