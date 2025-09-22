from logging import getLogger

from taser_cpp.navigation import (
    Dimensions,
    PathPlanner,
)

from taser.common.datatypes import Pose, VelocityCommand, Workspace
from taser.navigation import PurePursuitController
from taser_cpp import Pose2D


class PolygonNavigator:
    def __init__(
        self,
        workspace: Workspace,
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
        self._step = 0
        self._path = None

        self._logger = getLogger(__name__)

        self._planner = PathPlanner(
            num_rrt_samples, dt, wheel_base, v_max, Dimensions(*workspace.tuple())
        )

        self._set_polygons(polygons)

        # TODO: Fix controller outputting high angular velocities
        # self._controller = MPCControllerCpp(
        #     dt=dt,
        #     N=mpc_horizon,
        #     v_max=v_max,
        #     omega_max=w_max,
        # )

        self._controller = PurePursuitController(
            lookahead_base=0.25,
            lookahead_gain=0.6,
            v_max=v_max,
            w_max=w_max,
            curve_slowdown=1.0,
        )

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

        if not dubins_path:
            self._logger.warning(f"No dubins path found from {start} to {goal}")
            return []

        self._path_cpp = self._planner.sample_path(dubins_path)
        self._path_cpp.extend(self._path_cpp[-1] for _ in range(2 * self._mpc_horizon))
        self._path = [Pose(p.x, p.y, p.theta) for p in self._path_cpp]

        self._path_vel_cpp = self._planner.get_velocity_profile(self._path_cpp)
        self._path_vel = [VelocityCommand(v=v.v, w=v.omega) for v in self._path_vel_cpp]

        # self._step = 0
        self._controller.set_path(self._path)

        return self._path

    def step(self, current_pose: Pose, v_current: float = None) -> VelocityCommand:
        cmd, reached, info = self._controller.step(current_pose, v_current)
        return cmd, reached

    # def step(self, current_pose: Pose, v_current: float = None) -> VelocityCommand:
    #     if self._path is None:
    #         raise ValueError("Path not set. Call plan_path() first.")

    #     if self._step + self._mpc_horizon >= len(self._path):
    #         return VelocityCommand(0.0, 0.0)

    #     local_path = self._path[self._step : self._step + self._mpc_horizon]
    #     local_path_vel = self._path_vel[self._step : self._step + self._mpc_horizon]

    #     cmd = self._controller.step(
    #         current_pose,
    #         local_path,
    #         local_path_vel,
    #     )

    #     if abs(cmd.w) > 10.0:
    #         self._logger.info(
    #             f"High omega command: {cmd}. Local path: {local_path}. Local path velocities: {local_path_vel}"
    #         )

    #     self._step += 1
    #     return cmd

    @property
    def inflated_polygons(self) -> list[list[Pose]]:
        return self._inflated_polygons

    def _set_polygons(self, polygons: list[list[Pose]]):
        polygons_cpp = [[Pose2D(p.x, p.y) for p in poly] for poly in polygons]
        self._inflated_polygons = self._planner.set_obstacles(polygons_cpp)
