from taser.common.datatypes import Pose2D, VelocityCommand, Workspace
from taser.navigation import PurePursuitController, RRTStarPathPlanner


class PolygonNavigator:
    def __init__(
        self,
        workspace: Workspace,
        polygons: list[list[Pose2D]],
        v_max: float,
        w_max: float,
        wheel_base: float,
        dt: float,
        num_rrt_samples: int,
        mpc_horizon: int,
    ):
        # self._mpc_horizon = mpc_horizon
        # self._step = 0
        # self._path = None

        self._planner = RRTStarPathPlanner(
            polygons=polygons,
            num_rrt_samples=num_rrt_samples,
            mpc_horizon=mpc_horizon,
            dt=dt,
            wheel_base=wheel_base,
            v_max=v_max,
            workspace=workspace,
        )

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
        start: Pose2D,
        goal: Pose2D,
        polygons: list[list[Pose2D]] = None,
    ) -> list[Pose2D]:
        if polygons is not None:
            self._set_polygons(polygons)

        path, path_vel = self._planner.plan(start, goal)
        self._controller.set_path(path)

        return path

    def step(self, current_pose: Pose2D, v_current: float = None) -> VelocityCommand:
        cmd, reached, info = self._controller.step(current_pose, v_current)
        return cmd, reached

    # def step(self, current_pose: Pose2D, v_current: float = None) -> VelocityCommand:
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
    def inflated_polygons(self) -> list[list[Pose2D]]:
        return self._planner.inflated_polygons
