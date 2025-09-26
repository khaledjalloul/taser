import logging

import cvxpy as cp
import numpy as np
from taser_cpp.navigation import Controller

from taser.common.datatypes import Pose2D, VelocityCommand
from taser_cpp import BaseVelocity
from taser_cpp import Pose2D as Pose2DCpp

logging.basicConfig(level=logging.INFO)


class MPCController:
    def __init__(self, dt: float, N: int, v_max: float, omega_max: float):
        self.dt = dt
        self.N = N
        self.v_max = v_max
        self.omega_max = omega_max

        self.nx = 3
        self.nu = 2

        self.Q = np.diag([10, 10, 10])
        self.R = np.diag([0.5, 0.1])

    def step(
        self, x0: Pose2D, x_ref: list[Pose2D], u_ref: list[VelocityCommand]
    ) -> VelocityCommand:
        if len(x_ref) < self.N:
            logging.warning(
                "Cannot step base MPC controller, reference path x_ref has less than N elements."
            )
            return VelocityCommand()
        if len(u_ref) < self.N:
            logging.warning(
                "Cannot step base MPC controller, reference velocity profile u_ref has less than N elements."
            )
            return VelocityCommand()

        delta_x = cp.Variable((self.nx, self.N + 1))
        delta_u = cp.Variable((self.nu, self.N))

        cost = 0
        constraints = [
            delta_x[:, 0] == np.array(x0.tuple()) - x_ref[0].tuple(),
            # delta_x[:, -1] == 0
        ]

        for k in range(self.N):
            A, B = self.get_linearized_model(x_ref[k], u_ref[k])

            constraints += [
                delta_x[:, k + 1] == A @ delta_x[:, k] + B @ delta_u[:, k],
            ]
            cost += cp.quad_form(delta_x[:, k], self.Q) + cp.quad_form(
                delta_u[:, k], self.R
            )

        prob = cp.Problem(cp.Minimize(cost), constraints)
        result = prob.solve(solver=cp.OSQP)
        if np.isinf(result):
            logging.warning("Problem is infeasible")
            return VelocityCommand()

        u_opt = u_ref[0].tuple() + delta_u[:, 0].value

        return VelocityCommand(u_opt[0], u_opt[1])

    def get_linearized_model(
        self, x0: Pose2D, u0: VelocityCommand
    ) -> tuple[np.ndarray, np.ndarray]:
        A = np.eye(self.nx)
        A[0, 2] = -np.sin(x0.theta) * u0.v * self.dt
        A[1, 2] = np.cos(x0.theta) * u0.v * self.dt

        B = np.zeros((self.nx, self.nu))
        B[0, 0] = np.cos(x0.theta) * self.dt
        B[1, 0] = np.sin(x0.theta) * self.dt
        B[2, 1] = self.dt

        return A, B


class MPCControllerCpp:
    def __init__(self, dt: float, N: int, v_max: float, omega_max: float):
        self._controller = Controller(dt, N, v_max, omega_max)

    def step(
        self, x0: Pose2D, x_ref: list[Pose2D], u_ref: list[VelocityCommand]
    ) -> VelocityCommand:
        x0_cpp = Pose2DCpp(x0.x, x0.y, x0.theta)
        x_ref_cpp = [Pose2DCpp(p.x, p.y, p.theta) for p in x_ref]
        u_ref_cpp = [BaseVelocity(v.v, v.w) for v in u_ref]
        cmd_cpp = self._controller.step(x0_cpp, x_ref_cpp, u_ref_cpp)
        return VelocityCommand(cmd_cpp.v, cmd_cpp.omega)

    def get_linearized_model(
        self, x0: Pose2D, u0: VelocityCommand
    ) -> tuple[np.ndarray, np.ndarray]:
        x0_cpp = Pose2DCpp(x0.x, x0.y, x0.theta)
        u0_cpp = BaseVelocity(u0.v, u0.w)
        A_cpp, B_cpp = self._controller.get_linearized_model(x0_cpp, u0_cpp)
        return np.array(A_cpp), np.array(B_cpp)
