# type: ignore

import numpy as np
import cvxpy as cp

from wheeled_humanoid import Pose2D, BaseVelocity


class Controller:
    def __init__(self, dt: float, N: int, v_max: float, omega_max: float):
        self.dt = dt
        self.N = N
        self.v_max = v_max
        self.omega_max = omega_max

        self.nx = 3
        self.nu = 2

        self.Q = np.diag([10, 10, 10])
        self.R = np.diag([0.5, 0.1])

    def step(self, x0: Pose2D, x_ref: list[Pose2D], u_ref: list[BaseVelocity]) -> BaseVelocity:
        if len(x_ref) < self.N:
            print(
                "Cannot step base MPC controller, reference path x_ref has less than N elements.")
            return BaseVelocity()
        if len(u_ref) < self.N:
            print(
                "Cannot step base MPC controller, reference velocity profile u_ref has less than N elements.")
            return BaseVelocity()

        delta_x = cp.Variable((self.nx, self.N + 1))
        delta_u = cp.Variable((self.nu, self.N))

        cost = 0
        constraints = [
            delta_x[:, 0] == np.array(x0.list()) - x_ref[0].list(),
            delta_x[:, -1] == 0
        ]

        for k in range(self.N):
            A, B = self.get_linearized_model(x_ref[k], u_ref[k])

            constraints += [
                delta_x[:, k + 1] == A @ delta_x[:, k] + B @ delta_u[:, k],
            ]
            cost += cp.quad_form(delta_x[:, k], self.Q) + \
                cp.quad_form(delta_u[:, k], self.R)

        prob = cp.Problem(cp.Minimize(cost), constraints)
        result = prob.solve(solver=cp.OSQP)
        if np.isinf(result):
            print("Problem is infeasible")

        u_opt = u_ref[0].list() + delta_u[:, 0].value

        return BaseVelocity(u_opt[0], u_opt[1])

    def get_linearized_model(self, x0: Pose2D, u0: BaseVelocity) -> tuple[np.ndarray, np.ndarray]:
        A = np.eye(self.nx)
        A[0, 2] = -np.sin(x0.theta) * self.dt * u0.v
        A[1, 2] = np.cos(x0.theta) * self.dt * u0.v

        B = np.zeros((self.nx, self.nu))
        B[0, 0] = self.dt * np.cos(x0.theta)
        B[1, 0] = self.dt * np.sin(x0.theta)
        B[2, 1] = self.dt

        return A, B
