# type: ignore

import math

import matplotlib.pyplot as plt
import numpy as np

from taser.common.datatypes import Pose, VelocityCommand, Workspace
from taser.navigation import PolygonNavigator

L = 0.3
DT = 0.1
V_MAX = 2.0
W_MAX = 1.0
NUM_RRT_SAMPLES = 120
MPC_HORIZON = 10

WORKSPACE = Workspace(x_min=0, x_max=7, y_min=0, y_max=7)
START = Pose(1, 1, 0)
GOAL = Pose(6, 6, 0)


def plot_controller_step(
    ax: plt.Axes,
    robot: Pose,
    path: list[Pose],  # Path to follow
    trajectory: tuple[list[float], list[float]],  # Actual trajectory of the robot
    polygons: np.ndarray,
    inflated_polygons: np.ndarray,
) -> None:
    ax.clear()
    ax.set_facecolor("white")
    ax.grid(color="lightgray")
    ax.set_xlim(WORKSPACE.x_min, WORKSPACE.x_max)
    ax.set_ylim(WORKSPACE.y_min, WORKSPACE.y_max)
    ax.set_aspect("equal", adjustable="box")

    # Plot obstacles
    for p in inflated_polygons:
        x = [vertex.x for vertex in p]
        y = [vertex.y for vertex in p]
        ax.fill(x, y, color="pink", alpha=0.5)

    for p in polygons:
        x = [vertex.x for vertex in p]
        y = [vertex.y for vertex in p]
        ax.fill(x, y, color="red", alpha=0.5)

    # Plot start and goal poses
    ax.scatter(START.x, START.y, marker="o", color="green", zorder=3)
    ax.scatter(GOAL.x, GOAL.y, marker="o", color="orange", zorder=3)

    # Plot path
    ax.plot(
        [p.x for p in path],
        [p.y for p in path],
        marker="o",
        linestyle="--",
        color="blue",
        markersize=3,
        label="Path",
    )

    # Plot robot pose, size, and orientation
    ax.scatter(robot.x, robot.y, marker="o", color="purple", zorder=3)
    ax.add_patch(
        plt.Circle((robot.x, robot.y), L / 2, fill=False, color="purple", zorder=3)
    )
    ax.quiver(
        robot.x,
        robot.y,
        np.cos(robot.theta),
        np.sin(robot.theta),
        color="purple",
        scale_units="xy",
        zorder=3,
        scale=2,
        width=0.005,
    )

    # Plot robot trajectory
    ax.plot(*trajectory, color="purple", label="Trajectory")


if __name__ == "__main__":
    polygons = [
        [Pose(1, 3), Pose(1, 5), Pose(4, 5), Pose(4, 3), Pose(3, 2.5)],
        [Pose(5, 2), Pose(5, 4), Pose(6, 4), Pose(6, 2)],
    ]

    navigator = PolygonNavigator(
        workspace=WORKSPACE,
        polygons=polygons,
        v_max=V_MAX,
        w_max=W_MAX,
        wheel_base=L,
        dt=DT,
        num_rrt_samples=NUM_RRT_SAMPLES,
        mpc_horizon=MPC_HORIZON,
    )
    path = navigator.plan_path(START, GOAL)

    robot = Pose(x=START.x, y=START.y, theta=START.theta)
    cmd = VelocityCommand(0.0, 0.0)

    fig, plt_ax = plt.subplots(1, 1)
    traj_x, traj_y = [robot.x], [robot.y]

    for _ in range(1200):  # up to 60 s
        plot_controller_step(
            ax=plt_ax,
            path=path,
            trajectory=(traj_x, traj_y),
            robot=robot,
            polygons=polygons,
            inflated_polygons=navigator.inflated_polygons,
        )
        plt.pause(DT)

        if math.dist((robot.x, robot.y), (GOAL.x, GOAL.y)) < 0.1:
            break

        cmd, reached = navigator.step(robot, cmd.v)

        robot.x += cmd.v * math.cos(robot.theta) * DT
        robot.y += cmd.v * math.sin(robot.theta) * DT
        theta = robot.theta + cmd.w * DT
        # Wrap between -pi and pi
        robot.theta = math.atan2(math.sin(theta), math.cos(theta))

        traj_x.append(robot.x)
        traj_y.append(robot.y)

    plt.show()
