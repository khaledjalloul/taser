import math

import matplotlib.pyplot as plt
import numpy as np

from taser.common.datatypes import Pose, VelocityCommand, Workspace
from taser.navigation import GridNavigator, OccupancyGrid

L = 1
DT = 0.05
V_MAX = 3.0
W_MAX = 2

WORKSPACE = Workspace(x_min=-5, x_max=5, y_min=-5, y_max=5)
START = Pose(x=-4, y=-4, rz=3 * np.pi / 4)
GOAL = Pose(x=4, y=4, rz=0)


def set_up_occupancy_grid() -> OccupancyGrid:
    occupancy_grid = OccupancyGrid(workspace=WORKSPACE, cellsize=0.1)
    occupancy_grid.set([-2, 5, -4, -4], 1)
    occupancy_grid.set([-2, -2, -4, 3], 1)
    occupancy_grid.set([2, 2, 0, 5], 1)
    return occupancy_grid


def plot_controller_step(
    ax: plt.Axes,
    robot: Pose,
    path: list[Pose],  # Path to follow
    trajectory: tuple[list[float], list[float]],  # Actual trajectory of the robot
    occupancy_grid: np.ndarray,
    inflated_occupancy_grid: np.ndarray,
) -> None:
    ax.clear()
    ax.set_facecolor("white")
    ax.grid(color="lightgray")
    ax.set_xlim(WORKSPACE.x_min, WORKSPACE.x_max)
    ax.set_ylim(WORKSPACE.y_min, WORKSPACE.y_max)
    ax.set_aspect("equal", adjustable="box")

    # Plot inflated occupancy grid
    ax.imshow(
        occupancy_grid + inflated_occupancy_grid * 0.3,
        cmap="gray_r",
        origin="lower",
        interpolation="none",
        vmin=0,
        vmax=1,
        extent=WORKSPACE.tuple(),
    )

    # Plot start and goal poses and orientations
    ax.scatter(START.x, START.y, marker="o", color="green", zorder=3)
    ax.quiver(
        START.x,
        START.y,
        np.cos(START.rz),
        np.sin(START.rz),
        color="green",
        scale_units="xy",
        zorder=3,
        scale=2,
        width=0.005,
    )
    ax.scatter(GOAL.x, GOAL.y, marker="o", color="orange", zorder=3)
    ax.quiver(
        GOAL.x,
        GOAL.y,
        np.cos(GOAL.rz),
        np.sin(GOAL.rz),
        color="orange",
        scale_units="xy",
        zorder=3,
        scale=2,
        width=0.005,
    )

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
        np.cos(robot.rz),
        np.sin(robot.rz),
        color="purple",
        scale_units="xy",
        zorder=3,
        scale=2,
        width=0.005,
    )

    # Plot robot trajectory
    ax.plot(*trajectory, color="purple", label="Trajectory")


if __name__ == "__main__":
    occupancy_grid = set_up_occupancy_grid()

    navigator = GridNavigator(
        workspace=WORKSPACE,
        occupancy_grid=occupancy_grid,
        v_max=V_MAX,
        w_max=W_MAX,
        wheel_base=L,
    )
    path = navigator.plan_path(START, GOAL)

    robot = Pose(x=START.x, y=START.y, rz=START.rz)
    cmd = VelocityCommand(0.0, 0.0)
    reached = False

    fig, plt_ax = plt.subplots(1, 1)
    fig.set_size_inches(14, 10)

    traj_x, traj_y = [robot.x], [robot.y]

    for _ in range(1200):  # up to 60 s
        plot_controller_step(
            ax=plt_ax,
            path=path,
            trajectory=(traj_x, traj_y),
            robot=robot,
            occupancy_grid=occupancy_grid.grid,
            inflated_occupancy_grid=navigator.inflated_occupancy_grid,
        )
        plt.pause(DT)

        if reached:
            break

        cmd, reached = navigator.step(robot, cmd.v)

        robot.x += cmd.v * math.cos(robot.rz) * DT
        robot.y += cmd.v * math.sin(robot.rz) * DT
        rz = robot.rz + cmd.w * DT
        # Wrap between -pi and pi
        robot.rz = math.atan2(math.sin(rz), math.cos(rz))

        traj_x.append(robot.x)
        traj_y.append(robot.y)

    plt.show()
