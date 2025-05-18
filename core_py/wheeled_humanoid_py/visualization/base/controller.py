# type: ignore

import math
import matplotlib.pyplot as plt
import numpy as np
from wheeled_humanoid import Pose2D
from wheeled_humanoid.base import Dimensions, Kinematics, PathPlanner, get_euclidean_distance
from wheeled_humanoid.base import Controller
# from wheeled_humanoid_py.base import Controller


def plot_controller_step(ax: plt.Axes, base: Kinematics, start: Pose2D, goal: Pose2D,
                         obstacles: list[list], inflated_obstacles: list[list],
                         path: list[Pose2D]) -> None:
    ax.grid()
    ax.set_xlim(-2, 5)
    ax.set_ylim(-2, 5)
    ax.set_aspect('equal', adjustable='box')

    # Plot obstacles
    for obstacle in inflated_obstacles:
        x = [vertex.x for vertex in obstacle]
        y = [vertex.y for vertex in obstacle]
        ax.fill(x, y, color="pink", alpha=0.5)

    for obstacle in obstacles:
        x = [vertex.x for vertex in obstacle]
        y = [vertex.y for vertex in obstacle]
        ax.fill(x, y, color="red", alpha=0.5)

    # Plot start and goal
    ax.scatter(start.x, start.y, marker="o", color="green")
    ax.scatter(goal.x, goal.y, marker="o", color="orange")

    # Plot robot size and current pose
    ax.scatter(base.pose.x, base.pose.y,
               marker="o", color="purple", zorder=3)
    ax.add_patch(plt.Circle(
        (base.pose.x, base.pose.y),
        L / 2,
        fill=False,
        color='purple',
        zorder=3
    ))
    ax.quiver(base.pose.x, base.pose.y, np.cos(base.pose.theta), np.sin(base.pose.theta),
              color='purple', scale_units='xy', zorder=3, scale=2, width=0.005)

    # Plot generated path with point orientations
    for i in range(len(path) - 1):
        ax.plot(
            [path[i].x, path[i + 1].x],
            [path[i].y, path[i + 1].y],
            marker="o", color="blue", linewidth=0.5, markersize=3)
        # ax.quiver(path[i].x, path[i].y, np.cos(path[i].theta), np.sin(path[i].theta),
        #           color='black', scale_units='xy', zorder=2, scale=3, width=0.005)


if __name__ == "__main__":
    RRT_NUM_SAMPLES = 100
    L = 0.5
    wheel_radius = 0.3
    dt = 0.1  # Time step
    N = 10  # MPC horizon
    V = 3.0  # Desired velocity

    dim = Dimensions(-2, 5, -2, 5)

    base_mpc = Kinematics(L, wheel_radius, dt)
    base_no_mpc = Kinematics(L, wheel_radius, dt)
    controller = Controller(dt, N, 0.0, 0.0)
    rrt = PathPlanner(RRT_NUM_SAMPLES, dt, L, V, dim)

    start = Pose2D(0, 0, -math.pi / 4)
    goal = Pose2D(3, 3, math.pi / 4)
    base_mpc.pose = start
    base_no_mpc.pose = start

    obstacles = [
        [Pose2D(-1, 1), Pose2D(-1, 2), Pose2D(2, 2),
         Pose2D(2, 1), Pose2D(1, 0.5)],
        [Pose2D(3, 1), Pose2D(3, 2), Pose2D(4, 2), Pose2D(4, 1)]
    ]
    inflated_obstacles = rrt.set_obstacles(obstacles)

    dubins_path = rrt.generate_path(start, goal)
    path = rrt.sample_path(dubins_path)
    path.extend([path[-1] for _ in range(2 * N)])
    path_vel = rrt.get_velocity_profile(path)

    fig, axs = plt.subplots(1, 2)
    ax_mpc = axs[0]
    ax_no_mpc = axs[1]

    step = 0
    for _ in range(len(path) - N):
        ax_mpc.clear()
        ax_mpc.set_title("With Tracking MPC")
        ax_no_mpc.clear()
        ax_no_mpc.set_title("Without Tracking MPC")

        plot_controller_step(ax_mpc, base_mpc, start, goal,
                             obstacles, inflated_obstacles, path)
        plot_controller_step(ax_no_mpc, base_no_mpc, start, goal,
                             obstacles, inflated_obstacles, path)

        if get_euclidean_distance(base_mpc.pose, goal) < 0.1 and get_euclidean_distance(base_no_mpc.pose, goal) < 0.2:
            break

        plt.ginput(1, timeout=dt)

        local_path = path[step:step + N]
        local_path_vel = path_vel[step:step + N]

        vel = controller.step(base_mpc.pose, local_path, local_path_vel)
        base_mpc.set_base_velocity(vel.v, vel.omega)
        base_mpc.step(dt)

        base_no_mpc.set_base_velocity(
            local_path_vel[0].v, local_path_vel[0].omega)
        base_no_mpc.step(dt)

        step += 1

    plt.show()
