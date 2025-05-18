# type: ignore
import matplotlib.pyplot as plt
import numpy as np

from wheeled_humanoid import Pose2D
from wheeled_humanoid.base import Dimensions, PathPlanner, get_car_turning_radius, get_dubins_segment

from dubins import plot_dubins_segment


if __name__ == "__main__":
    RRT_NUM_SAMPLES = 100
    L = 0.5
    N = 20  # Number of path samples
    dubins_radius = get_car_turning_radius(L, np.pi / 4)

    start = Pose2D()
    goal = Pose2D(3, 3)

    rrt = PathPlanner(RRT_NUM_SAMPLES, 0.1, L)
    obstacles = [
        [Pose2D(-1, 1), Pose2D(-1, 2), Pose2D(2, 2),
         Pose2D(2, 1), Pose2D(1, 0.5)],
        [Pose2D(3, 1), Pose2D(3, 2), Pose2D(4, 2), Pose2D(4, 1)]
    ]
    inflated_obstacles = rrt.set_obstacles(obstacles)

    points = [start]
    parent_idxs = [-1]
    distances = [0]

    dim = Dimensions(-2, 5, -2, 5)

    for sample in range(RRT_NUM_SAMPLES):
        points, parent_idxs, distances = rrt.sample_new_point(
            points, parent_idxs, distances, dim, sample)

        plt.clf()
        plt.grid()
        plt.xlim(dim.x_min, dim.x_max)
        plt.ylim(dim.y_min, dim.y_max)
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')

        # Plot obstacles
        for obstacle in inflated_obstacles:
            x = [vertex.x for vertex in obstacle]
            y = [vertex.y for vertex in obstacle]
            plt.fill(x, y, color="pink", alpha=0.5)

        for obstacle in obstacles:
            x = [vertex.x for vertex in obstacle]
            y = [vertex.y for vertex in obstacle]
            plt.fill(x, y, color="red", alpha=0.5)

        # Plot start and goal
        plt.scatter(start.x, start.y, marker="o", color="green", zorder=3)
        plt.scatter(goal.x, goal.y, marker="o", color="orange")

        # Plot robot size and orientation
        ax.add_patch(plt.Circle(
            (start.x, start.y),
            L / 2,
            fill=False,
            color='green',
            zorder=3
        ))
        plt.quiver(start.x, start.y, np.cos(start.theta), np.sin(start.theta),
                   color='green', scale_units='xy', zorder=3, scale=2, width=0.005)

        # Plot generated dubins segments so far
        for i, point in enumerate(points):
            if parent_idxs[i] != -1:
                segment = get_dubins_segment(
                    points[parent_idxs[i]], point, dubins_radius)
                plot_dubins_segment(segment, ax)

        plt.ginput(1, timeout=0.0001)

    # Generate the final dubins path and plot it
    dubins_path = rrt.generate_path(start, goal)
    for segment in dubins_path:
        plot_dubins_segment(segment, ax, 'green', linewidth=3)

    # Plot the sampled path with point orientations
    path = rrt.sample_path(dubins_path, N)
    for s in path:
        plt.scatter(s.x, s.y, marker="o", color="orange", zorder=2)
        plt.quiver(s.x, s.y, np.cos(s.theta), np.sin(s.theta),
                   color='black', scale_units='xy', zorder=2, scale=3, width=0.005)

    plt.show()
