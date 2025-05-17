# type: ignore
import math
import matplotlib.pyplot as plt

from wheeled_humanoid import Pose2D
from wheeled_humanoid.base import Dimensions, PathPlanner, get_car_turning_radius, get_dubins_segment

from dubins import plot_dubins_segment


if __name__ == "__main__":
    NUM_SAMPLES = 100
    L = 0.5
    dubins_radius = get_car_turning_radius(L, math.pi / 4)

    start = Pose2D()
    goal = Pose2D(3, 3)

    rrt = PathPlanner(NUM_SAMPLES, 0.1, L)
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

    for sample in range(NUM_SAMPLES):
        points, parent_idxs, distances = rrt.sample_new_point(
            points, parent_idxs, distances, dim, sample)

        plt.clf()
        plt.grid()
        plt.xlim(dim.x_min, dim.x_max)
        plt.ylim(dim.y_min, dim.y_max)
        plt.scatter(start.x, start.y, marker="o", color="green")
        plt.scatter(goal.x, goal.y, marker="o", color="orange")
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')

        for obstacle in inflated_obstacles:
            x = [vertex.x for vertex in obstacle]
            y = [vertex.y for vertex in obstacle]
            plt.fill(x, y, color="pink", alpha=0.5)

        for obstacle in obstacles:
            x = [vertex.x for vertex in obstacle]
            y = [vertex.y for vertex in obstacle]
            plt.fill(x, y, color="red", alpha=0.5)

        for i, point in enumerate(points):
            if parent_idxs[i] != -1:
                segment = get_dubins_segment(
                    points[parent_idxs[i]], point, dubins_radius)
                plot_dubins_segment(segment, ax)

        plt.ginput(1, timeout=0.0001)

    path = rrt.generate_path(start, goal)
    for segment in path:
        plot_dubins_segment(segment, ax, 'green', linewidth=3)

    plt.show()
