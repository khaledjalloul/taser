# type: ignore
import matplotlib.pyplot as plt

from wheeled_humanoid import Pose2D, RRTPathPlanner

NUM_SAMPLES = 100

start = Pose2D()
goal = Pose2D(3, 3)

rrt = RRTPathPlanner(NUM_SAMPLES, 0.1, 0.0)
obstacles = [
    [Pose2D(-1, 1), Pose2D(-1, 2), Pose2D(2, 2), Pose2D(2, 1)],
    [Pose2D(3, 1), Pose2D(3, 2), Pose2D(4, 2), Pose2D(4, 1)]
]
rrt.set_obstacles(obstacles)

points = [start]
parent_idxs = [-1]
distances = [0]

dim = rrt.get_dimensions(start, goal)

for sample in range(NUM_SAMPLES):
    points, parent_idxs, distances = rrt.sample_new_point(
        points, parent_idxs, distances, dim, sample)

    plt.clf()
    plt.grid()
    plt.xlim(dim.x_min, dim.x_max)
    plt.ylim(dim.y_min, dim.y_max)
    plt.scatter(start.x, start.y, marker="o", color="green")
    plt.scatter(goal.x, goal.y, marker="o", color="orange")

    for obstacle in obstacles:
        x = [vertex.x for vertex in obstacle]
        y = [vertex.y for vertex in obstacle]
        plt.fill(x, y, color="red", alpha=0.5)

    for i, point in enumerate(points):
        if parent_idxs[i] != -1:
            plt.plot(
                [point.x, points[parent_idxs[i]].x],
                [point.y, points[parent_idxs[i]].y],
                marker="o", color="blue", linewidth=0.5)

    plt.ginput(1, timeout=0.001)


path = rrt.generate_path(start, goal)
for i in range(len(path) - 1):
    plt.plot(
        [path[i].x, path[i + 1].x],
        [path[i].y, path[i + 1].y],
        marker="o", color="green", linewidth=5)

plt.show()
