# type: ignore

import math
import matplotlib.pyplot as plt
import numpy as np
# from base_controller import BaseController
from wheeled_humanoid import BaseController
from wheeled_humanoid import BaseKinematics, Pose2D, RRTPathPlanner


RRT_NUM_SAMPLES = 100
L = 0.5
wheel_radius = 0.3
dt = 0.1
N = 10
v_max = 1.0
omega_max = 2.0

base = BaseKinematics(L, wheel_radius, dt)
controller = BaseController(dt, N, v_max, omega_max)

start = Pose2D(0, 0, np.random.uniform(0, 2 * math.pi))
goal = Pose2D(3, 3, math.pi / 4)
base.pose = start

rrt = RRTPathPlanner(RRT_NUM_SAMPLES, dt)
obstacles = [
    [Pose2D(-1, 1), Pose2D(-1, 2), Pose2D(2, 2), Pose2D(2, 1)],
    [Pose2D(3, 1), Pose2D(3, 2), Pose2D(4, 2), Pose2D(4, 1)]
]
rrt.set_obstacles(obstacles)

for _ in range(50):
    if np.sqrt((base.pose.x - goal.x) ** 2 + (base.pose.y - goal.y) ** 2) < 0.1:
        break

    path = rrt.generate_path(base.pose, goal)
    path = rrt.interpolate_path(path, N)
    path_vel = rrt.get_velocity_profile(path)

    plt.clf()
    plt.grid()
    plt.xlim(-1, 4)
    plt.ylim(-1, 4)

    for obstacle in obstacles:
        x = [vertex.x for vertex in obstacle]
        y = [vertex.y for vertex in obstacle]
        plt.fill(x, y, color="red", alpha=0.5)

    for i in range(len(path) - 1):
        plt.plot(
            [path[i].x, path[i + 1].x],
            [path[i].y, path[i + 1].y],
            marker="o", color="blue", linewidth=0.5, markersize=3)

    plt.scatter(start.x, start.y, marker="o", color="green")
    plt.scatter(base.pose.x, base.pose.y, marker="o", color="purple")
    plt.scatter(goal.x, goal.y, marker="o", color="orange")

    plt.ginput(1, timeout=0.1)

    vel = controller.step(base.pose, path, path_vel)
    base.set_base_velocity(vel.v, vel.omega)
    base.step(dt)

plt.show()
