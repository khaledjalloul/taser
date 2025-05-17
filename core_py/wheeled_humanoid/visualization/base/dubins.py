# type: ignore
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

from wheeled_humanoid import Pose2D
from wheeled_humanoid.base import Direction, DubinsSegment, Circle, get_dubins_segment, get_tangent, get_turning_circles


def plot_dubins_segment(segment: DubinsSegment, ax: plt.Axes):
    arc = segment.arc
    arc_start_theta_vec = (arc.start.x - arc.center.x,
                           arc.start.y - arc.center.y)
    arc_start_theta = np.atan2(
        arc_start_theta_vec[1], arc_start_theta_vec[0])
    arc_rot_angle = 0 if arc.direction == Direction.LEFT else - \
        arc.arc_angle * 180 / np.pi

    ax.add_patch(Arc(
        (arc.center.x, arc.center.y),
        height=2 * arc.radius,
        width=2 * arc.radius,
        facecolor="none",
        edgecolor="purple",
        theta1=arc_start_theta * 180 / np.pi,
        theta2=(arc_start_theta + arc.arc_angle) * 180 / np.pi,
        angle=arc_rot_angle,
        zorder=2
    ))

    line = segment.line
    plt.plot(
        [line.start.x, line.end.x],
        [line.start.y, line.end.y],
        color="purple",
        linewidth=1
    )


if __name__ == "__main__":
    while True:
        plt.clf()
        plt.title("Dubins Path Utils")
        plt.grid()
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')

        # Turning circles
        pose = Pose2D(-1.5, 2.5, np.random.uniform(0, 2 * np.pi))
        circles = get_turning_circles(pose, 1)

        plt.scatter(pose.x, pose.y, marker="o", color="green")

        for c in circles:
            ax.add_patch(plt.Circle(
                (c.center.x, c.center.y),
                c.radius,
                fill=False,
                color='blue',
                zorder=2
            ))

        # Tangent
        circle = Circle(
            Pose2D(2, 3),
            np.random.uniform(0.3, 1),
            np.random.choice([Direction.LEFT, Direction.RIGHT])
        )
        target = Pose2D(3, 1)
        tangent = get_tangent(circle, target)

        ax.add_patch(plt.Circle(
            (circle.center.x, circle.center.y),
            circle.radius,
            fill=False,
            color='red',
            zorder=2
        ))
        plt.plot(
            [tangent.start.x, tangent.end.x],
            [tangent.start.y, tangent.end.y],
            marker="o",
            color="orange"
        )

        # Dubins segment
        start = Pose2D(0, -3, np.random.uniform(0, 2 * np.pi))
        x = np.random.uniform(2, 4)
        x = np.random.choice([x, -x])
        y = np.random.uniform(-4, 0)
        goal = Pose2D(x, y)
        segment = get_dubins_segment(start, goal, 1)

        plt.scatter(start.x, start.y, marker="o", color="purple")
        plt.scatter(goal.x, goal.y, marker="o", color="purple")
        plot_dubins_segment(segment, ax)

        plt.ginput(2, timeout=1)
