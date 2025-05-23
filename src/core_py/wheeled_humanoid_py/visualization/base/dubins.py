# type: ignore
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

from wheeled_humanoid import Pose2D
from wheeled_humanoid.base import Direction, DubinsSegment, Circle, get_dubins_segment, get_tangent, get_turning_circles


def plot_dubins_segment(segment: DubinsSegment,
                        ax: plt.Axes,
                        color: str = "purple",
                        linewidth: float = 1,
                        quiver: bool = False) -> None:
    arc = segment.arc
    start_theta = np.arctan2(
        arc.start.y - arc.center.y,
        arc.start.x - arc.center.x) * 180 / np.pi
    end_theta = start_theta + arc.angle * 180 / np.pi

    ax.add_patch(Arc(
        (arc.center.x, arc.center.y),
        height=2 * arc.radius,
        width=2 * arc.radius,
        facecolor="none",
        edgecolor=color,
        # Matplotlib arc is always drawn counter-clockwise
        theta1=start_theta if arc.direction == Direction.LEFT else end_theta,
        theta2=end_theta if arc.direction == Direction.LEFT else start_theta,
        zorder=2,
        linewidth=linewidth,
    ))

    line = segment.line
    plt.plot(
        [line.start.x, line.end.x],
        [line.start.y, line.end.y],
        color=color,
        linewidth=linewidth,
    )

    if quiver:
        plt.quiver(arc.start.x, arc.start.y, np.cos(arc.start.theta), np.sin(arc.start.theta),
                   color='black', scale_units='xy', zorder=2, scale=1.3, width=0.005)
        plt.quiver(line.end.x, line.end.y, np.cos(line.end.theta), np.sin(line.end.theta),
                   color='black', scale_units='xy', zorder=2, scale=1.3, width=0.005)


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

        plt.quiver(pose.x, pose.y, np.cos(pose.theta), np.sin(pose.theta),
                   color='black', scale_units='xy', zorder=2, scale=1, width=0.005)

        for c in circles:
            ax.add_patch(plt.Circle(
                (c.center.x, c.center.y),
                c.radius,
                fill=False,
                color='blue' if c.direction == Direction.LEFT else 'green',
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

        plot_dubins_segment(segment, ax, quiver=True)

        plt.ginput(1, timeout=1)
