import matplotlib.pyplot as plt
from roboticstoolbox.robot import Robot

from taser.common.datatypes import Pose, TaserJointState
from taser.common.model import URDF_PATH
from taser.manipulation import PickController

DT = 0.05

if __name__ == "__main__":
    robot = Robot.URDF(str(URDF_PATH))

    controller = PickController()
    q = TaserJointState()
    dq = TaserJointState()

    target = Pose(x=0.5, y=0.0, z=0.0)
    controller.set_target(target)

    fig = plt.figure()
    ax: plt.Axes = fig.add_subplot(111, projection="3d")

    while True:
        dq, done = controller.step(q)

        if done:
            break

        q.left_arm += dq.left_arm * DT
        q.right_arm += dq.right_arm * DT

        ax.clear()

        ax.set_xlim([-0.5, 0.5])
        ax.set_ylim([-0.5, 0.5])
        ax.set_zlim([-0.5, 0.5])

        links = robot.links

        for link_to_plot in ["left_arm_eef", "right_arm_eef"]:
            while link_to_plot != "base_link":
                link_idx = next(
                    i for i, l in enumerate(links) if l.name == link_to_plot
                )
                link = links[link_idx]
                link_pose = robot.fkine(
                    q.ordered_rtb,
                    end=link.name,
                    start="base_link",
                )

                parent_link = link.parent.name
                parent_link_pose = robot.fkine(
                    q.ordered_rtb,
                    end=parent_link,
                    start="base_link",
                )

                ax.plot(
                    [parent_link_pose.t[0], link_pose.t[0]],
                    [parent_link_pose.t[1], link_pose.t[1]],
                    [parent_link_pose.t[2], link_pose.t[2]],
                    color="blue" if "left" in link_to_plot else "red",
                    marker="o",
                )

                link_to_plot = parent_link

        plt.pause(DT)

    plt.show()
