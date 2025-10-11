from pathlib import Path

import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from roboticstoolbox.robot import Robot

from taser.common.datatypes import Pose, TaserJointState
from taser.manipulation import GraspController

DT = 0.05

if __name__ == "__main__":
    taser_ros_share_dir = get_package_share_directory("taser_ros")
    urdf_path = Path(taser_ros_share_dir) / "robot_description" / "urdf" / "taser.urdf"

    robot = Robot.URDF(str(urdf_path))

    controller = GraspController()
    q = TaserJointState()
    pose_start_left = controller._left_arm.get_eef_position(q)
    pose_start_right = controller._right_arm.get_eef_position(q)

    print("Start left:", pose_start_left)
    print("Start right:", pose_start_right)

    target = Pose(x=0.5, y=0.0, z=0.0)
    controller.set_target(target)

    fig = plt.figure()
    ax: plt.Axes = fig.add_subplot(111, projection="3d")

    for _ in range(10000):
        dq = controller.step(q)
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

                # print("Plotting link", link.name)
                # print("q", q.left_arm if "left" in link_to_plot else q.right_arm)

                link_pose = robot.fkine(
                    q.ordered,
                    end=link.name,
                    start="base_link",
                )
                # print("Link pose:", link_pose.t)

                parent_link = link.parent.name
                # parent_link_idx = next(
                #     i for i, l in enumerate(links) if l.name == parent_link.name
                # )
                parent_link_pose = robot.fkine(
                    q.ordered,
                    end=parent_link,
                    start="base_link",
                )
                # print("Parent link pose:", parent_link_pose.t)

                ax.plot(
                    [parent_link_pose.t[0], link_pose.t[0]],
                    [parent_link_pose.t[1], link_pose.t[1]],
                    [parent_link_pose.t[2], link_pose.t[2]],
                    color="blue" if "left" in link_to_plot else "red",
                    marker="o",
                )

                link_to_plot = parent_link

        plt.pause(DT)

        # left_pos = controller._left_arm.get_eef_position(q)
        # right_pos = controller._right_arm.get_eef_position(q)
        # print(left_pos, right_pos)

    plt.show()
