import matplotlib.pyplot as plt

from taser.common.datatypes import Pose, TaserJointState
from taser.manipulation import ManipulationKinematics, PickController

DT = 0.05
TARGET_CUBE_SIZE = 0.3

if __name__ == "__main__":
    left_arm = ManipulationKinematics(arm="left")
    right_arm = ManipulationKinematics(arm="right")

    controller = PickController()
    q = TaserJointState()
    dq = TaserJointState()

    target = Pose(x=0.5, y=0.0, z=0.0)
    controller.set_target(target)

    fig = plt.figure()
    ax: plt.Axes = fig.add_subplot(111, projection="3d")

    while True:
        left_arm_pos = left_arm.get_eef_position(q)
        right_arm_pos = right_arm.get_eef_position(q)

        # Simulate picking up the target
        if (
            abs(left_arm_pos.x - controller._target_pos_left_b.x) < 0.05
            and abs(right_arm_pos.x - controller._target_pos_right_b.x) < 0.05
            and abs(left_arm_pos.z - controller._target_pos_left_b.z) < 0.05
            and abs(right_arm_pos.z - controller._target_pos_right_b.z) < 0.05
        ):
            target.z = (left_arm_pos.z + right_arm_pos.z) / 2

        controller.set_target(target)
        dq, done = controller.step(q)

        if done:
            break

        q.left_arm += dq.left_arm * DT
        q.right_arm += dq.right_arm * DT

        ax.clear()

        ax.set_xlim([-0.5, 0.5])
        ax.set_ylim([-0.5, 0.5])
        ax.set_zlim([-0.5, 0.5])
        ax.set_aspect("equal", adjustable="box")


        links = left_arm._arm.links

        for link_to_plot in ["left_arm_eef", "right_arm_eef"]:
            while link_to_plot != "base_link":
                link_idx = next(
                    i for i, l in enumerate(links) if l.name == link_to_plot
                )
                link = links[link_idx]
                link_pose = left_arm._arm.fkine(
                    q.ordered_rtb,
                    end=link.name,
                    start="base_link",
                )

                parent_link = link.parent.name
                parent_link_pose = left_arm._arm.fkine(
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

        # Plot target
        ax.bar3d(
            target.x - TARGET_CUBE_SIZE / 2,
            target.y - TARGET_CUBE_SIZE / 2,
            target.z - TARGET_CUBE_SIZE / 2,
            TARGET_CUBE_SIZE,
            TARGET_CUBE_SIZE,
            TARGET_CUBE_SIZE,
            color="purple",
        )

        plt.pause(DT)

    plt.show()
