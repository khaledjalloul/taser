from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from roboticstoolbox.robot import Robot
from spatialmath import SE3

from taser.common.datatypes import Pose, TaserJointState


class ManipulationKinematics:
    def __init__(self, arm: str):
        self._arm_side = arm

        taser_ros_share_dir = get_package_share_directory("taser_ros")
        urdf_path = (
            Path(taser_ros_share_dir) / "robot_description" / "urdf" / "taser.urdf"
        )

        self._arm = Robot.URDF(file_path=str(urdf_path), gripper=f"{arm}_arm_eef")

    def get_eef_position(self, q: TaserJointState) -> Pose:
        T = self._arm.fkine(
            q=q.ordered,
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
        )

        return Pose(
            x=T.t[0],
            y=T.t[1],
            z=T.t[2],
            rx=T.rpy()[0],
            ry=T.rpy()[1],
            rz=T.rpy()[2],
        )

    def get_q(self, pose: Pose) -> np.ndarray:
        T = SE3.Trans(pose.x, pose.y, pose.z)

        # if pose.rx or pose.ry or pose.rz:
        #     T = T * SE3.RPY([pose.rx, pose.ry, pose.rz], unit="rad")

        return self._arm.ikine_LM(
            Tep=T,
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
            mask=[1, 1, 1, 0, 0, 0],
        ).q

    def get_dq(self, v_lin: np.ndarray, q: TaserJointState) -> np.ndarray:
        J = self._arm.jacob0(
            q=q.ordered,
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
        )
        J_pos = J[0:3, :]

        return np.linalg.pinv(J_pos) @ v_lin

    def get_traj(
        self,
        pose_start: Pose,
        pose_end: Pose,
        type: str,
        time: float,
        dt: float,
    ) -> np.ndarray:
        n_points = int(time / dt)
        T_start = SE3.Trans(pose_start.x, pose_start.y, pose_start.z)
        T_end = SE3.Trans(pose_end.x, pose_end.y, pose_end.z)
        traj = self._arm.jtraj(
            T_start,
            T_end,
            n_points,
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
        )
        if type == "q":
            return traj.q
        elif type == "dq":
            return traj.qd


if __name__ == "__main__":
    kinematics = ManipulationKinematics(arm="left")

    q_start = TaserJointState(left_arm=np.array([0.1, 0.1, 0.1]))
    pose_start = kinematics.get_eef_position(q=q_start)
    print("Start Pose:", pose_start)

    pose_end = Pose(x=0.5, y=0.0, z=0.0)
    q_end = kinematics.get_q(pose=pose_end)
    print("End Joint Angles:", q_end)

    dq = kinematics.get_dq(v_lin=np.array([0.1, 0.0, 0.0]), q=q_start)
    print("Joint Velocities for 0.1 m/s in x:", dq)

    q_traj = kinematics.get_traj(
        pose_start=pose_start,
        pose_end=pose_end,
        type="q",
        time=5.0,
        dt=0.1,
    )
    print("Joint Trajectory:", q_traj.shape)
