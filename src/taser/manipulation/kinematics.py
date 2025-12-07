import numpy as np
from roboticstoolbox.robot import Robot
from spatialmath import SE3

from taser.common.datatypes import Pose, TaserJointState
from taser.common.model import URDF_PATH


class ManipulationKinematics:
    def __init__(self, arm: str):
        self._arm_side = arm

        self._arm = Robot.URDF(file_path=str(URDF_PATH))

    def get_eef_position(self, q: TaserJointState) -> Pose:
        T = self._arm.fkine(
            q=q.to("rtb"),
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

    def get_eef_velocity(self, q: TaserJointState, dq: TaserJointState) -> np.ndarray:
        J = self._arm.jacob0(
            q=q.to("rtb"),
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
        )
        v = J @ (dq.left_arm if self._arm_side == "left" else dq.right_arm)
        return v[0:3]

    def get_q(self, pose: Pose, q0: list[float] = None) -> tuple[np.ndarray, bool]:
        T = SE3.Trans(pose.x, pose.y, pose.z)

        # if pose.rx or pose.ry or pose.rz:
        #     T = T * SE3.RPY([pose.rx, pose.ry, pose.rz], unit="rad")

        sol = self._arm.ikine_LM(
            Tep=T,
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
            mask=[1, 1, 1, 0, 0, 0],
            q0=q0,
        )

        return sol.q, sol.success

    def get_dq(
        self, v: np.ndarray, weights: np.ndarray, q: TaserJointState
    ) -> np.ndarray:
        J = self._arm.jacob0(
            q=q.to("rtb"),
            start="base_link",
            end=f"{self._arm_side}_arm_eef",
        )

        W = np.diag(weights)

        # Joint-space damping/regularization
        lam = 1e-2
        H = np.eye(J.shape[1])

        A = J.T @ W @ J + (lam**2) * H
        b = J.T @ W @ v
        return np.linalg.solve(A, b)

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
