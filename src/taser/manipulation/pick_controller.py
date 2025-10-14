from enum import Enum

import numpy as np

from taser.common.datatypes import Pose, TaserJointState
from taser.manipulation import ManipulationKinematics


def wrap_angle(angle: float) -> float:
    return np.arctan2(np.sin(angle), np.cos(angle))


class Stage(Enum):
    Align_XZ = 0
    Align_Y = 1
    Lift = 2
    Reset = 3


class PickController:
    def __init__(self):
        self._left_arm = ManipulationKinematics(arm="left")
        self._right_arm = ManipulationKinematics(arm="right")

        self._home_pos_left = self._left_arm.get_eef_position(q=TaserJointState())
        self._home_pos_right = self._right_arm.get_eef_position(q=TaserJointState())

        self._kp = 1

        self.reset()

    def reset(self):
        self._target_pos_left_b = self._home_pos_left
        self._target_pos_right_b = self._home_pos_right

        self._q_target_left = self._left_arm.get_q(self._target_pos_left_b)
        self._q_target_right = self._right_arm.get_q(self._target_pos_right_b)

        self._stage = Stage.Reset

    def set_target(self, target_position_b: Pose):
        self._target_pos_left_b = Pose(
            x=target_position_b.x,
            y=self._home_pos_left.y,
            z=target_position_b.z,
        )
        self._target_pos_right_b = Pose(
            x=target_position_b.x,
            y=self._home_pos_right.y,
            z=target_position_b.z,
        )

        self._q_target_left = self._left_arm.get_q(
            pose=self._target_pos_left_b,
            q0=[-0.425, 0.0, -1.1],
        )
        self._q_target_right = self._right_arm.get_q(
            pose=self._target_pos_right_b,
            q0=[-0.425, 0.0, -1.1],
        )

        self._stage = Stage.Align_XZ

    def step(self, q: TaserJointState) -> tuple[TaserJointState, bool]:
        if self._stage is None:
            return TaserJointState(), True

        left_pos_b = self._left_arm.get_eef_position(q)
        right_pos_b = self._right_arm.get_eef_position(q)

        if self._stage in [Stage.Reset, Stage.Align_XZ]:
            dq_left = self._kp * wrap_angle(self._q_target_left - q.left_arm)
            dq_right = self._kp * wrap_angle(self._q_target_right - q.right_arm)

            if (
                np.abs(self._target_pos_left_b.x - left_pos_b.x) < 0.05
                and np.abs(self._target_pos_right_b.x - right_pos_b.x) < 0.05
                and np.abs(self._target_pos_left_b.z - left_pos_b.z) < 0.05
                and np.abs(self._target_pos_right_b.z - right_pos_b.z) < 0.05
            ):
                if self._stage == Stage.Reset:
                    self._stage = None
                else:
                    self._stage = Stage.Align_Y

        if self._stage == Stage.Align_Y:
            dq_left = self._left_arm.get_dq(
                v=np.array([0, -0.2, 0, 0, 0, 0]),
                weights=np.array([0.5, 1, 0.5, 0, 0, 0]),
                q=q,
            )
            dq_right = self._right_arm.get_dq(
                v=np.array([0, 0.2, 0, 0, 0, 0]),
                weights=np.array([0.5, 1, 0.5, 0, 0, 0]),
                q=q,
            )

            if np.abs(left_pos_b.y) < 0.15 and np.abs(right_pos_b.y) < 0.15:
                self._stage = Stage.Lift

        if self._stage == Stage.Lift:
            dq_left = self._left_arm.get_dq(
                v=np.array([0, 0, 0.2, 0, 0, 0]),
                weights=np.array([0.5, 0.5, 1, 0, 0, 0]),
                q=q,
            )
            dq_right = self._right_arm.get_dq(
                v=np.array([0, 0, 0.2, 0, 0, 0]),
                weights=np.array([0.5, 0.5, 1, 0, 0, 0]),
                q=q,
            )

            if left_pos_b.z > 0.2 and right_pos_b.z > 0.2:
                self._stage = None

        return TaserJointState(left_arm=dq_left, right_arm=dq_right), False
