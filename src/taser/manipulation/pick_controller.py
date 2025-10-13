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


class PickController:
    def __init__(self):
        self._left_arm = ManipulationKinematics(arm="left")
        self._right_arm = ManipulationKinematics(arm="right")

        self._home_pos_left = self._left_arm.get_eef_position(q=TaserJointState())
        self._home_pos_right = self._right_arm.get_eef_position(q=TaserJointState())

        self._target_pos_b = None
        self._q_target_left = None
        self._q_target_right = None

        self._kp = 0.5
        self._stage = None

    def set_target(self, target_position_b: Pose | None):
        self._target_pos_b = target_position_b
        if self._target_pos_b is None:
            return

        target_pos_left_b = Pose(
            x=self._target_pos_b.x,
            y=self._home_pos_left.y,
            z=self._target_pos_b.z,
            # ry=-np.pi / 2,
        )
        target_pos_right_b = Pose(
            x=self._target_pos_b.x,
            y=self._home_pos_right.y,
            z=self._target_pos_b.z,
            # ry=-np.pi / 2,
        )

        self._q_target_left = self._left_arm.get_q(pose=target_pos_left_b)
        self._q_target_right = self._right_arm.get_q(pose=target_pos_right_b)

        self._stage = Stage.Align_XZ

    def step(self, q: TaserJointState, dq: TaserJointState) -> TaserJointState:
        if self._target_pos_b is None or self._stage is None:
            return TaserJointState()

        left_pos_b = self._left_arm.get_eef_position(q)
        right_pos_b = self._right_arm.get_eef_position(q)

        if self._stage == Stage.Align_XZ:
            is_x_left_aligned = np.abs(self._target_pos_b.x - left_pos_b.x) < 0.1
            is_x_right_aligned = np.abs(self._target_pos_b.x - right_pos_b.x) < 0.1

            is_z_left_aligned = np.abs(self._target_pos_b.z - left_pos_b.z) < 0.1
            is_z_right_aligned = np.abs(self._target_pos_b.z - right_pos_b.z) < 0.1

            if (
                is_x_left_aligned
                and is_x_right_aligned
                and is_z_left_aligned
                and is_z_right_aligned
            ):
                self._stage = Stage.Align_Y
            else:
                dq_left = self._kp * wrap_angle(self._q_target_left - q.left_arm)
                dq_right = self._kp * wrap_angle(self._q_target_right - q.right_arm)

        if self._stage == Stage.Align_Y:
            is_y_left_aligned = np.abs(left_pos_b.y) < 0.15
            is_y_right_aligned = np.abs(right_pos_b.y) < 0.15

            if is_y_left_aligned and is_y_right_aligned:
                self._stage = Stage.Lift
            else:
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

        if self._stage == Stage.Lift:
            if left_pos_b.z > 0.2 and right_pos_b.z > 0.2:
                self._stage = None

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

        # return TaserJointState(
        #     left_arm=np.clip(dq_left, -1.0, 1.0),
        #     right_arm=np.clip(dq_right, -1.0, 1.0),
        # )

        return TaserJointState(left_arm=dq_left, right_arm=dq_right)
