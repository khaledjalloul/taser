import numpy as np

from taser.common.datatypes import Pose, TaserJointState
from taser.manipulation import ManipulationKinematics

KP = 2.0
Y_VELOCITY = 0.2
Z_VELOCITY = 0.4

XZ_THRESHOLD = 0.05
Y_THRESHOLD = 0.15
HEIGHT_THRESHOLD = 0.4


def wrap_angle(angle: float) -> float:
    return np.arctan2(np.sin(angle), np.cos(angle))


class PickController:
    def __init__(self):
        self._left_arm = ManipulationKinematics(arm="left")
        self._right_arm = ManipulationKinematics(arm="right")

        self._home_pos_left = self._left_arm.get_eef_position(q=TaserJointState())
        self._home_pos_right = self._right_arm.get_eef_position(q=TaserJointState())

        self.reset()

    def reset(self):
        self._target_pos_left_b = self._home_pos_left
        self._target_pos_right_b = self._home_pos_right

        self._q_target_left = self._left_arm.get_q(self._target_pos_left_b)
        self._q_target_right = self._right_arm.get_q(self._target_pos_right_b)

        self._resetting = True

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

        self._resetting = False

    def step(self, q: TaserJointState) -> tuple[TaserJointState, bool]:
        left_pos_b = self._left_arm.get_eef_position(q)
        right_pos_b = self._right_arm.get_eef_position(q)

        if not (
            np.abs(self._target_pos_left_b.x - left_pos_b.x) < XZ_THRESHOLD
            and np.abs(self._target_pos_right_b.x - right_pos_b.x) < XZ_THRESHOLD
            and np.abs(self._target_pos_left_b.z - left_pos_b.z) < XZ_THRESHOLD
            and np.abs(self._target_pos_right_b.z - right_pos_b.z) < XZ_THRESHOLD
        ):
            dq_left = KP * wrap_angle(self._q_target_left - q.left_arm)
            dq_right = KP * wrap_angle(self._q_target_right - q.right_arm)
            return TaserJointState(left_arm=dq_left, right_arm=dq_right), False

        if self._resetting:
            return TaserJointState(), True

        if not (
            np.abs(left_pos_b.y) < Y_THRESHOLD and np.abs(right_pos_b.y) < Y_THRESHOLD
        ):
            dq_left = self._left_arm.get_dq(
                v=np.array([0, -Y_VELOCITY, 0, 0, 0, 0]),
                weights=np.array([0.5, 1, 0.5, 0, 0, 0]),
                q=q,
            )
            dq_right = self._right_arm.get_dq(
                v=np.array([0, Y_VELOCITY, 0, 0, 0, 0]),
                weights=np.array([0.5, 1, 0.5, 0, 0, 0]),
                q=q,
            )
            return TaserJointState(left_arm=dq_left, right_arm=dq_right), False

        if not (left_pos_b.z > HEIGHT_THRESHOLD and right_pos_b.z > HEIGHT_THRESHOLD):
            dq_left = self._left_arm.get_dq(
                v=np.array([0, 0, Z_VELOCITY, 0, 0, 0]),
                weights=np.array([0.5, 0.5, 1, 0, 0, 0]),
                q=q,
            )
            dq_right = self._right_arm.get_dq(
                v=np.array([0, 0, Z_VELOCITY, 0, 0, 0]),
                weights=np.array([0.5, 0.5, 1, 0, 0, 0]),
                q=q,
            )
            return TaserJointState(left_arm=dq_left, right_arm=dq_right), False

        return TaserJointState(), True
