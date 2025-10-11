import numpy as np

from taser.common.datatypes import Pose, TaserJointState
from taser.manipulation import ManipulationKinematics


class GraspController:
    def __init__(self):
        self._left_arm = ManipulationKinematics(arm="left")
        self._right_arm = ManipulationKinematics(arm="right")

        self._home_pos_left = self._left_arm.get_eef_position(q=TaserJointState())
        self._home_pos_right = self._right_arm.get_eef_position(q=TaserJointState())

        self._target_pos_b = None
        self._q_target_left = None
        self._q_target_right = None

        self._kp = 0.5

    def set_target(self, target_position_b: Pose | None):
        self._target_pos_b = target_position_b

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

    def step(self, q: TaserJointState) -> TaserJointState:
        if self._target_pos_b is None:
            return TaserJointState()

        left_pos_b = self._left_arm.get_eef_position(q)
        right_pos_b = self._right_arm.get_eef_position(q)

        is_x_aligned = (
            np.abs(self._target_pos_b.x - left_pos_b.x) < 0.15
            and np.abs(self._target_pos_b.x - right_pos_b.x) < 0.15
        )
        is_z_aligned = (
            np.abs(self._target_pos_b.z - left_pos_b.z) < 0.15
            and np.abs(self._target_pos_b.z - right_pos_b.z) < 0.15
        )

        if not is_x_aligned or not is_z_aligned:
            dq_left = self._kp * (self._q_target_left - q.left_arm)
            dq_right = self._kp * (self._q_target_right - q.right_arm)

            return TaserJointState(left_arm=dq_left, right_arm=dq_right)

        v_desired_left = np.array([0.0, -0.2, 0.0])
        v_desired_right = np.array([0.0, 0.2, 0.0])

        if np.abs(left_pos_b.y) < 0.2:
            v_desired_left = np.zeros(3)
        if np.abs(right_pos_b.y) < 0.2:
            v_desired_right = np.zeros(3)

        dq_left = self._left_arm.get_dq(v_lin=v_desired_left, q=q)
        dq_right = self._right_arm.get_dq(v_lin=v_desired_right, q=q)

        return TaserJointState(left_arm=dq_left, right_arm=dq_right)
