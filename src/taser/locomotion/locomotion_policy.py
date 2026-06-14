from pathlib import Path

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

from taser.common.datatypes import TaserJointState
from taser.locomotion import __file__ as locomotion_path

MODELS_PATH = Path(locomotion_path).parent / "models"

V_MAX = 2.0
W_MAX = 2.0

LOCK_VELOCITY = np.array([-0.5, 7.0, -0.5, 7.0])
LOCK_POS = np.array([0.0, 0.0, 0.0, 0.0])
UNLOCK_POS = np.array([0.15, -np.deg2rad(90), 0.15, -np.deg2rad(90)])
LOCK_WAIT_TIME_S = 3.0

ARM_KP = 3.0


class LocomotionPolicy:
    def __init__(self):
        self._track_velocity_policy = ort.InferenceSession(
            MODELS_PATH / "track-velocity.onnx",
            providers=["CPUExecutionProvider"],
        )

        self.should_lock_counter = 0

    def lock(self, lock_joint_positions: np.ndarray) -> tuple[np.ndarray, bool]:
        lock_errors = lock_joint_positions - LOCK_POS
        is_locking = np.any(np.abs(lock_errors) > 0.01)
        return LOCK_VELOCITY, is_locking

    def unlock(self, lock_joint_positions: np.ndarray) -> tuple[np.ndarray, bool]:
        unlock_errors = lock_joint_positions - UNLOCK_POS
        is_locking = np.any(np.abs(unlock_errors) > 0.01)
        return -LOCK_VELOCITY, is_locking

    def step(
        self,
        joint_positions: TaserJointState,
        joint_velocities: TaserJointState,
        base_quaternion_w: np.ndarray,  # [w, x, y, z]
        base_linear_velocity_w: np.ndarray,
        base_angular_velocity_w: np.ndarray,
        base_target_planar_velocity_b: np.ndarray,
        dt: float,
    ) -> TaserJointState:
        R_BI = R.from_quat(base_quaternion_w, scalar_first=True)
        R_BI = R_BI.as_matrix().transpose()
        base_linear_velocity_b = np.matmul(R_BI, base_linear_velocity_w)
        base_angular_velocity_b = np.matmul(R_BI, base_angular_velocity_w)

        self.should_lock_counter += 1
        if np.any(base_target_planar_velocity_b != 0):
            self.should_lock_counter = 0
        should_lock = self.should_lock_counter * dt > LOCK_WAIT_TIME_S

        lock_fn = self.lock if should_lock else self.unlock
        lock_velocities, is_locking = lock_fn(
            lock_joint_positions=joint_positions.locks
        )

        obs = np.concatenate(
            (
                # Proprio
                joint_positions.to("isaac"),
                joint_velocities.to("isaac"),
                base_linear_velocity_b,
                base_angular_velocity_b,
                base_quaternion_w,
                # Policy
                base_target_planar_velocity_b,
            ),
            dtype=np.float32,
        )

        if should_lock and not is_locking:
            wheel_velocities = np.zeros(2, dtype=np.float32)
        else:
            wheel_velocities = self._track_velocity_policy.run(
                input_feed={"obs": obs.reshape(1, -1)},
                output_names=["action"],
            )[0][0]  # First action, first batch element
            wheel_velocities *= 20.0  # Action scale

        return TaserJointState(
            left_arm=-ARM_KP * joint_positions.left_arm,
            left_gripper=-ARM_KP * joint_positions.left_gripper,
            right_arm=-ARM_KP * joint_positions.right_arm,
            right_gripper=-ARM_KP * joint_positions.right_gripper,
            locks=lock_velocities,
            wheels=wheel_velocities,
        )

    @property
    def v_max(self) -> float:
        return V_MAX

    @property
    def w_max(self) -> float:
        return W_MAX
