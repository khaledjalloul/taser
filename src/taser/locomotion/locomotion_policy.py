from pathlib import Path

import numpy as np
import onnxruntime as ort

from taser.common.datatypes import TaserJointState
from taser.locomotion import __file__ as locomotion_path

MODELS_PATH = Path(locomotion_path).parent / "models"

V_MAX = 3.0
W_MAX = 2.0


class LocomotionPolicy:
    def __init__(self):
        self._balance_policy = ort.InferenceSession(
            MODELS_PATH / "balance.onnx",
            providers=["CPUExecutionProvider"],
        )
        self._track_velocity_policy = ort.InferenceSession(
            MODELS_PATH / "track_velocity.onnx",
            providers=["CPUExecutionProvider"],
        )

        self._base_velocity_buffer = np.zeros((10, 10), dtype=np.float32)

    def step(
        self,
        joint_positions: TaserJointState,
        joint_velocities: TaserJointState,
        base_quaternion_w: np.ndarray,
        base_linear_velocity_b: np.ndarray,
        base_angular_velocity_b: np.ndarray,
        base_target_planar_velocity_b: np.ndarray,
    ) -> np.ndarray:
        self._base_velocity_buffer = np.roll(
            self._base_velocity_buffer,
            shift=-1,
            axis=0,
        )
        self._base_velocity_buffer[-1, :3] = base_linear_velocity_b
        self._base_velocity_buffer[-1, 3:6] = base_angular_velocity_b
        self._base_velocity_buffer[-1, 6:] = base_quaternion_w

        use_balance_policy = (
            np.all(base_target_planar_velocity_b == 0)
            and np.all(np.abs(self._base_velocity_buffer[:, :3]) < 0.2)
            and np.all(np.abs(self._base_velocity_buffer[:, 3:6]) < 0.5)
            and np.all(np.abs(self._base_velocity_buffer[:, 7:9]) < 0.02)
        )

        if use_balance_policy:
            policy = self._balance_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions.ordered_isaac,
                    joint_velocities.ordered_isaac,
                    base_linear_velocity_b,
                    base_angular_velocity_b,
                    base_quaternion_w,
                ),
                dtype=np.float32,
            )
            action_scale = 5.0
        else:
            policy = self._track_velocity_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions.ordered_isaac,
                    joint_velocities.ordered_isaac,
                    base_linear_velocity_b,
                    base_angular_velocity_b,
                    base_quaternion_w,
                    # Policy
                    base_target_planar_velocity_b,
                ),
                dtype=np.float32,
            )
            action_scale = 20.0

        action = policy.run(
            input_feed={"obs": obs.reshape(1, -1)},
            output_names=["action"],
        )[0][0]  # First action, first batch element
        action *= action_scale

        return action

    @property
    def v_max(self) -> float:
        return V_MAX

    @property
    def w_max(self) -> float:
        return W_MAX
