from pathlib import Path

import numpy as np
import onnxruntime as ort

from taser.locomotion import __file__ as locomotion_path

MODELS_PATH = Path(locomotion_path).parent / "models"


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

    def step(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_position_w: np.ndarray,
        base_quaternion_w: np.ndarray,
        base_linear_velocity_b: np.ndarray,
        base_angular_velocity_b: np.ndarray,
        base_target_planar_velocity_b: np.ndarray,
    ) -> np.ndarray:
        if False:
            policy = self._balance_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions,
                    joint_velocities,
                    base_linear_velocity_b,
                    base_angular_velocity_b,
                    base_quaternion_w,
                ),
                dtype=np.float32,
            )
        else:
            policy = self._track_velocity_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions,
                    joint_velocities,
                    base_linear_velocity_b,
                    base_angular_velocity_b,
                    base_quaternion_w,
                    # Policy
                    base_target_planar_velocity_b,
                ),
                dtype=np.float32,
            )

        action = policy.run(
            input_feed={"obs": obs.reshape(1, -1)},
            output_names=["action"],
        )[0][0]  # First action, first batch element
        action = action * 20.0

        return action
