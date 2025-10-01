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
        base_position: np.ndarray,
        base_quaternion: np.ndarray,
        base_linear_velocity: np.ndarray,
        base_angular_velocity: np.ndarray,
        base_target_planar_velocity: np.ndarray,
    ) -> np.ndarray:
        if np.all(base_target_planar_velocity == 0):
            policy = self._balance_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions,
                    joint_velocities,
                    base_quaternion,
                    base_linear_velocity,
                    base_angular_velocity,
                    # Policy
                    base_position,
                ),
                axis=-1,
            )
        else:
            policy = self._track_velocity_policy
            obs = np.concatenate(
                (
                    # Proprio
                    joint_positions,
                    joint_velocities,
                    base_quaternion,
                    base_linear_velocity,
                    base_angular_velocity,
                    # Policy
                    base_target_planar_velocity,
                ),
                axis=-1,
            )

        action = policy.run(input_feed={"obs": obs}, output_names=["action"])[0]
        action = action * 3.0

        return action
