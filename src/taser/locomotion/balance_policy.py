from pathlib import Path

import numpy as np
import onnxruntime as ort

from taser.locomotion import __file__ as locomotion_path

MODEL_PATH = Path(locomotion_path).parent / "models" / "balance.onnx"


class BalancePolicy:
    def __init__(self):
        self.policy = ort.InferenceSession(
            MODEL_PATH, providers=["CPUExecutionProvider"]
        )

    def step(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_position: np.ndarray,
        base_quaternion: np.ndarray,
        base_velocity: np.ndarray,
    ) -> np.ndarray:
        obs = np.concatenate(
            (
                # Proprio
                joint_positions,
                joint_velocities,
                base_quaternion,
                base_velocity,
                # Policy
                base_position,
            ),
            axis=-1,
        )

        action = self.policy.run(input_feed={"obs": obs}, output_names=["action"])[0]
        action = action * 20.0 / np.pi

        return action
