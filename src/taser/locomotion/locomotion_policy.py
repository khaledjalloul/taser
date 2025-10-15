from pathlib import Path

import numpy as np
import onnxruntime as ort

from taser.common.datatypes import TaserJointState
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
        joint_positions: TaserJointState,
        joint_velocities: TaserJointState,
        base_quaternion_w: np.ndarray,
        base_linear_velocity_b: np.ndarray,
        base_angular_velocity_b: np.ndarray,
        base_target_planar_velocity_b: np.ndarray,
    ) -> np.ndarray:
        if (
            np.all(base_target_planar_velocity_b == 0)
            and np.all(joint_velocities.wheels < 5)
            and np.all(np.abs(base_linear_velocity_b) < 0.5)
            and np.all(np.abs(base_angular_velocity_b) < 0.5)
        ):
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
