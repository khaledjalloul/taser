from pathlib import Path

import numpy as np
import torch

from taser.isaaclab.rl.custom import ActorCritic
from taser.locomotion import __file__ as locomotion_path

MODEL_PATH = Path(locomotion_path).parent / "models" / "balance.pth"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
OBS_DIM = 29
ACT_DIM = 2


class BalancePolicy:
    def __init__(self):
        self._model = ActorCritic(obs_dim=OBS_DIM, act_dim=ACT_DIM).to(DEVICE)
        self._model.load(MODEL_PATH)
        self._model.eval()

    def step(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_position: np.ndarray,
        base_quaternion: np.ndarray,
        base_velocity: np.ndarray,
    ) -> np.ndarray:
        proprio_obs = np.concatenate(
            (
                joint_positions,
                joint_velocities,
                base_quaternion,
                base_velocity,
            ),
            axis=-1,
        )

        policy_obs = np.concatenate(
            (base_position,),
            axis=-1,
        )

        obs_dict = {
            "proprio": torch.tensor(proprio_obs, dtype=torch.float32, device=DEVICE),
            "policy": torch.tensor(policy_obs, dtype=torch.float32, device=DEVICE),
        }

        with torch.inference_mode():
            action_dist, _ = self._model(obs_dict)
            action = action_dist.mean.cpu().numpy().flatten()

        action = action * 20.0 / np.pi

        return action
