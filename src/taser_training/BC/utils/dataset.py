from dataclasses import dataclass, field
from typing import ClassVar

import h5py
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

OBS_DIM = 24  # 6 jp, 6 jv, 6 eef, 6 target
ACTION_DIM = 6  # 6 joint velocities


@dataclass
class GPTEpisode:
    observations: np.ndarray = field(default_factory=lambda: np.empty((0, OBS_DIM)))
    actions: np.ndarray = field(default_factory=lambda: np.empty((0, ACTION_DIM)))

    indices: ClassVar[dict] = {
        "joint_positions": slice(0, 6),
        "joint_velocities": slice(6, 12),
        "eef_positions": slice(12, 18),
        "target_positions": slice(18, 24),
    }

    def add_observation(self, observation: np.ndarray) -> None:
        self.observations = np.vstack([self.observations, observation])

    def add_action(self, action: np.ndarray) -> None:
        self.actions = np.vstack([self.actions, action])

    def save_to_h5(
        self,
        file_path: str,
        episode_id: str,
    ):
        with h5py.File(file_path, "a") as f:
            if "episodes" not in f:
                episodes_group = f.create_group("episodes")
            else:
                episodes_group = f["episodes"]

            if episode_id in episodes_group:
                raise ValueError(f"Episode {episode_id} already exists in {file_path}")

            ep_grp = episodes_group.create_group(episode_id)

            ep_grp.create_dataset("observations", data=self.observations)
            ep_grp.create_dataset("actions", data=self.actions)

    @staticmethod
    def collect_observation(
        joint_positions: np.ndarray,  # (T, num_joints)
        joint_velocities: np.ndarray,  # (T, num_joints)
        eef_positions: np.ndarray,  # (T, 6)
        target_positions: np.ndarray,  # (T, 6)
    ) -> np.ndarray:
        return np.concatenate(
            [
                joint_positions,
                joint_velocities,
                eef_positions,
                target_positions,
            ],
            axis=-1,
        )

    @staticmethod
    def from_h5(file_path: str, episode_id: str) -> "GPTEpisode":
        with h5py.File(file_path, "r") as f:
            if "episodes" not in f or episode_id not in f["episodes"]:
                raise ValueError(f"Episode {episode_id} not found in {file_path}")

            ep_grp = f["episodes"][episode_id]
            obs = ep_grp["observations"]
            actions = ep_grp["actions"]

            return GPTEpisode(observations=obs[:], actions=actions[:])


class GPTDataset(Dataset):
    def __init__(self, file_path: str, action_chunk_size: int):
        super().__init__()
        self.action_chunk_size = action_chunk_size

        self.obs = []
        self.actions = []

        with h5py.File(file_path, "r") as f:
            if "episodes" not in f:
                raise ValueError(f"No episodes found in {file_path}")

            episodes_group = f["episodes"]
            episode_ids = sorted(episodes_group.keys())

            for ep_id in episode_ids:
                ep_grp = episodes_group[ep_id]
                obs = ep_grp["observations"][:]
                actions = ep_grp["actions"][:]

                self.obs.append(torch.from_numpy(obs).float())
                self.actions.append(torch.from_numpy(actions).float())

    def __len__(self) -> int:
        return len(self.obs)

    def __getitem__(self, index: int):
        obs = self.obs[index]
        actions = self.actions[index]
        T = obs.shape[0]

        start_idx = torch.randint(0, T, (1,)).item()
        o = obs[start_idx]

        # Get actions with extra steps for chunking
        chunk_end = start_idx + self.action_chunk_size
        a = actions[start_idx:chunk_end]
        eef_pos = obs[start_idx:chunk_end, GPTEpisode.indices["eef_positions"]]

        # Handle end padding if we requested actions past the end of the trajectory
        if chunk_end > T:
            end_pad = chunk_end - T
            a = F.pad(a, (0, 0, 0, end_pad))
            eef_pos = F.pad(eef_pos, (0, 0, 0, end_pad))

        return {"observations": o, "actions": a, "eef_pos": eef_pos}

    def get_full_episode(self, index: int):
        return {"observations": self.obs[index], "actions": self.actions[index]}
