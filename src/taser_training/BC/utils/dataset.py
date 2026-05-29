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

    indices: ClassVar[dict[str, slice]] = {
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
    def __init__(
        self, file_path: str, action_chunk_size: int, obs_history_len: int = 1
    ):
        super().__init__()
        self.action_chunk_size = action_chunk_size
        self.obs_history_len = obs_history_len

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
                # actions = ep_grp["actions"][:]
                actions = obs[:, GPTEpisode.indices["joint_positions"]]
                actions = np.roll(actions, shift=-1, axis=0)
                actions[-1, :] = actions[-2, :]

                self.obs.append(torch.from_numpy(obs).float())
                self.actions.append(torch.from_numpy(actions).float())

        # Compute normalization stats
        all_obs = torch.cat(self.obs, dim=0)
        self.obs_mean = all_obs.mean(dim=0)
        self.obs_std = all_obs.std(dim=0)
        self.obs_std[self.obs_std < 1e-6] = 1.0

        all_actions = torch.cat(self.actions, dim=0)
        # self.action_max = torch.quantile(all_actions.abs(), 0.98, dim=0)
        self.action_mean = all_actions.mean(dim=0)
        self.action_std = all_actions.std(dim=0)
        self.action_std[self.action_std < 1e-6] = 1.0

    def __len__(self) -> int:
        return len(self.obs)

    def __getitem__(self, index: int):
        obs = self.obs[index]  # T, obs_dim
        obs_padded = F.pad(obs, (0, 0, self.obs_history_len - 1, 0))
        actions = self.actions[index]
        T = obs.shape[0]

        start_idx = torch.randint(0, T, (1,)).item()
        end_idx = start_idx + self.obs_history_len

        o = obs_padded[start_idx:end_idx]

        # Get actions with extra steps for chunking
        chunk_end_idx = end_idx + self.action_chunk_size - 1

        a = actions[start_idx:chunk_end_idx]
        eef_pos = obs[start_idx:chunk_end_idx, GPTEpisode.indices["eef_positions"]]

        if chunk_end_idx > T:
            pad_len = chunk_end_idx - T
            a = F.pad(a, (0, 0, 0, pad_len))
            eef_pos = F.pad(eef_pos, (0, 0, 0, pad_len))

        a = (a - self.action_mean) / self.action_std

        return {"observations": o, "actions": a, "eef_pos": eef_pos}

    def get_full_episode(self, index: int):
        return {"observations": self.obs[index], "actions": self.actions[index]}
