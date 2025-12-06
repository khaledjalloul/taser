from dataclasses import dataclass
from typing import ClassVar

import h5py
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

from taser_training.BC.model.gpt_config import GPTConfig


@dataclass
class GPTEpisode:
    indices: ClassVar["GPTEpisode"]
    joint_positions: np.ndarray  # (T, num_joints)
    joint_velocities: np.ndarray  # (T, num_joints)
    eef_positions: np.ndarray  # (T, 6)
    target_positions: np.ndarray  # (T, 6)
    actions: np.ndarray  # (T, num_joints)


GPTEpisode.indices = GPTEpisode(
    joint_positions=[0, 1, 2, 3, 4, 5],
    joint_velocities=[6, 7, 8, 9, 10, 11],
    eef_positions=[12, 13, 14, 15, 16, 17],
    target_positions=[18, 19, 20, 21, 22, 23],
    actions=[0, 1, 2, 3, 4, 5, 6],
)


def save_episode_to_h5(
    file_path: str,
    episode_id: str,
    episode: GPTEpisode,
):
    """
    Append a single episode to dataset.h5 under /episodes/<episode_id>.
    Creates the file and groups if they don't exist.
    """
    with h5py.File(
        file_path, "a"
    ) as f:  # "a" = create if not exists, else open for read/write
        # Create top-level 'episodes' group if missing
        if "episodes" not in f:
            episodes_group = f.create_group("episodes")
        else:
            episodes_group = f["episodes"]

        # Make sure we don't overwrite an existing episode accidentally
        if episode_id in episodes_group:
            raise ValueError(f"Episode {episode_id} already exists in {file_path}")

        ep_grp = episodes_group.create_group(episode_id)

        # Create observations group
        obs_grp = ep_grp.create_group("observations")
        obs_grp.create_dataset("joint_positions", data=episode.joint_positions)
        obs_grp.create_dataset("joint_velocities", data=episode.joint_velocities)
        obs_grp.create_dataset("eef_positions", data=episode.eef_positions)
        obs_grp.create_dataset("target_positions", data=episode.target_positions)

        # Create actions dataset
        ep_grp.create_dataset("actions", data=episode.actions)


def load_episode_from_h5(file_path: str, episode_id: str) -> GPTEpisode:
    """
    Load a single episode from dataset.h5 under /episodes/<episode_id>.
    """
    with h5py.File(file_path, "r") as f:
        if "episodes" not in f or episode_id not in f["episodes"]:
            raise ValueError(f"Episode {episode_id} not found in {file_path}")

        ep_grp = f["episodes"][episode_id]
        obs_grp = ep_grp["observations"]

        joint_positions = obs_grp["joint_positions"][:]
        joint_velocities = obs_grp["joint_velocities"][:]
        eef_positions = obs_grp["eef_positions"][:]
        target_positions = obs_grp["target_positions"][:]
        actions = ep_grp["actions"][:]

        return GPTEpisode(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            eef_positions=eef_positions,
            target_positions=target_positions,
            actions=actions,
        )


def load_dataset_from_h5(file_path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Load the entire dataset from h5 file.
    Returns:
        observations: (Total_T, Obs_Dim)
        actions: (Total_T, Action_Dim)
    """
    all_obs = []
    all_actions = []

    with h5py.File(file_path, "r") as f:
        if "episodes" not in f:
            raise ValueError(f"No episodes found in {file_path}")

        episodes_group = f["episodes"]
        # Sort episode keys to ensure deterministic order
        episode_ids = sorted(episodes_group.keys())

        for ep_id in episode_ids:
            ep_grp = episodes_group[ep_id]
            obs_grp = ep_grp["observations"]

            # Load components
            joint_positions = obs_grp["joint_positions"][:]
            joint_velocities = obs_grp["joint_velocities"][:]
            eef_positions = obs_grp["eef_positions"][:]
            target_positions = obs_grp["target_positions"][:]

            actions = ep_grp["actions"][:]

            # Concatenate observations: joint_pos, joint_vel, eef_pos, target_pos
            obs = np.concatenate(
                [
                    joint_positions,
                    joint_velocities,
                    eef_positions,
                    target_positions,
                ],
                axis=-1,
            )

            all_obs.append(obs)
            all_actions.append(actions)

    # Concatenate all episodes
    return np.concatenate(all_obs, axis=0), np.concatenate(all_actions, axis=0)


class GPTDataset(Dataset):
    def __init__(
        self,
        file_path: str,
        gpt_cfg: GPTConfig,
        device,
    ):
        super().__init__()
        self.gpt_cfg = gpt_cfg

        obs_np, actions_np = load_dataset_from_h5(file_path)
        self.obs = torch.tensor(obs_np, dtype=torch.float32, device=device)
        self.actions = torch.tensor(actions_np, dtype=torch.float32, device=device)

    def __len__(self) -> int:
        return len(self.obs)

    def __getitem__(self, index: int):
        obs = self.obs[index]
        actions = self.actions[index]
        T = obs.shape[0]

        start_idx = torch.randint(0, T, (1,)).item()
        o = obs[start_idx]

        # Get actions with extra steps for chunking
        chunk_end = start_idx + self.gpt_cfg.action_chunk_size - 1
        a = actions[start_idx:chunk_end]
        eef_pos = obs[start_idx:chunk_end, GPTEpisode.indices.eef_positions]

        # Handle end padding if we requested actions past the end of the trajectory
        if chunk_end > T:
            end_pad = chunk_end - T
            a = F.pad(a, (0, 0, 0, end_pad))
            eef_pos = F.pad(eef_pos, (0, 0, 0, end_pad))

        return {"observations": o, "actions": a, "eef_pos": eef_pos}
