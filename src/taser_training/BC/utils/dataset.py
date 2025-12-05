import h5py
import numpy as np

def save_episode_to_h5(
    file_path: str,
    episode_id: str,
    obs_qpos: np.ndarray,   # (T, nq)
    obs_qvel: np.ndarray,   # (T, nq)
    actions: np.ndarray,    # (T, na)
    reward: np.ndarray,     # (T,)
    done: np.ndarray,       # (T,)
    t: np.ndarray           # (T,) timestamps or step indices
):
    """
    Append a single episode to dataset.h5 under /episodes/<episode_id>.
    Creates the file and groups if they don't exist.
    """
    with h5py.File(file_path, "a") as f:  # "a" = create if not exists, else open for read/write
        # Create top-level 'episodes' group if missing
        if "episodes" not in f:
            episodes_group = f.create_group("episodes")
        else:
            episodes_group = f["episodes"]

        # Make sure we don't overwrite an existing episode accidentally
        if episode_id in episodes_group:
            raise ValueError(f"Episode {episode_id} already exists in {file_path}")

        ep_grp = episodes_group.create_group(episode_id)

        # Create datasets inside this episode group
        ep_grp.create_dataset("obs_qpos", data=obs_qpos)
        ep_grp.create_dataset("obs_qvel", data=obs_qvel)
        ep_grp.create_dataset("actions", data=actions)
        ep_grp.create_dataset("reward", data=reward)
        ep_grp.create_dataset("done", data=done)
        ep_grp.create_dataset("t", data=t)
