import torch
from isaaclab.envs import ManagerBasedEnv


def _get_body_index(env: ManagerBasedEnv, body_name: str) -> int:
    return env.scene["robot"].data.body_names.index(body_name)


def base_pos_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body position in the world frame."""
    return env.scene["robot"].data.body_pos_w[:, _get_body_index(env, "base_link")]


def base_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body position in the environment frame."""
    return base_pos_w(env) - env.scene.env_origins


def base_quat_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body quaternion in the world frame."""
    return env.scene["robot"].data.body_quat_w[:, _get_body_index(env, "base_link")]


def base_vel_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body velocity in the world frame."""
    return env.scene["robot"].data.body_vel_w[:, _get_body_index(env, "base_link")]
