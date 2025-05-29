import torch
from isaaclab.envs import ManagerBasedEnv


def root_com_pos_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root center of mass position in world frame."""
    return env.scene['robot'].data.root_com_pos_w


def root_com_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root center of mass position in body frame."""
    return env.scene['robot'].data.root_com_pos_w - env.scene.env_origins


def root_com_quat_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root center of mass quaternion in world frame."""
    return env.scene['robot'].data.root_com_quat_w


def root_com_vel_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root center of mass velocity in world frame."""
    return env.scene['robot'].data.root_com_vel_w


def root_com_state_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root center of mass state in world frame."""
    return env.scene['robot'].data.root_com_state_w
