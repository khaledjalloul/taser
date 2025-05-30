import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.utils.math import quat_conjugate, quat_mul


def _get_body_index(env: ManagerBasedEnv, body_name: str) -> int:
    return env.scene['robot'].data.body_names.index(body_name)


# World frame


def base_pos_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body position in the world frame."""
    return env.scene['robot'].data.body_pos_w[:, _get_body_index(env, 'base')]


def base_quat_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body quaternion in the world frame."""
    return env.scene['robot'].data.body_quat_w[:, _get_body_index(env, 'base')]


def base_lin_vel_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body linear velocity in the world frame."""
    return env.scene['robot'].data.body_lin_vel_w[:, _get_body_index(env, 'base_wrapper')]


def base_ang_vel_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body angular velocity in the world frame."""
    return env.scene['robot'].data.body_ang_vel_w[:, _get_body_index(env, 'base_wrapper')]


def base_vel_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body velocity in the world frame."""
    return env.scene['robot'].data.body_vel_w[:, _get_body_index(env, 'base_wrapper')]


def base_state_w(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body state (pose and velocity) in the world frame."""
    return env.scene['robot'].data.body_state_w[:, _get_body_index(env, 'base')]


# Environment frame


def base_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body position in the environment frame."""
    return base_pos_w(env) - env.scene.env_origins


# Robot base frame


def base_lin_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body linear velocity in the robot base frame."""
    q = base_quat_w(env)
    q_conj = quat_conjugate(q)

    w = torch.zeros(env.scene.num_envs, 1).to(env.device)
    lin_vel_quat_w = torch.cat([w, base_lin_vel_w(env)], dim=-1)
    lin_vel_quat_b = quat_mul(quat_mul(q_conj, lin_vel_quat_w), q)

    return lin_vel_quat_b[:, 1:4]


def base_ang_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body angular velocity in the robot base frame."""
    q = base_quat_w(env)
    q_conj = quat_conjugate(q)

    w = torch.zeros(env.scene.num_envs, 1).to(env.device)
    ang_vel_quat_w = torch.cat([w, base_ang_vel_w(env)], dim=-1)
    ang_vel_quat_b = quat_mul(quat_mul(q_conj, ang_vel_quat_w), q)

    return ang_vel_quat_b[:, 1:4]


def base_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the base body velocity in the robot base frame."""
    return torch.cat([base_lin_vel_b(env), base_ang_vel_b(env)], dim=-1)


# Robot root frame


def root_lin_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """
    Get the root body linear velocity in the robot root frame."""
    return env.scene["robot"].data.root_lin_vel_b


def root_ang_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root body angular velocity in the robot root frame."""
    return env.scene["robot"].data.root_ang_vel_b


def root_vel_b(env: ManagerBasedEnv) -> torch.Tensor:
    """Get the root body velocity in the robot root frame."""
    return torch.cat([root_lin_vel_b(env), root_ang_vel_b(env)], dim=-1)
