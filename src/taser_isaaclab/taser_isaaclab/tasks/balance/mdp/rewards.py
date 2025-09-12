import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass
from taser_isaaclab.common.obs_utils import base_pos_b, base_vel_w


def position_reward(env: ManagerBasedEnv, std: float = 1.0):
    """Get the distances from the robot's position to the environment's origin."""
    return torch.exp(
        -(torch.linalg.vector_norm(base_pos_b(env)[:, :2], dim=-1) ** 2) / (2 * std**2)
    )


def spinning_velocity(env: ManagerBasedEnv):
    """Get the spinning velocity (z-axis) of the robot."""
    return torch.clamp(torch.abs(base_vel_w(env)[:, 5]), max=0.5)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    alive_reward = RewardTermCfg(func=mdp.is_alive, weight=1.0)

    termination_penalty = RewardTermCfg(func=mdp.is_terminated, weight=-10.0)

    tilt_penalty = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    pos_reward = RewardTermCfg(func=position_reward, weight=3.0)

    spin_penalty = RewardTermCfg(func=spinning_velocity, weight=-2.0)
