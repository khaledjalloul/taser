import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from taser.isaaclab.common.obs_utils import base_vel_w, root_planar_vel_b


def velocity_tracking_reward(env: ManagerBasedEnv, std: float):
    """Get the reward for tracking the target velocity."""
    planar_vel = root_planar_vel_b(env)

    lin_vel_reward = torch.exp(
        -((planar_vel[:, 0] - env.target_vel_b[:, 0]) ** 2) / (2 * std**2)
    )
    ang_vel_reward = torch.exp(
        -((planar_vel[:, 1] - env.target_vel_b[:, 1]) ** 2) / (2 * std**2)
    )

    return lin_vel_reward + ang_vel_reward


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

    vel_reward_1 = RewardTermCfg(
        func=velocity_tracking_reward, weight=0.5, params={"std": 20.0}
    )

    vel_reward_2 = RewardTermCfg(
        func=velocity_tracking_reward, weight=0.5, params={"std": 10.0}
    )

    vel_reward_3 = RewardTermCfg(
        func=velocity_tracking_reward, weight=0.5, params={"std": 1.0}
    )

    vel_reward_4 = RewardTermCfg(
        func=velocity_tracking_reward, weight=0.5, params={"std": 0.1}
    )
