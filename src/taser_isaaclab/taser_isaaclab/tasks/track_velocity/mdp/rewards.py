import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from taser_isaaclab.common.obs_utils import base_vel_w, root_planar_vel_b


def velocity_tracking_reward(env: ManagerBasedEnv):
    """Get the reward for tracking the target velocity."""
    lin_std = 1.0
    ang_std = 0.2
    planar_vel = root_planar_vel_b(env)
    lin_reward = torch.exp(-torch.abs(planar_vel[:, 0] - env.target_vel_b[:, 0]) ** 2 / lin_std ** 2)
    ang_reward = torch.exp(-torch.abs(planar_vel[:, 1] - env.target_vel_b[:, 1]) ** 2 / ang_std ** 2)
    return lin_reward + ang_reward


def spinning_velocity(env: ManagerBasedEnv):
    """Get the spinning velocity (z-axis) of the robot."""
    return torch.clamp(torch.abs(base_vel_w(env)[:, 5]), max=0.5)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Constant running reward
    alive = RewardTermCfg(func=mdp.is_alive, weight=1.0)

    # Termination penalty
    terminating = RewardTermCfg(func=mdp.is_terminated, weight=-5.0)

    # Velocity tracking reward
    vel = RewardTermCfg(func=velocity_tracking_reward, weight=5.0)

    # Tilt penalty
    tilt = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # Spinning penalty
    spin = RewardTermCfg(func=spinning_velocity, weight=-1.0)
