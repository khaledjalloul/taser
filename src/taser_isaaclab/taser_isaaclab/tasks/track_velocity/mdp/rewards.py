import torch
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from taser_isaaclab.common.obs_utils import base_vel_b


def velocity_error(env: ManagerBasedEnv):
    """Get the velocity error between the robot's velocity and the target velocity."""
    base_vel = base_vel_b(env)[:, [0, 5]]  # [x, omega]
    return torch.linalg.vector_norm(base_vel - env.target_vel_b, dim=-1)


def spinning_velocity(env: ManagerBasedEnv):
    """Get the spinning velocity (z-axis) of the robot."""
    return torch.clamp(torch.abs(base_vel_b(env)[:, 5]), max=0.5)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Constant running reward
    alive = RewardTermCfg(func=mdp.is_alive, weight=1.0)

    # Termination penalty
    terminating = RewardTermCfg(func=mdp.is_terminated, weight=-5.0)

    # Velocity error penalty
    vel = RewardTermCfg(func=velocity_error, weight=-5.0)

    # Tilt penalty
    tilt = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-3.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # Spinning penalty
    spin = RewardTermCfg(func=spinning_velocity, weight=-1.0)
