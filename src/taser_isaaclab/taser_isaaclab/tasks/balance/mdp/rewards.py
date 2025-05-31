import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from taser_isaaclab.common.obs_utils import base_pos_b, base_vel_w


# TODO: Try to change this to a reward with gaussian function
def position_error(env: ManagerBasedEnv):
    """Get the distances from the robot's position to the environment's origin."""
    return torch.clamp(torch.linalg.vector_norm(base_pos_b(env)[:, :2], dim=-1), max=1.0)


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

    # Tilt penalty
    tilt = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # Position error penalty: Keep x and y close to 0
    pos = RewardTermCfg(func=position_error, weight=-3.0)

    # Spinning penalty
    spin = RewardTermCfg(func=spinning_velocity, weight=-2.0)
