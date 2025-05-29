import torch
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from taser_isaaclab.common import root_com_pos_b, root_com_vel_w


def distance_to_origin(env: ManagerBasedEnv):
    """Get the distances from the robot's root center of mass to the origin."""
    return torch.clamp(torch.linalg.vector_norm(root_com_pos_b(env)[:, :2], dim=-1), max=1.0)


def spinning_velocity(env: ManagerBasedEnv):
    """Get the spinning velocity (z-axis) of the robot."""
    return torch.clamp(torch.abs(root_com_vel_w(env)[:, 5]), max=0.5)


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

    # Distance to origin penalty: Keep x and y close to 0
    pos = RewardTermCfg(func=distance_to_origin, weight=-3.0)

    # Spinning penalty
    spin = RewardTermCfg(func=spinning_velocity, weight=-2.0)
