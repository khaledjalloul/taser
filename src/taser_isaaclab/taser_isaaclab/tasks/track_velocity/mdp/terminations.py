import numpy as np
import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass
from taser_isaaclab.common.obs_utils import root_planar_vel_b


def lin_vel_error_too_large(env: ManagerBasedEnv):
    return (
        torch.abs(env.target_vel_b[:, 0] - root_planar_vel_b(env)[:, 0])
        > env._max_lin_vel * 1.2
    )


def ang_vel_error_too_large(env: ManagerBasedEnv):
    return (
        torch.abs(env.target_vel_b[:, 1] - root_planar_vel_b(env)[:, 1])
        > env._max_ang_vel * 1.2
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    robot_falling = TerminationTermCfg(
        func=mdp.bad_orientation,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "limit_angle": float(np.deg2rad(70.0)),
        },
    )

    lin_vel_error_too_large = TerminationTermCfg(
        func=lin_vel_error_too_large,
    )

    ang_vel_error_too_large = TerminationTermCfg(
        func=ang_vel_error_too_large,
    )
