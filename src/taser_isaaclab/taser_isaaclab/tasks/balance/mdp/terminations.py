import numpy as np
from isaaclab.envs import mdp
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # Robot falling
    robot_falling = TerminationTermCfg(
        func=mdp.bad_orientation,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "limit_angle": float(np.deg2rad(70.0)),
        },
    )
