import isaaclab.envs.mdp as mdp
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=10.0)

    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-50.0)

    # (3) Tilt penalty
    tilt = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-30.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
