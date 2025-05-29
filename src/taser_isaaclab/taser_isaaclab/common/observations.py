import torch
import isaaclab.envs.mdp as mdp
from isaaclab.assets import Articulation
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.utils import configclass


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel_rel = ObservationTermCfg(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


def concat_obs(obs: dict, robot: Articulation) -> torch.Tensor:
    """Concatenate the observations into a single tensor."""
    policy_obs = obs["policy"]
    com_state_w = robot.data.root_com_state_w

    obs = torch.cat((policy_obs, com_state_w), dim=-1)
    obs = torch.nan_to_num(obs, nan=0.0)

    return obs
