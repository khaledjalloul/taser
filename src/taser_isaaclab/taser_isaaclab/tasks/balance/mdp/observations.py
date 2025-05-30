import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.utils import configclass

from taser_isaaclab.common.obs_utils import base_pos_b, base_quat_w, root_vel_b


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)
        base_pos_b = ObservationTermCfg(func=base_pos_b)
        base_quat_w = ObservationTermCfg(func=base_quat_w)
        root_vel_b = ObservationTermCfg(func=root_vel_b)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
