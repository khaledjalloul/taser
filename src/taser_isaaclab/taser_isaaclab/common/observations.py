import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.utils import configclass

from taser_isaaclab.common import root_com_pos_b, root_com_quat_w, root_com_vel_w


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel_rel = ObservationTermCfg(func=mdp.joint_vel_rel)
        root_com_pos_b = ObservationTermCfg(func=root_com_pos_b)
        root_com_quat_w = ObservationTermCfg(func=root_com_quat_w)
        root_com_vel_w = ObservationTermCfg(func=root_com_vel_w)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
