from isaaclab.envs import mdp
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg
from isaaclab.utils import configclass

from taser.isaaclab.common.obs_utils import base_pos_b, base_quat_w, base_vel_w


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Proprioceptive information
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)

        # Base position in environment frame to stay close to the origin
        base_pos_b = ObservationTermCfg(func=base_pos_b)

        # Base orientation useful for balancing
        base_quat_w = ObservationTermCfg(func=base_quat_w)

        # Base velocity useful for balancing
        # NOTE: Using world frame instead of body frame since the former is more stable
        # and leads to better performance
        base_vel_w = ObservationTermCfg(func=base_vel_w)

    # observation groups
    policy: PolicyCfg = PolicyCfg()
