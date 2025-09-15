from isaaclab.envs import mdp
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg
from isaaclab.utils import configclass

from taser.isaaclab.common.obs_utils import base_quat_w, base_vel_w, root_planar_vel_b


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Proprioceptive information
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)

        # Base orientation useful for balancing
        base_quat_w = ObservationTermCfg(func=base_quat_w)

        # Base velocity useful for balancing
        base_vel_w = ObservationTermCfg(func=base_vel_w)

        # Root planar velocity useful for tracking
        root_planar_vel_b = ObservationTermCfg(func=root_planar_vel_b)

        # Target planar velocity
        target_vel_b = ObservationTermCfg(func=lambda env: env.target_vel_b)

        # Velocity error
        vel_error = ObservationTermCfg(
            func=lambda env: root_planar_vel_b(env) - env.target_vel_b
        )

    # observation groups
    policy: PolicyCfg = PolicyCfg()
