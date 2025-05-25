from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass

from taser_isaaclab.common import ActionsCfg, ObservationsCfg
from taser_isaaclab.tasks.moving.mdp import EventsCfg, RewardsCfg, TerminationsCfg
from taser_isaaclab.tasks.moving.scene import SceneCfg


@configclass
class TaserEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the TASER robot environment."""

    scene = SceneCfg(num_envs=1, env_spacing=10)

    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventsCfg()

    rewards = RewardsCfg()
    terminations = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 0.01
        self.sim.render_interval = 5
