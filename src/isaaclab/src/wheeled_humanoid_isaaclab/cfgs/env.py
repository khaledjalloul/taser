from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass

from wheeled_humanoid_isaaclab.cfgs.actions import ActionsCfg
from wheeled_humanoid_isaaclab.cfgs.event import EventCfg
from wheeled_humanoid_isaaclab.cfgs.observations import ObservationsCfg
from wheeled_humanoid_isaaclab.cfgs.scene import WheeledHumanoidSceneCfg
from wheeled_humanoid_isaaclab.cfgs.rewards import RewardsCfg
from wheeled_humanoid_isaaclab.cfgs.terminations import TerminationsCfg


@configclass
class WheeledHumanoidEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the wheeled humanoid environment."""

    scene = WheeledHumanoidSceneCfg(
        num_envs=1,
        env_spacing=10
    )
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
