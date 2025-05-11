from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.utils import configclass

from cfgs.actions import ActionsCfg
from cfgs.event import EventCfg
from cfgs.observations import ObservationsCfg
from cfgs.scene import WheeledHumanoidSceneCfg


@configclass
class WheeledHumanoidEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the wheeled humanoid environment."""

    scene = WheeledHumanoidSceneCfg(
        num_envs=1,
        env_spacing=10
    )
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz
