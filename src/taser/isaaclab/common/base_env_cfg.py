import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass


@configclass
class TaserBaseSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    num_envs: int = 1
    env_spacing: float = 3.0

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
    )

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )


@configclass
class TaserBaseEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the TASER robot environment."""

    scene = TaserBaseSceneCfg()

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 1
        self.episode_length_s = 60
